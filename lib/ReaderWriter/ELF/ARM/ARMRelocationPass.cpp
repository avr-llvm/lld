//===--------- lib/ReaderWriter/ELF/ARM/ARMRelocationPass.cpp -------------===//
//
//                             The LLVM Linker
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
///
/// \file
/// \brief Defines the relocation processing pass for ARM. This includes
///   GOT and PLT entries, TLS, COPY, and ifunc.
///
/// This also includes additional behavior that gnu-ld and gold implement but
/// which is not specified anywhere.
///
//===----------------------------------------------------------------------===//

#include "ARMRelocationPass.h"
#include "ARMLinkingContext.h"
#include "Atoms.h"
#include "lld/Core/Simple.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/Support/Debug.h"

using namespace lld;
using namespace lld::elf;
using namespace llvm::ELF;

// ARM B/BL instructions of static relocation veneer.
// TODO: consider different instruction set for archs below ARMv5
// (one as for Thumb may be used though it's less optimal).
static const uint8_t Veneer_ARM_B_BL_StaticAtomContent[8] = {
    0x04, 0xf0, 0x1f, 0xe5,  // ldr pc, [pc, #-4]
    0x00, 0x00, 0x00, 0x00   // <target_symbol_address>
};

// Thumb B/BL instructions of static relocation veneer.
// TODO: consider different instruction set for archs above ARMv5
// (one as for ARM may be used since it's more optimal).
static const uint8_t Veneer_THM_B_BL_StaticAtomContent[8] = {
    0x78, 0x47,              // bx pc
    0x00, 0x00,              // nop
    0xfe, 0xff, 0xff, 0xea   // b <target_symbol_address>
};

// .got values
static const uint8_t ARMGotAtomContent[4] = {0};

namespace {
/// \brief Atoms that hold veneer code.
class VeneerAtom : public SimpleELFDefinedAtom {
  StringRef _section;

public:
  VeneerAtom(const File &f, StringRef secName)
      : SimpleELFDefinedAtom(f), _section(secName) {}

  Scope scope() const override { return DefinedAtom::scopeTranslationUnit; }

  SectionChoice sectionChoice() const override {
    return DefinedAtom::sectionBasedOnContent;
  }

  StringRef customSectionName() const override { return _section; }

  ContentType contentType() const override {
    return DefinedAtom::typeCode;
  }

  uint64_t size() const override { return rawContent().size(); }

  ContentPermissions permissions() const override { return permR_X; }

  Alignment alignment() const override { return Alignment(2); }

  StringRef name() const override { return _name; }
  std::string _name;
};

/// \brief Atoms that hold veneer for statically relocated
/// ARM B/BL instructions.
class Veneer_ARM_B_BL_StaticAtom : public VeneerAtom {
public:
  Veneer_ARM_B_BL_StaticAtom(const File &f, StringRef secName)
      : VeneerAtom(f, secName) {}

  ArrayRef<uint8_t> rawContent() const override {
    return llvm::makeArrayRef(Veneer_ARM_B_BL_StaticAtomContent);
  }
};

/// \brief Atoms that hold veneer for statically relocated
/// Thumb B/BL instructions.
class Veneer_THM_B_BL_StaticAtom : public VeneerAtom {
public:
  Veneer_THM_B_BL_StaticAtom(const File &f, StringRef secName)
      : VeneerAtom(f, secName) {}

  DefinedAtom::CodeModel codeModel() const override {
    return DefinedAtom::codeARMThumb;
  }

  ArrayRef<uint8_t> rawContent() const override {
    return llvm::makeArrayRef(Veneer_THM_B_BL_StaticAtomContent);
  }
};

/// \brief Atoms that are used by ARM dynamic linking
class ARMGOTAtom : public GOTAtom {
public:
  ARMGOTAtom(const File &f, StringRef secName) : GOTAtom(f, secName) {}

  ArrayRef<uint8_t> rawContent() const override {
    return llvm::makeArrayRef(ARMGotAtomContent);
  }

  Alignment alignment() const override { return Alignment(2); }
};

class ELFPassFile : public SimpleFile {
public:
  ELFPassFile(const ELFLinkingContext &eti) : SimpleFile("ELFPassFile") {
    setOrdinal(eti.getNextOrdinalAndIncrement());
  }

  llvm::BumpPtrAllocator _alloc;
};

/// \brief CRTP base for handling relocations.
template <class Derived> class ARMRelocationPass : public Pass {
  /// \brief Handle a specific reference.
  void handleReference(const DefinedAtom &atom, const Reference &ref) {
    DEBUG_WITH_TYPE(
        "ARM", llvm::dbgs() << "\t" << LLVM_FUNCTION_NAME << "()"
            << ":   Name of Defined Atom: " << atom.name().str();
        llvm::dbgs() << "   kindValue: " << ref.kindValue() << "\n");
    if (ref.kindNamespace() != Reference::KindNamespace::ELF)
      return;
    assert(ref.kindArch() == Reference::KindArch::ARM);
    switch (ref.kindValue()) {
    case R_ARM_JUMP24:
    case R_ARM_THM_JUMP24:
      static_cast<Derived *>(this)->handleVeneer(atom, ref);
      break;
    case R_ARM_TLS_IE32:
      static_cast<Derived *>(this)->handleTLSIE32(ref);
      break;
    }
  }

protected:
  std::error_code handleVeneer(const DefinedAtom &atom, const Reference &ref) {
    // Target symbol and relocated place should have different
    // instruction sets in order a veneer to be generated in between.
    const auto *target = dyn_cast<DefinedAtom>(ref.target());
    if (!target || target->codeModel() == atom.codeModel())
      return std::error_code();

    // TODO: For unconditional jump instructions (R_ARM_CALL and R_ARM_THM_CALL)
    // fixup isn't possible without veneer generation for archs below ARMv5.

    // Veneers may only be generated for STT_FUNC target symbols
    // or for symbols located in sections different to the place of relocation.
    const auto kindValue = ref.kindValue();
    StringRef secName = atom.customSectionName();
    if (DefinedAtom::typeCode != target->contentType() &&
        !target->customSectionName().equals(secName)) {
      StringRef kindValStr;
      if (!this->_ctx.registry().referenceKindToString(
              ref.kindNamespace(), ref.kindArch(), kindValue, kindValStr)) {
        kindValStr = "unknown";
      }

      std::string errStr =
          (Twine("Reference of type ") + Twine(kindValue) + " (" + kindValStr +
           ") from " + atom.name() + "+" + Twine(ref.offsetInAtom()) + " to " +
           ref.target()->name() + "+" + Twine(ref.addend()) +
           " cannot be effected without a veneer").str();

      llvm_unreachable(errStr.c_str());
    }

    const Atom *veneer = nullptr;
    switch (kindValue) {
    case R_ARM_JUMP24:
      veneer = static_cast<Derived *>(this)
                   ->getVeneer_ARM_B_BL(target, secName);
      break;
    case R_ARM_THM_JUMP24:
      veneer = static_cast<Derived *>(this)
                   ->getVeneer_THM_B_BL(target, secName);
      break;
    default:
      llvm_unreachable("Unhandled reference type for veneer generation");
    }

    assert(veneer && "The veneer is not set");
    const_cast<Reference &>(ref).setTarget(veneer);
    return std::error_code();
  }

  std::error_code handleTLSIE32(const Reference &ref) {
    if (const auto *target = dyn_cast<DefinedAtom>(ref.target())) {
      const_cast<Reference &>(ref).setTarget(
          static_cast<Derived *>(this)->getTLSTPOFF32(target));
      return std::error_code();
    }
    llvm_unreachable("R_ARM_TLS_IE32 reloc targets wrong atom type");
  }

  /// \brief Create a GOT entry for TLS with reloc type and addend specified.
  template <Reference::KindValue R_ARM_TLS, Reference::Addend A = 0>
  const GOTAtom *getGOTTLSEntry(const DefinedAtom *da) {
    auto got = _gotMap.find(da);
    if (got != _gotMap.end())
      return got->second;
    auto g = new (_file._alloc) ARMGOTAtom(_file, ".got");
    g->addReferenceELF_ARM(R_ARM_TLS, 0, da, A);
#ifndef NDEBUG
    g->_name = "__got_tls_";
    g->_name += da->name();
#endif
    _gotMap[da] = g;
    _gotVector.push_back(g);
    return g;
  }

public:
  ARMRelocationPass(const ELFLinkingContext &ctx) : _file(ctx), _ctx(ctx) {}

  /// \brief Do the pass.
  ///
  /// The goal here is to first process each reference individually. Each call
  /// to handleReference may modify the reference itself and/or create new
  /// atoms which must be stored in one of the maps below.
  ///
  /// After all references are handled, the atoms created during that are all
  /// added to mf.
  void perform(std::unique_ptr<MutableFile> &mf) override {
    ScopedTask task(getDefaultDomain(), "ARM GOT/PLT Pass");
    DEBUG_WITH_TYPE(
        "ARM", llvm::dbgs() << "Undefined Atoms" << "\n";
        for (const auto &atom
             : mf->undefined()) {
          llvm::dbgs() << " Name of Atom: " << atom->name().str() << "\n";
        }

        llvm::dbgs() << "Shared Library Atoms" << "\n";
        for (const auto &atom
             : mf->sharedLibrary()) {
          llvm::dbgs() << " Name of Atom: " << atom->name().str() << "\n";
        }

        llvm::dbgs() << "Absolute Atoms" << "\n";
        for (const auto &atom
             : mf->absolute()) {
          llvm::dbgs() << " Name of Atom: " << atom->name().str() << "\n";
        }

        llvm::dbgs() << "Defined Atoms" << "\n";
        for (const auto &atom
             : mf->defined()) {
          llvm::dbgs() << " Name of Atom: " << atom->name().str() << "\n";
        });

    // Process all references.
    for (const auto &atom : mf->defined()) {
      for (const auto &ref : *atom) {
        handleReference(*atom, *ref);
      }
    }

    // Add all created atoms to the link.
    uint64_t ordinal = 0;
    for (auto &got : _gotVector) {
      got->setOrdinal(ordinal++);
      mf->addAtom(*got);
    }
    for (auto &veneer : _veneerVector) {
      veneer->setOrdinal(ordinal++);
      mf->addAtom(*veneer);
    }
  }

protected:
  /// \brief Owner of all the Atoms created by this pass.
  ELFPassFile _file;
  const ELFLinkingContext &_ctx;

  /// \brief Map Atoms to their GOT entries.
  llvm::DenseMap<const Atom *, GOTAtom *> _gotMap;

  /// \brief Map Atoms to their veneers.
  llvm::DenseMap<const Atom *, VeneerAtom *> _veneerMap;

  /// \brief the list of GOT/PLT atoms
  std::vector<GOTAtom *> _gotVector;

  /// \brief the list of veneer atoms.
  std::vector<VeneerAtom *> _veneerVector;
};

/// This implements the static relocation model. Meaning GOT and PLT entries are
/// not created for references that can be directly resolved. These are
/// converted to a direct relocation. For entries that do require a GOT or PLT
/// entry, that entry is statically bound.
///
/// TLS always assumes module 1 and attempts to remove indirection.
class ARMStaticRelocationPass final
    : public ARMRelocationPass<ARMStaticRelocationPass> {
public:
  ARMStaticRelocationPass(const elf::ARMLinkingContext &ctx)
      : ARMRelocationPass(ctx) {}

  /// \brief Get the veneer for ARM B/BL instructions.
  const VeneerAtom *getVeneer_ARM_B_BL(const DefinedAtom *da,
                                       StringRef secName) {
    auto veneer = _veneerMap.find(da);
    if (_veneerMap.end() != veneer)
      return veneer->second;

    auto v = new (_file._alloc) Veneer_ARM_B_BL_StaticAtom(_file, secName);
    v->addReferenceELF_ARM(R_ARM_ABS32, 4, da, 0);

    v->_name = "__";
    v->_name += da->name();
    v->_name += "_from_arm";

    _veneerMap[da] = v;
    _veneerVector.push_back(v);
    return v;
  }

  /// \brief Get the veneer for Thumb B/BL instructions.
  const VeneerAtom *getVeneer_THM_B_BL(const DefinedAtom *da,
                                       StringRef secName) {
    auto veneer = _veneerMap.find(da);
    if (_veneerMap.end() != veneer)
      return veneer->second;

    auto v = new (_file._alloc) Veneer_THM_B_BL_StaticAtom(_file, secName);
    v->addReferenceELF_ARM(R_ARM_JUMP24, 4, da, 0);

    v->_name = "__";
    v->_name += da->name();
    v->_name += "_from_thumb";

    _veneerMap[da] = v;
    _veneerVector.push_back(v);
    return v;
  }

  /// \brief Create a GOT entry for R_ARM_TLS_TPOFF32 reloc.
  const GOTAtom *getTLSTPOFF32(const DefinedAtom *da) {
    return getGOTTLSEntry<R_ARM_TLS_LE32>(da);
  }
};

} // end of anon namespace

std::unique_ptr<Pass>
lld::elf::createARMRelocationPass(const ARMLinkingContext &ctx) {
  switch (ctx.getOutputELFType()) {
  case llvm::ELF::ET_EXEC:
    if (ctx.isDynamic())
      llvm_unreachable("Unhandled output file type");
    return llvm::make_unique<ARMStaticRelocationPass>(ctx);
  default:
    llvm_unreachable("Unhandled output file type");
  }
}
