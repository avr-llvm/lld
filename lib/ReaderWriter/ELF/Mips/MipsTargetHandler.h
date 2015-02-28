//===- lib/ReaderWriter/ELF/Mips/MipsTargetHandler.h ----------------------===//
//
//                             The LLVM Linker
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
#ifndef LLD_READER_WRITER_ELF_MIPS_MIPS_TARGET_HANDLER_H
#define LLD_READER_WRITER_ELF_MIPS_MIPS_TARGET_HANDLER_H

#include "DefaultTargetHandler.h"
#include "MipsDynamicLibraryWriter.h"
#include "MipsELFReader.h"
#include "MipsExecutableWriter.h"
#include "MipsLinkingContext.h"
#include "MipsRelocationHandler.h"
#include "MipsSectionChunks.h"
#include "TargetLayout.h"
#include "llvm/ADT/DenseSet.h"

namespace lld {
namespace elf {

/// \brief TargetLayout for Mips
template <class ELFType>
class MipsTargetLayout final : public TargetLayout<ELFType> {
public:
  MipsTargetLayout(const MipsLinkingContext &ctx)
      : TargetLayout<ELFType>(ctx),
        _gotSection(new (_alloc) MipsGOTSection<ELFType>(ctx)),
        _pltSection(new (_alloc) MipsPLTSection<ELFType>(ctx)) {}

  const MipsGOTSection<ELFType> &getGOTSection() const { return *_gotSection; }
  const MipsPLTSection<ELFType> &getPLTSection() const { return *_pltSection; }

  AtomSection<ELFType> *
  createSection(StringRef name, int32_t type,
                DefinedAtom::ContentPermissions permissions,
                Layout::SectionOrder order) override {
    if (type == DefinedAtom::typeGOT && name == ".got")
      return _gotSection;
    if (type == DefinedAtom::typeStub && name == ".plt")
      return _pltSection;
    return DefaultLayout<ELFType>::createSection(name, type, permissions,
                                                 order);
  }

  /// \brief GP offset relative to .got section.
  uint64_t getGPOffset() const { return 0x7FF0; }

  /// \brief Get '_gp' symbol atom layout.
  AtomLayout *getGP() {
    if (!_gpAtom.hasValue()) {
      auto atom = this->findAbsoluteAtom("_gp");
      _gpAtom = atom != this->absoluteAtoms().end() ? *atom : nullptr;
    }
    return *_gpAtom;
  }

  /// \brief Get '_gp_disp' symbol atom layout.
  AtomLayout *getGPDisp() {
    if (!_gpDispAtom.hasValue()) {
      auto atom = this->findAbsoluteAtom("_gp_disp");
      _gpDispAtom = atom != this->absoluteAtoms().end() ? *atom : nullptr;
    }
    return *_gpDispAtom;
  }

  /// \brief Return the section order for a input section
  Layout::SectionOrder getSectionOrder(StringRef name, int32_t contentType,
                                       int32_t contentPermissions) override {
    if ((contentType == DefinedAtom::typeStub) && (name.startswith(".text")))
      return DefaultLayout<ELFType>::ORDER_TEXT;

    return DefaultLayout<ELFType>::getSectionOrder(name, contentType,
                                                   contentPermissions);
  }

private:
  llvm::BumpPtrAllocator _alloc;
  MipsGOTSection<ELFType> *_gotSection;
  MipsPLTSection<ELFType> *_pltSection;
  llvm::Optional<AtomLayout *> _gpAtom;
  llvm::Optional<AtomLayout *> _gpDispAtom;
};

/// \brief Mips Runtime file.
template <class ELFType>
class MipsRuntimeFile final : public CRuntimeFile<ELFType> {
public:
  MipsRuntimeFile(MipsLinkingContext &ctx)
      : CRuntimeFile<ELFType>(ctx, "Mips runtime file") {}
};

/// \brief Auxiliary class holds relocation's names table.
class MipsRelocationStringTable {
  static const Registry::KindStrings kindStrings[];

public:
  static void registerTable(Registry &registry);
};

/// \brief TargetHandler for Mips
template <class ELFT>
class MipsTargetHandler final : public DefaultTargetHandler<ELFT> {
public:
  MipsTargetHandler(MipsLinkingContext &ctx)
      : _ctx(ctx), _runtimeFile(new MipsRuntimeFile<ELFT>(ctx)),
        _targetLayout(new MipsTargetLayout<ELFT>(ctx)),
        _relocationHandler(createMipsRelocationHandler<ELFT>(ctx)) {}

  MipsTargetLayout<ELFT> &getTargetLayout() override { return *_targetLayout; }

  std::unique_ptr<Reader> getObjReader() override {
    return std::unique_ptr<Reader>(new MipsELFObjectReader<ELFT>(_ctx));
  }

  std::unique_ptr<Reader> getDSOReader() override {
    return std::unique_ptr<Reader>(new MipsELFDSOReader<ELFT>(_ctx));
  }

  const TargetRelocationHandler &getRelocationHandler() const override {
    return *_relocationHandler;
  }

  std::unique_ptr<Writer> getWriter() override {
    switch (_ctx.getOutputELFType()) {
    case llvm::ELF::ET_EXEC:
      return std::unique_ptr<Writer>(
          new MipsExecutableWriter<ELFT>(_ctx, *_targetLayout));
    case llvm::ELF::ET_DYN:
      return std::unique_ptr<Writer>(
          new MipsDynamicLibraryWriter<ELFT>(_ctx, *_targetLayout));
    case llvm::ELF::ET_REL:
      llvm_unreachable("TODO: support -r mode");
    default:
      llvm_unreachable("unsupported output type");
    }
  }

  void registerRelocationNames(Registry &registry) override {
    MipsRelocationStringTable::registerTable(registry);
  }

private:
  MipsLinkingContext &_ctx;
  std::unique_ptr<MipsRuntimeFile<ELFT>> _runtimeFile;
  std::unique_ptr<MipsTargetLayout<ELFT>> _targetLayout;
  std::unique_ptr<TargetRelocationHandler> _relocationHandler;
};

template <class ELFT> class MipsSymbolTable : public SymbolTable<ELFT> {
public:
  typedef llvm::object::Elf_Sym_Impl<ELFT> Elf_Sym;

  MipsSymbolTable(const ELFLinkingContext &ctx)
      : SymbolTable<ELFT>(ctx, ".symtab",
                          DefaultLayout<ELFT>::ORDER_SYMBOL_TABLE) {}

  void addDefinedAtom(Elf_Sym &sym, const DefinedAtom *da,
                      int64_t addr) override {
    SymbolTable<ELFT>::addDefinedAtom(sym, da, addr);

    switch (da->codeModel()) {
    case DefinedAtom::codeMipsMicro:
      sym.st_other |= llvm::ELF::STO_MIPS_MICROMIPS;
      break;
    case DefinedAtom::codeMipsMicroPIC:
      sym.st_other |= llvm::ELF::STO_MIPS_MICROMIPS | llvm::ELF::STO_MIPS_PIC;
      break;
    default:
      break;
    }
  }

  void finalize(bool sort = true) override {
    SymbolTable<ELFT>::finalize(sort);

    for (auto &ste : this->_symbolTable) {
      if (!ste._atom)
        continue;
      if (const auto *da = dyn_cast<DefinedAtom>(ste._atom)) {
        if (da->codeModel() == DefinedAtom::codeMipsMicro ||
            da->codeModel() == DefinedAtom::codeMipsMicroPIC) {
          // Adjust dynamic microMIPS symbol value. That allows a dynamic
          // linker to recognize and handle this symbol correctly.
          ste._symbol.st_value = ste._symbol.st_value | 1;
        }
      }
    }
  }
};

template <class ELFT>
class MipsDynamicSymbolTable : public DynamicSymbolTable<ELFT> {
public:
  MipsDynamicSymbolTable(const ELFLinkingContext &ctx,
                         MipsTargetLayout<ELFT> &layout)
      : DynamicSymbolTable<ELFT>(ctx, layout, ".dynsym",
                                 DefaultLayout<ELFT>::ORDER_DYNAMIC_SYMBOLS),
        _targetLayout(layout) {}

  void sortSymbols() override {
    typedef typename DynamicSymbolTable<ELFT>::SymbolEntry SymbolEntry;
    std::stable_sort(this->_symbolTable.begin(), this->_symbolTable.end(),
                     [this](const SymbolEntry &A, const SymbolEntry &B) {
      if (A._symbol.getBinding() != STB_GLOBAL &&
          B._symbol.getBinding() != STB_GLOBAL)
        return A._symbol.getBinding() < B._symbol.getBinding();

      return _targetLayout.getGOTSection().compare(A._atom, B._atom);
    });
  }

  void finalize() override {
    DynamicSymbolTable<ELFT>::finalize();

    const auto &pltSection = _targetLayout.getPLTSection();

    for (auto &ste : this->_symbolTable) {
      const Atom *a = ste._atom;
      if (!a)
        continue;
      if (auto *layout = pltSection.findPLTLayout(a)) {
        a = layout->_atom;
        // Under some conditions a dynamic symbol table record should hold
        // a symbol value of the corresponding PLT entry. For details look
        // at the PLT entry creation code in the class MipsRelocationPass.
        // Let's update atomLayout fields for such symbols.
        assert(!ste._atomLayout);
        ste._symbol.st_value = layout->_virtualAddr;
        ste._symbol.st_other |= ELF::STO_MIPS_PLT;
      }

      if (const auto *da = dyn_cast<DefinedAtom>(a)) {
        if (da->codeModel() == DefinedAtom::codeMipsMicro ||
            da->codeModel() == DefinedAtom::codeMipsMicroPIC) {
          // Adjust dynamic microMIPS symbol value. That allows a dynamic
          // linker to recognize and handle this symbol correctly.
          ste._symbol.st_value = ste._symbol.st_value | 1;
        }
      }
    }
  }

private:
  MipsTargetLayout<ELFT> &_targetLayout;
};

} // end namespace elf
} // end namespace lld

#endif
