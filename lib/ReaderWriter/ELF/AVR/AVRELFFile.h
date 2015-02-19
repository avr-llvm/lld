//===- lib/ReaderWriter/ELF/AVRELFFile.h ---------------------------------===//
//
//                             The LLVM Linker
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
#ifndef LLD_READER_WRITER_ELF_AVR_AVR_ELF_FILE_H
#define LLD_READER_WRITER_ELF_AVR_AVR_ELF_FILE_H

#include "ELFReader.h"
#include "AVRLinkingContext.h"
#include "llvm/Support/Endian.h"

// FIXME: we often call rit->getType(false), rit->getSymbol(false)
// The boolean indicates `isMips64EL`. The function is deifined in LLVM.
// It is some weird template magic.

namespace llvm {
namespace object {

template <class ELFT>
struct Elf_RegInfo;

template <llvm::support::endianness TargetEndianness, std::size_t MaxAlign>
struct Elf_RegInfo<ELFType<TargetEndianness, MaxAlign, false>> {
  LLVM_ELF_IMPORT_TYPES(TargetEndianness, MaxAlign, false)
  Elf_Word ri_gprmask;     // bit-mask of used general registers
  Elf_Word ri_cprmask[4];  // bit-mask of used co-processor registers
  Elf_Addr ri_gp_value;    // gp register value
};

template <llvm::support::endianness TargetEndianness, std::size_t MaxAlign>
struct Elf_RegInfo<ELFType<TargetEndianness, MaxAlign, true>> {
  LLVM_ELF_IMPORT_TYPES(TargetEndianness, MaxAlign, true)
  Elf_Word ri_gprmask;     // bit-mask of used general registers
  Elf_Word ri_pad;         // unused padding field
  Elf_Word ri_cprmask[4];  // bit-mask of used co-processor registers
  Elf_Addr ri_gp_value;    // gp register value
};

template <class ELFT> struct Elf_AVR_Options {
  LLVM_ELF_IMPORT_TYPES(ELFT::TargetEndianness, ELFT::MaxAlignment,
                        ELFT::Is64Bits)
  uint8_t kind;     // Determines interpretation of variable part of descriptor
  uint8_t size;     // Byte size of descriptor, including this header
  Elf_Half section; // Section header index of section affected,
                    // or 0 for global options
  Elf_Word info;    // Kind-specific information
};

} // end namespace object.
} // end namespace llvm.

namespace lld {
namespace elf {

template <class ELFT> class AVRELFFile;

template <class ELFT>
class AVRELFDefinedAtom : public ELFDefinedAtom<ELFT> {
  typedef llvm::object::Elf_Sym_Impl<ELFT> Elf_Sym;
  typedef llvm::object::Elf_Shdr_Impl<ELFT> Elf_Shdr;

public:
  AVRELFDefinedAtom(const AVRELFFile<ELFT> &file, StringRef symbolName,
                     StringRef sectionName, const Elf_Sym *symbol,
                     const Elf_Shdr *section, ArrayRef<uint8_t> contentData,
                     unsigned int referenceStart, unsigned int referenceEnd,
                     std::vector<ELFReference<ELFT> *> &referenceList)
      : ELFDefinedAtom<ELFT>(file, symbolName, sectionName, symbol, section,
                             contentData, referenceStart, referenceEnd,
                             referenceList) {}

  const AVRELFFile<ELFT>& file() const override {
    return static_cast<const AVRELFFile<ELFT> &>(this->_owningFile);
  }

  DefinedAtom::CodeModel codeModel() const override {
    /*switch (this->_symbol->st_other & llvm::ELF::STO_AVR_AVR16) {
    case llvm::ELF::STO_AVR_AVR16:
      return DefinedAtom::codeAVR16;
    case llvm::ELF::STO_AVR_PIC:
      return DefinedAtom::codeAVRPIC;
    case llvm::ELF::STO_AVR_MICROAVR:
      return DefinedAtom::codeAVRMicro;
    case llvm::ELF::STO_AVR_MICROAVR | llvm::ELF::STO_AVR_PIC:
      return DefinedAtom::codeAVRMicroPIC;
    default:
      return DefinedAtom::codeNA;
    }*/
    llvm_unreachable("unimplemented");
    return DefinedAtom::codeNA;
  }
};

template <class ELFT> class AVRELFFile : public ELFFile<ELFT> {
public:
  AVRELFFile(std::unique_ptr<MemoryBuffer> mb, AVRLinkingContext &ctx)
      : ELFFile<ELFT>(std::move(mb), ctx) {}

  static ErrorOr<std::unique_ptr<AVRELFFile>>
  create(std::unique_ptr<MemoryBuffer> mb, AVRLinkingContext &ctx) {
    return std::unique_ptr<AVRELFFile<ELFT>>(
        new AVRELFFile<ELFT>(std::move(mb), ctx));
  }

private:
  typedef llvm::object::Elf_Sym_Impl<ELFT> Elf_Sym;
  typedef llvm::object::Elf_Shdr_Impl<ELFT> Elf_Shdr;
  typedef llvm::object::Elf_Rel_Impl<ELFT, false> Elf_Rel;
  typedef typename llvm::object::ELFFile<ELFT>::Elf_Rel_Iter Elf_Rel_Iter;

  ErrorOr<ELFDefinedAtom<ELFT> *> handleDefinedSymbol(
      StringRef symName, StringRef sectionName, const Elf_Sym *sym,
      const Elf_Shdr *sectionHdr, ArrayRef<uint8_t> contentData,
      unsigned int referenceStart, unsigned int referenceEnd,
      std::vector<ELFReference<ELFT> *> &referenceList) override {
    return new (this->_readerStorage) AVRELFDefinedAtom<ELFT>(
        *this, symName, sectionName, sym, sectionHdr, contentData,
        referenceStart, referenceEnd, referenceList);
  }

  const Elf_Shdr *findSectionByType(uint64_t type) {
    for (const Elf_Shdr &section : this->_objFile->sections())
      if (section.sh_type == type)
        return &section;
    return nullptr;
  }

  const Elf_Shdr *findSectionByFlags(uint64_t flags) {
    for (const Elf_Shdr &section : this->_objFile->sections())
      if (section.sh_flags & flags)
        return &section;
    return nullptr;
  }
  
  void createRelocationReferences(const Elf_Sym &symbol,
                                  ArrayRef<uint8_t> symContent,
                                  ArrayRef<uint8_t> secContent,
                                  range<Elf_Rel_Iter> rels) override {
    for (Elf_Rel_Iter rit = rels.begin(), eit = rels.end(); rit != eit; ++rit) {
      if (rit->r_offset < symbol.st_value ||
          symbol.st_value + symContent.size() <= rit->r_offset)
        continue;

      this->_references.push_back(new (this->_readerStorage) ELFReference<ELFT>(
          rit->r_offset - symbol.st_value, this->kindArch(),
          rit->getType(false), rit->getSymbol(false)));

      auto addend = getAddend(*rit, secContent);
      auto pairRelType = getPairRelocation(*rit);
      if (pairRelType != llvm::ELF::R_AVR_NONE) {
        addend <<= 16;
        auto mit = findMatchingRelocation(pairRelType, rit, eit);
        if (mit != eit)
          addend += int16_t(getAddend(*mit, secContent));
        else
          // FIXME (simon): Show detailed warning.
          llvm::errs() << "lld warning: cannot matching LO16 relocation\n";
      }
      this->_references.back()->setAddend(addend);
    }
  }

  static Reference::Addend readAddend(const uint8_t *data, uint32_t mask,
                                      bool shuffle) {
    using namespace llvm::support;
    uint32_t ins = endian::read<uint32_t, ELFT::TargetEndianness, 1>(data);
    if (shuffle)
      ins = ((ins & 0xffff) << 16) | ((ins & 0xffff0000) >> 16);
    return ins & mask;
  }

  Reference::Addend getAddend(const Elf_Rel &ri,
                              const ArrayRef<uint8_t> content) const {
    using namespace llvm::support;
    
    const uint8_t *ap = content.data() + ri.r_offset;

    switch (ri.getType(false)) {
    case llvm::ELF::R_AVR_32:
      return readAddend(ap, 0xffffffff, false);
    case llvm::ELF::R_AVR_7_PCREL:
    case llvm::ELF::R_AVR_LDS_STS_16:
      return readAddend(ap, 0x7f, false);
    case llvm::ELF::R_AVR_13_PCREL:
      return readAddend(ap, 0xfff, false);
    case llvm::ELF::R_AVR_16:
    case llvm::ELF::R_AVR_16_PM:
    case llvm::ELF::R_AVR_16_LDST:
      return readAddend(ap, 0xffff, false);
    case llvm::ELF::R_AVR_PORT6:
      return readAddend(ap, 0x3f, false);
    case llvm::ELF::R_AVR_PORT5:
      return readAddend(ap, 0x1f, false);
    case llvm::ELF::R_AVR_LO8_LDI:
    case llvm::ELF::R_AVR_HI8_LDI:
    case llvm::ELF::R_AVR_HH8_LDI:
    case llvm::ELF::R_AVR_LO8_LDI_NEG:
    case llvm::ELF::R_AVR_HI8_LDI_NEG:
    case llvm::ELF::R_AVR_HH8_LDI_NEG:
    case llvm::ELF::R_AVR_LO8_LDI_PM:
    case llvm::ELF::R_AVR_HI8_LDI_PM:
    case llvm::ELF::R_AVR_HH8_LDI_PM:
    case llvm::ELF::R_AVR_LO8_LDI_PM_NEG:
    case llvm::ELF::R_AVR_HI8_LDI_PM_NEG:
    case llvm::ELF::R_AVR_HH8_LDI_PM_NEG:
    case llvm::ELF::R_AVR_MS8_LDI:
    case llvm::ELF::R_AVR_MS8_LDI_NEG:
    case llvm::ELF::R_AVR_LO8_LDI_GS:
    case llvm::ELF::R_AVR_HI8_LDI_GS:
    case llvm::ELF::R_AVR_8:
    case llvm::ELF::R_AVR_8_LO8:
    case llvm::ELF::R_AVR_8_HI8:
    case llvm::ELF::R_AVR_8_HLO8:
      return readAddend(ap, 0xff, false);
    case llvm::ELF::R_AVR_CALL:
      llvm_unreachable("addend R_AVR_CALL not implemented");
      return 0;
    case llvm::ELF::R_AVR_LDI:
      llvm_unreachable("addend R_AVR_LDI not implemented");
      return 0;
    case llvm::ELF::R_AVR_6:
    case llvm::ELF::R_AVR_6_ADIW:
      return readAddend(ap, 0x3f, false);
    case llvm::ELF::R_AVR_SYM_DIFF:
      llvm_unreachable("addend R_AVR_SYM_DIFF not implemented");
      return 0;
    default:
      llvm_unreachable("unimplemented relocation addend");
      return 0;
    }
  }

  uint32_t getPairRelocation(const Elf_Rel &rel) {
    // FIXME: implement

    switch (rel.getType(false)) {
      default:
      // Nothing to do.
      return llvm::ELF::R_AVR_NONE;
    }
  }

  Elf_Rel_Iter findMatchingRelocation(uint32_t pairRelType, Elf_Rel_Iter rit,
                                      Elf_Rel_Iter eit) {
    return std::find_if(rit, eit, [&](const Elf_Rel &rel) {
      return rel.getType(false) == pairRelType &&
             rel.getSymbol(false) == rit->getSymbol(false);
    });
  }
};

template <class ELFT> class AVRDynamicFile : public DynamicFile<ELFT> {
public:
  AVRDynamicFile(const AVRLinkingContext &context, StringRef name)
      : DynamicFile<ELFT>(context, name) {}
};

} // elf
} // lld

#endif
