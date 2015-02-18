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
    
    llvm_unreachable("unimplemented");
    return 0;
    
    /*const uint8_t *ap = content.data() + ri.r_offset;
    switch (ri.getType(isAVR64EL())) {
    case llvm::ELF::R_AVR_32:
    case llvm::ELF::R_AVR_GPREL32:
    case llvm::ELF::R_AVR_PC32:
      return readAddend(ap, 0xffffffff, false);
    case llvm::ELF::R_AVR_26:
      return readAddend(ap, 0x3ffffff, false) << 2;
    case llvm::ELF::R_AVR_HI16:
    case llvm::ELF::R_AVR_LO16:
    case llvm::ELF::R_AVR_GPREL16:
    case llvm::ELF::R_AVR_GOT16:
    case llvm::ELF::R_AVR_TLS_DTPREL_HI16:
    case llvm::ELF::R_AVR_TLS_DTPREL_LO16:
    case llvm::ELF::R_AVR_TLS_TPREL_HI16:
    case llvm::ELF::R_AVR_TLS_TPREL_LO16:
      return readAddend(ap, 0xffff, false);
    case llvm::ELF::R_MICROAVR_TLS_DTPREL_HI16:
    case llvm::ELF::R_MICROAVR_TLS_DTPREL_LO16:
    case llvm::ELF::R_MICROAVR_TLS_TPREL_HI16:
    case llvm::ELF::R_MICROAVR_TLS_TPREL_LO16:
      return readAddend(ap, 0xffff, true);
    case llvm::ELF::R_MICROAVR_26_S1:
      return readAddend(ap, 0x3ffffff, true) << 1;
    case llvm::ELF::R_MICROAVR_HI16:
    case llvm::ELF::R_MICROAVR_LO16:
    case llvm::ELF::R_MICROAVR_GOT16:
      return readAddend(ap, 0xffff, true);
    case llvm::ELF::R_MICROAVR_PC16_S1:
      return readAddend(ap, 0xffff, true) << 1;
    case llvm::ELF::R_MICROAVR_PC7_S1:
      return readAddend(ap, 0x7f, false) << 1;
    case llvm::ELF::R_MICROAVR_PC10_S1:
      return readAddend(ap, 0x3ff, false) << 1;
    case llvm::ELF::R_MICROAVR_PC23_S2:
      return readAddend(ap, 0x7fffff, true) << 2;
    case llvm::ELF::R_AVR_CALL16:
    case llvm::ELF::R_AVR_TLS_GD:
    case llvm::ELF::R_AVR_TLS_LDM:
    case llvm::ELF::R_AVR_TLS_GOTTPREL:
    case llvm::ELF::R_MICROAVR_CALL16:
    case llvm::ELF::R_MICROAVR_TLS_GD:
    case llvm::ELF::R_MICROAVR_TLS_LDM:
    case llvm::ELF::R_MICROAVR_TLS_GOTTPREL:
      return 0;
    default:
      return 0;
    }*/
  }

  uint32_t getPairRelocation(const Elf_Rel &rel) {
    
    llvm_unreachable("unimplemented");
    /*switch (rel.getType(isAVR64EL())) {
    case llvm::ELF::R_AVR_HI16:
      return llvm::ELF::R_AVR_LO16;
    case llvm::ELF::R_AVR_GOT16:
      if (isLocalBinding(rel))
        return llvm::ELF::R_AVR_LO16;
      break;
    case llvm::ELF::R_MICROAVR_HI16:
      return llvm::ELF::R_MICROAVR_LO16;
    case llvm::ELF::R_MICROAVR_GOT16:
      if (isLocalBinding(rel))
        return llvm::ELF::R_MICROAVR_LO16;
      break;
    default:
      // Nothing to do.
      break;
    }*/
    return llvm::ELF::R_AVR_NONE;
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
