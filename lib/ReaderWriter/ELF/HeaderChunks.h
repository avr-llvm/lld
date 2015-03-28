//===- lib/ReaderWriter/ELF/HeaderChunks.h --------------------------------===//
//
//                             The LLVM Linker
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLD_READER_WRITER_ELF_HEADER_CHUNKS_H
#define LLD_READER_WRITER_ELF_HEADER_CHUNKS_H

#include "SegmentChunks.h"
#include "llvm/Object/ELF.h"
#include "llvm/Support/Allocator.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ELF.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/FileOutputBuffer.h"
#include "llvm/Support/Format.h"

/// \brief An Header represents the Elf[32/64]_Ehdr structure at the
///        start of an ELF executable file.
namespace lld {
namespace elf {
template <class ELFT> class ELFHeader : public Chunk<ELFT> {
public:
  typedef llvm::object::Elf_Ehdr_Impl<ELFT> Elf_Ehdr;

  ELFHeader(const ELFLinkingContext &);

  void e_ident(int I, unsigned char C) { _eh.e_ident[I] = C; }
  void e_type(uint16_t type)           { _eh.e_type = type; }
  void e_machine(uint16_t machine)     { _eh.e_machine = machine; }
  void e_version(uint32_t version)     { _eh.e_version = version; }
  void e_entry(int64_t entry)         { _eh.e_entry = entry; }
  void e_phoff(int64_t phoff)         { _eh.e_phoff = phoff; }
  void e_shoff(int64_t shoff)         { _eh.e_shoff = shoff; }
  void e_flags(uint32_t flags)         { _eh.e_flags = flags; }
  void e_ehsize(uint16_t ehsize)       { _eh.e_ehsize = ehsize; }
  void e_phentsize(uint16_t phentsize) { _eh.e_phentsize = phentsize; }
  void e_phnum(uint16_t phnum)         { _eh.e_phnum = phnum; }
  void e_shentsize(uint16_t shentsize) { _eh.e_shentsize = shentsize; }
  void e_shnum(uint16_t shnum)         { _eh.e_shnum = shnum; }
  void e_shstrndx(uint16_t shstrndx)   { _eh.e_shstrndx = shstrndx; }
  uint64_t fileSize() const { return sizeof(Elf_Ehdr); }

  static bool classof(const Chunk<ELFT> *c) {
    return c->Kind() == Chunk<ELFT>::Kind::ELFHeader;
  }

  int getContentType() const { return Chunk<ELFT>::ContentType::Header; }

  void write(ELFWriter *writer, TargetLayout<ELFT> &layout,
             llvm::FileOutputBuffer &buffer);

  virtual void doPreFlight() {}

  void finalize() {
    _eh.e_ident[llvm::ELF::EI_CLASS] =
        (ELFT::Is64Bits) ? llvm::ELF::ELFCLASS64 : llvm::ELF::ELFCLASS32;
    _eh.e_ident[llvm::ELF::EI_DATA] =
        (ELFT::TargetEndianness == llvm::support::little)
            ? llvm::ELF::ELFDATA2LSB
            : llvm::ELF::ELFDATA2MSB;
    _eh.e_type = this->_context.getOutputELFType();
    _eh.e_machine = this->_context.getOutputMachine();
  }

private:
  Elf_Ehdr _eh;
};

template <class ELFT>
ELFHeader<ELFT>::ELFHeader(const ELFLinkingContext &context)
    : Chunk<ELFT>("elfhdr", Chunk<ELFT>::Kind::ELFHeader, context) {
  this->_alignment = ELFT::Is64Bits ? 8 : 4;
  this->_fsize = sizeof(Elf_Ehdr);
  this->_msize = sizeof(Elf_Ehdr);
  memset(_eh.e_ident, 0, llvm::ELF::EI_NIDENT);
  e_ident(llvm::ELF::EI_MAG0, 0x7f);
  e_ident(llvm::ELF::EI_MAG1, 'E');
  e_ident(llvm::ELF::EI_MAG2, 'L');
  e_ident(llvm::ELF::EI_MAG3, 'F');
  e_ehsize(sizeof(Elf_Ehdr));
  e_flags(0);
}

template <class ELFT>
void ELFHeader<ELFT>::write(ELFWriter *writer, TargetLayout<ELFT> &layout,
                            llvm::FileOutputBuffer &buffer) {
  uint8_t *chunkBuffer = buffer.getBufferStart();
  uint8_t *atomContent = chunkBuffer + this->fileOffset();
  memcpy(atomContent, &_eh, fileSize());
}

/// \brief An ProgramHeader represents the Elf[32/64]_Phdr structure at the
///        start of an ELF executable file.
template<class ELFT>
class ProgramHeader : public Chunk<ELFT> {
public:
  typedef llvm::object::Elf_Phdr_Impl<ELFT> Elf_Phdr;
  typedef typename std::vector<Elf_Phdr *>::iterator PhIterT;
  typedef typename std::reverse_iterator<PhIterT> ReversePhIterT;

  /// \brief Find a program header entry, given the type of entry that
  /// we are looking for
  class FindPhdr {
  public:
    FindPhdr(uint64_t type, uint64_t flags, uint64_t flagsClear)
             : _type(type)
             , _flags(flags)
             , _flagsClear(flagsClear) {
    }

    bool operator()(const llvm::object::Elf_Phdr_Impl<ELFT> *j) const {
      return ((j->p_type == _type) &&
              ((j->p_flags & _flags) == _flags) &&
              (!(j->p_flags & _flagsClear)));
    }
  private:
    uint64_t _type;
    uint64_t _flags;
    uint64_t _flagsClear;
  };

  ProgramHeader(const ELFLinkingContext &context)
      : Chunk<ELFT>("elfphdr", Chunk<ELFT>::Kind::ProgramHeader, context) {
    this->_alignment = ELFT::Is64Bits ? 8 : 4;
    resetProgramHeaders();
  }

  bool addSegment(Segment<ELFT> *segment);

  void resetProgramHeaders() { _phi = _ph.begin(); }

  uint64_t fileSize() const { return sizeof(Elf_Phdr) * _ph.size(); }

  static bool classof(const Chunk<ELFT> *c) {
    return c->Kind() == Chunk<ELFT>::Kind::ProgramHeader;
  }

  void write(ELFWriter *writer, TargetLayout<ELFT> &layout,
             llvm::FileOutputBuffer &buffer);

  /// \brief find a program header entry in the list of program headers
  ReversePhIterT
  findProgramHeader(uint64_t type, uint64_t flags, uint64_t flagClear) {
    return std::find_if(_ph.rbegin(), _ph.rend(),
                        FindPhdr(type, flags, flagClear));
  }

  PhIterT begin() {
    return _ph.begin();
  }

  PhIterT end() {
    return _ph.end();
  }

  ReversePhIterT rbegin() { return _ph.rbegin(); }

  ReversePhIterT rend() { return _ph.rend(); }

  virtual void doPreFlight() {}

  void finalize() {}

  int64_t entsize() { return sizeof(Elf_Phdr); }

  int64_t numHeaders() {
    return _ph.size();
  }

  int getContentType() const { return Chunk<ELFT>::ContentType::Header; }

private:
  Elf_Phdr *allocateProgramHeader(bool &allocatedNew) {
    Elf_Phdr *phdr;
    if (_phi == _ph.end()) {
      phdr = new (_allocator) Elf_Phdr;
      _ph.push_back(phdr);
      _phi = _ph.end();
      allocatedNew = true;
    } else {
      phdr = (*_phi);
      ++_phi;
    }
    return phdr;
  }

  std::vector<Elf_Phdr *> _ph;
  PhIterT _phi;
  llvm::BumpPtrAllocator _allocator;
};

template <class ELFT>
bool ProgramHeader<ELFT>::addSegment(Segment<ELFT> *segment) {
  bool allocatedNew = false;
  ELFLinkingContext::OutputMagic outputMagic = this->_context.getOutputMagic();
  // For segments that are not a loadable segment, we
  // just pick the values directly from the segment as there
  // wouldnt be any slices within that
  if (segment->segmentType() != llvm::ELF::PT_LOAD) {
    Elf_Phdr *phdr = allocateProgramHeader(allocatedNew);
    phdr->p_type = segment->segmentType();
    phdr->p_offset = segment->fileOffset();
    phdr->p_vaddr = segment->virtualAddr();
    phdr->p_paddr = segment->virtualAddr();
    phdr->p_filesz = segment->fileSize();
    phdr->p_memsz = segment->memSize();
    phdr->p_flags = segment->flags();
    phdr->p_align = segment->alignment();
    this->_fsize = fileSize();
    this->_msize = this->_fsize;
    return allocatedNew;
  }
  // For all other segments, use the slice
  // to derive program headers
  for (auto slice : segment->slices()) {
    Elf_Phdr *phdr = allocateProgramHeader(allocatedNew);
    phdr->p_type = segment->segmentType();
    phdr->p_offset = slice->fileOffset();
    phdr->p_vaddr = slice->virtualAddr();
    phdr->p_paddr = slice->virtualAddr();
    phdr->p_filesz = slice->fileSize();
    phdr->p_memsz = slice->memSize();
    phdr->p_flags = segment->flags();
    phdr->p_align = slice->alignment();
    uint64_t segPageSize = segment->pageSize();
    uint64_t sliceAlign = slice->alignment();
    // Alignment of PT_LOAD segments are set to the page size, but if the
    // alignment of the slice is greater than the page size, set the alignment
    // of the segment appropriately.
    if (outputMagic != ELFLinkingContext::OutputMagic::NMAGIC &&
        outputMagic != ELFLinkingContext::OutputMagic::OMAGIC) {
      phdr->p_align = (phdr->p_type == llvm::ELF::PT_LOAD)
          ? (segPageSize < sliceAlign) ? sliceAlign : segPageSize
          : sliceAlign;
    } else
      phdr->p_align = slice->alignment();
  }
  this->_fsize = fileSize();
  this->_msize = this->_fsize;

  return allocatedNew;
}

template <class ELFT>
void ProgramHeader<ELFT>::write(ELFWriter *writer, TargetLayout<ELFT> &layout,
                                llvm::FileOutputBuffer &buffer) {
  uint8_t *chunkBuffer = buffer.getBufferStart();
  uint8_t *dest = chunkBuffer + this->fileOffset();
  for (auto phi : _ph) {
    memcpy(dest, phi, sizeof(Elf_Phdr));
    dest += sizeof(Elf_Phdr);
  }
}

/// \brief An SectionHeader represents the Elf[32/64]_Shdr structure
/// at the end of the file
template<class ELFT>
class SectionHeader : public Chunk<ELFT> {
public:
  typedef llvm::object::Elf_Shdr_Impl<ELFT> Elf_Shdr;

  SectionHeader(const ELFLinkingContext &, int32_t order);

  void appendSection(OutputSection<ELFT> *section);

  void updateSection(Section<ELFT> *section);

  static bool classof(const Chunk<ELFT> *c) {
    return c->getChunkKind() == Chunk<ELFT>::Kind::SectionHeader;
  }

  void setStringSection(StringTable<ELFT> *s) {
    _stringSection = s;
  }

  void write(ELFWriter *writer, TargetLayout<ELFT> &layout,
             llvm::FileOutputBuffer &buffer);

  virtual void doPreFlight() {}

  void finalize() {}

  uint64_t fileSize() const { return sizeof(Elf_Shdr) * _sectionInfo.size(); }

  uint64_t entsize() { return sizeof(Elf_Shdr); }

  int getContentType() const { return Chunk<ELFT>::ContentType::Header; }

  uint64_t numHeaders() { return _sectionInfo.size(); }

private:
  StringTable<ELFT> *_stringSection;
  std::vector<Elf_Shdr*>                  _sectionInfo;
  llvm::BumpPtrAllocator                  _sectionAllocate;
};

template <class ELFT>
SectionHeader<ELFT>::SectionHeader(const ELFLinkingContext &context,
                                   int32_t order)
    : Chunk<ELFT>("shdr", Chunk<ELFT>::Kind::SectionHeader, context) {
  this->_fsize = 0;
  this->_alignment = 8;
  this->setOrder(order);
  // The first element in the list is always NULL
  Elf_Shdr *nullshdr = new (_sectionAllocate.Allocate<Elf_Shdr>()) Elf_Shdr;
  ::memset(nullshdr, 0, sizeof (Elf_Shdr));
  _sectionInfo.push_back(nullshdr);
  this->_fsize += sizeof (Elf_Shdr);
}

template <class ELFT>
void SectionHeader<ELFT>::appendSection(OutputSection<ELFT> *section) {
  Elf_Shdr *shdr = new (_sectionAllocate.Allocate<Elf_Shdr>()) Elf_Shdr;
  shdr->sh_name   = _stringSection->addString(section->name());
  shdr->sh_type   = section->type();
  shdr->sh_flags  = section->flags();
  shdr->sh_offset = section->fileOffset();
  shdr->sh_addr   = section->virtualAddr();
  if (section->isLoadableSection())
    shdr->sh_size = section->memSize();
  else
    shdr->sh_size = section->fileSize();
  shdr->sh_link = section->link();
  shdr->sh_info   = section->shinfo();
  shdr->sh_addralign = section->alignment();
  shdr->sh_entsize = section->entsize();
  _sectionInfo.push_back(shdr);
}

template<class ELFT>
void
SectionHeader<ELFT>::updateSection(Section<ELFT> *section) {
  Elf_Shdr *shdr = _sectionInfo[section->ordinal()];
  shdr->sh_type = section->getType();
  shdr->sh_flags = section->getFlags();
  shdr->sh_offset = section->fileOffset();
  shdr->sh_addr   = section->virtualAddr();
  shdr->sh_size   = section->fileSize();
  shdr->sh_link = section->getLink();
  shdr->sh_info = section->getInfo();
  shdr->sh_addralign = section->alignment();
  shdr->sh_entsize = section->getEntSize();
}

template <class ELFT>
void SectionHeader<ELFT>::write(ELFWriter *writer, TargetLayout<ELFT> &layout,
                                llvm::FileOutputBuffer &buffer) {
  uint8_t *chunkBuffer = buffer.getBufferStart();
  uint8_t *dest = chunkBuffer + this->fileOffset();
  for (auto shi : _sectionInfo) {
    memcpy(dest, shi, sizeof(Elf_Shdr));
    dest += sizeof(Elf_Shdr);
  }
  _stringSection->write(writer, layout, buffer);
}
} // end namespace elf
} // end namespace lld

#endif
