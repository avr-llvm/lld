//===- lib/ReaderWriter/ELF/SegmentChunks.h -------------------------------===//
//
//                             The LLVM Linker
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLD_READER_WRITER_ELF_SEGMENT_CHUNKS_H
#define LLD_READER_WRITER_ELF_SEGMENT_CHUNKS_H

#include "Chunk.h"
#include "Layout.h"
#include "SectionChunks.h"
#include "Writer.h"
#include "lld/Core/range.h"
#include "lld/Core/Writer.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/Object/ELF.h"
#include "llvm/Support/Allocator.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ELF.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/FileOutputBuffer.h"
#include <memory>

namespace lld {
namespace elf {

template <typename ELFT> class DefaultLayout;

/// \brief A segment can be divided into segment slices
///        depending on how the segments can be split
template<class ELFT>
class SegmentSlice {
public:
  typedef typename std::vector<Chunk<ELFT> *>::iterator SectionIter;

  SegmentSlice() { }

  /// Set the start of the slice.
  void setStart(int32_t s) { _startSection = s; }

  // Set the segment slice start and end iterators. This is used to walk through
  // the sections that are part of the Segment slice
  void setSections(range<SectionIter> sections) { _sections = sections; }

  // Return the fileOffset of the slice
  uint64_t fileOffset() const { return _offset; }

  void setFileOffset(uint64_t offset) { _offset = offset; }

  // Return the size of the slice
  uint64_t fileSize() const { return _fsize; }

  void setFileSize(uint64_t filesz) { _fsize = filesz; }

  // Return the start of the slice
  int32_t startSection() const { return _startSection; }

  // Return the start address of the slice
  uint64_t virtualAddr() const { return _addr; }

  // Return the memory size of the slice
  uint64_t memSize() const { return _memSize; }

  // Return the alignment of the slice
  uint64_t alignment() const { return _alignment; }

  void setMemSize(uint64_t memsz) { _memSize = memsz; }

  void setVirtualAddr(uint64_t addr) { _addr = addr; }

  void setAlign(uint64_t align) { _alignment = align; }

  static bool compare_slices(SegmentSlice<ELFT> *a, SegmentSlice<ELFT> *b) {
    return a->startSection() < b->startSection();
  }

  range<SectionIter> sections() { return _sections; }

private:
  range<SectionIter> _sections;
  int32_t _startSection;
  uint64_t _addr;
  uint64_t _offset;
  uint64_t _alignment;
  uint64_t _fsize;
  uint64_t _memSize;
};

/// \brief A segment contains a set of sections, that have similar properties
//  the sections are already separated based on different flags and properties
//  the segment is just a way to concatenate sections to segments
template<class ELFT>
class Segment : public Chunk<ELFT> {
public:
  typedef typename std::vector<SegmentSlice<ELFT> *>::iterator SliceIter;
  typedef typename std::vector<Chunk<ELFT> *>::iterator SectionIter;

  Segment(const ELFLinkingContext &context, StringRef name,
          const Layout::SegmentType type);

  /// \brief the Order of segments that appear in the output file
  enum SegmentOrder {
    permUnknown,
    permRWX,
    permRX,
    permR,
    permRWL,
    permRW,
    permNonAccess
  };

  /// append a section to a segment
  virtual void append(Chunk<ELFT> *chunk);

  /// Sort segments depending on the property
  /// If we have a Program Header segment, it should appear first
  /// If we have a INTERP segment, that should appear after the Program Header
  /// All Loadable segments appear next in this order
  /// All Read Write Execute segments follow
  /// All Read Execute segments appear next
  /// All Read only segments appear first
  /// All Write execute segments follow
  static bool compareSegments(Segment<ELFT> *sega, Segment<ELFT> *segb);

  /// \brief Start assigning file offset to the segment chunks The fileoffset
  /// needs to be page at the start of the segment and in addition the
  /// fileoffset needs to be aligned to the max section alignment within the
  /// segment. This is required so that the ELF property p_poffset % p_align =
  /// p_vaddr mod p_align holds true.
  /// The algorithm starts off by assigning the startOffset thats passed in as
  /// parameter to the first section in the segment, if the difference between
  /// the newly computed offset is greater than a page, then we create a segment
  /// slice, as it would be a waste of virtual memory just to be filled with
  /// zeroes
  void assignFileOffsets(uint64_t startOffset);

  /// \brief Assign virtual addresses to the slices
  void assignVirtualAddress(uint64_t addr);

  // Write the Segment
  void write(ELFWriter *writer, TargetLayout<ELFT> &layout,
             llvm::FileOutputBuffer &buffer);

  int64_t flags() const;

  /// Prepend a generic chunk to the segment.
  void prepend(Chunk<ELFT> *c) {
    _sections.insert(_sections.begin(), c);
  }

  /// Finalize the segment before assigning File Offsets / Virtual addresses
  void doPreFlight() {}

  /// Finalize the segment, before we want to write the segment header
  /// information
  void finalize() {
    // We want to finalize the segment values for now only for non loadable
    // segments, since those values are not set in the Layout
    if (_segmentType == llvm::ELF::PT_LOAD)
      return;
    // The size is the difference of the
    // last section to the first section, especially for TLS because
    // the TLS segment contains both .tdata/.tbss
    this->setFileOffset(_sections.front()->fileOffset());
    this->setVirtualAddr(_sections.front()->virtualAddr());
    size_t startFileOffset = _sections.front()->fileOffset();
    size_t startAddr = _sections.front()->virtualAddr();
    for (auto ai : _sections) {
      this->_fsize = ai->fileOffset() + ai->fileSize() - startFileOffset;
      this->_msize = ai->virtualAddr() + ai->memSize() - startAddr;
    }
  }

  // For LLVM RTTI
  static bool classof(const Chunk<ELFT> *c) {
    return c->kind() == Chunk<ELFT>::Kind::ELFSegment;
  }

  // Getters
  int32_t sectionCount() const { return _sections.size(); }

  /// \brief, this function returns the type of segment (PT_*)
  Layout::SegmentType segmentType() { return _segmentType; }

  /// \brief return the segment type depending on the content,
  /// If the content corresponds to Code, this will return Segment::Code
  /// If the content corresponds to Data, this will return Segment::Data
  /// If the content corresponds to TLS, this will return Segment::TLS
  virtual int getContentType() const {
    int64_t fl = flags();
    switch (_segmentType) {
    case llvm::ELF::PT_LOAD: {
      if (fl && llvm::ELF::PF_X)
        return Chunk<ELFT>::ContentType::Code;
      if (fl && llvm::ELF::PF_W)
        return Chunk<ELFT>::ContentType::Data;
    }
    case llvm::ELF::PT_TLS:
      return Chunk<ELFT>::ContentType::TLS;
    case llvm::ELF::PT_NOTE:
      return Chunk<ELFT>::ContentType::Note;
    default:
      return Chunk<ELFT>::ContentType::Unknown;
    }
  }

  int pageSize() const { return this->_context.getPageSize(); }

  int rawflags() const { return _atomflags; }

  int64_t atomflags() const {
    switch (_atomflags) {

    case DefinedAtom::permUnknown:
      return permUnknown;

    case DefinedAtom::permRWX:
      return permRWX;

    case DefinedAtom::permR_X:
      return permRX;

    case DefinedAtom::permR__:
      return permR;

    case DefinedAtom::permRW_L:
      return permRWL;

    case DefinedAtom::permRW_:
      return permRW;

    case DefinedAtom::perm___:
    default:
      return permNonAccess;
    }
  }

  int64_t numSlices() const { return _segmentSlices.size(); }

  range<SliceIter> slices() { return _segmentSlices; }

  Chunk<ELFT> *firstSection() { return _sections[0]; }

private:

  /// \brief Check if the chunk needs to be aligned
  bool needAlign(Chunk<ELFT> *chunk) const {
    if (chunk->getContentType() == Chunk<ELFT>::ContentType::Data &&
        _outputMagic == ELFLinkingContext::OutputMagic::NMAGIC)
      return true;
    return false;
  }

  // Cached value of outputMagic
  ELFLinkingContext::OutputMagic _outputMagic;

protected:
  /// \brief Section or some other chunk type.
  std::vector<Chunk<ELFT> *> _sections;
  std::vector<SegmentSlice<ELFT> *> _segmentSlices;
  Layout::SegmentType _segmentType;
  uint64_t _flags;
  int64_t _atomflags;
  llvm::BumpPtrAllocator _segmentAllocate;
};

/// This chunk represents a linker script expression that needs to be calculated
/// at the time the virtual addresses for the parent segment are being assigned.
template <class ELFT> class ExpressionChunk : public Chunk<ELFT> {
public:
  ExpressionChunk(ELFLinkingContext &ctx, const script::SymbolAssignment *expr)
      : Chunk<ELFT>(StringRef(), Chunk<ELFT>::Kind::Expression, ctx),
        _expr(expr), _linkerScriptSema(ctx.linkerScriptSema()) {
    this->_alignment = 1;
  }

  static bool classof(const Chunk<ELFT> *c) {
    return c->kind() == Chunk<ELFT>::Kind::Expression;
  }

  int getContentType() const override {
    return Chunk<ELFT>::ContentType::Unknown;
  }
  void write(ELFWriter *, TargetLayout<ELFT> &,
             llvm::FileOutputBuffer &) override {}
  void doPreFlight() override {}
  void finalize() override {}

  std::error_code evalExpr(uint64_t &curPos) {
    return _linkerScriptSema.evalExpr(_expr, curPos);
  }

private:
  const script::SymbolAssignment *_expr;
  script::Sema &_linkerScriptSema;
};

/// \brief A Program Header segment contains a set of chunks instead of sections
/// The segment doesn't contain any slice
template <class ELFT> class ProgramHeaderSegment : public Segment<ELFT> {
public:
  ProgramHeaderSegment(const ELFLinkingContext &context)
      : Segment<ELFT>(context, "PHDR", llvm::ELF::PT_PHDR) {
    this->_alignment = 8;
    this->_flags = (llvm::ELF::SHF_ALLOC | llvm::ELF::SHF_EXECINSTR);
  }

  /// Finalize the segment, before we want to write the segment header
  /// information
  void finalize() {
    // If the segment is of type Program Header, then the values fileOffset
    // and the fileSize need to be picked up from the last section, the first
    // section points to the ELF header and the second chunk points to the
    // actual program headers
    this->setFileOffset(this->_sections.back()->fileOffset());
    this->setVirtualAddr(this->_sections.back()->virtualAddr());
    this->_fsize = this->_sections.back()->fileSize();
    this->_msize = this->_sections.back()->memSize();
  }

};

template <class ELFT>
Segment<ELFT>::Segment(const ELFLinkingContext &context, StringRef name,
                       const Layout::SegmentType type)
    : Chunk<ELFT>(name, Chunk<ELFT>::Kind::ELFSegment, context),
      _segmentType(type), _flags(0), _atomflags(0) {
  this->_alignment = 0;
  this->_fsize = 0;
  _outputMagic = context.getOutputMagic();
}

// This function actually is used, but not in all instantiations of Segment.
LLVM_ATTRIBUTE_UNUSED
static DefinedAtom::ContentPermissions toAtomPerms(uint64_t flags) {
  switch (flags & (SHF_ALLOC | SHF_WRITE | SHF_EXECINSTR)) {
  case SHF_ALLOC | SHF_WRITE | SHF_EXECINSTR:
    return DefinedAtom::permRWX;
  case SHF_ALLOC | SHF_EXECINSTR:
    return DefinedAtom::permR_X;
  case SHF_ALLOC:
    return DefinedAtom::permR__;
  case SHF_ALLOC | SHF_WRITE:
    return DefinedAtom::permRW_;
  default:
    return DefinedAtom::permUnknown;
  }
}

template <class ELFT> void Segment<ELFT>::append(Chunk<ELFT> *chunk) {
  _sections.push_back(chunk);
  Section<ELFT> *section = dyn_cast<Section<ELFT>>(chunk);
  if (!section)
    return;
  if (_flags < section->getFlags())
    _flags |= section->getFlags();
  if (_atomflags < toAtomPerms(_flags))
    _atomflags = toAtomPerms(_flags);
  if (this->_alignment < section->alignment())
    this->_alignment = section->alignment();
}

template <class ELFT>
bool Segment<ELFT>::compareSegments(Segment<ELFT> *sega, Segment<ELFT> *segb) {
  int64_t type1 = sega->segmentType();
  int64_t type2 = segb->segmentType();

  if (type1 == type2)
    return sega->atomflags() < segb->atomflags();

  // The single PT_PHDR segment is required to precede any loadable
  // segment.  We simply make it always first.
  if (type1 == llvm::ELF::PT_PHDR)
    return true;
  if (type2 == llvm::ELF::PT_PHDR)
    return false;

  // The single PT_INTERP segment is required to precede any loadable
  // segment.  We simply make it always second.
  if (type1 == llvm::ELF::PT_INTERP)
    return true;
  if (type2 == llvm::ELF::PT_INTERP)
    return false;

  // We then put PT_LOAD segments before any other segments.
  if (type1 == llvm::ELF::PT_LOAD)
    return true;
  if (type2 == llvm::ELF::PT_LOAD)
    return false;

  // We put the PT_GNU_RELRO segment last, because that is where the
  // dynamic linker expects to find it
  if (type1 == llvm::ELF::PT_GNU_RELRO)
    return false;
  if (type2 == llvm::ELF::PT_GNU_RELRO)
    return true;

  // We put the PT_TLS segment last except for the PT_GNU_RELRO
  // segment, because that is where the dynamic linker expects to find
  if (type1 == llvm::ELF::PT_TLS)
    return false;
  if (type2 == llvm::ELF::PT_TLS)
    return true;

  // Otherwise compare the types to establish an arbitrary ordering.
  // FIXME: Should figure out if we should just make all other types compare
  // equal, but if so, we should probably do the same for atom flags and change
  // users of this to use stable_sort.
  return type1 < type2;
}

template <class ELFT>
void Segment<ELFT>::assignFileOffsets(uint64_t startOffset) {
  uint64_t fileOffset = startOffset;
  uint64_t curSliceFileOffset = fileOffset;
  bool isDataPageAlignedForNMagic = false;
  bool alignSegments = this->_context.alignSegments();
  uint64_t p_align = this->_context.getPageSize();
  uint64_t lastVirtualAddress = 0;

  this->setFileOffset(startOffset);
  for (auto &slice : slices()) {
    bool isFirstSection = true;
    for (auto section : slice->sections()) {
      // Handle linker script expressions, which may change the offset
      if (!isFirstSection)
        if (auto expr = dyn_cast<ExpressionChunk<ELFT>>(section))
          fileOffset += expr->virtualAddr() - lastVirtualAddress;
      // Align fileoffset to the alignment of the section.
      fileOffset = llvm::RoundUpToAlignment(fileOffset, section->alignment());
      // If the linker outputmagic is set to OutputMagic::NMAGIC, align the Data
      // to a page boundary
      if (isFirstSection &&
          _outputMagic != ELFLinkingContext::OutputMagic::NMAGIC &&
          _outputMagic != ELFLinkingContext::OutputMagic::OMAGIC) {
        // Align to a page only if the output is not
        // OutputMagic::NMAGIC/OutputMagic::OMAGIC
        if (alignSegments)
          fileOffset = llvm::RoundUpToAlignment(fileOffset, p_align);
        else {
          // Align according to ELF spec.
          // in p75, http://www.sco.com/developers/devspecs/gabi41.pdf
          uint64_t virtualAddress = slice->virtualAddr();
          Section<ELFT> *sect = dyn_cast<Section<ELFT>>(section);
          if (sect && sect->isLoadableSection() &&
              ((virtualAddress & (p_align - 1)) !=
               (fileOffset & (p_align - 1))))
            fileOffset = llvm::RoundUpToAlignment(fileOffset, p_align) +
                         (virtualAddress % p_align);
        }
      } else if (!isDataPageAlignedForNMagic && needAlign(section)) {
        fileOffset =
            llvm::RoundUpToAlignment(fileOffset, this->_context.getPageSize());
        isDataPageAlignedForNMagic = true;
      }
      if (isFirstSection) {
        slice->setFileOffset(fileOffset);
        isFirstSection = false;
        curSliceFileOffset = fileOffset;
      }
      section->setFileOffset(fileOffset);
      fileOffset += section->fileSize();
      lastVirtualAddress = section->virtualAddr() + section->memSize();
    }
    slice->setFileSize(fileOffset - curSliceFileOffset);
  }
  this->setFileSize(fileOffset - startOffset);
}

/// \brief Assign virtual addresses to the slices
template <class ELFT> void Segment<ELFT>::assignVirtualAddress(uint64_t addr) {
  int startSection = 0;
  int currSection = 0;
  SectionIter startSectionIter;

  // slice align is set to the max alignment of the chunks that are
  // contained in the slice
  uint64_t sliceAlign = 0;
  // Current slice size
  uint64_t curSliceSize = 0;
  // Current Slice File Offset
  uint64_t curSliceAddress = 0;

  startSectionIter = _sections.begin();
  startSection = 0;
  bool isFirstSection = true;
  bool isDataPageAlignedForNMagic = false;
  uint64_t startAddr = addr;
  SegmentSlice<ELFT> *slice = nullptr;
  uint64_t tlsStartAddr = 0;
  bool alignSegments = this->_context.alignSegments();
  StringRef prevOutputSectionName = StringRef();

  for (auto si = _sections.begin(); si != _sections.end(); ++si) {
    // If this is first section in the segment, page align the section start
    // address. The linker needs to align the data section to a page boundary
    // only if NMAGIC is set.
    if (isFirstSection) {
      isFirstSection = false;
      if (alignSegments &&
          _outputMagic != ELFLinkingContext::OutputMagic::NMAGIC &&
          _outputMagic != ELFLinkingContext::OutputMagic::OMAGIC)
        // Align to a page only if the output is not
        // OutputMagic::NMAGIC/OutputMagic::OMAGIC
        startAddr =
            llvm::RoundUpToAlignment(startAddr, this->_context.getPageSize());
      else if (!isDataPageAlignedForNMagic && needAlign(*si)) {
        // If the linker outputmagic is set to OutputMagic::NMAGIC, align the
        // Data to a page boundary.
        startAddr =
            llvm::RoundUpToAlignment(startAddr, this->_context.getPageSize());
        isDataPageAlignedForNMagic = true;
      }
      // align the startOffset to the section alignment
      uint64_t newAddr = llvm::RoundUpToAlignment(startAddr, (*si)->alignment());
      // Handle linker script expressions, which *may update newAddr* if the
      // expression assigns to "."
      if (auto expr = dyn_cast<ExpressionChunk<ELFT>>(*si))
        expr->evalExpr(newAddr);
      curSliceAddress = newAddr;
      sliceAlign = (*si)->alignment();
      (*si)->setVirtualAddr(curSliceAddress);

      // Handle TLS.
      if (auto section = dyn_cast<Section<ELFT>>(*si)) {
        if (section->getSegmentType() == llvm::ELF::PT_TLS) {
          tlsStartAddr =
              llvm::RoundUpToAlignment(tlsStartAddr, (*si)->alignment());
          section->assignVirtualAddress(tlsStartAddr);
          tlsStartAddr += (*si)->memSize();
        } else {
          section->assignVirtualAddress(newAddr);
        }
      }
      // TBSS section is special in that it doesn't contribute to memory of any
      // segment. If we see a tbss section, don't add memory size to addr The
      // fileOffset is automatically taken care of since TBSS section does not
      // end up using file size
      if ((*si)->order() != DefaultLayout<ELFT>::ORDER_TBSS)
        curSliceSize = (*si)->memSize();
    } else {
      uint64_t curAddr = curSliceAddress + curSliceSize;
      if (!isDataPageAlignedForNMagic && needAlign(*si)) {
        // If the linker outputmagic is set to OutputMagic::NMAGIC, align the
        // Data
        // to a page boundary
        curAddr =
            llvm::RoundUpToAlignment(curAddr, this->_context.getPageSize());
        isDataPageAlignedForNMagic = true;
      }
      uint64_t newAddr = llvm::RoundUpToAlignment(curAddr, (*si)->alignment());
      // Handle linker script expressions, which *may update newAddr* if the
      // expression assigns to "."
      if (auto expr = dyn_cast<ExpressionChunk<ELFT>>(*si))
        expr->evalExpr(newAddr);
      Section<ELFT> *sec = dyn_cast<Section<ELFT>>(*si);
      StringRef curOutputSectionName;
      if (sec)
        curOutputSectionName = sec->outputSectionName();
      else {
        // If this is a linker script expression, propagate the name of the
        // previous section instead
        if (isa<ExpressionChunk<ELFT>>(*si))
          curOutputSectionName = prevOutputSectionName;
        else
          curOutputSectionName = (*si)->name();
      }
      bool autoCreateSlice = true;
      if (curOutputSectionName == prevOutputSectionName)
        autoCreateSlice = false;
      // If the newAddress computed is more than a page away, let's create
      // a separate segment, so that memory is not used up while running.
      // Dont create a slice, if the new section falls in the same output
      // section as the previous section.
      if (autoCreateSlice &&
          ((newAddr - curAddr) > this->_context.getPageSize()) &&
          (_outputMagic != ELFLinkingContext::OutputMagic::NMAGIC &&
           _outputMagic != ELFLinkingContext::OutputMagic::OMAGIC)) {
        auto sliceIter =
            std::find_if(_segmentSlices.begin(), _segmentSlices.end(),
                         [startSection](SegmentSlice<ELFT> *s) -> bool {
              return s->startSection() == startSection;
            });
        if (sliceIter == _segmentSlices.end()) {
          slice = new (_segmentAllocate.Allocate<SegmentSlice<ELFT>>())
            SegmentSlice<ELFT>();
          _segmentSlices.push_back(slice);
        } else {
          slice = (*sliceIter);
        }
        slice->setStart(startSection);
        slice->setSections(make_range(startSectionIter, si));
        slice->setMemSize(curSliceSize);
        slice->setAlign(sliceAlign);
        slice->setVirtualAddr(curSliceAddress);
        // Start new slice
        curSliceAddress = newAddr;
        (*si)->setVirtualAddr(curSliceAddress);
        startSectionIter = si;
        startSection = currSection;
        if (auto section = dyn_cast<Section<ELFT>>(*si))
          section->assignVirtualAddress(newAddr);
        curSliceSize = newAddr - curSliceAddress + (*si)->memSize();
        sliceAlign = (*si)->alignment();
      } else {
        if (sliceAlign < (*si)->alignment())
          sliceAlign = (*si)->alignment();
        (*si)->setVirtualAddr(newAddr);
        // Handle TLS.
        if (auto section = dyn_cast<Section<ELFT>>(*si)) {
          if (section->getSegmentType() == llvm::ELF::PT_TLS) {
            tlsStartAddr =
                llvm::RoundUpToAlignment(tlsStartAddr, (*si)->alignment());
            section->assignVirtualAddress(tlsStartAddr);
            tlsStartAddr += (*si)->memSize();
          } else {
            section->assignVirtualAddress(newAddr);
          }
        }
        // TBSS section is special in that it doesn't contribute to memory of
        // any segment. If we see a tbss section, don't add memory size to addr
        // The fileOffset is automatically taken care of since TBSS section does
        // not end up using file size.
        if ((*si)->order() != DefaultLayout<ELFT>::ORDER_TBSS)
          curSliceSize = newAddr - curSliceAddress + (*si)->memSize();
        else
          curSliceSize = newAddr - curSliceAddress;
      }
      prevOutputSectionName = curOutputSectionName;
    }
    currSection++;
  }
  auto sliceIter = std::find_if(_segmentSlices.begin(), _segmentSlices.end(),
                                [startSection](SegmentSlice<ELFT> *s) -> bool {
    return s->startSection() == startSection;
  });
  if (sliceIter == _segmentSlices.end()) {
    slice = new (_segmentAllocate.Allocate<SegmentSlice<ELFT>>())
      SegmentSlice<ELFT>();
    _segmentSlices.push_back(slice);
  } else {
    slice = (*sliceIter);
  }
  slice->setStart(startSection);
  slice->setVirtualAddr(curSliceAddress);
  slice->setMemSize(curSliceSize);
  slice->setSections(make_range(startSectionIter, _sections.end()));
  slice->setAlign(sliceAlign);

  // Set the segment memory size and the virtual address.
  this->setMemSize(curSliceAddress - startAddr + curSliceSize);
  this->setVirtualAddr(curSliceAddress);
  std::stable_sort(_segmentSlices.begin(), _segmentSlices.end(),
                   SegmentSlice<ELFT>::compare_slices);
}

// Write the Segment
template <class ELFT>
void Segment<ELFT>::write(ELFWriter *writer, TargetLayout<ELFT> &layout,
                          llvm::FileOutputBuffer &buffer) {
  for (auto slice : slices())
    for (auto section : slice->sections())
      section->write(writer, layout, buffer);
}

template<class ELFT>
int64_t
Segment<ELFT>::flags() const {
  int64_t fl = 0;
  if (_flags & llvm::ELF::SHF_ALLOC)
    fl |= llvm::ELF::PF_R;
  if (_flags & llvm::ELF::SHF_WRITE)
    fl |= llvm::ELF::PF_W;
  if (_flags & llvm::ELF::SHF_EXECINSTR)
    fl |= llvm::ELF::PF_X;
  return fl;
}
} // end namespace elf
} // end namespace lld

#endif
