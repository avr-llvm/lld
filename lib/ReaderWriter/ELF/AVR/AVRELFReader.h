//===- lib/ReaderWriter/ELF/AVRELFReader.h -------------------------------===//
//
//                             The LLVM Linker
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
#ifndef LLD_READER_WRITER_ELF_AVR_AVR_ELF_READER_H
#define LLD_READER_WRITER_ELF_AVR_AVR_ELF_READER_H

#include "ELFReader.h"
#include "AVRELFFile.h"
#include "AVRLinkingContext.h"

namespace lld {
namespace elf {

struct AVRELFFileCreateTraits {
  typedef llvm::ErrorOr<std::unique_ptr<lld::File>> result_type;

  template <class ELFT>
  static result_type create(std::unique_ptr<llvm::MemoryBuffer> mb,
                            AVRLinkingContext &ctx) {
    return lld::elf::AVRELFFile<ELFT>::create(std::move(mb), ctx);
  }
};

struct AVRDynamicFileCreateELFTraits {
  typedef llvm::ErrorOr<std::unique_ptr<lld::SharedLibraryFile>> result_type;

  template <class ELFT>
  static result_type create(std::unique_ptr<llvm::MemoryBuffer> mb,
                            AVRLinkingContext &ctx) {
    return lld::elf::AVRDynamicFile<ELFT>::create(std::move(mb), ctx);
  }
};

template <class ELFT>
class AVRELFObjectReader
    : public ELFObjectReader<ELFT, AVRELFFileCreateTraits,
                             AVRLinkingContext> {
  typedef ELFObjectReader<ELFT, AVRELFFileCreateTraits, AVRLinkingContext>
      BaseReaderType;

public:
  AVRELFObjectReader(AVRLinkingContext &ctx)
      : BaseReaderType(ctx, llvm::ELF::EM_AVR) {}
};

template <class ELFT>
class AVRELFDSOReader
    : public ELFDSOReader<ELFT, AVRDynamicFileCreateELFTraits,
                          AVRLinkingContext> {
  typedef ELFDSOReader<ELFT, AVRDynamicFileCreateELFTraits, AVRLinkingContext>
      BaseReaderType;

public:
  AVRELFDSOReader(AVRLinkingContext &ctx)
      : BaseReaderType(ctx, llvm::ELF::EM_AVR) {}
};

} // namespace elf
} // namespace lld

#endif
