//===- lib/ReaderWriter/ELF/AVR/AVRExecutableWriter.h -------------------===//
//
//                             The LLVM Linker
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
#ifndef LLD_READER_WRITER_ELF_AVR_AVR_EXECUTABLE_WRITER_H
#define LLD_READER_WRITER_ELF_AVR_AVR_EXECUTABLE_WRITER_H

#include "ExecutableWriter.h"
#include "AVRELFWriters.h"
#include "AVRLinkingContext.h"

namespace lld {
namespace elf {

template <typename ELFT> class AVRTargetLayout;

template <class ELFT>
class AVRExecutableWriter : public ExecutableWriter<ELFT> {
public:
  AVRExecutableWriter(AVRLinkingContext &ctx, AVRTargetLayout<ELFT> &layout);

protected:
  // Add any runtime files and their atoms to the output
  bool createImplicitFiles(std::vector<std::unique_ptr<File>> &) override;

  void finalizeDefaultAtomValues() override;
  std::error_code setELFHeader() override;

private:
  AVRELFWriter<ELFT> _writeHelper;
  AVRTargetLayout<ELFT> &_avrTargetLayout;
};

template <class ELFT>
AVRExecutableWriter<ELFT>::AVRExecutableWriter(AVRLinkingContext &ctx,
                                                 AVRTargetLayout<ELFT> &layout)
    : ExecutableWriter<ELFT>(ctx, layout), _writeHelper(ctx, layout),
      _avrTargetLayout(layout) {}

template <class ELFT>
std::error_code AVRExecutableWriter<ELFT>::setELFHeader() {
  std::error_code ec = ExecutableWriter<ELFT>::setELFHeader();
  if (ec)
    return ec;

  _writeHelper.setELFHeader(*this->_elfHeader);
  return std::error_code();
}

template <class ELFT>
bool AVRExecutableWriter<ELFT>::createImplicitFiles(
    std::vector<std::unique_ptr<File>> &result) {
  ExecutableWriter<ELFT>::createImplicitFiles(result);
  result.push_back(std::move(_writeHelper.createRuntimeFile()));
  return true;
}

template <class ELFT>
void AVRExecutableWriter<ELFT>::finalizeDefaultAtomValues() {
  // Finalize the atom values that are part of the parent.
  ExecutableWriter<ELFT>::finalizeDefaultAtomValues();
  _writeHelper.finalizeAVRRuntimeAtomValues();
}

} // namespace elf
} // namespace lld

#endif
