//===- lib/ReaderWriter/ELF/AVR/AVRRelocationPass.h ---------------------===//
//
//                             The LLVM Linker
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
#ifndef LLD_READER_WRITER_ELF_AVR_AVR_RELOCATION_PASS_H
#define LLD_READER_WRITER_ELF_AVR_AVR_RELOCATION_PASS_H

#include <memory>

namespace lld {
class Pass;

namespace elf {
class AVRLinkingContext;

std::unique_ptr<Pass> createAVRRelocationPass(AVRLinkingContext &ctx);

} // end namespace elf
} // end namespace lld

#endif
