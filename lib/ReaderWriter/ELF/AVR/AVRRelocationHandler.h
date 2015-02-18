//===- lld/ReaderWriter/ELF/AVR/AVRRelocationHandler.h ------------------===//
//
//                             The LLVM Linker
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
#ifndef LLD_READER_WRITER_ELF_AVR_AVR_RELOCATION_HANDLER_H
#define LLD_READER_WRITER_ELF_AVR_AVR_RELOCATION_HANDLER_H

namespace lld {
namespace elf {

class AVRLinkingContext;
class TargetRelocationHandler;

template <class ELFT>
std::unique_ptr<TargetRelocationHandler>
createAVRRelocationHandler(AVRLinkingContext &ctx);

} // elf
} // lld

#endif
