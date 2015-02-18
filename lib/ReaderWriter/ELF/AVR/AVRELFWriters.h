//===- lib/ReaderWriter/ELF/AVR/AVRELFWriters.h -------------------------===//
//
//                             The LLVM Linker
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
#ifndef LLD_READER_WRITER_ELF_AVR_AVR_ELF_WRITERS_H
#define LLD_READER_WRITER_ELF_AVR_AVR_ELF_WRITERS_H

#include "AVRLinkingContext.h"
#include "OutputELFWriter.h"

namespace lld {
namespace elf {

template <class ELFT> class AVRRuntimeFile;

template <class ELFT> class AVRTargetLayout;

template <typename ELFT> class AVRELFWriter {
public:
  AVRELFWriter(AVRLinkingContext &ctx, AVRTargetLayout<ELFT> &targetLayout)
      : _ctx(ctx), _targetLayout(targetLayout) {}

  void setELFHeader(ELFHeader<ELFT> &elfHeader) {
    elfHeader.e_version(1);
    elfHeader.e_ident(llvm::ELF::EI_VERSION, llvm::ELF::EV_CURRENT);
    elfHeader.e_ident(llvm::ELF::EI_OSABI, llvm::ELF::ELFOSABI_NONE);
    if (_targetLayout.findOutputSection(".got.plt"))
      elfHeader.e_ident(llvm::ELF::EI_ABIVERSION, 1);
    else
      elfHeader.e_ident(llvm::ELF::EI_ABIVERSION, 0);

    // FIXME: Insert the proper e_flags
    elfHeader.e_flags(0x00);
  }

  void finalizeAVRRuntimeAtomValues() {
    if (!_ctx.isDynamic())
      return;

    auto gotSection = _targetLayout.findOutputSection(".got");
    auto got = gotSection ? gotSection->virtualAddr() : 0;
    auto gp = gotSection ? got + _targetLayout.getGPOffset() : 0;

    setAtomValue("_GLOBAL_OFFSET_TABLE_", got);
    setAtomValue("_gp", gp);
    setAtomValue("_gp_disp", gp);
    setAtomValue("__gnu_local_gp", gp);
  }

  std::unique_ptr<AVRRuntimeFile<ELFT>> createRuntimeFile() {
    auto file = llvm::make_unique<AVRRuntimeFile<ELFT>>(_ctx);
    if (_ctx.isDynamic()) {
      file->addAbsoluteAtom("_GLOBAL_OFFSET_TABLE_");
      file->addAbsoluteAtom("_gp");
      file->addAbsoluteAtom("_gp_disp");
      file->addAbsoluteAtom("__gnu_local_gp");
    }
    return file;
  }

private:
  AVRLinkingContext &_ctx;
  AVRTargetLayout<ELFT> &_targetLayout;

  void setAtomValue(StringRef name, uint64_t value) {
    auto atom = _targetLayout.findAbsoluteAtom(name);
    assert(atom != _targetLayout.absoluteAtoms().end());
    (*atom)->_virtualAddr = value;
  }
};

} // elf
} // lld

#endif
