//===- lib/ReaderWriter/ELF/AVR/AVRLinkingContext.cpp -------------------===//
//
//                             The LLVM Linker
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "Atoms.h"
#include "AVRCtorsOrderPass.h"
#include "AVRLinkingContext.h"
#include "AVRRelocationPass.h"
#include "AVRTargetHandler.h"

using namespace lld;
using namespace lld::elf;

std::unique_ptr<ELFLinkingContext>
AVRLinkingContext::create(llvm::Triple triple) {
  if (triple.getArch() == llvm::Triple::avr)
    return std::unique_ptr<ELFLinkingContext>(new AVRLinkingContext(triple));
  return nullptr;
}

typedef std::unique_ptr<TargetHandlerBase> TargetHandlerBasePtr;

static TargetHandlerBasePtr createTarget(llvm::Triple triple,
                                         AVRLinkingContext &ctx) {
  switch (triple.getArch()) {
  case llvm::Triple::avr:
    return TargetHandlerBasePtr(new AVRTargetHandler<AVRELFType>(ctx));
  default:
    llvm_unreachable("unhandled arch");
  }
}

AVRLinkingContext::AVRLinkingContext(llvm::Triple triple)
    : ELFLinkingContext(triple, createTarget(triple, *this)) {}

uint64_t AVRLinkingContext::getBaseAddress() const {
  if (_baseAddress == 0 && getOutputELFType() == llvm::ELF::ET_EXEC)
    return getTriple().isArch64Bit() ? 0x120000000 : 0x400000;
  return _baseAddress;
}

StringRef AVRLinkingContext::entrySymbolName() const {
  if (_outputELFType == elf::ET_EXEC && _entrySymbolName.empty())
    return "__start";
  return _entrySymbolName;
}

StringRef AVRLinkingContext::getDefaultInterpreter() const {
  return getTriple().isArch64Bit() ? "/lib64/ld.so.1" : "/lib/ld.so.1";
}

void AVRLinkingContext::addPasses(PassManager &pm) {
  auto pass = createAVRRelocationPass(*this);
  if (pass)
    pm.add(std::move(pass));
  ELFLinkingContext::addPasses(pm);
  pm.add(llvm::make_unique<elf::AVRCtorsOrderPass>());
}

