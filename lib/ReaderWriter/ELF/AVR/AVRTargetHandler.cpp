//===- lib/ReaderWriter/ELF/AVR/AVRTargetHandler.cpp --------------------===//
//
//                             The LLVM Linker
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "AVRTargetHandler.h"

using namespace lld;
using namespace elf;

void AVRRelocationStringTable::registerTable(Registry &registry) {
  registry.addKindTable(Reference::KindNamespace::ELF,
                        Reference::KindArch::AVR, kindStrings);
}

#define ELF_RELOC(name, value) LLD_KIND_STRING_ENTRY(name),

// FIXME: this can't be right - this is the Mips stuff.
const Registry::KindStrings AVRRelocationStringTable::kindStrings[] = {
#include "llvm/Support/ELFRelocs/AVR.def"
  LLD_KIND_STRING_ENTRY(LLD_R_AVR_GLOBAL_GOT),
  LLD_KIND_STRING_ENTRY(LLD_R_AVR_32_HI16),
  LLD_KIND_STRING_ENTRY(LLD_R_AVR_GLOBAL_26),
  LLD_KIND_STRING_ENTRY(LLD_R_AVR_HI16),
  LLD_KIND_STRING_ENTRY(LLD_R_AVR_LO16),
  LLD_KIND_STRING_ENTRY(LLD_R_AVR_STO_PLT),
  LLD_KIND_STRING_ENTRY(LLD_R_MICROAVR_GLOBAL_26_S1),
  LLD_KIND_STRING_END
};

#undef ELF_RELOC
