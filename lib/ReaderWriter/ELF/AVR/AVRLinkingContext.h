//===- lib/ReaderWriter/ELF/AVR/AVRLinkingContext.h ---------------------===//
//
//                             The LLVM Linker
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
#ifndef LLD_READER_WRITER_ELF_AVR_AVR_LINKING_CONTEXT_H
#define LLD_READER_WRITER_ELF_AVR_AVR_LINKING_CONTEXT_H

#include "lld/ReaderWriter/ELFLinkingContext.h"

namespace lld {
namespace elf {

/// \brief AVR internal references.
enum {
  /// \brief Do nothing but mark GOT entry as a global one.
  LLD_R_AVR_GLOBAL_GOT = 1024,
  /// \brief Apply high 16 bits of symbol + addend.
  LLD_R_AVR_32_HI16 = 1025,
  /// \brief The same as R_AVR_26 but for global symbols.
  LLD_R_AVR_GLOBAL_26 = 1026,
  /// \brief Setup hi 16 bits using the symbol this reference refers to.
  LLD_R_AVR_HI16 = 1027,
  /// \brief Setup low 16 bits using the symbol this reference refers to.
  LLD_R_AVR_LO16 = 1028,
  /// \brief Represents a reference between PLT and dynamic symbol.
  LLD_R_AVR_STO_PLT = 1029,
  /// \brief The same as R_MICROAVR_26_S1 but for global symbols.
  LLD_R_MICROAVR_GLOBAL_26_S1 = 1030,
};

class AVRLinkingContext final : public ELFLinkingContext {
public:
  static std::unique_ptr<ELFLinkingContext> create(llvm::Triple);
  AVRLinkingContext(llvm::Triple triple);

  // ELFLinkingContext
  uint64_t getBaseAddress() const override;
  StringRef entrySymbolName() const override;
  StringRef getDefaultInterpreter() const override;
  void addPasses(PassManager &pm) override;
  bool isRelaOutputFormat() const override { return false; }
};

} // elf
} // lld

#endif
