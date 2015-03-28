//===- lib/ReaderWriter/ELF/Mips/MipsLinkingContext.h ---------------------===//
//
//                             The LLVM Linker
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
#ifndef LLD_READER_WRITER_ELF_MIPS_MIPS_LINKING_CONTEXT_H
#define LLD_READER_WRITER_ELF_MIPS_MIPS_LINKING_CONTEXT_H

#include "MipsELFFlagsMerger.h"
#include "lld/ReaderWriter/ELFLinkingContext.h"

namespace lld {
namespace elf {

/// \brief Mips internal references.
enum {
  /// \brief Do nothing but mark GOT entry as a global one.
  LLD_R_MIPS_GLOBAL_GOT = 1024,
  /// \brief Apply high 16 bits of symbol + addend.
  LLD_R_MIPS_32_HI16 = 1025,
  /// \brief The same as R_MIPS_26 but for global symbols.
  LLD_R_MIPS_GLOBAL_26 = 1026,
  /// \brief Setup hi 16 bits using the symbol this reference refers to.
  LLD_R_MIPS_HI16 = 1027,
  /// \brief Setup low 16 bits using the symbol this reference refers to.
  LLD_R_MIPS_LO16 = 1028,
  /// \brief Represents a reference between PLT and dynamic symbol.
  LLD_R_MIPS_STO_PLT = 1029,
  /// \brief The same as R_MICROMIPS_26_S1 but for global symbols.
  LLD_R_MICROMIPS_GLOBAL_26_S1 = 1030,
  /// \brief Apply high 32+16 bits of symbol + addend.
  LLD_R_MIPS_64_HI16 = 1031,
};

typedef llvm::object::ELFType<llvm::support::little, 2, false> Mips32ELType;
typedef llvm::object::ELFType<llvm::support::little, 2, true> Mips64ELType;
typedef llvm::object::ELFType<llvm::support::big, 2, false> Mips32BEType;
typedef llvm::object::ELFType<llvm::support::big, 2, true> Mips64BEType;

class MipsLinkingContext final : public ELFLinkingContext {
public:
  static std::unique_ptr<ELFLinkingContext> create(llvm::Triple);
  MipsLinkingContext(llvm::Triple triple);

  uint32_t getMergedELFFlags() const;
  MipsELFFlagsMerger &getELFFlagsMerger();

  // ELFLinkingContext
  uint64_t getBaseAddress() const override;
  StringRef entrySymbolName() const override;
  StringRef getDefaultInterpreter() const override;
  void addPasses(PassManager &pm) override;
  bool isRelaOutputFormat() const override { return false; }
  bool isDynamicRelocation(const Reference &r) const override;
  bool isCopyRelocation(const Reference &r) const override;
  bool isPLTRelocation(const Reference &r) const override;

private:
  MipsELFFlagsMerger _flagsMerger;
};

} // elf
} // lld

#endif
