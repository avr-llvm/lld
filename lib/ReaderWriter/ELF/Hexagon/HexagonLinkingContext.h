//===- lib/ReaderWriter/ELF/Hexagon/HexagonLinkingContext.h ---------------===//
//
//                             The LLVM Linker
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLD_READER_WRITER_ELF_HEXAGON_HEXAGON_LINKING_CONTEXT_H
#define LLD_READER_WRITER_ELF_HEXAGON_HEXAGON_LINKING_CONTEXT_H

#include "lld/ReaderWriter/ELFLinkingContext.h"
#include "llvm/Object/ELF.h"
#include "llvm/Support/ELF.h"

namespace lld {
namespace elf {

typedef llvm::object::ELFType<llvm::support::little, 2, false> HexagonELFType;

class HexagonLinkingContext final : public ELFLinkingContext {
public:
  static std::unique_ptr<ELFLinkingContext> create(llvm::Triple);
  HexagonLinkingContext(llvm::Triple triple);

  void addPasses(PassManager &) override;

  bool isDynamicRelocation(const Reference &r) const override {
    if (r.kindNamespace() != Reference::KindNamespace::ELF)
      return false;
    switch (r.kindValue()) {
    case llvm::ELF::R_HEX_RELATIVE:
    case llvm::ELF::R_HEX_GLOB_DAT:
      return true;
    default:
      return false;
    }
  }

  bool isPLTRelocation(const Reference &r) const override {
    if (r.kindNamespace() != Reference::KindNamespace::ELF)
      return false;
    switch (r.kindValue()) {
    case llvm::ELF::R_HEX_JMP_SLOT:
      return true;
    default:
      return false;
    }
  }

  /// \brief Hexagon has only one relative relocation
  /// a) for supporting relative relocs - R_HEX_RELATIVE
  bool isRelativeReloc(const Reference &r) const override {
    if (r.kindNamespace() != Reference::KindNamespace::ELF)
      return false;
    switch (r.kindValue()) {
    case llvm::ELF::R_HEX_RELATIVE:
      return true;
    default:
      return false;
    }
  }
};

} // elf
} // lld

#endif // LLD_READER_WRITER_ELF_HEXAGON_HEXAGON_LINKING_CONTEXT_H
