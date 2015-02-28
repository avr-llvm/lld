//===- lib/ReaderWriter/ELF/X86_64/X86_64RelocationHandler.h --------------===//
//
//                             The LLVM Linker
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef X86_64_RELOCATION_HANDLER_H
#define X86_64_RELOCATION_HANDLER_H

#include "X86_64TargetHandler.h"

namespace lld {
namespace elf {
typedef llvm::object::ELFType<llvm::support::little, 2, true> X86_64ELFType;

class X86_64TargetLayout;

class X86_64TargetRelocationHandler final : public TargetRelocationHandler {
public:
  X86_64TargetRelocationHandler(X86_64TargetLayout &layout)
      : _tlsSize(0), _x86_64Layout(layout) {}

  std::error_code applyRelocation(ELFWriter &, llvm::FileOutputBuffer &,
                                  const lld::AtomLayout &,
                                  const Reference &) const override;

private:
  // Cached size of the TLS segment.
  mutable uint64_t _tlsSize;
  X86_64TargetLayout &_x86_64Layout;
};

} // end namespace elf
} // end namespace lld

#endif // X86_64_RELOCATION_HANDLER_H
