//===--------- lib/ReaderWriter/ELF/ARM/ARMTargetHandler.h ----------------===//
//
//                             The LLVM Linker
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLD_READER_WRITER_ELF_ARM_ARM_TARGET_HANDLER_H
#define LLD_READER_WRITER_ELF_ARM_ARM_TARGET_HANDLER_H

#include "ARMELFFile.h"
#include "ARMELFReader.h"
#include "ARMRelocationHandler.h"
#include "DefaultTargetHandler.h"
#include "TargetLayout.h"

#include "lld/Core/Simple.h"
#include "llvm/ADT/Optional.h"
#include <map>

namespace lld {
namespace elf {
typedef llvm::object::ELFType<llvm::support::little, 2, false> ARMELFType;
class ARMLinkingContext;

template <class ELFT> class ARMTargetLayout : public TargetLayout<ELFT> {
public:
  ARMTargetLayout(ARMLinkingContext &context)
      : TargetLayout<ELFT>(context) {}

  uint64_t getTPOffset() {
    if (_tpOff.hasValue())
      return *_tpOff;

    for (const auto &phdr : *this->_programHeader) {
      if (phdr->p_type == llvm::ELF::PT_TLS) {
        _tpOff = llvm::RoundUpToAlignment(TCB_SIZE, phdr->p_align);
        return *_tpOff;
      }
    }
    llvm_unreachable("TLS segment not found");
  }

private:
  // TCB block size of the TLS.
  enum { TCB_SIZE = 0x8 };

  // Cached value of the TLS offset from the $tp pointer.
  llvm::Optional<uint64_t> _tpOff;
};

class ARMTargetHandler final : public DefaultTargetHandler<ARMELFType> {
public:
  ARMTargetHandler(ARMLinkingContext &context);

  ARMTargetLayout<ARMELFType> &getTargetLayout() override {
    return *(_armTargetLayout.get());
  }

  void registerRelocationNames(Registry &registry) override;

  const ARMTargetRelocationHandler &getRelocationHandler() const override {
    return *(_armRelocationHandler.get());
  }

  std::unique_ptr<Reader> getObjReader() override {
    return std::unique_ptr<Reader>(new ARMELFObjectReader(_context));
  }

  std::unique_ptr<Reader> getDSOReader() override {
    return std::unique_ptr<Reader>(new ARMELFDSOReader(_context));
  }

  std::unique_ptr<Writer> getWriter() override;

private:
  static const Registry::KindStrings kindStrings[];
  ARMLinkingContext &_context;
  std::unique_ptr<ARMTargetLayout<ARMELFType>> _armTargetLayout;
  std::unique_ptr<ARMTargetRelocationHandler> _armRelocationHandler;
};

} // end namespace elf
} // end namespace lld

#endif // LLD_READER_WRITER_ELF_ARM_ARM_TARGET_HANDLER_H
