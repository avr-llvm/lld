//===- lib/ReaderWriter/ELF/AVR/AVRTargetHandler.h ----------------------===//
//
//                             The LLVM Linker
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
#ifndef LLD_READER_WRITER_ELF_AVR_AVR_TARGET_HANDLER_H
#define LLD_READER_WRITER_ELF_AVR_AVR_TARGET_HANDLER_H

#include "DefaultTargetHandler.h"
#include "AVRELFReader.h"
#include "AVRExecutableWriter.h"
#include "AVRLinkingContext.h"
#include "AVRRelocationHandler.h"
#include "AVRSectionChunks.h"
#include "TargetLayout.h"
#include "llvm/ADT/DenseSet.h"

namespace lld {
namespace elf {

typedef llvm::object::ELFType<llvm::support::little, 2, false> AVRELFType;

/// \brief TargetLayout for AVR
template <class ELFT>
class AVRTargetLayout final : public TargetLayout<ELFT> {
public:
  AVRTargetLayout(AVRLinkingContext &ctx)
      : TargetLayout<ELFT>(ctx) { }
  
  /// \brief GP offset relative to .got section.
  uint64_t getGPOffset() const { return 0x7FF0; }

  /// \brief Get '_gp' symbol atom layout.
  AtomLayout *getGP() {
    if (!_gpAtom.hasValue()) {
      auto atom = this->findAbsoluteAtom("_gp");
      _gpAtom = atom != this->absoluteAtoms().end() ? *atom : nullptr;
    }
    return *_gpAtom;
  }

  /// \brief Get '_gp_disp' symbol atom layout.
  AtomLayout *getGPDisp() {
    if (!_gpDispAtom.hasValue()) {
      auto atom = this->findAbsoluteAtom("_gp_disp");
      _gpDispAtom = atom != this->absoluteAtoms().end() ? *atom : nullptr;
    }
    return *_gpDispAtom;
  }

private:
  llvm::BumpPtrAllocator _alloc;
  llvm::Optional<AtomLayout *> _gpAtom;
  llvm::Optional<AtomLayout *> _gpDispAtom;
};

/// \brief AVR Runtime file.
template <class ELFT>
class AVRRuntimeFile final : public RuntimeFile<ELFT> {
public:
  AVRRuntimeFile(AVRLinkingContext &ctx)
      : RuntimeFile<ELFT>(ctx, "AVR runtime file") {}
};

/// \brief Auxiliary class holds relocation's names table.
class AVRRelocationStringTable {
  static const Registry::KindStrings kindStrings[];

public:
  static void registerTable(Registry &registry);
};

/// \brief TargetHandler for AVR
template <class ELFT>
class AVRTargetHandler final : public DefaultTargetHandler<ELFT> {
public:
  AVRTargetHandler(AVRLinkingContext &ctx)
      : _ctx(ctx), _runtimeFile(new AVRRuntimeFile<ELFT>(ctx)),
        _targetLayout(new AVRTargetLayout<ELFT>(ctx)),
        _relocationHandler(createAVRRelocationHandler<ELFT>(ctx)) {}

  AVRTargetLayout<ELFT> &getTargetLayout() override { return *_targetLayout; }

  std::unique_ptr<Reader> getObjReader() override {
    return std::unique_ptr<Reader>(new AVRELFObjectReader<ELFT>(_ctx));
  }

  std::unique_ptr<Reader> getDSOReader() override {
    return std::unique_ptr<Reader>(new AVRELFDSOReader<ELFT>(_ctx));
  }

  const TargetRelocationHandler &getRelocationHandler() const override {
    return *_relocationHandler;
  }

  std::unique_ptr<Writer> getWriter() override {
    switch (_ctx.getOutputELFType()) {
    case llvm::ELF::ET_EXEC:
      return std::unique_ptr<Writer>(
          new AVRExecutableWriter<ELFT>(_ctx, *_targetLayout));
    case llvm::ELF::ET_REL:
      llvm_unreachable("TODO: support -r mode");
    default:
      llvm_unreachable("unsupported output type");
    }
  }

  void registerRelocationNames(Registry &registry) override {
    AVRRelocationStringTable::registerTable(registry);
  }

private:
  AVRLinkingContext &_ctx;
  std::unique_ptr<AVRRuntimeFile<ELFT>> _runtimeFile;
  std::unique_ptr<AVRTargetLayout<ELFT>> _targetLayout;
  std::unique_ptr<TargetRelocationHandler> _relocationHandler;
};

template <class ELFT> class AVRSymbolTable : public SymbolTable<ELFT> {
public:
  typedef llvm::object::Elf_Sym_Impl<ELFT> Elf_Sym;

  AVRSymbolTable(const ELFLinkingContext &ctx)
      : SymbolTable<ELFT>(ctx, ".symtab",
                          DefaultLayout<ELFT>::ORDER_SYMBOL_TABLE) {}
};

} // end namespace elf
} // end namespace lld

#endif
