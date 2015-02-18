//===- lib/ReaderWriter/ELF/AVR/AVRRelocationPass.cpp -------------------===//
//
//                             The LLVM Linker
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "AVRELFFile.h"
#include "AVRLinkingContext.h"
#include "AVRRelocationPass.h"
#include "AVRTargetHandler.h"
#include "llvm/ADT/DenseSet.h"

using namespace lld;
using namespace lld::elf;
using namespace llvm::ELF;

// Lazy resolver
static const uint8_t avrGot0AtomContent[] = {
  0x00, 0x00, 0x00, 0x00
};

// Module pointer
static const uint8_t avrGotModulePointerAtomContent[] = {
  0x00, 0x00, 0x00, 0x80
};

// TLS GD Entry
static const uint8_t avrGotTlsGdAtomContent[] = {
  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00
};

// Regular PLT0 entry
static const uint8_t avrPlt0AtomContent[] = {
  0x00, 0x00, 0x1c, 0x3c, // lui   $28, %hi(&GOTPLT[0])
  0x00, 0x00, 0x99, 0x8f, // lw    $25, %lo(&GOTPLT[0])($28)
  0x00, 0x00, 0x9c, 0x27, // addiu $28, $28, %lo(&GOTPLT[0])
  0x23, 0xc0, 0x1c, 0x03, // subu  $24, $24, $28
  0x21, 0x78, 0xe0, 0x03, // move  $15, $31
  0x82, 0xc0, 0x18, 0x00, // srl   $24, $24, 2
  0x09, 0xf8, 0x20, 0x03, // jalr  $25
  0xfe, 0xff, 0x18, 0x27  // subu  $24, $24, 2
};

// microavr PLT0 entry
static const uint8_t microavrPlt0AtomContent[] = {
  0x80, 0x79, 0x00, 0x00, // addiupc $3,  (&GOTPLT[0]) - .
  0x23, 0xff, 0x00, 0x00, // lw      $25, 0($3)
  0x35, 0x05,             // subu    $2,  $2, $3
  0x25, 0x25,             // srl     $2,  $2, 2
  0x02, 0x33, 0xfe, 0xff, // subu    $24, $2, 2
  0xff, 0x0d,             // move    $15, $31
  0xf9, 0x45,             // jalrs   $25
  0x83, 0x0f,             // move    $28, $3
  0x00, 0x0c              // nop
};

// Regular PLT entry
static const uint8_t avrPltAAtomContent[] = {
  0x00, 0x00, 0x0f, 0x3c, // lui   $15, %hi(.got.plt entry)
  0x00, 0x00, 0xf9, 0x8d, // l[wd] $25, %lo(.got.plt entry)($15)
  0x08, 0x00, 0x20, 0x03, // jr    $25
  0x00, 0x00, 0xf8, 0x25  // addiu $24, $15, %lo(.got.plt entry)
};

// microavr PLT entry
static const uint8_t microavrPltAtomContent[] = {
  0x00, 0x79, 0x00, 0x00, // addiupc $2, (.got.plt entry) - .
  0x22, 0xff, 0x00, 0x00, // lw $25, 0($2)
  0x99, 0x45,             // jr $25
  0x02, 0x0f              // move $24, $2
};

// LA25 stub entry
static const uint8_t avrLA25AtomContent[] = {
  0x00, 0x00, 0x19, 0x3c, // lui   $25, %hi(func)
  0x00, 0x00, 0x00, 0x08, // j     func
  0x00, 0x00, 0x39, 0x27, // addiu $25, $25, %lo(func)
  0x00, 0x00, 0x00, 0x00  // nop
};

// microavr LA25 stub entry
static const uint8_t microavrLA25AtomContent[] = {
  0xb9, 0x41, 0x00, 0x00, // lui   $25, %hi(func)
  0x00, 0xd4, 0x00, 0x00, // j     func
  0x39, 0x33, 0x00, 0x00, // addiu $25, $25, %lo(func)
  0x00, 0x00, 0x00, 0x00  // nop
};

namespace {

/// \brief Abstract base class represent avr GOT entries.
class AVRGOTAtom : public GOTAtom {
public:
  AVRGOTAtom(const File &f) : GOTAtom(f, ".got") {}

  Alignment alignment() const override { return Alignment(2); }
};

/// \brief avr GOT entry initialized by zero.
class GOT0Atom : public AVRGOTAtom {
public:
  GOT0Atom(const File &f) : AVRGOTAtom(f) {}

  ArrayRef<uint8_t> rawContent() const override {
    return llvm::makeArrayRef(avrGot0AtomContent);
  }
};

/// \brief avr GOT entry initialized by zero.
class GOTModulePointerAtom : public AVRGOTAtom {
public:
  GOTModulePointerAtom(const File &f) : AVRGOTAtom(f) {}

  ArrayRef<uint8_t> rawContent() const override {
    return llvm::makeArrayRef(avrGotModulePointerAtomContent);
  }
};


class RelocationPassFile : public SimpleFile {
public:
  RelocationPassFile(const ELFLinkingContext &ctx)
      : SimpleFile("RelocationPassFile") {
    setOrdinal(ctx.getNextOrdinalAndIncrement());
  }

  llvm::BumpPtrAllocator _alloc;
};

template <typename ELFT> class RelocationPass : public Pass {
public:
  RelocationPass(AVRLinkingContext &ctx) { }

  void perform(std::unique_ptr<MutableFile> &mf) override;
};

template <typename ELFT>
void RelocationPass<ELFT>::perform(std::unique_ptr<MutableFile> &mf) {
  llvm_unreachable("unimplemented");
}

} // end anon namespace

static std::unique_ptr<Pass> createPass(AVRLinkingContext &ctx) {
  switch (ctx.getTriple().getArch()) {
  case llvm::Triple::avr:
    return llvm::make_unique<RelocationPass<AVRELFType>>(ctx);
  default:
    llvm_unreachable("Unhandled arch");
  }
}

std::unique_ptr<Pass>
lld::elf::createAVRRelocationPass(AVRLinkingContext &ctx) {
  switch (ctx.getOutputELFType()) {
  case llvm::ELF::ET_EXEC:
  case llvm::ELF::ET_DYN:
    return createPass(ctx);
  case llvm::ELF::ET_REL:
    return nullptr;
  default:
    llvm_unreachable("Unhandled output file type");
  }

}
