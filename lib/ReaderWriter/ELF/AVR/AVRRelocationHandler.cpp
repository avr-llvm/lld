//===- lib/ReaderWriter/ELF/AVR/AVRRelocationHandler.cpp ----------------===//
//
//                             The LLVM Linker
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "AVRTargetHandler.h"
#include "AVRLinkingContext.h"
#include "AVRRelocationHandler.h"
#include "lld/ReaderWriter/RelocationHelperFunctions.h"

using namespace lld;
using namespace elf;
using namespace llvm::ELF;
using namespace llvm::support;

static inline void applyReloc(uint32_t &ins, uint32_t result, uint32_t mask) {
  ins = (ins & ~mask) | (result & mask);
}

template <size_t BITS, class T> inline T signExtend(T val) {
  if (val & (T(1) << (BITS - 1)))
    val |= T(-1) << BITS;
  return val;
}

/// \brief R_AVR_32
/// local/external: word32 S + A (truncate)
static void reloc32(uint32_t &ins, uint64_t S, int64_t A) {
  applyReloc(ins, S + A, 0xffffffff);
}

static void relocLO8LDI(uint32_t &ins, uint64_t S, int64_t A) {
  llvm_unreachable("R_AVR_LO8_LDI relocation not implemented");
}

namespace {

template <class ELFT> class RelocationHandler : public TargetRelocationHandler {
public:
  RelocationHandler(AVRLinkingContext &ctx) : _ctx(ctx) {}

  std::error_code applyRelocation(ELFWriter &writer,
                                  llvm::FileOutputBuffer &buf,
                                  const lld::AtomLayout &atom,
                                  const Reference &ref) const override;

private:
  AVRLinkingContext &_ctx;

  AVRTargetLayout<ELFT> &getTargetLayout() const {
    return static_cast<AVRTargetLayout<ELFT> &>(
        _ctx.getTargetHandler<ELFT>().getTargetLayout());
  }
};

template <class ELFT>
std::error_code RelocationHandler<ELFT>::applyRelocation(
    ELFWriter &writer, llvm::FileOutputBuffer &buf, const lld::AtomLayout &atom,
    const Reference &ref) const {
  if (ref.kindNamespace() != lld::Reference::KindNamespace::ELF)
    return std::error_code();
  assert(ref.kindArch() == Reference::KindArch::AVR);

  uint8_t *atomContent = buf.getBufferStart() + atom._fileOffset;
  uint8_t *location = atomContent + ref.offsetInAtom();
  uint64_t targetVAddress = writer.addressOfAtom(ref.target());
  //uint64_t relocVAddress = atom._virtualAddr + ref.offsetInAtom();
  uint32_t ins = endian::read<uint32_t, little, 2>(location);

  switch (ref.kindValue()) {
  case R_AVR_NONE:
    break;
  case R_AVR_32:
    reloc32(ins, targetVAddress, ref.addend());
    break;
  case R_AVR_LO8_LDI:
    relocLO8LDI(ins, targetVAddress, ref.addend());
    break;
  default:
    return make_unhandled_reloc_error();
  }

  endian::write<uint32_t, little, 2>(location, ins);
  return std::error_code();
}

} // end anon namespace

namespace lld {
namespace elf {

template <>
std::unique_ptr<TargetRelocationHandler>
createAVRRelocationHandler<AVRELFType>(AVRLinkingContext &ctx) {
  return std::unique_ptr<TargetRelocationHandler>(
      new RelocationHandler<AVRELFType>(ctx));
}

} // elf
} // lld
