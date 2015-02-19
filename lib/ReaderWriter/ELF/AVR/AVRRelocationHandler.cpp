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

/// \brief Applies a relocation.
/// \param ins The unrelocated data.
/// \param result The fixup value.
/// \param mask A mask containing the bits we want to modify.
static inline void applyReloc(uint32_t &ins, uint32_t result, uint32_t mask) {
  // (integer of the bits we don't want to touch) |
  // (integer of the bits we are touching)
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

/// \brief R_AVR_7_PCREL
static void relocpc7(uint32_t &ins, uint64_t P, uint64_t S, int64_t A) {
  int32_t result = S + A - P;
  applyReloc(ins, result >> 1, 0x7f);
}

/// \brief R_AVR_13_PCREL
static void relocpc13(uint32_t &ins, uint64_t P, uint64_t S, int64_t A) {
  int32_t result = S + A - P;
  applyReloc(ins, result >> 1, 0x1fff);
}

/// \brief R_AVR_16
static void reloc16(uint32_t &ins, uint64_t S, int64_t A) {
  applyReloc(ins, S+A, 0xffff);
}

/// \brief R_AVR_LO8_LDI
static void reloc8loldi(uint32_t &ins, uint64_t S, int64_t A) {
  llvm_unreachable("R_AVR_LO8_LDI relocation not implemented");
}

/// \brief R_AVR_6
static void reloc6(uint32_t &ins, uint64_t S, int64_t A) {
  applyReloc(ins, S+A, 0x3f);
}

/// \brief R_AVR_8
static void reloc8(uint32_t &ins, uint64_t S, int64_t A) {
  applyReloc(ins, S+A, 0xff);
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
  uint64_t relocVAddress = atom._virtualAddr + ref.offsetInAtom();
  uint32_t ins = endian::read<uint32_t, little, 2>(location);

  switch (ref.kindValue()) {
  case R_AVR_NONE:
    break;
  case R_AVR_32:
    reloc32(ins, targetVAddress, ref.addend());
    break;
  case R_AVR_LO8_LDI:
    reloc8loldi(ins, targetVAddress, ref.addend());
    break;
  case R_AVR_7_PCREL:
    relocpc7(ins, relocVAddress, targetVAddress, ref.addend());
	break;
  case R_AVR_13_PCREL:
	relocpc13(ins, relocVAddress, targetVAddress, ref.addend());
	break;
  case R_AVR_16:
	reloc16(ins, targetVAddress, ref.addend());
	break;
  case R_AVR_16_PM:
	llvm_unreachable("unimplemented relocation type: R_AVR_16_PM");
	break;
  case R_AVR_HI8_LDI:
	llvm_unreachable("unimplemented relocation type: R_AVR_HI8_LDI");
	break;
  case R_AVR_HH8_LDI:
	llvm_unreachable("unimplemented relocation type: R_AVR_HH8_LDI");
	break;
  case R_AVR_LO8_LDI_NEG:
	llvm_unreachable("unimplemented relocation type: R_AVR_LO8_LDI_NEG");
	break;
  case R_AVR_HI8_LDI_NEG:
	llvm_unreachable("unimplemented relocation type: R_AVR_HI8_LDI_NEG");
	break;
  case R_AVR_HH8_LDI_NEG:
	llvm_unreachable("unimplemented relocation type: R_AVR_HH8_LDI_NEG");
	break;
  case R_AVR_LO8_LDI_PM:
	llvm_unreachable("unimplemented relocation type: R_AVR_LO8_LDI_PM");
	break;
  case R_AVR_HI8_LDI_PM:
	llvm_unreachable("unimplemented relocation type: R_AVR_HI8_LDI_PM");
	break;
  case R_AVR_HH8_LDI_PM:
	llvm_unreachable("unimplemented relocation type: R_AVR_HH8_LDI_PM");
	break;
  case R_AVR_LO8_LDI_PM_NEG:
	llvm_unreachable("unimplemented relocation type: R_AVR_LO8_LDI_PM_NEG");
	break;
  case R_AVR_HI8_LDI_PM_NEG:
	llvm_unreachable("unimplemented relocation type: R_AVR_HI8_LDI_PM_NEG");
	break;
  case R_AVR_HH8_LDI_PM_NEG:
	llvm_unreachable("unimplemented relocation type: R_AVR_HH8_LDI_PM_NEG");
	break;
  case R_AVR_CALL:
	llvm_unreachable("unimplemented relocation type: R_AVR_CALL");
	break;
  case R_AVR_LDI:
	llvm_unreachable("unimplemented relocation type: R_AVR_LDI");
	break;
  case R_AVR_6:
	reloc6(ins, targetVAddress, ref.addend());
	break;
  case R_AVR_6_ADIW:
	llvm_unreachable("unimplemented relocation type: R_AVR_6_ADIW");
	break;
  case R_AVR_MS8_LDI:
	llvm_unreachable("unimplemented relocation type: R_AVR_MS8_LDI");
	break;
  case R_AVR_MS8_LDI_NEG:
	llvm_unreachable("unimplemented relocation type: R_AVR_MS8_LDI_NEG");
	break;
  case R_AVR_LO8_LDI_GS:
	llvm_unreachable("unimplemented relocation type: R_AVR_LO8_LDI_GS");
	break;
  case R_AVR_HI8_LDI_GS:
	llvm_unreachable("unimplemented relocation type: R_AVR_HI8_LDI_GS");
	break;
  case R_AVR_8:
	reloc8(ins, targetVAddress, ref.addend());
	break;
  case R_AVR_8_LO8:
	llvm_unreachable("unimplemented relocation type: R_AVR_8_LO8");
	break;
  case R_AVR_8_HI8:
	llvm_unreachable("unimplemented relocation type: R_AVR_8_HI8");
	break;
  case R_AVR_8_HLO8:
	llvm_unreachable("unimplemented relocation type: R_AVR_8_HLO8");
	break;
  case R_AVR_SYM_DIFF:
	llvm_unreachable("unimplemented relocation type: R_AVR_SYM_DIFF");
	break;
  case R_AVR_16_LDST:
	llvm_unreachable("unimplemented relocation type: R_AVR_16_LDST");
	break;
  case R_AVR_LDS_STS_16:
	llvm_unreachable("unimplemented relocation type: R_AVR_LDS_STS_16");
	break;
  case R_AVR_PORT6:
	llvm_unreachable("unimplemented relocation type: R_AVR_PORT6");
	break;
  case R_AVR_PORT5:
	llvm_unreachable("unimplemented relocation type: R_AVR_PORT5");
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
