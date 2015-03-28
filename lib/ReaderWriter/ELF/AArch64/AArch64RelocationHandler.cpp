//===- lib/ReaderWriter/ELF/AArch64/AArch64RelocationHandler.cpp ----------===//
//
//                             The LLVM Linker
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "AArch64TargetHandler.h"
#include "AArch64LinkingContext.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/Endian.h"
#include "llvm/Support/MathExtras.h"

using namespace lld;
using namespace lld::elf;
using namespace llvm::support::endian;

#define PAGE(X) ((X) & ~0x0FFFL)

/// \brief Check X is in the interval (-2^(bits-1), 2^bits]
static bool withinSignedUnsignedRange(int64_t X, int bits) {
  return isIntN(bits - 1, X) || isUIntN(bits, X);
}

/// \brief R_AARCH64_ABS64 - word64: S + A
static void relocR_AARCH64_ABS64(uint8_t *location, uint64_t P, uint64_t S,
                                 int64_t A) {
  int64_t result = (int64_t)S + A;
  DEBUG_WITH_TYPE(
      "AArch64", llvm::dbgs() << "\t\tHandle " << LLVM_FUNCTION_NAME << " -";
      llvm::dbgs() << " S: 0x" << Twine::utohexstr(S);
      llvm::dbgs() << " A: 0x" << Twine::utohexstr(A);
      llvm::dbgs() << " P: 0x" << Twine::utohexstr(P);
      llvm::dbgs() << " result: 0x" << Twine::utohexstr(result) << "\n");
  write64le(location, result | read64le(location));
}

/// \brief R_AARCH64_PREL32 - word32: S + A - P
static void relocR_AARCH64_PREL32(uint8_t *location, uint64_t P, uint64_t S,
                                  int64_t A) {
  int32_t result = (int32_t)((S + A) - P);
  write32le(location, result + (int32_t)read32le(location));
}

/// \brief R_AARCH64_ABS32 - word32:  S + A
static std::error_code relocR_AARCH64_ABS32(uint8_t *location, uint64_t P,
                                            uint64_t S, int64_t A) {
  int64_t result = S + A;
  if (!withinSignedUnsignedRange(result, 32))
    return make_out_of_range_reloc_error();
  DEBUG_WITH_TYPE(
      "AArch64", llvm::dbgs() << "\t\tHandle " << LLVM_FUNCTION_NAME << " -";
      llvm::dbgs() << " S: 0x" << Twine::utohexstr(S);
      llvm::dbgs() << " A: 0x" << Twine::utohexstr(A);
      llvm::dbgs() << " P: 0x" << Twine::utohexstr(P);
      llvm::dbgs() << " result: 0x" << Twine::utohexstr(result) << "\n");
  write32le(location, result | read32le(location));
  return std::error_code();
}

/// \brief R_AARCH64_ADR_PREL_PG_HI21 - Page(S+A) - Page(P)
static void relocR_AARCH64_ADR_PREL_PG_HI21(uint8_t *location, uint64_t P,
                                            uint64_t S, int64_t A) {
  uint64_t result = (PAGE(S + A) - PAGE(P));
  result = result >> 12;
  uint32_t immlo = result & 0x3;
  uint32_t immhi = result & 0x1FFFFC;
  immlo = immlo << 29;
  immhi = immhi << 3;
  DEBUG_WITH_TYPE(
      "AArch64", llvm::dbgs() << "\t\tHandle " << LLVM_FUNCTION_NAME << " -";
      llvm::dbgs() << " S: " << Twine::utohexstr(S);
      llvm::dbgs() << " A: " << Twine::utohexstr(A);
      llvm::dbgs() << " P: " << Twine::utohexstr(P);
      llvm::dbgs() << " immhi: " << Twine::utohexstr(immhi);
      llvm::dbgs() << " immlo: " << Twine::utohexstr(immlo);
      llvm::dbgs() << " result: " << Twine::utohexstr(result) << "\n");
  write32le(location, immlo | immhi | read32le(location));
  // TODO: Make sure this is correct!
}

/// \brief R_AARCH64_ADR_PREL_LO21 - S + A - P
static void relocR_AARCH64_ADR_PREL_LO21(uint8_t *location, uint64_t P,
                                         uint64_t S, int64_t A) {
  uint64_t result = (S + A) - P;
  uint32_t immlo = result & 0x3;
  uint32_t immhi = result & 0x1FFFFC;
  immlo = immlo << 29;
  immhi = immhi << 3;
  DEBUG_WITH_TYPE(
      "AArch64", llvm::dbgs() << "\t\tHandle " << LLVM_FUNCTION_NAME << " -";
      llvm::dbgs() << " S: " << Twine::utohexstr(S);
      llvm::dbgs() << " A: " << Twine::utohexstr(A);
      llvm::dbgs() << " P: " << Twine::utohexstr(P);
      llvm::dbgs() << " immhi: " << Twine::utohexstr(immhi);
      llvm::dbgs() << " immlo: " << Twine::utohexstr(immlo);
      llvm::dbgs() << " result: " << Twine::utohexstr(result) << "\n");
  write32le(location, immlo | immhi | read32le(location));
  // TODO: Make sure this is correct!
}

/// \brief R_AARCH64_ADD_ABS_LO12_NC
static void relocR_AARCH64_ADD_ABS_LO12_NC(uint8_t *location, uint64_t P,
                                           uint64_t S, int64_t A) {
  int32_t result = (int32_t)((S + A) & 0xFFF);
  result <<= 10;
  DEBUG_WITH_TYPE(
      "AArch64", llvm::dbgs() << "\t\tHandle " << LLVM_FUNCTION_NAME << " -";
      llvm::dbgs() << " S: " << Twine::utohexstr(S);
      llvm::dbgs() << " A: " << Twine::utohexstr(A);
      llvm::dbgs() << " P: " << Twine::utohexstr(P);
      llvm::dbgs() << " result: " << Twine::utohexstr(result) << "\n");
  write32le(location, result | read32le(location));
}

static void relocJump26(uint8_t *location, uint64_t P, uint64_t S, int64_t A) {
  int32_t result = (int32_t)((S + A) - P);
  result &= 0x0FFFFFFC;
  result >>= 2;
  DEBUG_WITH_TYPE(
      "AArch64", llvm::dbgs() << "\t\tHandle " << LLVM_FUNCTION_NAME << " -";
      llvm::dbgs() << " S: " << Twine::utohexstr(S);
      llvm::dbgs() << " A: " << Twine::utohexstr(A);
      llvm::dbgs() << " P: " << Twine::utohexstr(P);
      llvm::dbgs() << " result: " << Twine::utohexstr(result) << "\n");
  write32le(location, result | read32le(location));
}

/// \brief R_AARCH64_CONDBR19
static void relocR_AARCH64_CONDBR19(uint8_t *location, uint64_t P, uint64_t S,
                                    int64_t A) {
  int32_t result = (int32_t)((S + A) - P);
  result &= 0x01FFFFC;
  result <<= 3;
  DEBUG_WITH_TYPE(
      "AArch64", llvm::dbgs() << "\t\tHandle " << LLVM_FUNCTION_NAME << " -";
      llvm::dbgs() << " S: " << Twine::utohexstr(S);
      llvm::dbgs() << " A: " << Twine::utohexstr(A);
      llvm::dbgs() << " P: " << Twine::utohexstr(P);
      llvm::dbgs() << " result: " << Twine::utohexstr(result) << "\n");
  write32le(location, result | read32le(location));
}

/// \brief R_AARCH64_LDST8_ABS_LO12_NC - S + A
static void relocR_AARCH64_LDST8_ABS_LO12_NC(uint8_t *location, uint64_t P,
                                             uint64_t S, int64_t A) {
  int32_t result = (int32_t)((S + A) & 0xFFF);
  result <<= 10;
  DEBUG_WITH_TYPE(
      "AArch64", llvm::dbgs() << "\t\tHandle " << LLVM_FUNCTION_NAME << " -";
      llvm::dbgs() << " S: " << Twine::utohexstr(S);
      llvm::dbgs() << " A: " << Twine::utohexstr(A);
      llvm::dbgs() << " P: " << Twine::utohexstr(P);
      llvm::dbgs() << " result: " << Twine::utohexstr(result) << "\n");
  write32le(location, result | read32le(location));
}

/// \brief R_AARCH64_LDST16_ABS_LO12_NC
static void relocR_AARCH64_LDST16_ABS_LO12_NC(uint8_t *location, uint64_t P,
                                              uint64_t S, int64_t A) {
  int32_t result = (int32_t)(S + A);
  result &= 0x0FFC;
  result <<= 9;
  DEBUG_WITH_TYPE(
      "AArch64", llvm::dbgs() << "\t\tHandle " << LLVM_FUNCTION_NAME << " -";
      llvm::dbgs() << " S: " << Twine::utohexstr(S);
      llvm::dbgs() << " A: " << Twine::utohexstr(A);
      llvm::dbgs() << " P: " << Twine::utohexstr(P);
      llvm::dbgs() << " result: " << Twine::utohexstr(result) << "\n");
  write32le(location, result | read32le(location));
}

/// \brief R_AARCH64_LDST32_ABS_LO12_NC
static void relocR_AARCH64_LDST32_ABS_LO12_NC(uint8_t *location, uint64_t P,
                                              uint64_t S, int64_t A) {
  int32_t result = (int32_t)(S + A);
  result &= 0x0FFC;
  result <<= 8;
  DEBUG_WITH_TYPE(
      "AArch64", llvm::dbgs() << "\t\tHandle " << LLVM_FUNCTION_NAME << " -";
      llvm::dbgs() << " S: " << Twine::utohexstr(S);
      llvm::dbgs() << " A: " << Twine::utohexstr(A);
      llvm::dbgs() << " P: " << Twine::utohexstr(P);
      llvm::dbgs() << " result: " << Twine::utohexstr(result) << "\n");
  write32le(location, result | read32le(location));
}

/// \brief R_AARCH64_LDST64_ABS_LO12_NC
static void relocR_AARCH64_LDST64_ABS_LO12_NC(uint8_t *location, uint64_t P,
                                              uint64_t S, int64_t A) {
  int32_t result = (int32_t)(S + A);
  result &= 0x0FF8;
  result <<= 7;
  DEBUG_WITH_TYPE(
      "AArch64", llvm::dbgs() << "\t\tHandle " << LLVM_FUNCTION_NAME << " -";
      llvm::dbgs() << " S: " << Twine::utohexstr(S);
      llvm::dbgs() << " A: " << Twine::utohexstr(A);
      llvm::dbgs() << " P: " << Twine::utohexstr(P);
      llvm::dbgs() << " result: " << Twine::utohexstr(result) << "\n");
  write32le(location, result | read32le(location));
}

/// \brief R_AARCH64_LDST128_ABS_LO12_NC
static void relocR_AARCH64_LDST128_ABS_LO12_NC(uint8_t *location, uint64_t P,
                                               uint64_t S, int64_t A) {
  int32_t result = (int32_t)(S + A);
  result &= 0x0FF8;
  result <<= 6;
  DEBUG_WITH_TYPE(
      "AArch64", llvm::dbgs() << "\t\tHandle " << LLVM_FUNCTION_NAME << " -";
      llvm::dbgs() << " S: " << Twine::utohexstr(S);
      llvm::dbgs() << " A: " << Twine::utohexstr(A);
      llvm::dbgs() << " P: " << Twine::utohexstr(P);
      llvm::dbgs() << " result: " << Twine::utohexstr(result) << "\n");
  write32le(location, result | read32le(location));
}

static void relocR_AARCH64_ADR_GOT_PAGE(uint8_t *location, uint64_t P,
                                        uint64_t S, int64_t A) {
  uint64_t result = PAGE(S + A) - PAGE(P);
  result >>= 12;
  uint32_t immlo = result & 0x3;
  uint32_t immhi = result & 0x1FFFFC;
  immlo = immlo << 29;
  immhi = immhi << 3;
  DEBUG_WITH_TYPE(
      "AArch64", llvm::dbgs() << "\t\tHandle " << LLVM_FUNCTION_NAME << " -";
      llvm::dbgs() << " S: " << Twine::utohexstr(S);
      llvm::dbgs() << " A: " << Twine::utohexstr(A);
      llvm::dbgs() << " P: " << Twine::utohexstr(P);
      llvm::dbgs() << " immhi: " << Twine::utohexstr(immhi);
      llvm::dbgs() << " immlo: " << Twine::utohexstr(immlo);
      llvm::dbgs() << " result: " << Twine::utohexstr(result) << "\n");
  write32le(location, result | read32le(location));
}

// R_AARCH64_LD64_GOT_LO12_NC
static void relocR_AARCH64_LD64_GOT_LO12_NC(uint8_t *location, uint64_t P,
                                            uint64_t S, int64_t A) {
  int32_t result = S + A;
  DEBUG_WITH_TYPE(
      "AArch64", llvm::dbgs() << "\t\tHandle " << LLVM_FUNCTION_NAME << " -";
      llvm::dbgs() << " S: " << Twine::utohexstr(S);
      llvm::dbgs() << " A: " << Twine::utohexstr(A);
      llvm::dbgs() << " P: " << Twine::utohexstr(P);
      llvm::dbgs() << " result: " << Twine::utohexstr(result) << "\n");
  result &= 0xFF8;
  result <<= 7;
  write32le(location, result | read32le(location));
}

// ADD_AARCH64_GOTRELINDEX
static void relocADD_AARCH64_GOTRELINDEX(uint8_t *location, uint64_t P,
                                         uint64_t S, int64_t A) {
  int32_t result = S + A;
  DEBUG_WITH_TYPE(
      "AArch64", llvm::dbgs() << "\t\tHandle " << LLVM_FUNCTION_NAME << " -";
      llvm::dbgs() << " S: " << Twine::utohexstr(S);
      llvm::dbgs() << " A: " << Twine::utohexstr(A);
      llvm::dbgs() << " P: " << Twine::utohexstr(P);
      llvm::dbgs() << " result: " << Twine::utohexstr(result) << "\n");
  result &= 0xFFF;
  result <<= 10;
  write32le(location, result | read32le(location));
}

// R_AARCH64_TLSIE_ADR_GOTTPREL_PAGE21
static void relocR_AARCH64_TLSIE_ADR_GOTTPREL_PAGE21(uint8_t *location,
                                                     uint64_t P, uint64_t S,
                                                     int64_t A) {
  int64_t result = PAGE(S + A) - PAGE(P);
  result >>= 12;
  uint32_t immlo = result & 0x3;
  uint32_t immhi = result & 0x1FFFFC;
  immlo = immlo << 29;
  immhi = immhi << 3;
  DEBUG_WITH_TYPE(
      "AArch64", llvm::dbgs() << "\t\tHandle " << LLVM_FUNCTION_NAME << " -";
      llvm::dbgs() << " S: " << Twine::utohexstr(S);
      llvm::dbgs() << " A: " << Twine::utohexstr(A);
      llvm::dbgs() << " P: " << Twine::utohexstr(P);
      llvm::dbgs() << " immhi: " << Twine::utohexstr(immhi);
      llvm::dbgs() << " immlo: " << Twine::utohexstr(immlo);
      llvm::dbgs() << " result: " << Twine::utohexstr(result) << "\n");
  write32le(location, immlo | immhi | read32le(location));
}

// R_AARCH64_TLSIE_LD64_GOTTPREL_LO12_NC
static void relocR_AARCH64_TLSIE_LD64_GOTTPREL_LO12_NC(uint8_t *location,
                                                       uint64_t P, uint64_t S,
                                                       int64_t A) {
  int32_t result = S + A;
  result &= 0xFF8;
  result <<= 7;
  DEBUG_WITH_TYPE(
      "AArch64", llvm::dbgs() << "\t\tHandle " << LLVM_FUNCTION_NAME << " -";
      llvm::dbgs() << " S: " << Twine::utohexstr(S);
      llvm::dbgs() << " A: " << Twine::utohexstr(A);
      llvm::dbgs() << " P: " << Twine::utohexstr(P);
      llvm::dbgs() << " result: " << Twine::utohexstr(result) << "\n");
  write32le(location, result | read32le(location));
}

/// \brief R_AARCH64_TLSLE_ADD_TPREL_HI12
static void relocR_AARCH64_TLSLE_ADD_TPREL_HI12(uint8_t *location, uint64_t P,
                                                uint64_t S, int64_t A) {
  int32_t result = S + A;
  result &= 0x0FFF000;
  result >>= 2;
  DEBUG_WITH_TYPE(
      "AArch64", llvm::dbgs() << "\t\tHandle " << LLVM_FUNCTION_NAME << " -";
      llvm::dbgs() << " S: " << Twine::utohexstr(S);
      llvm::dbgs() << " A: " << Twine::utohexstr(A);
      llvm::dbgs() << " P: " << Twine::utohexstr(P);
      llvm::dbgs() << " result: " << Twine::utohexstr(result) << "\n");
  write32le(location, result | read32le(location));
}

/// \brief R_AARCH64_TLSLE_ADD_TPREL_LO12_NC
static void relocR_AARCH64_TLSLE_ADD_TPREL_LO12_NC(uint8_t *location,
                                                   uint64_t P, uint64_t S,
                                                   int64_t A) {
  int32_t result = S + A;
  result &= 0x0FFF;
  result <<= 10;
  DEBUG_WITH_TYPE(
      "AArch64", llvm::dbgs() << "\t\tHandle " << LLVM_FUNCTION_NAME << " -";
      llvm::dbgs() << " S: " << Twine::utohexstr(S);
      llvm::dbgs() << " A: " << Twine::utohexstr(A);
      llvm::dbgs() << " P: " << Twine::utohexstr(P);
      llvm::dbgs() << " result: " << Twine::utohexstr(result) << "\n");
  write32le(location, result | read32le(location));
}

std::error_code AArch64TargetRelocationHandler::applyRelocation(
    ELFWriter &writer, llvm::FileOutputBuffer &buf, const lld::AtomLayout &atom,
    const Reference &ref) const {
  uint8_t *atomContent = buf.getBufferStart() + atom._fileOffset;
  uint8_t *location = atomContent + ref.offsetInAtom();
  uint64_t targetVAddress = writer.addressOfAtom(ref.target());
  uint64_t relocVAddress = atom._virtualAddr + ref.offsetInAtom();

  if (ref.kindNamespace() != Reference::KindNamespace::ELF)
    return std::error_code();
  assert(ref.kindArch() == Reference::KindArch::AArch64);
  switch (ref.kindValue()) {
  case R_AARCH64_NONE:
    break;
  case R_AARCH64_ABS64:
    relocR_AARCH64_ABS64(location, relocVAddress, targetVAddress, ref.addend());
    break;
  case R_AARCH64_PREL32:
    relocR_AARCH64_PREL32(location, relocVAddress, targetVAddress,
                          ref.addend());
    break;
  case R_AARCH64_ABS32:
    return relocR_AARCH64_ABS32(location, relocVAddress, targetVAddress,
                                ref.addend());
  // Runtime only relocations. Ignore here.
  case R_AARCH64_RELATIVE:
  case R_AARCH64_IRELATIVE:
  case R_AARCH64_JUMP_SLOT:
  case R_AARCH64_GLOB_DAT:
    break;
  case R_AARCH64_ADR_PREL_PG_HI21:
    relocR_AARCH64_ADR_PREL_PG_HI21(location, relocVAddress, targetVAddress,
                                    ref.addend());
    break;
  case R_AARCH64_ADR_PREL_LO21:
    relocR_AARCH64_ADR_PREL_LO21(location, relocVAddress, targetVAddress,
                                 ref.addend());
    break;
  case R_AARCH64_ADD_ABS_LO12_NC:
    relocR_AARCH64_ADD_ABS_LO12_NC(location, relocVAddress, targetVAddress,
                                   ref.addend());
    break;
  case R_AARCH64_CALL26:
  case R_AARCH64_JUMP26:
    relocJump26(location, relocVAddress, targetVAddress, ref.addend());
    break;
  case R_AARCH64_CONDBR19:
    relocR_AARCH64_CONDBR19(location, relocVAddress, targetVAddress,
                            ref.addend());
    break;
  case R_AARCH64_ADR_GOT_PAGE:
    relocR_AARCH64_ADR_GOT_PAGE(location, relocVAddress, targetVAddress,
                                ref.addend());
    break;
  case R_AARCH64_LD64_GOT_LO12_NC:
    relocR_AARCH64_LD64_GOT_LO12_NC(location, relocVAddress, targetVAddress,
                                    ref.addend());
    break;
  case R_AARCH64_LDST8_ABS_LO12_NC:
    relocR_AARCH64_LDST8_ABS_LO12_NC(location, relocVAddress, targetVAddress,
                                     ref.addend());
    break;
  case R_AARCH64_LDST16_ABS_LO12_NC:
    relocR_AARCH64_LDST16_ABS_LO12_NC(location, relocVAddress, targetVAddress,
                                      ref.addend());
    break;
  case R_AARCH64_LDST32_ABS_LO12_NC:
    relocR_AARCH64_LDST32_ABS_LO12_NC(location, relocVAddress, targetVAddress,
                                      ref.addend());
    break;
  case R_AARCH64_LDST64_ABS_LO12_NC:
    relocR_AARCH64_LDST64_ABS_LO12_NC(location, relocVAddress, targetVAddress,
                                      ref.addend());
    break;
  case R_AARCH64_LDST128_ABS_LO12_NC:
    relocR_AARCH64_LDST128_ABS_LO12_NC(location, relocVAddress, targetVAddress,
                                       ref.addend());
    break;
  case ADD_AARCH64_GOTRELINDEX:
    relocADD_AARCH64_GOTRELINDEX(location, relocVAddress, targetVAddress,
                                 ref.addend());
    break;
  case R_AARCH64_TLSIE_ADR_GOTTPREL_PAGE21:
    relocR_AARCH64_TLSIE_ADR_GOTTPREL_PAGE21(location, relocVAddress,
                                             targetVAddress, ref.addend());
    break;
  case R_AARCH64_TLSIE_LD64_GOTTPREL_LO12_NC:
    relocR_AARCH64_TLSIE_LD64_GOTTPREL_LO12_NC(location, relocVAddress,
                                               targetVAddress, ref.addend());
    break;
  case R_AARCH64_TLSLE_ADD_TPREL_HI12:
    relocR_AARCH64_TLSLE_ADD_TPREL_HI12(location, relocVAddress, targetVAddress,
                                        ref.addend());
    break;
  case R_AARCH64_TLSLE_ADD_TPREL_LO12_NC:
    relocR_AARCH64_TLSLE_ADD_TPREL_LO12_NC(location, relocVAddress,
                                           targetVAddress, ref.addend());
    break;
  default:
    return make_unhandled_reloc_error();
  }

  return std::error_code();
}
