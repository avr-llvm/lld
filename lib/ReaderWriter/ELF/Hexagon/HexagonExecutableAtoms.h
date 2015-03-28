//===- lib/ReaderWriter/ELF/Hexagon/HexagonExecutableAtoms.h --------------===//
//
//                             The LLVM Linker
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLD_READER_WRITER_ELF_HEXAGON_HEXAGON_EXECUTABLE_ATOM_H
#define LLD_READER_WRITER_ELF_HEXAGON_HEXAGON_EXECUTABLE_ATOM_H

#include "ELFFile.h"

namespace lld {
namespace elf {
typedef llvm::object::ELFType<llvm::support::little, 2, false> HexagonELFType;
class HexagonLinkingContext;

template <class HexagonELFType> class HexagonRuntimeFile
  : public RuntimeFile<HexagonELFType> {
public:
  HexagonRuntimeFile(HexagonLinkingContext &context)
      : RuntimeFile<HexagonELFType>(context, "Hexagon runtime file") {}
};
} // elf
} // lld

#endif // LLD_READER_WRITER_ELF_HEXAGON_HEXAGON_EXECUTABLE_ATOM_H
