//===- lib/ReaderWriter/ELF/Mips/MipsExecutableWriter.h -------------------===//
//
//                             The LLVM Linker
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
#ifndef LLD_READER_WRITER_ELF_MIPS_MIPS_EXECUTABLE_WRITER_H
#define LLD_READER_WRITER_ELF_MIPS_MIPS_EXECUTABLE_WRITER_H

#include "ExecutableWriter.h"
#include "MipsDynamicTable.h"
#include "MipsELFWriters.h"
#include "MipsLinkingContext.h"

namespace lld {
namespace elf {

template <typename ELFT> class MipsTargetLayout;

template <class ELFT>
class MipsExecutableWriter : public ExecutableWriter<ELFT> {
public:
  MipsExecutableWriter(MipsLinkingContext &ctx, MipsTargetLayout<ELFT> &layout);

protected:
  void buildDynamicSymbolTable(const File &file) override;

  // Add any runtime files and their atoms to the output
  bool createImplicitFiles(std::vector<std::unique_ptr<File>> &) override;

  void finalizeDefaultAtomValues() override;
  std::error_code setELFHeader() override;

  unique_bump_ptr<SymbolTable<ELFT>> createSymbolTable() override;
  unique_bump_ptr<DynamicTable<ELFT>> createDynamicTable() override;

  unique_bump_ptr<DynamicSymbolTable<ELFT>>
      createDynamicSymbolTable() override;

private:
  MipsELFWriter<ELFT> _writeHelper;
  MipsTargetLayout<ELFT> &_mipsTargetLayout;
};

template <class ELFT>
MipsExecutableWriter<ELFT>::MipsExecutableWriter(MipsLinkingContext &ctx,
                                                 MipsTargetLayout<ELFT> &layout)
    : ExecutableWriter<ELFT>(ctx, layout), _writeHelper(ctx, layout),
      _mipsTargetLayout(layout) {}

template <class ELFT>
std::error_code MipsExecutableWriter<ELFT>::setELFHeader() {
  std::error_code ec = ExecutableWriter<ELFT>::setELFHeader();
  if (ec)
    return ec;

  StringRef entryName = this->_context.entrySymbolName();
  if (const AtomLayout *al = this->_layout.findAtomLayoutByName(entryName)) {
    const auto *ea = cast<DefinedAtom>(al->_atom);
    if (ea->codeModel() == DefinedAtom::codeMipsMicro ||
        ea->codeModel() == DefinedAtom::codeMipsMicroPIC)
      // Adjust entry symbol value if this symbol is microMIPS encoded.
      this->_elfHeader->e_entry(al->_virtualAddr | 1);
  }

  _writeHelper.setELFHeader(*this->_elfHeader);
  return std::error_code();
}

template <class ELFT>
void MipsExecutableWriter<ELFT>::buildDynamicSymbolTable(const File &file) {
  // MIPS ABI requires to add to dynsym even undefined symbols
  // if they have a corresponding entries in a global part of GOT.
  for (auto sec : this->_layout.sections())
    if (auto section = dyn_cast<AtomSection<ELFT>>(sec))
      for (const auto &atom : section->atoms()) {
        if (_writeHelper.hasGlobalGOTEntry(atom->_atom)) {
          this->_dynamicSymbolTable->addSymbol(atom->_atom, section->ordinal(),
                                               atom->_virtualAddr, atom);
          continue;
        }

        const DefinedAtom *da = dyn_cast<const DefinedAtom>(atom->_atom);
        if (!da)
          continue;

        if (da->dynamicExport() != DefinedAtom::dynamicExportAlways &&
            !this->_context.isDynamicallyExportedSymbol(da->name()) &&
            !(this->_context.shouldExportDynamic() &&
              da->scope() == Atom::Scope::scopeGlobal))
          continue;

        this->_dynamicSymbolTable->addSymbol(atom->_atom, section->ordinal(),
                                             atom->_virtualAddr, atom);
      }

  for (const UndefinedAtom *a : file.undefined())
    // FIXME (simon): Consider to move this check to the
    // MipsELFUndefinedAtom class method. That allows to
    // handle more complex coditions in the future.
    if (_writeHelper.hasGlobalGOTEntry(a))
      this->_dynamicSymbolTable->addSymbol(a, ELF::SHN_UNDEF);

  // Skip our immediate parent class method
  // ExecutableWriter<ELFT>::buildDynamicSymbolTable because we replaced it
  // with our own version. Call OutputELFWriter directly.
  OutputELFWriter<ELFT>::buildDynamicSymbolTable(file);
}

template <class ELFT>
bool MipsExecutableWriter<ELFT>::createImplicitFiles(
    std::vector<std::unique_ptr<File>> &result) {
  ExecutableWriter<ELFT>::createImplicitFiles(result);
  result.push_back(std::move(_writeHelper.createRuntimeFile()));
  return true;
}

template <class ELFT>
void MipsExecutableWriter<ELFT>::finalizeDefaultAtomValues() {
  // Finalize the atom values that are part of the parent.
  ExecutableWriter<ELFT>::finalizeDefaultAtomValues();
  _writeHelper.finalizeMipsRuntimeAtomValues();
}

template <class ELFT>
unique_bump_ptr<SymbolTable<ELFT>>
    MipsExecutableWriter<ELFT>::createSymbolTable() {
  return unique_bump_ptr<SymbolTable<ELFT>>(new (
      this->_alloc) MipsSymbolTable<ELFT>(this->_context));
}

/// \brief create dynamic table
template <class ELFT>
unique_bump_ptr<DynamicTable<ELFT>>
    MipsExecutableWriter<ELFT>::createDynamicTable() {
  return unique_bump_ptr<DynamicTable<ELFT>>(new (
      this->_alloc) MipsDynamicTable<ELFT>(this->_context, _mipsTargetLayout));
}

/// \brief create dynamic symbol table
template <class ELFT>
unique_bump_ptr<DynamicSymbolTable<ELFT>>
    MipsExecutableWriter<ELFT>::createDynamicSymbolTable() {
  return unique_bump_ptr<DynamicSymbolTable<ELFT>>(
      new (this->_alloc) MipsDynamicSymbolTable<ELFT>(
          this->_context, _mipsTargetLayout));
}

} // namespace elf
} // namespace lld

#endif
