//===- lib/ReaderWriter/ELF/OutputELFWriter.h ----------------------------===//
//
//                             The LLVM Linker
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
#ifndef LLD_READER_WRITER_ELF_OUTPUT_WRITER_H
#define LLD_READER_WRITER_ELF_OUTPUT_WRITER_H

#include "DefaultLayout.h"
#include "ELFFile.h"
#include "TargetLayout.h"
#include "lld/Core/Instrumentation.h"
#include "lld/Core/Parallel.h"
#include "lld/Core/SharedLibraryFile.h"
#include "lld/ReaderWriter/ELFLinkingContext.h"
#include "lld/Core/Writer.h"
#include "llvm/ADT/StringSet.h"
#include "llvm/Support/Path.h"

namespace lld {
namespace elf {
using namespace llvm;
using namespace llvm::object;

template <class ELFT> class OutputELFWriter;
template <class ELFT> class TargetLayout;

//===----------------------------------------------------------------------===//
//  OutputELFWriter Class
//===----------------------------------------------------------------------===//
/// \brief This acts as the base class for all the ELF writers that are output
/// for emitting an ELF output file. This class also acts as a common class for
/// creating static and dynamic executables. All the function in this class
/// can be overridden and an appropriate writer be created
template<class ELFT>
class OutputELFWriter : public ELFWriter {
public:
  typedef Elf_Shdr_Impl<ELFT> Elf_Shdr;
  typedef Elf_Sym_Impl<ELFT> Elf_Sym;
  typedef Elf_Dyn_Impl<ELFT> Elf_Dyn;

  OutputELFWriter(const ELFLinkingContext &context, TargetLayout<ELFT> &layout);

protected:
  // build the sections that need to be created
  virtual void createDefaultSections();

  // Build all the output sections
  void buildChunks(const File &file) override;

  // Build the output file
  virtual std::error_code buildOutput(const File &file);

  // Setup the ELF header.
  virtual std::error_code setELFHeader();

  // Write the file to the path specified
  std::error_code writeFile(const File &File, StringRef path) override;

  // Write to the output file.
  virtual std::error_code writeOutput(const File &file, StringRef path);

  // Get the size of the output file that the linker would emit.
  virtual uint64_t outputFileSize() const;

  // Build the atom to address map, this has to be called
  // before applying relocations
  virtual void buildAtomToAddressMap(const File &file);

  // Build the symbol table for static linking
  virtual void buildStaticSymbolTable(const File &file);

  // Build the dynamic symbol table for dynamic linking
  virtual void buildDynamicSymbolTable(const File &file);

  // Build the section header table
  virtual void buildSectionHeaderTable();

  // Assign sections that have no segments such as the symbol table,
  // section header table, string table etc
  virtual void assignSectionsWithNoSegments();

  // Add default atoms that need to be present in the output file
  virtual void addDefaultAtoms() = 0;

  // Add any runtime files and their atoms to the output
  bool createImplicitFiles(std::vector<std::unique_ptr<File>> &) override;

  // Finalize the default atom values
  virtual void finalizeDefaultAtomValues() = 0;

  // This is called by the write section to apply relocations
  uint64_t addressOfAtom(const Atom *atom) override {
    auto addr = _atomToAddressMap.find(atom);
    return addr == _atomToAddressMap.end() ? 0 : addr->second;
  }

  // This is a hook for creating default dynamic entries
  virtual void createDefaultDynamicEntries() {}

  /// \brief Create symbol table.
  virtual LLD_UNIQUE_BUMP_PTR(SymbolTable<ELFT>) createSymbolTable();

  /// \brief create dynamic table.
  virtual LLD_UNIQUE_BUMP_PTR(DynamicTable<ELFT>) createDynamicTable();

  /// \brief create dynamic symbol table.
  virtual LLD_UNIQUE_BUMP_PTR(DynamicSymbolTable<ELFT>)
      createDynamicSymbolTable();

  /// \brief Create entry in the dynamic symbols table for this atom.
  virtual bool isDynSymEntryRequired(const SharedLibraryAtom *sla) const {
    return _layout.isReferencedByDefinedAtom(sla);
  }

  /// \brief Create DT_NEEDED dynamic tage for the shared library.
  virtual bool isNeededTagRequired(const SharedLibraryAtom *sla) const {
    return false;
  }

  llvm::BumpPtrAllocator _alloc;

  const ELFLinkingContext &_context;
  TargetHandler<ELFT> &_targetHandler;

  typedef llvm::DenseMap<const Atom *, uint64_t> AtomToAddress;
  AtomToAddress _atomToAddressMap;
  TargetLayout<ELFT> &_layout;
  LLD_UNIQUE_BUMP_PTR(ELFHeader<ELFT>) _elfHeader;
  LLD_UNIQUE_BUMP_PTR(ProgramHeader<ELFT>) _programHeader;
  LLD_UNIQUE_BUMP_PTR(SymbolTable<ELFT>) _symtab;
  LLD_UNIQUE_BUMP_PTR(StringTable<ELFT>) _strtab;
  LLD_UNIQUE_BUMP_PTR(StringTable<ELFT>) _shstrtab;
  LLD_UNIQUE_BUMP_PTR(SectionHeader<ELFT>) _shdrtab;
  LLD_UNIQUE_BUMP_PTR(EHFrameHeader<ELFT>) _ehFrameHeader;
  /// \name Dynamic sections.
  /// @{
  LLD_UNIQUE_BUMP_PTR(DynamicTable<ELFT>) _dynamicTable;
  LLD_UNIQUE_BUMP_PTR(DynamicSymbolTable<ELFT>) _dynamicSymbolTable;
  LLD_UNIQUE_BUMP_PTR(StringTable<ELFT>) _dynamicStringTable;
  LLD_UNIQUE_BUMP_PTR(HashSection<ELFT>) _hashTable;
  llvm::StringSet<> _soNeeded;
  /// @}

private:
  static StringRef maybeGetSOName(Node *node);
};

//===----------------------------------------------------------------------===//
//  OutputELFWriter
//===----------------------------------------------------------------------===//
template <class ELFT>
OutputELFWriter<ELFT>::OutputELFWriter(const ELFLinkingContext &context,
                                       TargetLayout<ELFT> &layout)
    : _context(context), _targetHandler(context.getTargetHandler<ELFT>()),
      _layout(layout) {}

template <class ELFT>
void OutputELFWriter<ELFT>::buildChunks(const File &file) {
  ScopedTask task(getDefaultDomain(), "buildChunks");
  for (const DefinedAtom *definedAtom : file.defined()) {
    DefinedAtom::ContentType contentType = definedAtom->contentType();
    // Dont add COMDAT group atoms and GNU linkonce atoms, as they are used for
    // symbol resolution.
    // TODO: handle partial linking.
    if (contentType == DefinedAtom::typeGroupComdat ||
        contentType == DefinedAtom::typeGnuLinkOnce)
      continue;
    _layout.addAtom(definedAtom);
  }
  for (const AbsoluteAtom *absoluteAtom : file.absolute())
    _layout.addAtom(absoluteAtom);
}

template <class ELFT>
void OutputELFWriter<ELFT>::buildStaticSymbolTable(const File &file) {
  ScopedTask task(getDefaultDomain(), "buildStaticSymbolTable");
  for (auto sec : _layout.sections())
    if (auto section = dyn_cast<AtomSection<ELFT>>(sec))
      for (const auto &atom : section->atoms())
        _symtab->addSymbol(atom->_atom, section->ordinal(), atom->_virtualAddr);
  for (auto &atom : _layout.absoluteAtoms())
    _symtab->addSymbol(atom->_atom, ELF::SHN_ABS, atom->_virtualAddr);
  for (const UndefinedAtom *a : file.undefined())
    _symtab->addSymbol(a, ELF::SHN_UNDEF);
}

// Returns the DSO name for a given input file if it's a shared library
// file and not marked as --as-needed.
template <class ELFT>
StringRef OutputELFWriter<ELFT>::maybeGetSOName(Node *node) {
  if (auto *fnode = dyn_cast<FileNode>(node))
    if (!fnode->asNeeded())
      if (auto *file = dyn_cast<SharedLibraryFile>(fnode->getFile()))
        return file->getDSOName();
  return "";
}

template <class ELFT>
void OutputELFWriter<ELFT>::buildDynamicSymbolTable(const File &file) {
  ScopedTask task(getDefaultDomain(), "buildDynamicSymbolTable");
  for (const auto &sla : file.sharedLibrary()) {
    if (isDynSymEntryRequired(sla)) {
      _dynamicSymbolTable->addSymbol(sla, ELF::SHN_UNDEF);
      _soNeeded.insert(sla->loadName());
      continue;
    }
    if (isNeededTagRequired(sla))
      _soNeeded.insert(sla->loadName());
  }
  for (const std::unique_ptr<Node> &node : _context.getNodes()) {
    StringRef soname = maybeGetSOName(node.get());
    if (!soname.empty())
      _soNeeded.insert(soname);
  }
  // Never mark the dynamic linker as DT_NEEDED
  _soNeeded.erase(sys::path::filename(_context.getInterpreter()));
  for (const auto &loadName : _soNeeded) {
    Elf_Dyn dyn;
    dyn.d_tag = DT_NEEDED;
    dyn.d_un.d_val = _dynamicStringTable->addString(loadName.getKey());
    _dynamicTable->addEntry(dyn);
  }
  const auto &rpathList = _context.getRpathList();
  if (!rpathList.empty()) {
    auto rpath = new (_alloc) std::string(join(rpathList.begin(),
      rpathList.end(), ":"));
    Elf_Dyn dyn;
    dyn.d_tag = DT_RPATH;
    dyn.d_un.d_val = _dynamicStringTable->addString(*rpath);
    _dynamicTable->addEntry(dyn);
  }
  StringRef soname = _context.sharedObjectName();
  if (!soname.empty() && _context.getOutputELFType() == llvm::ELF::ET_DYN) {
    Elf_Dyn dyn;
    dyn.d_tag = DT_SONAME;
    dyn.d_un.d_val = _dynamicStringTable->addString(soname);
    _dynamicTable->addEntry(dyn);
  }
  // The dynamic symbol table need to be sorted earlier because the hash
  // table needs to be built using the dynamic symbol table. It would be
  // late to sort the symbols due to that in finalize. In the dynamic symbol
  // table finalize, we call the symbol table finalize and we don't want to
  // sort again
  _dynamicSymbolTable->sortSymbols();

  // Add the dynamic symbols into the hash table
  _dynamicSymbolTable->addSymbolsToHashTable();
}

template <class ELFT>
void OutputELFWriter<ELFT>::buildAtomToAddressMap(const File &file) {
  ScopedTask task(getDefaultDomain(), "buildAtomToAddressMap");
  int64_t totalAbsAtoms = _layout.absoluteAtoms().size();
  int64_t totalUndefinedAtoms = file.undefined().size();
  int64_t totalDefinedAtoms = 0;
  for (auto sec : _layout.sections())
    if (auto section = dyn_cast<AtomSection<ELFT> >(sec)) {
      totalDefinedAtoms += section->atoms().size();
      for (const auto &atom : section->atoms())
        _atomToAddressMap[atom->_atom] = atom->_virtualAddr;
    }
  // build the atomToAddressMap that contains absolute symbols too
  for (auto &atom : _layout.absoluteAtoms())
    _atomToAddressMap[atom->_atom] = atom->_virtualAddr;

  // Set the total number of atoms in the symbol table, so that appropriate
  // resizing of the string table can be done
  _symtab->setNumEntries(totalDefinedAtoms + totalAbsAtoms +
                         totalUndefinedAtoms);
}

template<class ELFT>
void OutputELFWriter<ELFT>::buildSectionHeaderTable() {
  ScopedTask task(getDefaultDomain(), "buildSectionHeaderTable");
  for (auto outputSection : _layout.outputSections()) {
    if (outputSection->kind() != Chunk<ELFT>::Kind::ELFSection &&
        outputSection->kind() != Chunk<ELFT>::Kind::AtomSection)
      continue;
    if (outputSection->hasSegment())
      _shdrtab->appendSection(outputSection);
  }
}

template<class ELFT>
void OutputELFWriter<ELFT>::assignSectionsWithNoSegments() {
  ScopedTask task(getDefaultDomain(), "assignSectionsWithNoSegments");
  for (auto outputSection : _layout.outputSections()) {
    if (outputSection->kind() != Chunk<ELFT>::Kind::ELFSection &&
        outputSection->kind() != Chunk<ELFT>::Kind::AtomSection)
      continue;
    if (!outputSection->hasSegment())
      _shdrtab->appendSection(outputSection);
  }
  _layout.assignFileOffsetsForMiscSections();
  for (auto sec : _layout.sections())
    if (auto section = dyn_cast<Section<ELFT>>(sec))
      if (!DefaultLayout<ELFT>::hasOutputSegment(section))
        _shdrtab->updateSection(section);
}

template <class ELFT>
bool OutputELFWriter<ELFT>::createImplicitFiles(
    std::vector<std::unique_ptr<File>> &) {
  return true;
}

template <class ELFT> void OutputELFWriter<ELFT>::createDefaultSections() {
  _elfHeader.reset(new (_alloc) ELFHeader<ELFT>(_context));
  _programHeader.reset(new (_alloc) ProgramHeader<ELFT>(_context));
  _layout.setHeader(_elfHeader.get());
  _layout.setProgramHeader(_programHeader.get());

  _symtab = std::move(this->createSymbolTable());
  _strtab.reset(new (_alloc) StringTable<ELFT>(
      _context, ".strtab", DefaultLayout<ELFT>::ORDER_STRING_TABLE));
  _shstrtab.reset(new (_alloc) StringTable<ELFT>(
      _context, ".shstrtab", DefaultLayout<ELFT>::ORDER_SECTION_STRINGS));
  _shdrtab.reset(new (_alloc) SectionHeader<ELFT>(
      _context, DefaultLayout<ELFT>::ORDER_SECTION_HEADERS));
  _layout.addSection(_symtab.get());
  _layout.addSection(_strtab.get());
  _layout.addSection(_shstrtab.get());
  _shdrtab->setStringSection(_shstrtab.get());
  _symtab->setStringSection(_strtab.get());
  _layout.addSection(_shdrtab.get());

  for (auto sec : _layout.sections()) {
    // TODO: use findOutputSection
    auto section = dyn_cast<Section<ELFT>>(sec);
    if (!section || section->outputSectionName() != ".eh_frame")
      continue;
    _ehFrameHeader.reset(new (_alloc) EHFrameHeader<ELFT>(
        _context, ".eh_frame_hdr", _layout,
        DefaultLayout<ELFT>::ORDER_EH_FRAMEHDR));
    _layout.addSection(_ehFrameHeader.get());
    break;
  }

  if (_context.isDynamic()) {
    _dynamicTable = std::move(createDynamicTable());
    _dynamicStringTable.reset(new (_alloc) StringTable<ELFT>(
        _context, ".dynstr", DefaultLayout<ELFT>::ORDER_DYNAMIC_STRINGS, true));
    _dynamicSymbolTable = std::move(createDynamicSymbolTable());
    _hashTable.reset(new (_alloc) HashSection<ELFT>(
        _context, ".hash", DefaultLayout<ELFT>::ORDER_HASH));
    // Set the hash table in the dynamic symbol table so that the entries in the
    // hash table can be created
    _dynamicSymbolTable->setHashTable(_hashTable.get());
    _hashTable->setSymbolTable(_dynamicSymbolTable.get());
    _layout.addSection(_dynamicTable.get());
    _layout.addSection(_dynamicStringTable.get());
    _layout.addSection(_dynamicSymbolTable.get());
    _layout.addSection(_hashTable.get());
    _dynamicSymbolTable->setStringSection(_dynamicStringTable.get());
    _dynamicTable->setSymbolTable(_dynamicSymbolTable.get());
    _dynamicTable->setHashTable(_hashTable.get());
    if (_layout.hasDynamicRelocationTable())
      _layout.getDynamicRelocationTable()->setSymbolTable(
          _dynamicSymbolTable.get());
    if (_layout.hasPLTRelocationTable())
      _layout.getPLTRelocationTable()->setSymbolTable(
          _dynamicSymbolTable.get());
  }
}

template <class ELFT>
LLD_UNIQUE_BUMP_PTR(SymbolTable<ELFT>)
    OutputELFWriter<ELFT>::createSymbolTable() {
  return LLD_UNIQUE_BUMP_PTR(SymbolTable<ELFT>)(new (_alloc) SymbolTable<ELFT>(
      this->_context, ".symtab", DefaultLayout<ELFT>::ORDER_SYMBOL_TABLE));
}

/// \brief create dynamic table
template <class ELFT>
LLD_UNIQUE_BUMP_PTR(DynamicTable<ELFT>)
    OutputELFWriter<ELFT>::createDynamicTable() {
  return LLD_UNIQUE_BUMP_PTR(
      DynamicTable<ELFT>)(new (_alloc) DynamicTable<ELFT>(
      this->_context, _layout, ".dynamic", DefaultLayout<ELFT>::ORDER_DYNAMIC));
}

/// \brief create dynamic symbol table
template <class ELFT>
LLD_UNIQUE_BUMP_PTR(DynamicSymbolTable<ELFT>)
    OutputELFWriter<ELFT>::createDynamicSymbolTable() {
  return LLD_UNIQUE_BUMP_PTR(
      DynamicSymbolTable<ELFT>)(new (_alloc) DynamicSymbolTable<ELFT>(
      this->_context, _layout, ".dynsym",
      DefaultLayout<ELFT>::ORDER_DYNAMIC_SYMBOLS));
}

template <class ELFT>
std::error_code OutputELFWriter<ELFT>::buildOutput(const File &file) {
  ScopedTask buildTask(getDefaultDomain(), "ELF Writer buildOutput");
  buildChunks(file);

  // Create the default sections like the symbol table, string table, and the
  // section string table
  createDefaultSections();

  // Set the Layout
  _layout.assignSectionsToSegments();

  // Create the dynamic table entries
  if (_context.isDynamic()) {
    _dynamicTable->createDefaultEntries();
    buildDynamicSymbolTable(file);
  }

  // Call the preFlight callbacks to modify the sections and the atoms
  // contained in them, in anyway the targets may want
  _layout.doPreFlight();

  _layout.assignVirtualAddress();

  // Finalize the default value of symbols that the linker adds
  finalizeDefaultAtomValues();

  // Build the Atom To Address map for applying relocations
  buildAtomToAddressMap(file);

  // Create symbol table and section string table
  // Do it only if -s is not specified.
  if (!_context.stripSymbols())
    buildStaticSymbolTable(file);

  // Finalize the layout by calling the finalize() functions
  _layout.finalize();

  // build Section Header table
  buildSectionHeaderTable();

  // assign Offsets and virtual addresses
  // for sections with no segments
  assignSectionsWithNoSegments();

  if (_context.isDynamic())
    _dynamicTable->updateDynamicTable();

  return std::error_code();
}

template <class ELFT> std::error_code OutputELFWriter<ELFT>::setELFHeader() {
  _elfHeader->e_type(_context.getOutputELFType());
  _elfHeader->e_machine(_context.getOutputMachine());
  _elfHeader->e_ident(ELF::EI_VERSION, 1);
  _elfHeader->e_ident(ELF::EI_OSABI, 0);
  _elfHeader->e_version(1);
  _elfHeader->e_phoff(_programHeader->fileOffset());
  _elfHeader->e_shoff(_shdrtab->fileOffset());
  _elfHeader->e_phentsize(_programHeader->entsize());
  _elfHeader->e_phnum(_programHeader->numHeaders());
  _elfHeader->e_shentsize(_shdrtab->entsize());
  _elfHeader->e_shnum(_shdrtab->numHeaders());
  _elfHeader->e_shstrndx(_shstrtab->ordinal());
  if (const auto *al = _layout.findAtomLayoutByName(_context.entrySymbolName()))
    _elfHeader->e_entry(al->_virtualAddr);
  else
    _elfHeader->e_entry(0);

  return std::error_code();
}

template <class ELFT> uint64_t OutputELFWriter<ELFT>::outputFileSize() const {
  return _shdrtab->fileOffset() + _shdrtab->fileSize();
}

template <class ELFT>
std::error_code OutputELFWriter<ELFT>::writeOutput(const File &file,
                                                   StringRef path) {
  std::unique_ptr<FileOutputBuffer> buffer;
  ScopedTask createOutputTask(getDefaultDomain(), "ELF Writer Create Output");
  std::error_code ec = FileOutputBuffer::create(path, outputFileSize(), buffer,
                                                FileOutputBuffer::F_executable);
  createOutputTask.end();

  if (ec)
    return ec;

  ScopedTask writeTask(getDefaultDomain(), "ELF Writer write to memory");

  // HACK: We have to write out the header and program header here even though
  // they are a member of a segment because only sections are written in the
  // following loop.

  // Finalize ELF Header / Program Headers.
  _elfHeader->finalize();
  _programHeader->finalize();

  _elfHeader->write(this, _layout, *buffer);
  _programHeader->write(this, _layout, *buffer);

  for (auto section : _layout.sections())
    section->write(this, _layout, *buffer);
  writeTask.end();

  ScopedTask commitTask(getDefaultDomain(), "ELF Writer commit to disk");
  return buffer->commit();
}

template <class ELFT>
std::error_code OutputELFWriter<ELFT>::writeFile(const File &file,
                                                 StringRef path) {
  std::error_code ec = buildOutput(file);
  if (ec)
    return ec;

  ec = setELFHeader();
  if (ec)
    return ec;

  return writeOutput(file, path);
}
} // namespace elf
} // namespace lld

#endif // LLD_READER_WRITER_ELF_OUTPUT_WRITER_H
