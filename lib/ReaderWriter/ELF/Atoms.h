//===- lib/ReaderWriter/ELF/Atoms.h ---------------------------------------===//
//
//                             The LLVM Linker
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLD_READER_WRITER_ELF_ATOMS_H
#define LLD_READER_WRITER_ELF_ATOMS_H

#include "TargetHandler.h"
#include "lld/Core/LLVM.h"
#include "lld/Core/Simple.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/StringSwitch.h"
#include <memory>
#include <vector>

namespace lld {
namespace elf {
template <class ELFT> class DynamicFile;
template <typename ELFT> class ELFFile;

/// \brief Relocation References: Defined Atoms may contain references that will
/// need to be patched before the executable is written.
///
/// Construction of ELFReferences is two pass process. ELFReferences are
/// instantiated while we are iterating over symbol tables to atomize
/// symbols. At that time we only know the index of relocation target symbol
/// (not target atom) about a relocation, so we store the index to
/// ELFREference. In the second pass, ELFReferences are revisited to update
/// target atoms by target symbol indexes.
template <class ELFT> class ELFReference : public Reference {
  typedef llvm::object::Elf_Rel_Impl<ELFT, false> Elf_Rel;
  typedef llvm::object::Elf_Rel_Impl<ELFT, true> Elf_Rela;
  typedef llvm::object::Elf_Sym_Impl<ELFT> Elf_Sym;

public:
  ELFReference(const Elf_Rela *rela, uint64_t off, Reference::KindArch arch,
               Reference::KindValue relocType, uint32_t idx)
      : Reference(Reference::KindNamespace::ELF, arch, relocType),
        _target(nullptr), _targetSymbolIndex(idx), _offsetInAtom(off),
        _addend(rela->r_addend) {}

  ELFReference(uint64_t off, Reference::KindArch arch,
               Reference::KindValue relocType, uint32_t idx)
      : Reference(Reference::KindNamespace::ELF, arch, relocType),
        _target(nullptr), _targetSymbolIndex(idx), _offsetInAtom(off),
        _addend(0) {}

  ELFReference(uint32_t edgeKind)
      : Reference(Reference::KindNamespace::all, Reference::KindArch::all,
                  edgeKind),
        _target(nullptr), _targetSymbolIndex(0), _offsetInAtom(0), _addend(0) {}

  uint64_t offsetInAtom() const override { return _offsetInAtom; }

  const Atom *target() const override { return _target; }

  /// \brief The symbol table index that contains the target reference.
  uint64_t targetSymbolIndex() const {
    return _targetSymbolIndex;
  }

  Addend addend() const override { return _addend; }

  virtual void setOffset(uint64_t off) { _offsetInAtom = off; }

  void setAddend(Addend A) override { _addend = A; }

  void setTarget(const Atom *newAtom) override { _target = newAtom; }

private:
  const Atom *_target;
  uint64_t _targetSymbolIndex;
  uint64_t _offsetInAtom;
  Addend _addend;
};

/// \brief These atoms store symbols that are fixed to a particular address.
/// This atom has no content its address will be used by the writer to fixup
/// references that point to it.
template <class ELFT> class ELFAbsoluteAtom : public AbsoluteAtom {
  typedef llvm::object::Elf_Sym_Impl<ELFT> Elf_Sym;

public:
  ELFAbsoluteAtom(const ELFFile<ELFT> &file, StringRef name,
                  const Elf_Sym *symbol, uint64_t value)
      : _owningFile(file), _name(name), _symbol(symbol), _value(value) {
  }

  const ELFFile<ELFT> &file() const override { return _owningFile; }

  Scope scope() const override {
    if (_symbol->getVisibility() == llvm::ELF::STV_HIDDEN)
      return scopeLinkageUnit;
    if (_symbol->getBinding() == llvm::ELF::STB_LOCAL)
      return scopeTranslationUnit;
    else
      return scopeGlobal;
  }

  StringRef name() const override { return _name; }

  uint64_t value() const override { return _value; }

private:
  const ELFFile<ELFT> &_owningFile;
  StringRef _name;
  const Elf_Sym *_symbol;
  uint64_t _value;
};

/// \brief ELFUndefinedAtom: These atoms store undefined symbols and are place
/// holders that will be replaced by defined atoms later in the linking process.
template <class ELFT> class ELFUndefinedAtom : public lld::UndefinedAtom {
  typedef llvm::object::Elf_Sym_Impl<ELFT> Elf_Sym;

public:
  ELFUndefinedAtom(const File &file, StringRef name, const Elf_Sym *symbol)
      : _owningFile(file), _name(name), _symbol(symbol) {}

  const File &file() const override { return _owningFile; }

  StringRef name() const override { return _name; }

  // A symbol in ELF can be undefined at build time if the symbol is a undefined
  // weak symbol.
  CanBeNull canBeNull() const override {
    if (_symbol->getBinding() == llvm::ELF::STB_WEAK)
      return CanBeNull::canBeNullAtBuildtime;
    else
      return CanBeNull::canBeNullNever;
  }

private:
  const File &_owningFile;
  StringRef _name;
  const Elf_Sym *_symbol;
};

/// \brief This atom stores defined symbols and will contain either data or
/// code.
template <class ELFT> class ELFDefinedAtom : public DefinedAtom {
  typedef llvm::object::Elf_Sym_Impl<ELFT> Elf_Sym;
  typedef llvm::object::Elf_Shdr_Impl<ELFT> Elf_Shdr;

public:
  ELFDefinedAtom(const ELFFile<ELFT> &file, StringRef symbolName,
                 StringRef sectionName, const Elf_Sym *symbol,
                 const Elf_Shdr *section, ArrayRef<uint8_t> contentData,
                 unsigned int referenceStart, unsigned int referenceEnd,
                 std::vector<ELFReference<ELFT> *> &referenceList)
      : _owningFile(file), _symbolName(symbolName), _sectionName(sectionName),
        _symbol(symbol), _section(section), _contentData(contentData),
        _referenceStartIndex(referenceStart), _referenceEndIndex(referenceEnd),
        _referenceList(referenceList), _contentType(typeUnknown),
        _permissions(permUnknown) {}

  ~ELFDefinedAtom() {}

  const ELFFile<ELFT> &file() const override { return _owningFile; }

  StringRef name() const override { return _symbolName; }

  uint64_t ordinal() const override { return _ordinal; }

  const Elf_Sym *symbol() const { return _symbol; }

  const Elf_Shdr *section() const { return _section; }

  uint64_t size() const override {
    // Common symbols are not allocated in object files,
    // so use st_size to tell how many bytes are required.
    if (_symbol && (_symbol->getType() == llvm::ELF::STT_COMMON ||
                    _symbol->st_shndx == llvm::ELF::SHN_COMMON))
      return (uint64_t) _symbol->st_size;

    return _contentData.size();
  }

  Scope scope() const override {
    if (!_symbol)
      return scopeGlobal;
    if (_symbol->getVisibility() == llvm::ELF::STV_HIDDEN)
      return scopeLinkageUnit;
    else if (_symbol->getBinding() != llvm::ELF::STB_LOCAL)
      return scopeGlobal;
    else
      return scopeTranslationUnit;
  }

  // FIXME: Need to revisit this in future.
  Interposable interposable() const override { return interposeNo; }

  Merge merge() const override {
    if (!_symbol)
      return mergeNo;

    if (_symbol->getBinding() == llvm::ELF::STB_WEAK)
      return mergeAsWeak;

    if ((_symbol->getType() == llvm::ELF::STT_COMMON) ||
        _symbol->st_shndx == llvm::ELF::SHN_COMMON)
      return mergeAsTentative;

    return mergeNo;
  }

  ContentType contentType() const override {
    if (_contentType != typeUnknown)
      return _contentType;

    ContentType ret = typeUnknown;
    uint64_t flags = _section->sh_flags;

    if (_section->sh_type == llvm::ELF::SHT_GROUP)
      return typeGroupComdat;

    if (!_symbol && _sectionName.startswith(".gnu.linkonce"))
      return typeGnuLinkOnce;

    if (!(flags & llvm::ELF::SHF_ALLOC))
      return _contentType = typeNoAlloc;

    if (_section->sh_flags ==
        (llvm::ELF::SHF_ALLOC | llvm::ELF::SHF_WRITE | llvm::ELF::SHF_TLS)) {
      return _contentType = _section->sh_type == llvm::ELF::SHT_NOBITS ? typeThreadZeroFill
                                                        : typeThreadData;
    }

    if ((_section->sh_flags == llvm::ELF::SHF_ALLOC) &&
        (_section->sh_type == llvm::ELF::SHT_PROGBITS))
      return _contentType = typeConstant;

    if (_symbol->getType() == llvm::ELF::STT_GNU_IFUNC)
      return _contentType = typeResolver;

    if (_symbol->st_shndx == llvm::ELF::SHN_COMMON)
      return _contentType = typeZeroFill;

    switch (_section->sh_type) {
    case llvm::ELF::SHT_PROGBITS:
      flags &= ~llvm::ELF::SHF_ALLOC;
      flags &= ~llvm::ELF::SHF_GROUP;
      switch (flags) {
      case llvm::ELF::SHF_EXECINSTR:
      case (llvm::ELF::SHF_WRITE|llvm::ELF::SHF_EXECINSTR):
        ret = typeCode;
        break;
      case llvm::ELF::SHF_WRITE:
        ret = typeData;
        break;
      case (llvm::ELF::SHF_MERGE|llvm::ELF::SHF_STRINGS):
      case llvm::ELF::SHF_STRINGS:
      case llvm::ELF::SHF_MERGE:
        ret = typeConstant;
        break;
      default:
        ret = typeCode;
        break;
      }
      break;
    case llvm::ELF::SHT_NOTE:
      flags &= ~llvm::ELF::SHF_ALLOC;
      switch (flags) {
      case llvm::ELF::SHF_WRITE:
        ret = typeRWNote;
        break;
      default:
        ret = typeRONote;
        break;
      }
      break;
    case llvm::ELF::SHT_NOBITS:
      ret = typeZeroFill;
      break;
    case llvm::ELF::SHT_NULL:
      if ((_symbol->getType() == llvm::ELF::STT_COMMON)
          || _symbol->st_shndx == llvm::ELF::SHN_COMMON)
        ret = typeZeroFill;
      break;
    case llvm::ELF::SHT_INIT_ARRAY:
    case llvm::ELF::SHT_FINI_ARRAY:
      ret = typeData;
      break;
    }

    return _contentType = ret;
  }

  Alignment alignment() const override {
    if (!_symbol)
      return Alignment(0);

    // Obtain proper value of st_value field.
    const auto symValue = getSymbolValue(_symbol);

    // Unallocated common symbols specify their alignment constraints in
    // st_value.
    if ((_symbol->getType() == llvm::ELF::STT_COMMON) ||
        _symbol->st_shndx == llvm::ELF::SHN_COMMON) {
      return Alignment(llvm::Log2_64(symValue));
    } else if (_section->sh_addralign == 0) {
      // sh_addralign of 0 means no alignment
      return Alignment(0, symValue);
    }
    return Alignment(llvm::Log2_64(_section->sh_addralign),
                     symValue % _section->sh_addralign);
  }

  // Do we have a choice for ELF?  All symbols live in explicit sections.
  SectionChoice sectionChoice() const override {
    switch (contentType()) {
    case typeCode:
    case typeData:
    case typeZeroFill:
    case typeThreadZeroFill:
    case typeThreadData:
    case typeConstant:
      if ((_sectionName == ".text") || (_sectionName == ".data") ||
          (_sectionName == ".bss") || (_sectionName == ".rodata") ||
          (_sectionName == ".tdata") || (_sectionName == ".tbss"))
        return sectionBasedOnContent;
    default:
      break;
    }
    return sectionCustomRequired;
  }

  StringRef customSectionName() const override {
    if ((contentType() == typeZeroFill) ||
        (_symbol && _symbol->st_shndx == llvm::ELF::SHN_COMMON))
      return ".bss";
    return _sectionName;
  }

  SectionPosition sectionPosition() const override {
    return sectionPositionAny;
  }

  // It isn't clear that __attribute__((used)) is transmitted to the ELF object
  // file.
  DeadStripKind deadStrip() const override { return deadStripNormal; }

  ContentPermissions permissions() const override {
    if (_permissions != permUnknown)
      return _permissions;

    uint64_t flags = _section->sh_flags;

    if (!(flags & llvm::ELF::SHF_ALLOC))
      return _permissions = perm___;

    switch (_section->sh_type) {
    // permRW_L is for sections modified by the runtime
    // loader.
    case llvm::ELF::SHT_REL:
    case llvm::ELF::SHT_RELA:
      return _permissions = permRW_L;

    case llvm::ELF::SHT_DYNAMIC:
    case llvm::ELF::SHT_PROGBITS:
    case llvm::ELF::SHT_NOTE:
      flags &= ~llvm::ELF::SHF_ALLOC;
      flags &= ~llvm::ELF::SHF_GROUP;
      switch (flags) {
      // Code
      case llvm::ELF::SHF_EXECINSTR:
        return _permissions = permR_X;
      case (llvm::ELF::SHF_WRITE|llvm::ELF::SHF_EXECINSTR):
        return _permissions = permRWX;
      // Data
      case llvm::ELF::SHF_WRITE:
        return _permissions = permRW_;
      // Strings
      case llvm::ELF::SHF_MERGE:
      case llvm::ELF::SHF_STRINGS:
        return _permissions = permR__;

      default:
        if (flags & llvm::ELF::SHF_WRITE)
          return _permissions = permRW_;
        return _permissions = permR__;
      }

    case llvm::ELF::SHT_NOBITS:
      return _permissions = permRW_;

    case llvm::ELF::SHT_INIT_ARRAY:
    case llvm::ELF::SHT_FINI_ARRAY:
      return _permissions = permRW_;

    default:
      return _permissions = perm___;
    }
  }

  ArrayRef<uint8_t> rawContent() const override { return _contentData; }

  DefinedAtom::reference_iterator begin() const override {
    uintptr_t index = _referenceStartIndex;
    const void *it = reinterpret_cast<const void*>(index);
    return reference_iterator(*this, it);
  }

  DefinedAtom::reference_iterator end() const override {
    uintptr_t index = _referenceEndIndex;
    const void *it = reinterpret_cast<const void*>(index);
    return reference_iterator(*this, it);
  }

  const Reference *derefIterator(const void *It) const override {
    uintptr_t index = reinterpret_cast<uintptr_t>(It);
    assert(index >= _referenceStartIndex);
    assert(index < _referenceEndIndex);
    return ((_referenceList)[index]);
  }

  void incrementIterator(const void *&It) const override {
    uintptr_t index = reinterpret_cast<uintptr_t>(It);
    ++index;
    It = reinterpret_cast<const void *>(index);
  }

  void addReference(ELFReference<ELFT> *reference) {
    _referenceList.push_back(reference);
    _referenceEndIndex = _referenceList.size();
  }

  virtual void setOrdinal(uint64_t ord) { _ordinal = ord; }

protected:
  /// Returns correct st_value for the symbol depending on the architecture.
  /// For most architectures it's just a regular st_value with no changes.
  virtual uint64_t getSymbolValue(const Elf_Sym *symbol) const {
    return symbol->st_value;
  }

protected:
  const ELFFile<ELFT> &_owningFile;
  StringRef _symbolName;
  StringRef _sectionName;
  const Elf_Sym *_symbol;
  const Elf_Shdr *_section;
  /// \brief Holds the bits that make up the atom.
  ArrayRef<uint8_t> _contentData;

  uint64_t _ordinal;
  unsigned int _referenceStartIndex;
  unsigned int _referenceEndIndex;
  std::vector<ELFReference<ELFT> *> &_referenceList;
  mutable ContentType _contentType;
  mutable ContentPermissions _permissions;
};

/// \brief This atom stores mergeable Strings
template <class ELFT> class ELFMergeAtom : public DefinedAtom {
  typedef llvm::object::Elf_Shdr_Impl<ELFT> Elf_Shdr;

public:
  ELFMergeAtom(const ELFFile<ELFT> &file, StringRef sectionName,
               const Elf_Shdr *section, ArrayRef<uint8_t> contentData,
               uint64_t offset)
      : _owningFile(file), _sectionName(sectionName), _section(section),
        _contentData(contentData), _offset(offset) {
  }

  const ELFFile<ELFT> &file() const override { return _owningFile; }

  StringRef name() const override { return ""; }

  virtual uint64_t section() const { return _section->sh_name; }

  virtual uint64_t offset() const { return _offset; }

  virtual void setOrdinal(uint64_t ord) { _ordinal = ord; }

  uint64_t ordinal() const override { return _ordinal; }

  uint64_t size() const override { return _contentData.size(); }

  Scope scope() const override { return scopeTranslationUnit; }

  Interposable interposable() const override { return interposeNo; }

  Merge merge() const override { return mergeByContent; }

  ContentType contentType() const override { return typeConstant; }

  Alignment alignment() const override {
    return Alignment(llvm::Log2_64(_section->sh_addralign));
  }

  SectionChoice sectionChoice() const override { return sectionCustomRequired; }

  StringRef customSectionName() const override { return _sectionName; }

  SectionPosition sectionPosition() const override {
    return sectionPositionAny;
  }

  DeadStripKind deadStrip() const override { return deadStripNormal; }

  ContentPermissions permissions() const override { return permR__; }

  ArrayRef<uint8_t> rawContent() const override { return _contentData; }

  DefinedAtom::reference_iterator begin() const override {
    uintptr_t index = 0;
    const void *it = reinterpret_cast<const void *>(index);
    return reference_iterator(*this, it);
  }

  DefinedAtom::reference_iterator end() const override {
    uintptr_t index = 0;
    const void *it = reinterpret_cast<const void *>(index);
    return reference_iterator(*this, it);
  }

  const Reference *derefIterator(const void *It) const override {
    return nullptr;
  }

  void incrementIterator(const void *&It) const override {}

private:

  const ELFFile<ELFT> &_owningFile;
  StringRef _sectionName;
  const Elf_Shdr *_section;
  /// \brief Holds the bits that make up the atom.
  ArrayRef<uint8_t> _contentData;
  uint64_t _ordinal;
  uint64_t _offset;
};

template <class ELFT> class ELFCommonAtom : public DefinedAtom {
  typedef llvm::object::Elf_Sym_Impl<ELFT> Elf_Sym;
public:
  ELFCommonAtom(const ELFFile<ELFT> &file,
                StringRef symbolName,
                const Elf_Sym *symbol)
      : _owningFile(file),
        _symbolName(symbolName),
        _symbol(symbol) {}

  const ELFFile<ELFT> &file() const override { return _owningFile; }

  StringRef name() const override { return _symbolName; }

  uint64_t ordinal() const override { return _ordinal; }

  virtual void setOrdinal(uint64_t ord) { _ordinal = ord; }

  uint64_t size() const override { return _symbol->st_size; }

  Scope scope() const override {
    if (_symbol->getVisibility() == llvm::ELF::STV_HIDDEN)
      return scopeLinkageUnit;
    else if (_symbol->getBinding() != llvm::ELF::STB_LOCAL)
      return scopeGlobal;
    else
      return scopeTranslationUnit;
  }

  Interposable interposable() const override { return interposeNo; }

  Merge merge() const override { return mergeAsTentative; }

  ContentType contentType() const override { return typeZeroFill; }

  Alignment alignment() const override {
    return Alignment(llvm::Log2_64(_symbol->st_value));
  }

  SectionChoice sectionChoice() const override { return sectionBasedOnContent; }

  StringRef customSectionName() const override { return ".bss"; }

  SectionPosition sectionPosition() const override {
    return sectionPositionAny;
  }

  DeadStripKind deadStrip() const override { return deadStripNormal; }

  ContentPermissions permissions() const override { return permRW_; }

  ArrayRef<uint8_t> rawContent() const override { return ArrayRef<uint8_t>(); }

  DefinedAtom::reference_iterator begin() const override {
    uintptr_t index = 0;
    const void *it = reinterpret_cast<const void *>(index);
    return reference_iterator(*this, it);
  }

  DefinedAtom::reference_iterator end() const override {
    uintptr_t index = 0;
    const void *it = reinterpret_cast<const void *>(index);
    return reference_iterator(*this, it);
  }
protected:

  virtual ~ELFCommonAtom() {}

  const Reference *derefIterator(const void *iter) const override {
    return nullptr;
  }

  void incrementIterator(const void *&iter) const override {}

  const ELFFile<ELFT> &_owningFile;
  StringRef _symbolName;
  const Elf_Sym *_symbol;
  uint64_t _ordinal;
};

/// \brief An atom from a shared library.
template <class ELFT> class ELFDynamicAtom : public SharedLibraryAtom {
  typedef llvm::object::Elf_Sym_Impl<ELFT> Elf_Sym;

public:
  ELFDynamicAtom(const DynamicFile<ELFT> &file, StringRef symbolName,
                 StringRef loadName, const Elf_Sym *symbol)
      : _owningFile(file), _symbolName(symbolName), _loadName(loadName),
        _symbol(symbol) {
  }

  const DynamicFile<ELFT> &file() const override { return _owningFile; }

  StringRef name() const override { return _symbolName; }

  virtual Scope scope() const {
    if (_symbol->getVisibility() == llvm::ELF::STV_HIDDEN)
      return scopeLinkageUnit;
    else if (_symbol->getBinding() != llvm::ELF::STB_LOCAL)
      return scopeGlobal;
    else
      return scopeTranslationUnit;
  }

  StringRef loadName() const override { return _loadName; }

  bool canBeNullAtRuntime() const override {
    return _symbol->getBinding() == llvm::ELF::STB_WEAK;
  }

  Type type() const override {
    switch (_symbol->getType()) {
    case llvm::ELF::STT_FUNC:
    case llvm::ELF::STT_GNU_IFUNC:
      return Type::Code;
    case llvm::ELF::STT_OBJECT:
      return Type::Data;
    default:
      return Type::Unknown;
    }
  }

  uint64_t size() const override {
    return _symbol->st_size;
  }

private:

  const DynamicFile<ELFT> &_owningFile;
  StringRef _symbolName;
  StringRef _loadName;
  const Elf_Sym *_symbol;
};

class SimpleELFDefinedAtom : public SimpleDefinedAtom {
public:
  SimpleELFDefinedAtom(const File &f) : SimpleDefinedAtom(f) {}

  void addReferenceELF(Reference::KindArch arch, Reference::KindValue kindValue,
                       uint64_t off, const Atom *target,
                       Reference::Addend addend) {
    this->addReference(Reference::KindNamespace::ELF, arch, kindValue, off,
                       target, addend);
  }

  void addReferenceELF_Hexagon(Reference::KindValue relocType, uint64_t off,
                               const Atom *t, Reference::Addend a) {
    this->addReferenceELF(Reference::KindArch::Hexagon, relocType, off, t, a);
  }

  void addReferenceELF_x86_64(Reference::KindValue relocType, uint64_t off,
                              const Atom *t, Reference::Addend a) {
    this->addReferenceELF(Reference::KindArch::x86_64, relocType, off, t, a);
  }

  void addReferenceELF_Mips(Reference::KindValue relocType, uint64_t off,
                            const Atom *t, Reference::Addend a) {
    this->addReferenceELF(Reference::KindArch::Mips, relocType, off, t, a);
  }

  void addReferenceELF_AArch64(Reference::KindValue relocType, uint64_t off,
                               const Atom *t, Reference::Addend a) {
    this->addReferenceELF(Reference::KindArch::AArch64, relocType, off, t, a);
  }

  void addReferenceELF_ARM(Reference::KindValue relocType, uint64_t off,
                           const Atom *t, Reference::Addend a) {
    this->addReferenceELF(Reference::KindArch::ARM, relocType, off, t, a);
  }
};

/// \brief Atom which represents an object for which a COPY relocation will be
///   generated.
class ObjectAtom : public SimpleELFDefinedAtom {
public:
  ObjectAtom(const File &f) : SimpleELFDefinedAtom(f) {}

  Scope scope() const override { return scopeGlobal; }

  SectionChoice sectionChoice() const override { return sectionBasedOnContent; }

  ContentType contentType() const override { return typeZeroFill; }

  uint64_t size() const override { return _size; }

  DynamicExport dynamicExport() const override { return dynamicExportAlways; }

  ContentPermissions permissions() const override { return permRW_; }

  ArrayRef<uint8_t> rawContent() const override { return ArrayRef<uint8_t>(); }

  Alignment alignment() const override {
    // The alignment should be 8 byte aligned
    return Alignment(3);
  }

  StringRef name() const override { return _name; }

  std::string _name;
  uint64_t _size;
};

class GOTAtom : public SimpleELFDefinedAtom {
  StringRef _section;

public:
  GOTAtom(const File &f, StringRef secName)
      : SimpleELFDefinedAtom(f), _section(secName) {}

  Scope scope() const override { return scopeTranslationUnit; }

  SectionChoice sectionChoice() const override { return sectionCustomRequired; }

  StringRef customSectionName() const override { return _section; }

  ContentType contentType() const override { return typeGOT; }

  uint64_t size() const override { return rawContent().size(); }

  ContentPermissions permissions() const override { return permRW_; }

  Alignment alignment() const override {
    // The alignment should be 8 byte aligned
    return Alignment(3);
  }

#ifndef NDEBUG
  StringRef name() const override { return _name; }
  std::string _name;
#else
  StringRef name() const override { return ""; }
#endif
};

class PLTAtom : public SimpleELFDefinedAtom {
  StringRef _section;

public:
  PLTAtom(const File &f, StringRef secName)
      : SimpleELFDefinedAtom(f), _section(secName) {}

  Scope scope() const override { return scopeTranslationUnit; }

  SectionChoice sectionChoice() const override { return sectionCustomRequired; }

  StringRef customSectionName() const override { return _section; }

  ContentType contentType() const override { return typeStub; }

  uint64_t size() const override { return rawContent().size(); }

  ContentPermissions permissions() const override { return permR_X; }

  Alignment alignment() const override {
    return Alignment(4); // 16
  }

#ifndef NDEBUG
  StringRef name() const override { return _name; }
  std::string _name;
#else
  StringRef name() const override { return ""; }
#endif
};

class PLT0Atom : public PLTAtom {

public:
  PLT0Atom(const File &f) : PLTAtom(f, ".plt") {
#ifndef NDEBUG
    _name = ".PLT0";
#endif
  }
};

class GLOBAL_OFFSET_TABLEAtom : public SimpleELFDefinedAtom {
public:
  GLOBAL_OFFSET_TABLEAtom(const File &f) : SimpleELFDefinedAtom(f) {}

  StringRef name() const override { return "_GLOBAL_OFFSET_TABLE_"; }

  Scope scope() const override { return scopeGlobal; }

  SectionChoice sectionChoice() const override { return sectionCustomRequired; }

  StringRef customSectionName() const override { return ".got.plt"; }

  ContentType contentType() const override { return typeGOT; }

  uint64_t size() const override { return 0; }

  ContentPermissions permissions() const override { return permRW_; }

  Alignment alignment() const override {
    // Needs 8 byte alignment
    return Alignment(3);
  }

  ArrayRef<uint8_t> rawContent() const override { return ArrayRef<uint8_t>(); }
};

class DYNAMICAtom : public SimpleELFDefinedAtom {
public:
  DYNAMICAtom(const File &f) : SimpleELFDefinedAtom(f) {}

  StringRef name() const override { return "_DYNAMIC"; }

  Scope scope() const override { return scopeLinkageUnit; }

  Merge merge() const override { return mergeNo; }

  SectionChoice sectionChoice() const override { return sectionCustomRequired; }

  StringRef customSectionName() const override { return ".dynamic"; }

  ContentType contentType() const override { return typeData; }

  uint64_t size() const override { return 0; }

  ContentPermissions permissions() const override { return permRW_; }

  Alignment alignment() const override { return Alignment(0); }

  ArrayRef<uint8_t> rawContent() const override { return ArrayRef<uint8_t>(); }
};

class InitFiniAtom : public SimpleELFDefinedAtom {
  StringRef _section;

public:
  InitFiniAtom(const File &f, StringRef secName)
      : SimpleELFDefinedAtom(f), _section(secName) {}

  Scope scope() const override { return scopeGlobal; }

  SectionChoice sectionChoice() const override { return sectionCustomRequired; }

  StringRef customSectionName() const override { return _section; }

  ContentType contentType() const override { return typeData; }

  uint64_t size() const override { return rawContent().size(); }

  ContentPermissions permissions() const override { return permRW_; }

  Alignment alignment() const override { return size(); }

#ifndef NDEBUG
  StringRef name() const override { return _name; }
  std::string _name;
#else
  StringRef name() const override { return ""; }
#endif
};

} // end namespace elf
} // end namespace lld

#endif
