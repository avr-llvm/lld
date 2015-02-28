//===- lib/ReaderWriter/YAML/ReaderWriterYAML.cpp -------------------------===//
//
//                             The LLVM Linker
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "lld/Core/ArchiveLibraryFile.h"
#include "lld/Core/DefinedAtom.h"
#include "lld/Core/Error.h"
#include "lld/Core/File.h"
#include "lld/Core/LLVM.h"
#include "lld/Core/Reader.h"
#include "lld/Core/Reference.h"
#include "lld/Core/Simple.h"
#include "lld/Core/Writer.h"
#include "lld/ReaderWriter/YamlContext.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/StringMap.h"
#include "llvm/ADT/Twine.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/Errc.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/Format.h"
#include "llvm/Support/MemoryBuffer.h"
#include "llvm/Support/YAMLTraits.h"
#include "llvm/Support/raw_ostream.h"
#include <memory>
#include <string>
#include <system_error>

using llvm::yaml::MappingTraits;
using llvm::yaml::ScalarEnumerationTraits;
using llvm::yaml::ScalarTraits;
using llvm::yaml::IO;
using llvm::yaml::SequenceTraits;
using llvm::yaml::DocumentListTraits;

using namespace lld;

/// The conversion of Atoms to and from YAML uses LLVM's YAML I/O.  This
/// file just defines template specializations on the lld types which control
/// how the mapping is done to and from YAML.

namespace {

/// Used when writing yaml files.
/// In most cases, atoms names are unambiguous, so references can just
/// use the atom name as the target (e.g. target: foo).  But in a few
/// cases that does not work, so ref-names are added.  These are labels
/// used only in yaml.  The labels do not exist in the Atom model.
///
/// One need for ref-names are when atoms have no user supplied name
/// (e.g. c-string literal).  Another case is when two object files with
/// identically named static functions are merged (ld -r) into one object file.
/// In that case referencing the function by name is ambiguous, so a unique
/// ref-name is added.
class RefNameBuilder {
public:
  RefNameBuilder(const lld::File &file)
      : _collisionCount(0), _unnamedCounter(0) {
    // visit all atoms
    for (const lld::DefinedAtom *atom : file.defined()) {
      // Build map of atoms names to detect duplicates
      if (!atom->name().empty())
        buildDuplicateNameMap(*atom);

      // Find references to unnamed atoms and create ref-names for them.
      for (const lld::Reference *ref : *atom) {
        // create refname for any unnamed reference target
        const lld::Atom *target = ref->target();
        if ((target != nullptr) && target->name().empty()) {
          std::string storage;
          llvm::raw_string_ostream buffer(storage);
          buffer << llvm::format("L%03d", _unnamedCounter++);
          StringRef newName = copyString(buffer.str());
          _refNames[target] = newName;
          DEBUG_WITH_TYPE("WriterYAML",
                          llvm::dbgs() << "unnamed atom: creating ref-name: '"
                                       << newName << "' ("
                                       << (const void *)newName.data() << ", "
                                       << newName.size() << ")\n");
        }
      }
    }
    for (const lld::UndefinedAtom *undefAtom : file.undefined()) {
      buildDuplicateNameMap(*undefAtom);
    }
    for (const lld::SharedLibraryAtom *shlibAtom : file.sharedLibrary()) {
      buildDuplicateNameMap(*shlibAtom);
    }
    for (const lld::AbsoluteAtom *absAtom : file.absolute()) {
      if (!absAtom->name().empty())
        buildDuplicateNameMap(*absAtom);
    }
  }

  void buildDuplicateNameMap(const lld::Atom &atom) {
    assert(!atom.name().empty());
    NameToAtom::iterator pos = _nameMap.find(atom.name());
    if (pos != _nameMap.end()) {
      // Found name collision, give each a unique ref-name.
      std::string Storage;
      llvm::raw_string_ostream buffer(Storage);
      buffer << atom.name() << llvm::format(".%03d", ++_collisionCount);
      StringRef newName = copyString(buffer.str());
      _refNames[&atom] = newName;
      DEBUG_WITH_TYPE("WriterYAML",
                      llvm::dbgs() << "name collsion: creating ref-name: '"
                                   << newName << "' ("
                                   << (const void *)newName.data()
                                   << ", " << newName.size() << ")\n");
      const lld::Atom *prevAtom = pos->second;
      AtomToRefName::iterator pos2 = _refNames.find(prevAtom);
      if (pos2 == _refNames.end()) {
        // Only create ref-name for previous if none already created.
        std::string Storage2;
        llvm::raw_string_ostream buffer2(Storage2);
        buffer2 << prevAtom->name() << llvm::format(".%03d", ++_collisionCount);
        StringRef newName2 = copyString(buffer2.str());
        _refNames[prevAtom] = newName2;
        DEBUG_WITH_TYPE("WriterYAML",
                        llvm::dbgs() << "name collsion: creating ref-name: '"
                                     << newName2 << "' ("
                                     << (const void *)newName2.data() << ", "
                                     << newName2.size() << ")\n");
      }
    } else {
      // First time we've seen this name, just add it to map.
      _nameMap[atom.name()] = &atom;
      DEBUG_WITH_TYPE("WriterYAML", llvm::dbgs()
                                        << "atom name seen for first time: '"
                                        << atom.name() << "' ("
                                        << (const void *)atom.name().data()
                                        << ", " << atom.name().size() << ")\n");
    }
  }

  bool hasRefName(const lld::Atom *atom) { return _refNames.count(atom); }

  StringRef refName(const lld::Atom *atom) {
    return _refNames.find(atom)->second;
  }

private:
  typedef llvm::StringMap<const lld::Atom *> NameToAtom;
  typedef llvm::DenseMap<const lld::Atom *, std::string> AtomToRefName;

  // Allocate a new copy of this string in _storage, so the strings
  // can be freed when RefNameBuilder is destroyed.
  StringRef copyString(StringRef str) {
    char *s = _storage.Allocate<char>(str.size());
    memcpy(s, str.data(), str.size());
    return StringRef(s, str.size());
  }

  unsigned int                         _collisionCount;
  unsigned int                         _unnamedCounter;
  NameToAtom                           _nameMap;
  AtomToRefName                        _refNames;
  llvm::BumpPtrAllocator               _storage;
};

/// Used when reading yaml files to find the target of a reference
/// that could be a name or ref-name.
class RefNameResolver {
public:
  RefNameResolver(const lld::File *file, IO &io);

  const lld::Atom *lookup(StringRef name) const {
    NameToAtom::const_iterator pos = _nameMap.find(name);
    if (pos != _nameMap.end())
      return pos->second;
    _io.setError(Twine("no such atom name: ") + name);
    return nullptr;
  }

  /// \brief Lookup a group parent when there is a reference of type
  /// kindGroupChild. If there was no group-parent produce an appropriate
  /// error.
  const lld::Atom *lookupGroupParent(StringRef name) const {
    NameToAtom::const_iterator pos = _groupMap.find(name);
    if (pos != _groupMap.end())
      return pos->second;
    _io.setError(Twine("no such group name: ") + name);
    return nullptr;
  }

private:
  typedef llvm::StringMap<const lld::Atom *> NameToAtom;

  void add(StringRef name, const lld::Atom *atom) {
    if (const lld::DefinedAtom *da = dyn_cast<DefinedAtom>(atom)) {
      if (da->isGroupParent()) {
        if (_groupMap.count(name)) {
          _io.setError(Twine("duplicate group name: ") + name);
        } else {
          _groupMap[name] = atom;
        }
        return;
      }
    }
    if (_nameMap.count(name)) {
      _io.setError(Twine("duplicate atom name: ") + name);
    } else {
      _nameMap[name] = atom;
    }
  }

  IO &_io;
  NameToAtom _nameMap;
  NameToAtom _groupMap;
};

// Used in NormalizedFile to hold the atoms lists.
template <typename T> class AtomList : public lld::File::atom_collection<T> {
public:
  virtual lld::File::atom_iterator<T> begin() const {
    return lld::File::atom_iterator<T>(
        *this,
        _atoms.empty() ? 0 : reinterpret_cast<const void *>(_atoms.data()));
  }
  virtual lld::File::atom_iterator<T> end() const {
    return lld::File::atom_iterator<T>(
        *this, _atoms.empty() ? 0 : reinterpret_cast<const void *>(
                                        _atoms.data() + _atoms.size()));
  }
  virtual const T *deref(const void *it) const {
    return *reinterpret_cast<const T *const *>(it);
  }
  virtual void next(const void *&it) const {
    const T *const *p = reinterpret_cast<const T *const *>(it);
    ++p;
    it = reinterpret_cast<const void *>(p);
  }
  virtual void push_back(const T *element) { _atoms.push_back(element); }
  virtual uint64_t size() const { return _atoms.size(); }
  std::vector<const T *> _atoms;
};

/// Mapping of kind: field in yaml files.
enum FileKinds {
  fileKindObjectAtoms, // atom based object file encoded in yaml
  fileKindArchive,     // static archive library encoded in yaml
  fileKindObjectELF,   // ELF object files encoded in yaml
  fileKindObjectMachO  // mach-o object files encoded in yaml
};

struct ArchMember {
  FileKinds         _kind;
  StringRef         _name;
  const lld::File  *_content;
};

// The content bytes in a DefinedAtom are just uint8_t but we want
// special formatting, so define a strong type.
LLVM_YAML_STRONG_TYPEDEF(uint8_t, ImplicitHex8)

// SharedLibraryAtoms have a bool canBeNull() method which we'd like to be
// more readable than just true/false.
LLVM_YAML_STRONG_TYPEDEF(bool, ShlibCanBeNull)

// lld::Reference::Kind is a tuple of <namespace, arch, value>.
// For yaml, we just want one string that encapsulates the tuple.
struct RefKind {
  Reference::KindNamespace  ns;
  Reference::KindArch       arch;
  Reference::KindValue      value;
};

} // namespace anon

LLVM_YAML_IS_SEQUENCE_VECTOR(ArchMember)
LLVM_YAML_IS_SEQUENCE_VECTOR(const lld::Reference *)
// Always write DefinedAtoms content bytes as a flow sequence.
LLVM_YAML_IS_FLOW_SEQUENCE_VECTOR(ImplicitHex8)
// for compatibility with gcc-4.7 in C++11 mode, add extra namespace
namespace llvm {
namespace yaml {

// This is a custom formatter for RefKind
template <> struct ScalarTraits<RefKind> {
  static void output(const RefKind &kind, void *ctxt, raw_ostream &out) {
    assert(ctxt != nullptr);
    YamlContext *info = reinterpret_cast<YamlContext *>(ctxt);
    assert(info->_registry);
    StringRef str;
    if (info->_registry->referenceKindToString(kind.ns, kind.arch, kind.value,
                                               str))
      out << str;
    else
      out << (int)(kind.ns) << "-" << (int)(kind.arch) << "-" << kind.value;
  }

  static StringRef input(StringRef scalar, void *ctxt, RefKind &kind) {
    assert(ctxt != nullptr);
    YamlContext *info = reinterpret_cast<YamlContext *>(ctxt);
    assert(info->_registry);
    if (info->_registry->referenceKindFromString(scalar, kind.ns, kind.arch,
                                                 kind.value))
      return StringRef();
    return StringRef("unknown reference kind");
  }

  static bool mustQuote(StringRef) { return false; }
};

template <> struct ScalarEnumerationTraits<lld::File::Kind> {
  static void enumeration(IO &io, lld::File::Kind &value) {
    io.enumCase(value, "object",         lld::File::kindObject);
    io.enumCase(value, "shared-library", lld::File::kindSharedLibrary);
    io.enumCase(value, "static-library", lld::File::kindArchiveLibrary);
  }
};

template <> struct ScalarEnumerationTraits<lld::Atom::Scope> {
  static void enumeration(IO &io, lld::Atom::Scope &value) {
    io.enumCase(value, "global", lld::Atom::scopeGlobal);
    io.enumCase(value, "hidden", lld::Atom::scopeLinkageUnit);
    io.enumCase(value, "static", lld::Atom::scopeTranslationUnit);
  }
};

template <> struct ScalarEnumerationTraits<lld::DefinedAtom::SectionChoice> {
  static void enumeration(IO &io, lld::DefinedAtom::SectionChoice &value) {
    io.enumCase(value, "content", lld::DefinedAtom::sectionBasedOnContent);
    io.enumCase(value, "custom",  lld::DefinedAtom::sectionCustomPreferred);
    io.enumCase(value, "custom-required",
                                 lld::DefinedAtom::sectionCustomRequired);
  }
};

template <> struct ScalarEnumerationTraits<lld::DefinedAtom::SectionPosition> {
  static void enumeration(IO &io, lld::DefinedAtom::SectionPosition &value) {
    io.enumCase(value, "start", lld::DefinedAtom::sectionPositionStart);
    io.enumCase(value, "early", lld::DefinedAtom::sectionPositionEarly);
    io.enumCase(value, "any",   lld::DefinedAtom::sectionPositionAny);
    io.enumCase(value, "end",   lld::DefinedAtom::sectionPositionEnd);
  }
};

template <> struct ScalarEnumerationTraits<lld::DefinedAtom::Interposable> {
  static void enumeration(IO &io, lld::DefinedAtom::Interposable &value) {
    io.enumCase(value, "no",           DefinedAtom::interposeNo);
    io.enumCase(value, "yes",          DefinedAtom::interposeYes);
    io.enumCase(value, "yes-and-weak", DefinedAtom::interposeYesAndRuntimeWeak);
  }
};

template <> struct ScalarEnumerationTraits<lld::DefinedAtom::Merge> {
  static void enumeration(IO &io, lld::DefinedAtom::Merge &value) {
    io.enumCase(value, "no",           lld::DefinedAtom::mergeNo);
    io.enumCase(value, "as-tentative", lld::DefinedAtom::mergeAsTentative);
    io.enumCase(value, "as-weak",      lld::DefinedAtom::mergeAsWeak);
    io.enumCase(value, "as-addressed-weak",
                                   lld::DefinedAtom::mergeAsWeakAndAddressUsed);
    io.enumCase(value, "by-content",   lld::DefinedAtom::mergeByContent);
    io.enumCase(value, "same-name-and-size",
                lld::DefinedAtom::mergeSameNameAndSize);
    io.enumCase(value, "largest", lld::DefinedAtom::mergeByLargestSection);
  }
};

template <> struct ScalarEnumerationTraits<lld::DefinedAtom::DeadStripKind> {
  static void enumeration(IO &io, lld::DefinedAtom::DeadStripKind &value) {
    io.enumCase(value, "normal", lld::DefinedAtom::deadStripNormal);
    io.enumCase(value, "never",  lld::DefinedAtom::deadStripNever);
    io.enumCase(value, "always", lld::DefinedAtom::deadStripAlways);
  }
};

template <> struct ScalarEnumerationTraits<lld::DefinedAtom::DynamicExport> {
  static void enumeration(IO &io, lld::DefinedAtom::DynamicExport &value) {
    io.enumCase(value, "normal", lld::DefinedAtom::dynamicExportNormal);
    io.enumCase(value, "always", lld::DefinedAtom::dynamicExportAlways);
  }
};

template <> struct ScalarEnumerationTraits<lld::DefinedAtom::CodeModel> {
  static void enumeration(IO &io, lld::DefinedAtom::CodeModel &value) {
    io.enumCase(value, "none", lld::DefinedAtom::codeNA);
    io.enumCase(value, "mips-pic", lld::DefinedAtom::codeMipsPIC);
    io.enumCase(value, "mips-micro", lld::DefinedAtom::codeMipsMicro);
    io.enumCase(value, "mips-micro-pic", lld::DefinedAtom::codeMipsMicroPIC);
    io.enumCase(value, "mips-16", lld::DefinedAtom::codeMips16);
    io.enumCase(value, "arm-thumb", lld::DefinedAtom::codeARMThumb);
  }
};

template <>
struct ScalarEnumerationTraits<lld::DefinedAtom::ContentPermissions> {
  static void enumeration(IO &io, lld::DefinedAtom::ContentPermissions &value) {
    io.enumCase(value, "---",     lld::DefinedAtom::perm___);
    io.enumCase(value, "r--",     lld::DefinedAtom::permR__);
    io.enumCase(value, "r-x",     lld::DefinedAtom::permR_X);
    io.enumCase(value, "rw-",     lld::DefinedAtom::permRW_);
    io.enumCase(value, "rwx",     lld::DefinedAtom::permRWX);
    io.enumCase(value, "rw-l",    lld::DefinedAtom::permRW_L);
    io.enumCase(value, "unknown", lld::DefinedAtom::permUnknown);
  }
};

template <> struct ScalarEnumerationTraits<lld::DefinedAtom::ContentType> {
  static void enumeration(IO &io, lld::DefinedAtom::ContentType &value) {
    io.enumCase(value, "unknown",         DefinedAtom::typeUnknown);
    io.enumCase(value, "code",            DefinedAtom::typeCode);
    io.enumCase(value, "stub",            DefinedAtom::typeStub);
    io.enumCase(value, "constant",        DefinedAtom::typeConstant);
    io.enumCase(value, "data",            DefinedAtom::typeData);
    io.enumCase(value, "quick-data",      DefinedAtom::typeDataFast);
    io.enumCase(value, "zero-fill",       DefinedAtom::typeZeroFill);
    io.enumCase(value, "zero-fill-quick", DefinedAtom::typeZeroFillFast);
    io.enumCase(value, "const-data",      DefinedAtom::typeConstData);
    io.enumCase(value, "got",             DefinedAtom::typeGOT);
    io.enumCase(value, "resolver",        DefinedAtom::typeResolver);
    io.enumCase(value, "branch-island",   DefinedAtom::typeBranchIsland);
    io.enumCase(value, "branch-shim",     DefinedAtom::typeBranchShim);
    io.enumCase(value, "stub-helper",     DefinedAtom::typeStubHelper);
    io.enumCase(value, "c-string",        DefinedAtom::typeCString);
    io.enumCase(value, "utf16-string",    DefinedAtom::typeUTF16String);
    io.enumCase(value, "unwind-cfi",      DefinedAtom::typeCFI);
    io.enumCase(value, "unwind-lsda",     DefinedAtom::typeLSDA);
    io.enumCase(value, "const-4-byte",    DefinedAtom::typeLiteral4);
    io.enumCase(value, "const-8-byte",    DefinedAtom::typeLiteral8);
    io.enumCase(value, "const-16-byte",   DefinedAtom::typeLiteral16);
    io.enumCase(value, "lazy-pointer",    DefinedAtom::typeLazyPointer);
    io.enumCase(value, "lazy-dylib-pointer",
                                          DefinedAtom::typeLazyDylibPointer);
    io.enumCase(value, "cfstring",        DefinedAtom::typeCFString);
    io.enumCase(value, "initializer-pointer",
                                          DefinedAtom::typeInitializerPtr);
    io.enumCase(value, "terminator-pointer",
                                          DefinedAtom::typeTerminatorPtr);
    io.enumCase(value, "c-string-pointer",DefinedAtom::typeCStringPtr);
    io.enumCase(value, "objc-class-pointer",
                                          DefinedAtom::typeObjCClassPtr);
    io.enumCase(value, "objc-category-list",
                                          DefinedAtom::typeObjC2CategoryList);
    io.enumCase(value, "objc-class1",     DefinedAtom::typeObjC1Class);
    io.enumCase(value, "dtraceDOF",       DefinedAtom::typeDTraceDOF);
    io.enumCase(value, "interposing-tuples",
                                          DefinedAtom::typeInterposingTuples);
    io.enumCase(value, "lto-temp",        DefinedAtom::typeTempLTO);
    io.enumCase(value, "compact-unwind",  DefinedAtom::typeCompactUnwindInfo);
    io.enumCase(value, "unwind-info",     DefinedAtom::typeProcessedUnwindInfo);
    io.enumCase(value, "tlv-thunk",       DefinedAtom::typeThunkTLV);
    io.enumCase(value, "tlv-data",        DefinedAtom::typeTLVInitialData);
    io.enumCase(value, "tlv-zero-fill",   DefinedAtom::typeTLVInitialZeroFill);
    io.enumCase(value, "tlv-initializer-ptr",
                                          DefinedAtom::typeTLVInitializerPtr);
    io.enumCase(value, "mach_header",     DefinedAtom::typeMachHeader);
    io.enumCase(value, "thread-data",     DefinedAtom::typeThreadData);
    io.enumCase(value, "thread-zero-fill",DefinedAtom::typeThreadZeroFill);
    io.enumCase(value, "ro-note",         DefinedAtom::typeRONote);
    io.enumCase(value, "rw-note",         DefinedAtom::typeRWNote);
    io.enumCase(value, "no-alloc",        DefinedAtom::typeNoAlloc);
    io.enumCase(value, "group-comdat", DefinedAtom::typeGroupComdat);
    io.enumCase(value, "gnu-linkonce", DefinedAtom::typeGnuLinkOnce);
  }
};

template <> struct ScalarEnumerationTraits<lld::UndefinedAtom::CanBeNull> {
  static void enumeration(IO &io, lld::UndefinedAtom::CanBeNull &value) {
    io.enumCase(value, "never",       lld::UndefinedAtom::canBeNullNever);
    io.enumCase(value, "at-runtime",  lld::UndefinedAtom::canBeNullAtRuntime);
    io.enumCase(value, "at-buildtime",lld::UndefinedAtom::canBeNullAtBuildtime);
  }
};

template <> struct ScalarEnumerationTraits<ShlibCanBeNull> {
  static void enumeration(IO &io, ShlibCanBeNull &value) {
    io.enumCase(value, "never",      false);
    io.enumCase(value, "at-runtime", true);
  }
};

template <>
struct ScalarEnumerationTraits<lld::SharedLibraryAtom::Type> {
  static void enumeration(IO &io, lld::SharedLibraryAtom::Type &value) {
    io.enumCase(value, "code",    lld::SharedLibraryAtom::Type::Code);
    io.enumCase(value, "data",    lld::SharedLibraryAtom::Type::Data);
    io.enumCase(value, "unknown", lld::SharedLibraryAtom::Type::Unknown);
  }
};

/// This is a custom formatter for lld::DefinedAtom::Alignment.  Values look
/// like:
///     2^3          # 8-byte aligned
///     7 mod 2^4    # 16-byte aligned plus 7 bytes
template <> struct ScalarTraits<lld::DefinedAtom::Alignment> {
  static void output(const lld::DefinedAtom::Alignment &value, void *ctxt,
                     raw_ostream &out) {
    if (value.modulus == 0) {
      out << llvm::format("2^%d", value.powerOf2);
    } else {
      out << llvm::format("%d mod 2^%d", value.modulus, value.powerOf2);
    }
  }

  static StringRef input(StringRef scalar, void *ctxt,
                         lld::DefinedAtom::Alignment &value) {
    value.modulus = 0;
    size_t modStart = scalar.find("mod");
    if (modStart != StringRef::npos) {
      StringRef modStr = scalar.slice(0, modStart);
      modStr = modStr.rtrim();
      unsigned int modulus;
      if (modStr.getAsInteger(0, modulus)) {
        return "malformed alignment modulus";
      }
      value.modulus = modulus;
      scalar = scalar.drop_front(modStart + 3);
      scalar = scalar.ltrim();
    }
    if (!scalar.startswith("2^")) {
      return "malformed alignment";
    }
    StringRef powerStr = scalar.drop_front(2);
    unsigned int power;
    if (powerStr.getAsInteger(0, power)) {
      return "malformed alignment power";
    }
    value.powerOf2 = power;
    if (value.modulus > (1 << value.powerOf2)) {
      return "malformed alignment, modulus too large for power";
    }
    return StringRef(); // returning empty string means success
  }

  static bool mustQuote(StringRef) { return false; }
};

template <> struct ScalarEnumerationTraits<FileKinds> {
  static void enumeration(IO &io, FileKinds &value) {
    io.enumCase(value, "object",        fileKindObjectAtoms);
    io.enumCase(value, "archive",       fileKindArchive);
    io.enumCase(value, "object-elf",    fileKindObjectELF);
    io.enumCase(value, "object-mach-o", fileKindObjectMachO);
  }
};

template <> struct MappingTraits<ArchMember> {
  static void mapping(IO &io, ArchMember &member) {
    io.mapOptional("kind",    member._kind, fileKindObjectAtoms);
    io.mapOptional("name",    member._name);
    io.mapRequired("content", member._content);
  }
};

// Declare that an AtomList is a yaml sequence.
template <typename T> struct SequenceTraits<AtomList<T> > {
  static size_t size(IO &io, AtomList<T> &seq) { return seq._atoms.size(); }
  static const T *&element(IO &io, AtomList<T> &seq, size_t index) {
    if (index >= seq._atoms.size())
      seq._atoms.resize(index + 1);
    return seq._atoms[index];
  }
};

// Used to allow DefinedAtom content bytes to be a flow sequence of
// two-digit hex numbers without the leading 0x (e.g. FF, 04, 0A)
template <> struct ScalarTraits<ImplicitHex8> {
  static void output(const ImplicitHex8 &val, void *, raw_ostream &out) {
    uint8_t num = val;
    out << llvm::format("%02X", num);
  }

  static StringRef input(StringRef str, void *, ImplicitHex8 &val) {
    unsigned long long n;
    if (getAsUnsignedInteger(str, 16, n))
      return "invalid two-digit-hex number";
    if (n > 0xFF)
      return "out of range two-digit-hex number";
    val = n;
    return StringRef(); // returning empty string means success
  }

  static bool mustQuote(StringRef) { return false; }
};

// YAML conversion for std::vector<const lld::File*>
template <> struct DocumentListTraits<std::vector<const lld::File *> > {
  static size_t size(IO &io, std::vector<const lld::File *> &seq) {
    return seq.size();
  }
  static const lld::File *&element(IO &io, std::vector<const lld::File *> &seq,
                                   size_t index) {
    if (index >= seq.size())
      seq.resize(index + 1);
    return seq[index];
  }
};

// YAML conversion for const lld::File*
template <> struct MappingTraits<const lld::File *> {

  class NormArchiveFile : public lld::ArchiveLibraryFile {
  public:
    NormArchiveFile(IO &io) : ArchiveLibraryFile(""), _path() {}
    NormArchiveFile(IO &io, const lld::File *file)
        : ArchiveLibraryFile(file->path()), _path(file->path()) {
      // If we want to support writing archives, this constructor would
      // need to populate _members.
    }

    const lld::File *denormalize(IO &io) { return this; }

    const atom_collection<lld::DefinedAtom> &defined() const override {
      return _noDefinedAtoms;
    }
    const atom_collection<lld::UndefinedAtom> &undefined() const override {
      return _noUndefinedAtoms;
    }
    virtual const atom_collection<lld::SharedLibraryAtom> &
    sharedLibrary() const override {
      return _noSharedLibraryAtoms;
    }
    const atom_collection<lld::AbsoluteAtom> &absolute() const override {
      return _noAbsoluteAtoms;
    }
    const File *find(StringRef name, bool dataSymbolOnly) const override {
      for (const ArchMember &member : _members) {
        for (const lld::DefinedAtom *atom : member._content->defined()) {
          if (name == atom->name()) {
            if (!dataSymbolOnly)
              return member._content;
            switch (atom->contentType()) {
            case lld::DefinedAtom::typeData:
            case lld::DefinedAtom::typeZeroFill:
              return member._content;
            default:
              break;
            }
          }
        }
      }
      return nullptr;
    }

    virtual std::error_code
    parseAllMembers(std::vector<std::unique_ptr<File>> &result) override {
      return std::error_code();
    }

    StringRef               _path;
    std::vector<ArchMember> _members;
  };

  class NormalizedFile : public lld::File {
  public:
    NormalizedFile(IO &io) : File("", kindObject), _io(io), _rnb(nullptr) {}
    NormalizedFile(IO &io, const lld::File *file)
        : File(file->path(), kindObject), _io(io),
          _rnb(new RefNameBuilder(*file)), _path(file->path()) {
      for (const lld::DefinedAtom *a : file->defined())
        _definedAtoms.push_back(a);
      for (const lld::UndefinedAtom *a : file->undefined())
        _undefinedAtoms.push_back(a);
      for (const lld::SharedLibraryAtom *a : file->sharedLibrary())
        _sharedLibraryAtoms.push_back(a);
      for (const lld::AbsoluteAtom *a : file->absolute())
        _absoluteAtoms.push_back(a);
    }
    const lld::File *denormalize(IO &io);

    const atom_collection<lld::DefinedAtom> &defined() const override {
      return _definedAtoms;
    }
    const atom_collection<lld::UndefinedAtom> &undefined() const override {
      return _undefinedAtoms;
    }
    virtual const atom_collection<lld::SharedLibraryAtom> &
    sharedLibrary() const override {
      return _sharedLibraryAtoms;
    }
    const atom_collection<lld::AbsoluteAtom> &absolute() const override {
      return _absoluteAtoms;
    }

    // Allocate a new copy of this string in _storage, so the strings
    // can be freed when File is destroyed.
    StringRef copyString(StringRef str) {
      char *s = _storage.Allocate<char>(str.size());
      memcpy(s, str.data(), str.size());
      return StringRef(s, str.size());
    }

    IO                                  &_io;
    std::unique_ptr<RefNameBuilder> _rnb;
    StringRef                            _path;
    AtomList<lld::DefinedAtom>           _definedAtoms;
    AtomList<lld::UndefinedAtom>         _undefinedAtoms;
    AtomList<lld::SharedLibraryAtom>     _sharedLibraryAtoms;
    AtomList<lld::AbsoluteAtom>          _absoluteAtoms;
    llvm::BumpPtrAllocator               _storage;
  };

  static void mapping(IO &io, const lld::File *&file) {
    YamlContext *info = reinterpret_cast<YamlContext *>(io.getContext());
    assert(info != nullptr);
    // Let any register tag handler process this.
    if (info->_registry && info->_registry->handleTaggedDoc(io, file))
      return;
    // If no registered handler claims this tag and there is no tag,
    // grandfather in as "!native".
    if (io.mapTag("!native", true) || io.mapTag("tag:yaml.org,2002:map"))
      mappingAtoms(io, file);
  }

  static void mappingAtoms(IO &io, const lld::File *&file) {
    MappingNormalizationHeap<NormalizedFile, const lld::File *> keys(io, file);
    YamlContext *info = reinterpret_cast<YamlContext *>(io.getContext());
    assert(info != nullptr);
    info->_file = keys.operator->();

    io.mapOptional("path",                 keys->_path);
    io.mapOptional("defined-atoms",        keys->_definedAtoms);
    io.mapOptional("undefined-atoms",      keys->_undefinedAtoms);
    io.mapOptional("shared-library-atoms", keys->_sharedLibraryAtoms);
    io.mapOptional("absolute-atoms",       keys->_absoluteAtoms);
  }

  static void mappingArchive(IO &io, const lld::File *&file) {
    MappingNormalizationHeap<NormArchiveFile, const lld::File *> keys(io, file);

    io.mapOptional("path",    keys->_path);
    io.mapOptional("members", keys->_members);
  }
};

// YAML conversion for const lld::Reference*
template <> struct MappingTraits<const lld::Reference *> {

  class NormalizedReference : public lld::Reference {
  public:
    NormalizedReference(IO &io)
        : lld::Reference(lld::Reference::KindNamespace::all,
                         lld::Reference::KindArch::all, 0),
          _target(nullptr), _targetName(), _offset(0), _addend(0) {}

    NormalizedReference(IO &io, const lld::Reference *ref)
        : lld::Reference(ref->kindNamespace(), ref->kindArch(),
                         ref->kindValue()),
          _target(nullptr), _targetName(targetName(io, ref)),
          _offset(ref->offsetInAtom()), _addend(ref->addend()) {
      _mappedKind.ns = ref->kindNamespace();
      _mappedKind.arch = ref->kindArch();
      _mappedKind.value = ref->kindValue();
    }

    const lld::Reference *denormalize(IO &io) {
      YamlContext *info = reinterpret_cast<YamlContext *>(io.getContext());
      assert(info != nullptr);
      typedef MappingTraits<const lld::File *>::NormalizedFile NormalizedFile;
      NormalizedFile *f = reinterpret_cast<NormalizedFile *>(info->_file);
      if (!_targetName.empty())
        _targetName = f->copyString(_targetName);
      DEBUG_WITH_TYPE("WriterYAML", llvm::dbgs()
                                        << "created Reference to name: '"
                                        << _targetName << "' ("
                                        << (const void *)_targetName.data()
                                        << ", " << _targetName.size() << ")\n");
      setKindNamespace(_mappedKind.ns);
      setKindArch(_mappedKind.arch);
      setKindValue(_mappedKind.value);
      return this;
    }
    void bind(const RefNameResolver &);
    static StringRef targetName(IO &io, const lld::Reference *ref);

    uint64_t offsetInAtom() const override { return _offset; }
    const lld::Atom *target() const override { return _target; }
    Addend addend() const override { return _addend; }
    void setAddend(Addend a) override { _addend = a; }
    void setTarget(const lld::Atom *a) override { _target = a; }

    const lld::Atom *_target;
    StringRef        _targetName;
    uint32_t         _offset;
    Addend           _addend;
    RefKind          _mappedKind;
  };

  static void mapping(IO &io, const lld::Reference *&ref) {
    MappingNormalizationHeap<NormalizedReference, const lld::Reference *> keys(
        io, ref);

    io.mapRequired("kind",   keys->_mappedKind);
    io.mapOptional("offset", keys->_offset);
    io.mapOptional("target", keys->_targetName);
    io.mapOptional("addend", keys->_addend, (lld::Reference::Addend)0);
  }
};

// YAML conversion for const lld::DefinedAtom*
template <> struct MappingTraits<const lld::DefinedAtom *> {

  class NormalizedAtom : public lld::DefinedAtom {
  public:
    NormalizedAtom(IO &io)
        : _file(fileFromContext(io)), _name(), _refName(), _contentType(),
          _alignment(0), _content(), _references(), _isGroupChild(false) {
      static uint32_t ordinalCounter = 1;
      _ordinal = ordinalCounter++;
    }
    NormalizedAtom(IO &io, const lld::DefinedAtom *atom)
        : _file(fileFromContext(io)), _name(atom->name()), _refName(),
          _scope(atom->scope()), _interpose(atom->interposable()),
          _merge(atom->merge()), _contentType(atom->contentType()),
          _alignment(atom->alignment()), _sectionChoice(atom->sectionChoice()),
          _sectionPosition(atom->sectionPosition()),
          _deadStrip(atom->deadStrip()), _dynamicExport(atom->dynamicExport()),
          _codeModel(atom->codeModel()),
          _permissions(atom->permissions()), _size(atom->size()),
          _sectionName(atom->customSectionName()) {
      for (const lld::Reference *r : *atom)
        _references.push_back(r);
      if (!atom->occupiesDiskSpace())
        return;
      ArrayRef<uint8_t> cont = atom->rawContent();
      _content.reserve(cont.size());
      for (uint8_t x : cont)
        _content.push_back(x);
    }
    const lld::DefinedAtom *denormalize(IO &io) {
      YamlContext *info = reinterpret_cast<YamlContext *>(io.getContext());
      assert(info != nullptr);
      typedef MappingTraits<const lld::File *>::NormalizedFile NormalizedFile;
      NormalizedFile *f = reinterpret_cast<NormalizedFile *>(info->_file);
      if (!_name.empty())
        _name = f->copyString(_name);
      if (!_refName.empty())
        _refName = f->copyString(_refName);
      if (!_sectionName.empty())
        _sectionName = f->copyString(_sectionName);
      DEBUG_WITH_TYPE("WriterYAML",
                      llvm::dbgs() << "created DefinedAtom named: '" << _name
                                   << "' (" << (const void *)_name.data()
                                   << ", " << _name.size() << ")\n");
      return this;
    }
    void bind(const RefNameResolver &);
    // Extract current File object from YAML I/O parsing context
    const lld::File &fileFromContext(IO &io) {
      YamlContext *info = reinterpret_cast<YamlContext *>(io.getContext());
      assert(info != nullptr);
      assert(info->_file != nullptr);
      return *info->_file;
    }

    const lld::File &file() const override { return _file; }
    StringRef name() const override { return _name; }
    uint64_t size() const override { return _size; }
    Scope scope() const override { return _scope; }
    Interposable interposable() const override { return _interpose; }
    Merge merge() const override { return _merge; }
    ContentType contentType() const override { return _contentType; }
    Alignment alignment() const override { return _alignment; }
    SectionChoice sectionChoice() const override { return _sectionChoice; }
    StringRef customSectionName() const override { return _sectionName; }
    SectionPosition sectionPosition() const override { return _sectionPosition; }
    DeadStripKind deadStrip() const override { return _deadStrip; }
    DynamicExport dynamicExport() const override { return _dynamicExport; }
    CodeModel codeModel() const override { return _codeModel; }
    ContentPermissions permissions() const override { return _permissions; }
    void setGroupChild(bool val) { _isGroupChild = val; }
    bool isGroupChild() const { return _isGroupChild; }
    ArrayRef<uint8_t> rawContent() const override {
      if (!occupiesDiskSpace())
        return ArrayRef<uint8_t>();
      return ArrayRef<uint8_t>(
          reinterpret_cast<const uint8_t *>(_content.data()), _content.size());
    }

    uint64_t ordinal() const override { return _ordinal; }

    reference_iterator begin() const override {
      uintptr_t index = 0;
      const void *it = reinterpret_cast<const void *>(index);
      return reference_iterator(*this, it);
    }
    reference_iterator end() const override {
      uintptr_t index = _references.size();
      const void *it = reinterpret_cast<const void *>(index);
      return reference_iterator(*this, it);
    }
    const lld::Reference *derefIterator(const void *it) const override {
      uintptr_t index = reinterpret_cast<uintptr_t>(it);
      assert(index < _references.size());
      return _references[index];
    }
    void incrementIterator(const void *&it) const override {
      uintptr_t index = reinterpret_cast<uintptr_t>(it);
      ++index;
      it = reinterpret_cast<const void *>(index);
    }

    const lld::File                    &_file;
    StringRef                           _name;
    StringRef                           _refName;
    Scope                               _scope;
    Interposable                        _interpose;
    Merge                               _merge;
    ContentType                         _contentType;
    Alignment                           _alignment;
    SectionChoice                       _sectionChoice;
    SectionPosition                     _sectionPosition;
    DeadStripKind                       _deadStrip;
    DynamicExport                       _dynamicExport;
    CodeModel                           _codeModel;
    ContentPermissions                  _permissions;
    uint32_t                            _ordinal;
    std::vector<ImplicitHex8>           _content;
    uint64_t                            _size;
    StringRef                           _sectionName;
    std::vector<const lld::Reference *> _references;
    bool _isGroupChild;
  };

  static void mapping(IO &io, const lld::DefinedAtom *&atom) {
    MappingNormalizationHeap<NormalizedAtom, const lld::DefinedAtom *> keys(
        io, atom);
    if (io.outputting()) {
      // If writing YAML, check if atom needs a ref-name.
      typedef MappingTraits<const lld::File *>::NormalizedFile NormalizedFile;
      YamlContext *info = reinterpret_cast<YamlContext *>(io.getContext());
      assert(info != nullptr);
      NormalizedFile *f = reinterpret_cast<NormalizedFile *>(info->_file);
      assert(f);
      assert(f->_rnb);
      if (f->_rnb->hasRefName(atom)) {
        keys->_refName = f->_rnb->refName(atom);
      }
    }

    io.mapOptional("name",             keys->_name,    StringRef());
    io.mapOptional("ref-name",         keys->_refName, StringRef());
    io.mapOptional("scope",            keys->_scope,
                                         DefinedAtom::scopeTranslationUnit);
    io.mapOptional("type",             keys->_contentType,
                                         DefinedAtom::typeCode);
    io.mapOptional("content",          keys->_content);
    io.mapOptional("size",             keys->_size, (uint64_t)keys->_content.size());
    io.mapOptional("interposable",     keys->_interpose,
                                         DefinedAtom::interposeNo);
    io.mapOptional("merge",            keys->_merge, DefinedAtom::mergeNo);
    io.mapOptional("alignment",        keys->_alignment,
                                         DefinedAtom::Alignment(0));
    io.mapOptional("section-choice",   keys->_sectionChoice,
                                         DefinedAtom::sectionBasedOnContent);
    io.mapOptional("section-name",     keys->_sectionName, StringRef());
    io.mapOptional("section-position", keys->_sectionPosition,
                                         DefinedAtom::sectionPositionAny);
    io.mapOptional("dead-strip",       keys->_deadStrip,
                                         DefinedAtom::deadStripNormal);
    io.mapOptional("dynamic-export",   keys->_dynamicExport,
                                         DefinedAtom::dynamicExportNormal);
    io.mapOptional("code-model",       keys->_codeModel, DefinedAtom::codeNA);
    // default permissions based on content type
    io.mapOptional("permissions",      keys->_permissions,
                                         DefinedAtom::permissions(
                                                          keys->_contentType));
    io.mapOptional("references",       keys->_references);
  }
};

// YAML conversion for const lld::UndefinedAtom*
template <> struct MappingTraits<const lld::UndefinedAtom *> {

  class NormalizedAtom : public lld::UndefinedAtom {
  public:
    NormalizedAtom(IO &io)
        : _file(fileFromContext(io)), _name(), _canBeNull(canBeNullNever),
          _fallback(nullptr) {}

    NormalizedAtom(IO &io, const lld::UndefinedAtom *atom)
        : _file(fileFromContext(io)), _name(atom->name()),
          _canBeNull(atom->canBeNull()), _fallback(atom->fallback()) {}

    const lld::UndefinedAtom *denormalize(IO &io) {
      YamlContext *info = reinterpret_cast<YamlContext *>(io.getContext());
      assert(info != nullptr);
      typedef MappingTraits<const lld::File *>::NormalizedFile NormalizedFile;
      NormalizedFile *f = reinterpret_cast<NormalizedFile *>(info->_file);
      if (!_name.empty())
        _name = f->copyString(_name);

      DEBUG_WITH_TYPE("WriterYAML",
                      llvm::dbgs() << "created UndefinedAtom named: '" << _name
                      << "' (" << (const void *)_name.data() << ", "
                      << _name.size() << ")\n");
      return this;
    }

    // Extract current File object from YAML I/O parsing context
    const lld::File &fileFromContext(IO &io) {
      YamlContext *info = reinterpret_cast<YamlContext *>(io.getContext());
      assert(info != nullptr);
      assert(info->_file != nullptr);
      return *info->_file;
    }

    const lld::File &file() const override { return _file; }
    StringRef name() const override { return _name; }
    CanBeNull canBeNull() const override { return _canBeNull; }
    const UndefinedAtom *fallback() const override { return _fallback; }

    const lld::File     &_file;
    StringRef            _name;
    CanBeNull            _canBeNull;
    const UndefinedAtom *_fallback;
  };

  static void mapping(IO &io, const lld::UndefinedAtom *&atom) {
    MappingNormalizationHeap<NormalizedAtom, const lld::UndefinedAtom *> keys(
        io, atom);

    io.mapRequired("name",        keys->_name);
    io.mapOptional("can-be-null", keys->_canBeNull,
                                  lld::UndefinedAtom::canBeNullNever);
    io.mapOptional("fallback",    keys->_fallback,
                                  (const lld::UndefinedAtom *)nullptr);
  }
};

// YAML conversion for const lld::SharedLibraryAtom*
template <> struct MappingTraits<const lld::SharedLibraryAtom *> {

  class NormalizedAtom : public lld::SharedLibraryAtom {
  public:
    NormalizedAtom(IO &io)
        : _file(fileFromContext(io)), _name(), _loadName(), _canBeNull(false),
          _type(Type::Unknown), _size(0) {}
    NormalizedAtom(IO &io, const lld::SharedLibraryAtom *atom)
        : _file(fileFromContext(io)), _name(atom->name()),
          _loadName(atom->loadName()), _canBeNull(atom->canBeNullAtRuntime()),
          _type(atom->type()), _size(atom->size()) {}

    const lld::SharedLibraryAtom *denormalize(IO &io) {
      YamlContext *info = reinterpret_cast<YamlContext *>(io.getContext());
      assert(info != nullptr);
      typedef MappingTraits<const lld::File *>::NormalizedFile NormalizedFile;
      NormalizedFile *f = reinterpret_cast<NormalizedFile *>(info->_file);
      if (!_name.empty())
        _name = f->copyString(_name);
      if (!_loadName.empty())
        _loadName = f->copyString(_loadName);

      DEBUG_WITH_TYPE("WriterYAML",
                      llvm::dbgs() << "created SharedLibraryAtom named: '"
                                   << _name << "' ("
                                   << (const void *)_name.data()
                                   << ", " << _name.size() << ")\n");
      return this;
    }

    // Extract current File object from YAML I/O parsing context
    const lld::File &fileFromContext(IO &io) {
      YamlContext *info = reinterpret_cast<YamlContext *>(io.getContext());
      assert(info != nullptr);
      assert(info->_file != nullptr);
      return *info->_file;
    }

    const lld::File &file() const override { return _file; }
    StringRef name() const override { return _name; }
    StringRef loadName() const override { return _loadName; }
    bool canBeNullAtRuntime() const override { return _canBeNull; }
    Type type() const override { return _type; }
    uint64_t size() const override { return _size; }

    const lld::File &_file;
    StringRef        _name;
    StringRef        _loadName;
    ShlibCanBeNull   _canBeNull;
    Type             _type;
    uint64_t         _size;
  };

  static void mapping(IO &io, const lld::SharedLibraryAtom *&atom) {

    MappingNormalizationHeap<NormalizedAtom, const lld::SharedLibraryAtom *>
    keys(io, atom);

    io.mapRequired("name",        keys->_name);
    io.mapOptional("load-name",   keys->_loadName);
    io.mapOptional("can-be-null", keys->_canBeNull, (ShlibCanBeNull) false);
    io.mapOptional("type",        keys->_type, SharedLibraryAtom::Type::Code);
    io.mapOptional("size",        keys->_size, uint64_t(0));
  }
};

// YAML conversion for const lld::AbsoluteAtom*
template <> struct MappingTraits<const lld::AbsoluteAtom *> {

  class NormalizedAtom : public lld::AbsoluteAtom {
  public:
    NormalizedAtom(IO &io)
        : _file(fileFromContext(io)), _name(), _scope(), _value(0) {}
    NormalizedAtom(IO &io, const lld::AbsoluteAtom *atom)
        : _file(fileFromContext(io)), _name(atom->name()),
          _scope(atom->scope()), _value(atom->value()) {}
    const lld::AbsoluteAtom *denormalize(IO &io) {
      YamlContext *info = reinterpret_cast<YamlContext *>(io.getContext());
      assert(info != nullptr);
      typedef MappingTraits<const lld::File *>::NormalizedFile NormalizedFile;
      NormalizedFile *f = reinterpret_cast<NormalizedFile *>(info->_file);
      if (!_name.empty())
        _name = f->copyString(_name);

      DEBUG_WITH_TYPE("WriterYAML",
                      llvm::dbgs() << "created AbsoluteAtom named: '" << _name
                                   << "' (" << (const void *)_name.data()
                                   << ", " << _name.size() << ")\n");
      return this;
    }
    // Extract current File object from YAML I/O parsing context
    const lld::File &fileFromContext(IO &io) {
      YamlContext *info = reinterpret_cast<YamlContext *>(io.getContext());
      assert(info != nullptr);
      assert(info->_file != nullptr);
      return *info->_file;
    }

    const lld::File &file() const override { return _file; }
    StringRef name() const override { return _name; }
    uint64_t value() const override { return _value; }
    Scope scope() const override { return _scope; }

    const lld::File &_file;
    StringRef        _name;
    StringRef        _refName;
    Scope            _scope;
    Hex64            _value;
  };

  static void mapping(IO &io, const lld::AbsoluteAtom *&atom) {
    MappingNormalizationHeap<NormalizedAtom, const lld::AbsoluteAtom *> keys(
        io, atom);

    if (io.outputting()) {
      typedef MappingTraits<const lld::File *>::NormalizedFile NormalizedFile;
      YamlContext *info = reinterpret_cast<YamlContext *>(io.getContext());
      assert(info != nullptr);
      NormalizedFile *f = reinterpret_cast<NormalizedFile *>(info->_file);
      assert(f);
      assert(f->_rnb);
      if (f->_rnb->hasRefName(atom)) {
        keys->_refName = f->_rnb->refName(atom);
      }
    }

    io.mapRequired("name",     keys->_name);
    io.mapOptional("ref-name", keys->_refName, StringRef());
    io.mapOptional("scope",    keys->_scope);
    io.mapRequired("value",    keys->_value);
  }
};

} // namespace llvm
} // namespace yaml

RefNameResolver::RefNameResolver(const lld::File *file, IO &io) : _io(io) {
  typedef MappingTraits<const lld::DefinedAtom *>::NormalizedAtom
  NormalizedAtom;
  for (const lld::DefinedAtom *a : file->defined()) {
    const auto *na = (const NormalizedAtom *)a;
    if (!na->_refName.empty())
      add(na->_refName, a);
    else if (!na->_name.empty())
      add(na->_name, a);
  }

  for (const lld::UndefinedAtom *a : file->undefined())
    add(a->name(), a);

  for (const lld::SharedLibraryAtom *a : file->sharedLibrary())
    add(a->name(), a);

  typedef MappingTraits<const lld::AbsoluteAtom *>::NormalizedAtom NormAbsAtom;
  for (const lld::AbsoluteAtom *a : file->absolute()) {
    const auto *na = (const NormAbsAtom *)a;
    if (na->_refName.empty())
      add(na->_name, a);
    else
      add(na->_refName, a);
  }
}

inline const lld::File *
MappingTraits<const lld::File *>::NormalizedFile::denormalize(IO &io) {
  typedef MappingTraits<const lld::DefinedAtom *>::NormalizedAtom
  NormalizedAtom;

  RefNameResolver nameResolver(this, io);
  // Now that all atoms are parsed, references can be bound.
  for (const lld::DefinedAtom *a : this->defined()) {
    auto *normAtom = (NormalizedAtom *)const_cast<DefinedAtom *>(a);
    normAtom->bind(nameResolver);
  }

  _definedAtoms._atoms.erase(
      std::remove_if(_definedAtoms._atoms.begin(), _definedAtoms._atoms.end(),
                     [](const DefinedAtom *a) {
        return ((const NormalizedAtom *)a)->isGroupChild();
      }),
      _definedAtoms._atoms.end());

  return this;
}

inline void MappingTraits<const lld::DefinedAtom *>::NormalizedAtom::bind(
    const RefNameResolver &resolver) {
  typedef MappingTraits<const lld::Reference *>::NormalizedReference
  NormalizedReference;
  for (const lld::Reference *ref : _references) {
    auto *normRef = (NormalizedReference *)const_cast<Reference *>(ref);
    normRef->bind(resolver);
  }
}

inline void MappingTraits<const lld::Reference *>::NormalizedReference::bind(
    const RefNameResolver &resolver) {
  typedef MappingTraits<const lld::DefinedAtom *>::NormalizedAtom NormalizedAtom;

  _target = resolver.lookup(_targetName);

  if (_mappedKind.ns == lld::Reference::KindNamespace::all &&
      _mappedKind.value == lld::Reference::kindGroupChild) {
    ((NormalizedAtom *)const_cast<Atom *>(_target))->setGroupChild(true);
  }
}

inline StringRef
MappingTraits<const lld::Reference *>::NormalizedReference::targetName(
    IO &io, const lld::Reference *ref) {
  if (ref->target() == nullptr)
    return StringRef();
  YamlContext *info = reinterpret_cast<YamlContext *>(io.getContext());
  assert(info != nullptr);
  typedef MappingTraits<const lld::File *>::NormalizedFile NormalizedFile;
  NormalizedFile *f = reinterpret_cast<NormalizedFile *>(info->_file);
  RefNameBuilder &rnb = *f->_rnb;
  if (rnb.hasRefName(ref->target()))
    return rnb.refName(ref->target());
  return ref->target()->name();
}

namespace lld {
namespace yaml {

class Writer : public lld::Writer {
public:
  Writer(const LinkingContext &context) : _context(context) {}

  std::error_code writeFile(const lld::File &file, StringRef outPath) override {
    // Create stream to path.
    std::error_code ec;
    llvm::raw_fd_ostream out(outPath, ec, llvm::sys::fs::F_Text);
    if (ec)
      return ec;

    // Create yaml Output writer, using yaml options for context.
    YamlContext yamlContext;
    yamlContext._linkingContext = &_context;
    yamlContext._registry = &_context.registry();
    llvm::yaml::Output yout(out, &yamlContext);

    // Write yaml output.
    const lld::File *fileRef = &file;
    yout << fileRef;

    return std::error_code();
  }

private:
  const LinkingContext &_context;
};

} // end namespace yaml

namespace {

/// Handles !native tagged yaml documents.
class NativeYamlIOTaggedDocumentHandler : public YamlIOTaggedDocumentHandler {
  bool handledDocTag(llvm::yaml::IO &io, const lld::File *&file) const override {
    if (io.mapTag("!native")) {
      MappingTraits<const lld::File *>::mappingAtoms(io, file);
      return true;
    }
    return false;
  }
};


/// Handles !archive tagged yaml documents.
class ArchiveYamlIOTaggedDocumentHandler : public YamlIOTaggedDocumentHandler {
  bool handledDocTag(llvm::yaml::IO &io, const lld::File *&file) const override {
    if (io.mapTag("!archive")) {
      MappingTraits<const lld::File *>::mappingArchive(io, file);
      return true;
    }
    return false;
  }
};



class YAMLReader : public Reader {
public:
  YAMLReader(const Registry &registry) : _registry(registry) {}

  bool canParse(file_magic, StringRef ext, const MemoryBuffer &) const override {
    return (ext.equals(".objtxt") || ext.equals(".yaml"));
  }

  std::error_code
  loadFile(std::unique_ptr<MemoryBuffer> mb, const class Registry &,
           std::vector<std::unique_ptr<File>> &result) const override {
    // Create YAML Input Reader.
    YamlContext yamlContext;
    yamlContext._registry = &_registry;
    yamlContext._path = mb->getBufferIdentifier();
    llvm::yaml::Input yin(mb->getBuffer(), &yamlContext);

    // Fill vector with File objects created by parsing yaml.
    std::vector<const lld::File *> createdFiles;
    yin >> createdFiles;

    // Error out now if there were parsing errors.
    if (yin.error())
      return make_error_code(lld::YamlReaderError::illegal_value);

    std::shared_ptr<MemoryBuffer> smb(mb.release());
    for (const File *file : createdFiles) {
      // Note: loadFile() should return vector of *const* File
      File *f = const_cast<File *>(file);
      f->setLastError(std::error_code());
      f->setSharedMemoryBuffer(smb);
      result.emplace_back(f);
    }
    return make_error_code(lld::YamlReaderError::success);
  }

private:
  const Registry &_registry;
};

} // anonymous namespace

void Registry::addSupportYamlFiles() {
  add(std::unique_ptr<Reader>(new YAMLReader(*this)));
  add(std::unique_ptr<YamlIOTaggedDocumentHandler>(
                                    new NativeYamlIOTaggedDocumentHandler()));
  add(std::unique_ptr<YamlIOTaggedDocumentHandler>(
                                    new ArchiveYamlIOTaggedDocumentHandler()));
}

std::unique_ptr<Writer> createWriterYAML(const LinkingContext &context) {
  return std::unique_ptr<Writer>(new lld::yaml::Writer(context));
}

} // end namespace lld
