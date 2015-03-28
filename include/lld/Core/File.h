//===- Core/File.h - A Container of Atoms ---------------------------------===//
//
//                             The LLVM Linker
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLD_CORE_FILE_H
#define LLD_CORE_FILE_H

#include "lld/Core/AbsoluteAtom.h"
#include "lld/Core/DefinedAtom.h"
#include "lld/Core/SharedLibraryAtom.h"
#include "lld/Core/UndefinedAtom.h"
#include "lld/Core/range.h"
#include "llvm/ADT/Optional.h"
#include "llvm/ADT/Twine.h"
#include "llvm/Support/ErrorHandling.h"
#include <functional>
#include <memory>
#include <mutex>
#include <vector>

namespace lld {

class LinkingContext;

/// Every Atom is owned by some File. A common scenario is for a single
/// object file (.o) to be parsed by some reader and produce a single
/// File object that represents the content of that object file.
///
/// To iterate through the Atoms in a File there are four methods that
/// return collections.  For instance to iterate through all the DefinedAtoms
/// in a File object use:
///      for (const DefinedAtoms *atom : file->defined()) {
///      }
///
/// The Atom objects in a File are owned by the File object.  The Atom objects
/// are destroyed when the File object is destroyed.
class File {
public:
  virtual ~File();

  /// \brief Kinds of files that are supported.
  enum Kind {
    kindObject,        ///< object file (.o)
    kindSharedLibrary, ///< shared library (.so)
    kindArchiveLibrary ///< archive (.a)
  };

  /// \brief Returns file kind.  Need for dyn_cast<> on File objects.
  Kind kind() const {
    return _kind;
  }

  /// This returns the path to the file which was used to create this object
  /// (e.g. "/tmp/foo.o"). If the file is a member of an archive file, the
  /// returned string includes the archive file name.
  StringRef path() const {
    if (_archivePath.empty())
      return _path;
    if (_archiveMemberPath.empty())
      _archiveMemberPath = (_archivePath + "(" + _path + ")").str();
    return _archiveMemberPath;
  }

  /// Returns the path of the archive file name if this file is instantiated
  /// from an archive file. Otherwise returns the empty string.
  StringRef archivePath() const { return _archivePath; }
  void setArchivePath(StringRef path) { _archivePath = path; }

  /// Returns the path name of this file. It doesn't include archive file name.
  StringRef memberPath() const { return _path; }

  /// Returns the command line order of the file.
  uint64_t ordinal() const {
    assert(_ordinal != UINT64_MAX);
    return _ordinal;
  }

  /// Returns true/false depending on whether an ordinal has been set.
  bool hasOrdinal() const { return (_ordinal != UINT64_MAX); }

  /// Sets the command line order of the file.
  void setOrdinal(uint64_t ordinal) const { _ordinal = ordinal; }

  template <typename T> class atom_iterator; // forward reference

  /// For allocating any objects owned by this File.
  llvm::BumpPtrAllocator &allocator() const {
    return _allocator;
  }

  /// \brief For use interating over DefinedAtoms in this File.
  typedef atom_iterator<DefinedAtom>  defined_iterator;

  /// \brief For use interating over UndefinedAtoms in this File.
  typedef atom_iterator<UndefinedAtom> undefined_iterator;

  /// \brief For use interating over SharedLibraryAtoms in this File.
  typedef atom_iterator<SharedLibraryAtom> shared_library_iterator;

  /// \brief For use interating over AbsoluteAtoms in this File.
  typedef atom_iterator<AbsoluteAtom> absolute_iterator;

  /// \brief Different object file readers may instantiate and manage atoms with
  /// different data structures.  This class is a collection abstraction.
  /// Each concrete File instance must implement these atom_collection
  /// methods to enable clients to interate the File's atoms.
  template <typename T>
  class atom_collection {
  public:
    virtual ~atom_collection() { }
    virtual atom_iterator<T> begin() const = 0;
    virtual atom_iterator<T> end() const = 0;
    virtual const T *deref(const void *it) const = 0;
    virtual void next(const void *&it) const = 0;
    virtual uint64_t size() const = 0;
    bool empty() const { return size() == 0; }
  };

  /// \brief The class is the iterator type used to iterate through a File's
  /// Atoms. This iterator delegates the work to the associated atom_collection
  /// object. There are four kinds of Atoms, so this iterator is templated on
  /// the four base Atom kinds.
  template <typename T>
  class atom_iterator : public std::iterator<std::forward_iterator_tag, T> {
  public:
    atom_iterator(const atom_collection<T> &c, const void *it)
              : _collection(&c), _it(it) { }

    const T *operator*() const {
      return _collection->deref(_it);
    }
    const T *operator->() const {
      return _collection->deref(_it);
    }

    friend bool operator==(const atom_iterator<T> &lhs, const atom_iterator<T> &rhs)  {
      return lhs._it == rhs._it;
    }

    friend bool operator!=(const atom_iterator<T> &lhs, const atom_iterator<T> &rhs)  {
      return !(lhs == rhs);
    }

    atom_iterator<T> &operator++() {
      _collection->next(_it);
      return *this;
    }
  private:
    const atom_collection<T> *_collection;
    const void               *_it;
  };

  /// \brief Must be implemented to return the atom_collection object for
  /// all DefinedAtoms in this File.
  virtual const atom_collection<DefinedAtom> &defined() const = 0;

  /// \brief Must be implemented to return the atom_collection object for
  /// all UndefinedAtomw in this File.
  virtual const atom_collection<UndefinedAtom> &undefined() const = 0;

  /// \brief Must be implemented to return the atom_collection object for
  /// all SharedLibraryAtoms in this File.
  virtual const atom_collection<SharedLibraryAtom> &sharedLibrary() const = 0;

  /// \brief Must be implemented to return the atom_collection object for
  /// all AbsoluteAtoms in this File.
  virtual const atom_collection<AbsoluteAtom> &absolute() const = 0;

  /// \brief If a file is parsed using a different method than doParse(),
  /// one must use this method to set the last error status, so that
  /// doParse will not be called twice. Only YAML reader uses this
  /// (because YAML reader does not read blobs but structured data).
  void setLastError(std::error_code err) { _lastError = err; }

  std::error_code parse();

  // This function is called just before the core linker tries to use
  // a file. Currently the PECOFF reader uses this to trigger the
  // driver to parse .drectve section (which contains command line options).
  // If you want to do something having side effects, don't do that in
  // doParse() because a file could be pre-loaded speculatively.
  // Use this hook instead.
  virtual void beforeLink() {}

  // Usually each file owns a std::unique_ptr<MemoryBuffer>.
  // However, there's one special case. If a file is an archive file,
  // the archive file and its children all shares the same memory buffer.
  // This method is used by the ArchiveFile to give its children
  // co-ownership of the buffer.
  void setSharedMemoryBuffer(std::shared_ptr<MemoryBuffer> mb) {
    _sharedMemoryBuffer = mb;
  }

protected:
  /// \brief only subclasses of File can be instantiated
  File(StringRef p, Kind kind)
      : _path(p), _kind(kind), _ordinal(UINT64_MAX) {}

  /// \brief Subclasses should override this method to parse the
  /// memory buffer passed to this file's constructor.
  virtual std::error_code doParse() { return std::error_code(); }

  /// \brief This is a convenience class for File subclasses which manage their
  /// atoms as a simple std::vector<>.
  template <typename T>
  class atom_collection_vector : public atom_collection<T> {
  public:
    atom_iterator<T> begin() const override {
      auto *it = _atoms.empty() ? nullptr
                                : reinterpret_cast<const void *>(_atoms.data());
      return atom_iterator<T>(*this, it);
    }

    atom_iterator<T> end() const override {
      auto *it = _atoms.empty() ? nullptr : reinterpret_cast<const void *>(
                                                _atoms.data() + _atoms.size());
      return atom_iterator<T>(*this, it);
    }

    const T *deref(const void *it) const override {
      return *reinterpret_cast<const T *const *>(it);
    }

    void next(const void *&it) const override {
      const T *const *p = reinterpret_cast<const T *const *>(it);
      ++p;
      it = reinterpret_cast<const void*>(p);
    }

    uint64_t size() const override { return _atoms.size(); }

    std::vector<const T *> _atoms;
  };

  /// \brief This is a convenience class for File subclasses which need to
  /// return an empty collection.
  template <typename T>
  class atom_collection_empty : public atom_collection<T> {
  public:
    atom_iterator<T> begin() const override {
      return atom_iterator<T>(*this, nullptr);
    }
    atom_iterator<T> end() const override {
      return atom_iterator<T>(*this, nullptr);
    }
    const T *deref(const void *it) const override {
      llvm_unreachable("empty collection should never be accessed");
    }
    void next(const void *&it) const override {}
    uint64_t size() const override { return 0; }
  };

  static atom_collection_empty<DefinedAtom>       _noDefinedAtoms;
  static atom_collection_empty<UndefinedAtom>     _noUndefinedAtoms;
  static atom_collection_empty<SharedLibraryAtom> _noSharedLibraryAtoms;
  static atom_collection_empty<AbsoluteAtom>      _noAbsoluteAtoms;
  mutable llvm::BumpPtrAllocator                  _allocator;

private:
  StringRef _path;
  std::string _archivePath;
  mutable std::string _archiveMemberPath;
  Kind              _kind;
  mutable uint64_t  _ordinal;
  std::shared_ptr<MemoryBuffer> _sharedMemoryBuffer;
  llvm::Optional<std::error_code> _lastError;
  std::mutex _parseMutex;
};

/// \brief A mutable File.
class MutableFile : public File {
public:
  /// \brief Add an atom to the file. Invalidates iterators for all returned
  /// containters.
  virtual void addAtom(const Atom&) = 0;

  typedef range<std::vector<const DefinedAtom *>::iterator> DefinedAtomRange;
  virtual DefinedAtomRange definedAtoms() = 0;

  virtual void
  removeDefinedAtomsIf(std::function<bool(const DefinedAtom *)> pred) = 0;

protected:
  /// \brief only subclasses of MutableFile can be instantiated
  MutableFile(StringRef p) : File(p, kindObject) {}
};

/// An ErrorFile represents a file that doesn't exist.
/// If you try to parse a file which doesn't exist, an instance of this
/// class will be returned. That's parse method always returns an error.
/// This is useful to delay erroring on non-existent files, so that we
/// can do unit testing a driver using non-existing file paths.
class ErrorFile : public File {
public:
  ErrorFile(StringRef path, std::error_code ec)
      : File(path, kindObject), _ec(ec) {}

  std::error_code doParse() override { return _ec; }

  const atom_collection<DefinedAtom> &defined() const override {
    llvm_unreachable("internal error");
  }
  const atom_collection<UndefinedAtom> &undefined() const override {
    llvm_unreachable("internal error");
  }
  const atom_collection<SharedLibraryAtom> &sharedLibrary() const override {
    llvm_unreachable("internal error");
  }
  const atom_collection<AbsoluteAtom> &absolute() const override {
    llvm_unreachable("internal error");
  }

private:
  std::error_code _ec;
};

} // end namespace lld

#endif
