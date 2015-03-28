//===- lib/ReaderWriter/ELF/Hexagon/HexagonTargetHandler.h ----------------===//
//
//                             The LLVM Linker
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef HEXAGON_TARGET_HANDLER_H
#define HEXAGON_TARGET_HANDLER_H

#include "DefaultTargetHandler.h"
#include "HexagonELFReader.h"
#include "HexagonExecutableAtoms.h"
#include "HexagonRelocationHandler.h"
#include "HexagonSectionChunks.h"
#include "TargetLayout.h"

namespace lld {
namespace elf {
class HexagonLinkingContext;

/// \brief TargetLayout for Hexagon
template <class HexagonELFType>
class HexagonTargetLayout final : public TargetLayout<HexagonELFType> {
public:
  enum HexagonSectionOrder {
    ORDER_SDATA = 205
  };

  HexagonTargetLayout(HexagonLinkingContext &hti)
      : TargetLayout<HexagonELFType>(hti), _sdataSection(nullptr),
        _gotSymAtom(nullptr), _cachedGotSymAtom(false) {
    _sdataSection = new (_alloc) SDataSection<HexagonELFType>(hti);
  }

  /// \brief Return the section order for a input section
  virtual Layout::SectionOrder getSectionOrder(
      StringRef name, int32_t contentType, int32_t contentPermissions) {
    if ((contentType == DefinedAtom::typeDataFast) ||
       (contentType == DefinedAtom::typeZeroFillFast))
      return ORDER_SDATA;

    return DefaultLayout<HexagonELFType>::getSectionOrder(name, contentType,
                                                          contentPermissions);
  }

  /// \brief Return the appropriate input section name.
  virtual StringRef getInputSectionName(const DefinedAtom *da) const {
    switch (da->contentType()) {
    case DefinedAtom::typeDataFast:
    case DefinedAtom::typeZeroFillFast:
      return ".sdata";
    default:
      break;
    }
    return DefaultLayout<HexagonELFType>::getInputSectionName(da);
  }

  /// \brief Gets or creates a section.
  virtual AtomSection<HexagonELFType> *
  createSection(StringRef name, int32_t contentType,
                DefinedAtom::ContentPermissions contentPermissions,
                Layout::SectionOrder sectionOrder) {
    if ((contentType == DefinedAtom::typeDataFast) ||
       (contentType == DefinedAtom::typeZeroFillFast))
      return _sdataSection;
    return DefaultLayout<HexagonELFType>::createSection(
        name, contentType, contentPermissions, sectionOrder);
  }

  /// \brief get the segment type for the section thats defined by the target
  virtual Layout::SegmentType
  getSegmentType(Section<HexagonELFType> *section) const {
    if (section->order() == ORDER_SDATA)
      return PT_LOAD;

    return DefaultLayout<HexagonELFType>::getSegmentType(section);
  }

  Section<HexagonELFType> *getSDataSection() const {
    return _sdataSection;
  }

  uint64_t getGOTSymAddr() {
    if (!_cachedGotSymAtom) {
      auto gotAtomIter = this->findAbsoluteAtom("_GLOBAL_OFFSET_TABLE_");
      _gotSymAtom = (*gotAtomIter);
      _cachedGotSymAtom = true;
    }
    if (_gotSymAtom)
      return _gotSymAtom->_virtualAddr;
    return 0;
  }

private:
  llvm::BumpPtrAllocator _alloc;
  SDataSection<HexagonELFType> *_sdataSection;
  AtomLayout *_gotSymAtom;
  bool _cachedGotSymAtom;
};

/// \brief TargetHandler for Hexagon
class HexagonTargetHandler final :
    public DefaultTargetHandler<HexagonELFType> {
public:
  HexagonTargetHandler(HexagonLinkingContext &targetInfo);

  void registerRelocationNames(Registry &registry) override;

  const HexagonTargetRelocationHandler &getRelocationHandler() const override {
    return *(_hexagonRelocationHandler.get());
  }

  HexagonTargetLayout<HexagonELFType> &getTargetLayout() override {
    return *(_hexagonTargetLayout.get());
  }

  std::unique_ptr<Reader> getObjReader() override {
    return std::unique_ptr<Reader>(
        new HexagonELFObjectReader(_hexagonLinkingContext));
  }

  std::unique_ptr<Reader> getDSOReader() override {
    return std::unique_ptr<Reader>(
        new HexagonELFDSOReader(_hexagonLinkingContext));
  }

  std::unique_ptr<Writer> getWriter() override;

private:
  llvm::BumpPtrAllocator _alloc;
  static const Registry::KindStrings kindStrings[];
  HexagonLinkingContext &_hexagonLinkingContext;
  std::unique_ptr<HexagonRuntimeFile<HexagonELFType> > _hexagonRuntimeFile;
  std::unique_ptr<HexagonTargetLayout<HexagonELFType>> _hexagonTargetLayout;
  std::unique_ptr<HexagonTargetRelocationHandler> _hexagonRelocationHandler;
};
} // end namespace elf
} // end namespace lld

#endif
