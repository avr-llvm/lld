//===- lib/ReaderWriter/MachO/MachOLinkingContext.cpp ---------------------===//
//
//                             The LLVM Linker
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "lld/ReaderWriter/MachOLinkingContext.h"
#include "ArchHandler.h"
#include "File.h"
#include "MachONormalizedFile.h"
#include "MachOPasses.h"
#include "lld/Core/ArchiveLibraryFile.h"
#include "lld/Core/PassManager.h"
#include "lld/Core/Reader.h"
#include "lld/Core/Writer.h"
#include "lld/Driver/Driver.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/ADT/Triple.h"
#include "llvm/Config/config.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/Errc.h"
#include "llvm/Support/Host.h"
#include "llvm/Support/MachO.h"
#include "llvm/Support/Path.h"
#include <algorithm>

#if defined(HAVE_CXXABI_H)
#include <cxxabi.h>
#endif

using lld::mach_o::ArchHandler;
using lld::mach_o::MachODylibFile;
using namespace llvm::MachO;

namespace lld {

bool MachOLinkingContext::parsePackedVersion(StringRef str, uint32_t &result) {
  result = 0;

  if (str.empty())
    return false;

  SmallVector<StringRef, 3> parts;
  llvm::SplitString(str, parts, ".");

  unsigned long long num;
  if (llvm::getAsUnsignedInteger(parts[0], 10, num))
    return true;
  if (num > 65535)
    return true;
  result = num << 16;

  if (parts.size() > 1) {
    if (llvm::getAsUnsignedInteger(parts[1], 10, num))
      return true;
    if (num > 255)
      return true;
    result |= (num << 8);
  }

  if (parts.size() > 2) {
    if (llvm::getAsUnsignedInteger(parts[2], 10, num))
      return true;
    if (num > 255)
      return true;
    result |= num;
  }

  return false;
}


MachOLinkingContext::ArchInfo MachOLinkingContext::_s_archInfos[] = {
  { "x86_64", arch_x86_64, true,  CPU_TYPE_X86_64,  CPU_SUBTYPE_X86_64_ALL },
  { "i386",   arch_x86,    true,  CPU_TYPE_I386,    CPU_SUBTYPE_X86_ALL },
  { "ppc",    arch_ppc,    false, CPU_TYPE_POWERPC, CPU_SUBTYPE_POWERPC_ALL },
  { "armv6",  arch_armv6,  true,  CPU_TYPE_ARM,     CPU_SUBTYPE_ARM_V6 },
  { "armv7",  arch_armv7,  true,  CPU_TYPE_ARM,     CPU_SUBTYPE_ARM_V7 },
  { "armv7s", arch_armv7s, true,  CPU_TYPE_ARM,     CPU_SUBTYPE_ARM_V7S },
  { "arm64",  arch_arm64,  true,  CPU_TYPE_ARM64,   CPU_SUBTYPE_ARM64_ALL },
  { "",       arch_unknown,false, 0,                0 }
};

MachOLinkingContext::Arch
MachOLinkingContext::archFromCpuType(uint32_t cputype, uint32_t cpusubtype) {
  for (ArchInfo *info = _s_archInfos; !info->archName.empty(); ++info) {
    if ((info->cputype == cputype) && (info->cpusubtype == cpusubtype))
      return info->arch;
  }
  return arch_unknown;
}

MachOLinkingContext::Arch
MachOLinkingContext::archFromName(StringRef archName) {
  for (ArchInfo *info = _s_archInfos; !info->archName.empty(); ++info) {
    if (info->archName.equals(archName))
      return info->arch;
  }
  return arch_unknown;
}

StringRef MachOLinkingContext::nameFromArch(Arch arch) {
  for (ArchInfo *info = _s_archInfos; !info->archName.empty(); ++info) {
    if (info->arch == arch)
      return info->archName;
  }
  return "<unknown>";
}

uint32_t MachOLinkingContext::cpuTypeFromArch(Arch arch) {
  assert(arch != arch_unknown);
  for (ArchInfo *info = _s_archInfos; !info->archName.empty(); ++info) {
    if (info->arch == arch)
      return info->cputype;
  }
  llvm_unreachable("Unknown arch type");
}

uint32_t MachOLinkingContext::cpuSubtypeFromArch(Arch arch) {
  assert(arch != arch_unknown);
  for (ArchInfo *info = _s_archInfos; !info->archName.empty(); ++info) {
    if (info->arch == arch)
      return info->cpusubtype;
  }
  llvm_unreachable("Unknown arch type");
}

bool MachOLinkingContext::isThinObjectFile(StringRef path, Arch &arch) {
  return mach_o::normalized::isThinObjectFile(path, arch);
}

bool MachOLinkingContext::sliceFromFatFile(const MemoryBuffer &mb,
                                           uint32_t &offset,
                                           uint32_t &size) {
  return mach_o::normalized::sliceFromFatFile(mb, _arch, offset, size);
}

MachOLinkingContext::MachOLinkingContext()
    : _outputMachOType(MH_EXECUTE), _outputMachOTypeStatic(false),
      _doNothing(false), _pie(false), _arch(arch_unknown), _os(OS::macOSX),
      _osMinVersion(0), _pageZeroSize(0), _pageSize(4096), _baseAddress(0),
      _compatibilityVersion(0), _currentVersion(0), _deadStrippableDylib(false),
      _printAtoms(false), _testingFileUsage(false), _keepPrivateExterns(false),
      _demangle(false), _archHandler(nullptr),
      _exportMode(ExportMode::globals),
      _debugInfoMode(DebugInfoMode::addDebugMap), _orderFileEntries(0) {}

MachOLinkingContext::~MachOLinkingContext() {}

void MachOLinkingContext::configure(HeaderFileType type, Arch arch, OS os,
                                    uint32_t minOSVersion) {
  _outputMachOType = type;
  _arch = arch;
  _os = os;
  _osMinVersion = minOSVersion;

  // If min OS not specified on command line, use reasonable defaults.
  if (minOSVersion == 0) {
    switch (_arch) {
    case arch_x86_64:
    case arch_x86:
      parsePackedVersion("10.8", _osMinVersion);
      _os = MachOLinkingContext::OS::macOSX;
      break;
    case arch_armv6:
    case arch_armv7:
    case arch_armv7s:
    case arch_arm64:
      parsePackedVersion("7.0", _osMinVersion);
      _os = MachOLinkingContext::OS::iOS;
      break;
    default:
      break;
    }
  }

  switch (_outputMachOType) {
  case llvm::MachO::MH_EXECUTE:
    // If targeting newer OS, use _main
    if (minOS("10.8", "6.0")) {
      _entrySymbolName = "_main";
    } else {
      // If targeting older OS, use start (in crt1.o)
      _entrySymbolName = "start";
    }

    // __PAGEZERO defaults to 4GB on 64-bit (except for PP64 which lld does not
    // support) and 4KB on 32-bit.
    if (is64Bit(_arch)) {
      _pageZeroSize = 0x100000000;
    } else {
      _pageZeroSize = 0x1000;
    }

    // Make PIE by default when targetting newer OSs.
    switch (os) {
      case OS::macOSX:
        if (minOSVersion >= 0x000A0700) // MacOSX 10.7
          _pie = true;
        break;
      case OS::iOS:
        if (minOSVersion >= 0x00040300) // iOS 4.3
          _pie = true;
       break;
       case OS::iOS_simulator:
        _pie = true;
       break;
       case OS::unknown:
       break;
    }
    break;
  case llvm::MachO::MH_DYLIB:
    _globalsAreDeadStripRoots = true;
    break;
  case llvm::MachO::MH_BUNDLE:
    break;
  case llvm::MachO::MH_OBJECT:
    _printRemainingUndefines = false;
    _allowRemainingUndefines = true;
  default:
    break;
  }

  // Set default segment page sizes based on arch.
  if (arch == arch_arm64)
    _pageSize = 4*4096;
}

uint32_t MachOLinkingContext::getCPUType() const {
  return cpuTypeFromArch(_arch);
}

uint32_t MachOLinkingContext::getCPUSubType() const {
  return cpuSubtypeFromArch(_arch);
}

bool MachOLinkingContext::is64Bit(Arch arch) {
  for (ArchInfo *info = _s_archInfos; !info->archName.empty(); ++info) {
    if (info->arch == arch) {
      return (info->cputype & CPU_ARCH_ABI64);
    }
  }
  // unknown archs are not 64-bit.
  return false;
}

bool MachOLinkingContext::isHostEndian(Arch arch) {
  assert(arch != arch_unknown);
  for (ArchInfo *info = _s_archInfos; !info->archName.empty(); ++info) {
    if (info->arch == arch) {
      return (info->littleEndian == llvm::sys::IsLittleEndianHost);
    }
  }
  llvm_unreachable("Unknown arch type");
}

bool MachOLinkingContext::isBigEndian(Arch arch) {
  assert(arch != arch_unknown);
  for (ArchInfo *info = _s_archInfos; !info->archName.empty(); ++info) {
    if (info->arch == arch) {
      return ! info->littleEndian;
    }
  }
  llvm_unreachable("Unknown arch type");
}



bool MachOLinkingContext::is64Bit() const {
  return is64Bit(_arch);
}

bool MachOLinkingContext::outputTypeHasEntry() const {
  switch (_outputMachOType) {
  case MH_EXECUTE:
  case MH_DYLINKER:
  case MH_PRELOAD:
    return true;
  default:
    return false;
  }
}

bool MachOLinkingContext::needsStubsPass() const {
  switch (_outputMachOType) {
  case MH_EXECUTE:
    return !_outputMachOTypeStatic;
  case MH_DYLIB:
  case MH_BUNDLE:
    return true;
  default:
    return false;
  }
}

bool MachOLinkingContext::needsGOTPass() const {
  // GOT pass not used in -r mode.
  if (_outputMachOType == MH_OBJECT)
    return false;
  // Only some arches use GOT pass.
  switch (_arch) {
    case arch_x86_64:
    case arch_arm64:
      return true;
    default:
      return false;
  }
}

bool MachOLinkingContext::needsCompactUnwindPass() const {
  switch (_outputMachOType) {
  case MH_EXECUTE:
  case MH_DYLIB:
  case MH_BUNDLE:
    return archHandler().needsCompactUnwind();
  default:
    return false;
  }
}

bool MachOLinkingContext::needsShimPass() const {
  // Shim pass only used in final executables.
  if (_outputMachOType == MH_OBJECT)
    return false;
  // Only 32-bit arm arches use Shim pass.
  switch (_arch) {
  case arch_armv6:
  case arch_armv7:
  case arch_armv7s:
    return true;
  default:
    return false;
  }
}

StringRef MachOLinkingContext::binderSymbolName() const {
  return archHandler().stubInfo().binderSymbolName;
}




bool MachOLinkingContext::minOS(StringRef mac, StringRef iOS) const {
  uint32_t parsedVersion;
  switch (_os) {
  case OS::macOSX:
    if (parsePackedVersion(mac, parsedVersion))
      return false;
    return _osMinVersion >= parsedVersion;
  case OS::iOS:
  case OS::iOS_simulator:
    if (parsePackedVersion(iOS, parsedVersion))
      return false;
    return _osMinVersion >= parsedVersion;
  case OS::unknown:
    break;
  }
  llvm_unreachable("target not configured for iOS or MacOSX");
}

bool MachOLinkingContext::addEntryPointLoadCommand() const {
  if ((_outputMachOType == MH_EXECUTE) && !_outputMachOTypeStatic) {
    return minOS("10.8", "6.0");
  }
  return false;
}

bool MachOLinkingContext::addUnixThreadLoadCommand() const {
  switch (_outputMachOType) {
  case MH_EXECUTE:
    if (_outputMachOTypeStatic)
      return true;
    else
      return !minOS("10.8", "6.0");
    break;
  case MH_DYLINKER:
  case MH_PRELOAD:
    return true;
  default:
    return false;
  }
}

bool MachOLinkingContext::pathExists(StringRef path) const {
  if (!_testingFileUsage)
    return llvm::sys::fs::exists(path.str());

  // Otherwise, we're in test mode: only files explicitly provided on the
  // command-line exist.
  std::string key = path.str();
  std::replace(key.begin(), key.end(), '\\', '/');
  return _existingPaths.find(key) != _existingPaths.end();
}

bool MachOLinkingContext::fileExists(StringRef path) const {
  bool found = pathExists(path);
  // Log search misses.
  if (!found)
    addInputFileNotFound(path);

  // When testing, file is never opened, so logging is done here.
  if (_testingFileUsage && found)
    addInputFileDependency(path);

  return found;
}

void MachOLinkingContext::setSysLibRoots(const StringRefVector &paths) {
  _syslibRoots = paths;
}

void MachOLinkingContext::addRpath(StringRef rpath) {
  _rpaths.push_back(rpath);
}

void MachOLinkingContext::addModifiedSearchDir(StringRef libPath,
                                               bool isSystemPath) {
  bool addedModifiedPath = false;

  // -syslibroot only applies to absolute paths.
  if (libPath.startswith("/")) {
    for (auto syslibRoot : _syslibRoots) {
      SmallString<256> path(syslibRoot);
      llvm::sys::path::append(path, libPath);
      if (pathExists(path)) {
        _searchDirs.push_back(path.str().copy(_allocator));
        addedModifiedPath = true;
      }
    }
  }

  if (addedModifiedPath)
    return;

  // Finally, if only one -syslibroot is given, system paths which aren't in it
  // get suppressed.
  if (_syslibRoots.size() != 1 || !isSystemPath) {
    if (pathExists(libPath)) {
      _searchDirs.push_back(libPath);
    }
  }
}

void MachOLinkingContext::addFrameworkSearchDir(StringRef fwPath,
                                                bool isSystemPath) {
  bool pathAdded = false;

  // -syslibroot only used with to absolute framework search paths.
  if (fwPath.startswith("/")) {
    for (auto syslibRoot : _syslibRoots) {
      SmallString<256> path(syslibRoot);
      llvm::sys::path::append(path, fwPath);
      if (pathExists(path)) {
        _frameworkDirs.push_back(path.str().copy(_allocator));
        pathAdded = true;
      }
    }
  }
  // If fwPath found in any -syslibroot, then done.
  if (pathAdded)
    return;

  // If only one -syslibroot, system paths not in that SDK are suppressed.
  if (isSystemPath && (_syslibRoots.size() == 1))
    return;

  // Only use raw fwPath if that directory exists.
  if (pathExists(fwPath))
    _frameworkDirs.push_back(fwPath);
}


ErrorOr<StringRef>
MachOLinkingContext::searchDirForLibrary(StringRef path,
                                         StringRef libName) const {
  SmallString<256> fullPath;
  if (libName.endswith(".o")) {
    // A request ending in .o is special: just search for the file directly.
    fullPath.assign(path);
    llvm::sys::path::append(fullPath, libName);
    if (fileExists(fullPath))
      return fullPath.str().copy(_allocator);
    return make_error_code(llvm::errc::no_such_file_or_directory);
  }

  // Search for dynamic library
  fullPath.assign(path);
  llvm::sys::path::append(fullPath, Twine("lib") + libName + ".dylib");
  if (fileExists(fullPath))
    return fullPath.str().copy(_allocator);

  // If not, try for a static library
  fullPath.assign(path);
  llvm::sys::path::append(fullPath, Twine("lib") + libName + ".a");
  if (fileExists(fullPath))
    return fullPath.str().copy(_allocator);

  return make_error_code(llvm::errc::no_such_file_or_directory);
}



ErrorOr<StringRef> MachOLinkingContext::searchLibrary(StringRef libName) const {
  SmallString<256> path;
  for (StringRef dir : searchDirs()) {
    ErrorOr<StringRef> ec = searchDirForLibrary(dir, libName);
    if (ec)
      return ec;
  }

  return make_error_code(llvm::errc::no_such_file_or_directory);
}


ErrorOr<StringRef> MachOLinkingContext::findPathForFramework(StringRef fwName) const{
  SmallString<256> fullPath;
  for (StringRef dir : frameworkDirs()) {
    fullPath.assign(dir);
    llvm::sys::path::append(fullPath, Twine(fwName) + ".framework", fwName);
    if (fileExists(fullPath))
      return fullPath.str().copy(_allocator);
  }

  return make_error_code(llvm::errc::no_such_file_or_directory);
}

bool MachOLinkingContext::validateImpl(raw_ostream &diagnostics) {
  // TODO: if -arch not specified, look at arch of first .o file.

  if (_currentVersion && _outputMachOType != MH_DYLIB) {
    diagnostics << "error: -current_version can only be used with dylibs\n";
    return false;
  }

  if (_compatibilityVersion && _outputMachOType != MH_DYLIB) {
    diagnostics
        << "error: -compatibility_version can only be used with dylibs\n";
    return false;
  }

  if (_deadStrippableDylib && _outputMachOType != MH_DYLIB) {
    diagnostics
        << "error: -mark_dead_strippable_dylib can only be used with dylibs.\n";
    return false;
  }

  if (!_bundleLoader.empty() && outputMachOType() != MH_BUNDLE) {
    diagnostics
        << "error: -bundle_loader can only be used with Mach-O bundles\n";
    return false;
  }

  // If -exported_symbols_list used, all exported symbols must be defined.
  if (_exportMode == ExportMode::whiteList) {
    for (const auto &symbol : _exportedSymbols)
      addInitialUndefinedSymbol(symbol.getKey());
  }

  // If -dead_strip, set up initial live symbols.
  if (deadStrip()) {
    // Entry point is live.
    if (outputTypeHasEntry())
      addDeadStripRoot(entrySymbolName());
    // Lazy binding helper is live.
    if (needsStubsPass())
      addDeadStripRoot(binderSymbolName());
    // If using -exported_symbols_list, make all exported symbols live.
    if (_exportMode == ExportMode::whiteList) {
      _globalsAreDeadStripRoots = false;
      for (const auto &symbol : _exportedSymbols)
        addDeadStripRoot(symbol.getKey());
    }
  }

  addOutputFileDependency(outputPath());

  return true;
}

void MachOLinkingContext::addPasses(PassManager &pm) {
  mach_o::addLayoutPass(pm, *this);
  if (needsStubsPass())
    mach_o::addStubsPass(pm, *this);
  if (needsCompactUnwindPass())
    mach_o::addCompactUnwindPass(pm, *this);
  if (needsGOTPass())
    mach_o::addGOTPass(pm, *this);
  if (needsShimPass())
    mach_o::addShimPass(pm, *this); // Shim pass must run after stubs pass.
}

Writer &MachOLinkingContext::writer() const {
  if (!_writer)
    _writer = createWriterMachO(*this);
  return *_writer;
}

ErrorOr<std::unique_ptr<MemoryBuffer>>
MachOLinkingContext::getMemoryBuffer(StringRef path) {
  addInputFileDependency(path);

  ErrorOr<std::unique_ptr<MemoryBuffer>> mbOrErr =
    MemoryBuffer::getFileOrSTDIN(path);
  if (std::error_code ec = mbOrErr.getError())
    return ec;
  std::unique_ptr<MemoryBuffer> mb = std::move(mbOrErr.get());

  // If buffer contains a fat file, find required arch in fat buffer
  // and switch buffer to point to just that required slice.
  uint32_t offset;
  uint32_t size;
  if (sliceFromFatFile(*mb, offset, size))
    return MemoryBuffer::getFileSlice(path, size, offset);
  return std::move(mb);
}

MachODylibFile* MachOLinkingContext::loadIndirectDylib(StringRef path) {
  ErrorOr<std::unique_ptr<MemoryBuffer>> mbOrErr = getMemoryBuffer(path);
  if (mbOrErr.getError())
    return nullptr;

  std::vector<std::unique_ptr<File>> files;
  if (registry().loadFile(std::move(mbOrErr.get()), files))
    return nullptr;
  assert(files.size() == 1 && "expected one file in dylib");
  files[0]->parse();
  MachODylibFile* result = reinterpret_cast<MachODylibFile*>(files[0].get());
  // Node object now owned by _indirectDylibs vector.
  _indirectDylibs.push_back(std::move(files[0]));
  return result;
}


MachODylibFile* MachOLinkingContext::findIndirectDylib(StringRef path) {
  // See if already loaded.
  auto pos = _pathToDylibMap.find(path);
  if (pos != _pathToDylibMap.end())
    return pos->second;

  // Search -L paths if of the form "libXXX.dylib"
  std::pair<StringRef, StringRef> split = path.rsplit('/');
  StringRef leafName = split.second;
  if (leafName.startswith("lib") && leafName.endswith(".dylib")) {
    // FIXME: Need to enhance searchLibrary() to only look for .dylib
    auto libPath = searchLibrary(leafName);
    if (!libPath.getError()) {
      return loadIndirectDylib(libPath.get());
    }
  }

  // Try full path with sysroot.
  for (StringRef sysPath : _syslibRoots) {
    SmallString<256> fullPath;
    fullPath.assign(sysPath);
    llvm::sys::path::append(fullPath, path);
    if (pathExists(fullPath))
      return loadIndirectDylib(fullPath);
  }

  // Try full path.
  if (pathExists(path)) {
    return loadIndirectDylib(path);
  }

  return nullptr;
}

uint32_t MachOLinkingContext::dylibCurrentVersion(StringRef installName) const {
  auto pos = _pathToDylibMap.find(installName);
  if (pos != _pathToDylibMap.end())
    return pos->second->currentVersion();
  else
    return 0x1000; // 1.0
}

uint32_t MachOLinkingContext::dylibCompatVersion(StringRef installName) const {
  auto pos = _pathToDylibMap.find(installName);
  if (pos != _pathToDylibMap.end())
    return pos->second->compatVersion();
  else
    return 0x1000; // 1.0
}

bool MachOLinkingContext::createImplicitFiles(
                            std::vector<std::unique_ptr<File> > &result) {
  // Add indirect dylibs by asking each linked dylib to add its indirects.
  // Iterate until no more dylibs get loaded.
  size_t dylibCount = 0;
  while (dylibCount != _allDylibs.size()) {
    dylibCount = _allDylibs.size();
    for (MachODylibFile *dylib : _allDylibs) {
      dylib->loadReExportedDylibs([this] (StringRef path) -> MachODylibFile* {
                                  return findIndirectDylib(path); });
    }
  }

  // Let writer add output type specific extras.
  return writer().createImplicitFiles(result);
}


void MachOLinkingContext::registerDylib(MachODylibFile *dylib,
                                        bool upward) const {
  _allDylibs.insert(dylib);
  _pathToDylibMap[dylib->installName()] = dylib;
  // If path is different than install name, register path too.
  if (!dylib->path().equals(dylib->installName()))
    _pathToDylibMap[dylib->path()] = dylib;
  if (upward)
    _upwardDylibs.insert(dylib);
}


bool MachOLinkingContext::isUpwardDylib(StringRef installName) const {
  for (MachODylibFile *dylib : _upwardDylibs) {
    if (dylib->installName().equals(installName))
      return true;
  }
  return false;
}

ArchHandler &MachOLinkingContext::archHandler() const {
  if (!_archHandler)
    _archHandler = ArchHandler::create(_arch);
  return *_archHandler;
}


void MachOLinkingContext::addSectionAlignment(StringRef seg, StringRef sect,
                                                               uint8_t align2) {
  SectionAlign entry;
  entry.segmentName = seg;
  entry.sectionName = sect;
  entry.align2 = align2;
  _sectAligns.push_back(entry);
}

bool MachOLinkingContext::sectionAligned(StringRef seg, StringRef sect,
                                                        uint8_t &align2) const {
  for (const SectionAlign &entry : _sectAligns) {
    if (seg.equals(entry.segmentName) && sect.equals(entry.sectionName)) {
      align2 = entry.align2;
      return true;
    }
  }
  return false;
}


void MachOLinkingContext::addExportSymbol(StringRef sym) {
  // Support old crufty export lists with bogus entries.
  if (sym.endswith(".eh") || sym.startswith(".objc_category_name_")) {
    llvm::errs() << "warning: ignoring " << sym << " in export list\n";
    return;
  }
  // Only i386 MacOSX uses old ABI, so don't change those.
  if ((_os != OS::macOSX) || (_arch != arch_x86)) {
    // ObjC has two differnent ABIs.  Be nice and allow one export list work for
    // both ABIs by renaming symbols.
    if (sym.startswith(".objc_class_name_")) {
      std::string abi2className("_OBJC_CLASS_$_");
      abi2className += sym.substr(17);
      _exportedSymbols.insert(copy(abi2className));
      std::string abi2metaclassName("_OBJC_METACLASS_$_");
      abi2metaclassName += sym.substr(17);
      _exportedSymbols.insert(copy(abi2metaclassName));
      return;
    }
  }

  // FIXME: Support wildcards.
  _exportedSymbols.insert(sym);
}

bool MachOLinkingContext::exportSymbolNamed(StringRef sym) const {
  switch (_exportMode) {
  case ExportMode::globals:
    llvm_unreachable("exportSymbolNamed() should not be called in this mode");
    break;
  case ExportMode::whiteList:
    return _exportedSymbols.count(sym);
  case ExportMode::blackList:
    return !_exportedSymbols.count(sym);
  }
  llvm_unreachable("_exportMode unknown enum value");
}

std::string MachOLinkingContext::demangle(StringRef symbolName) const {
  // Only try to demangle symbols if -demangle on command line
  if (!demangleSymbols())
    return symbolName;

  // Only try to demangle symbols that look like C++ symbols
  if (!symbolName.startswith("__Z"))
    return symbolName;

#if defined(HAVE_CXXABI_H)
  SmallString<256> symBuff;
  StringRef nullTermSym = Twine(symbolName).toNullTerminatedStringRef(symBuff);
  // Mach-O has extra leading underscore that needs to be removed.
  const char *cstr = nullTermSym.data() + 1;
  int status;
  char *demangled = abi::__cxa_demangle(cstr, nullptr, nullptr, &status);
  if (demangled != NULL) {
    std::string result(demangled);
    // __cxa_demangle() always uses a malloc'ed buffer to return the result.
    free(demangled);
    return result;
  }
#endif

  return symbolName;
}

std::error_code MachOLinkingContext::createDependencyFile(StringRef path) {
  std::error_code ec;
  _dependencyInfo = std::unique_ptr<llvm::raw_fd_ostream>(new
                         llvm::raw_fd_ostream(path, ec, llvm::sys::fs::F_None));
  if (ec) {
    _dependencyInfo.reset();
    return ec;
  }

  char linkerVersionOpcode = 0x00;
  *_dependencyInfo << linkerVersionOpcode;
  *_dependencyInfo << "lld";     // FIXME
  *_dependencyInfo << '\0';

  return std::error_code();
}

void MachOLinkingContext::addInputFileDependency(StringRef path) const {
  if (!_dependencyInfo)
    return;

  char inputFileOpcode = 0x10;
  *_dependencyInfo << inputFileOpcode;
  *_dependencyInfo << path;
  *_dependencyInfo << '\0';
}

void MachOLinkingContext::addInputFileNotFound(StringRef path) const {
  if (!_dependencyInfo)
    return;

  char inputFileOpcode = 0x11;
  *_dependencyInfo << inputFileOpcode;
  *_dependencyInfo << path;
  *_dependencyInfo << '\0';
}

void MachOLinkingContext::addOutputFileDependency(StringRef path) const {
  if (!_dependencyInfo)
    return;

  char outputFileOpcode = 0x40;
  *_dependencyInfo << outputFileOpcode;
  *_dependencyInfo << path;
  *_dependencyInfo << '\0';
}

void MachOLinkingContext::appendOrderedSymbol(StringRef symbol,
                                              StringRef filename) {
  // To support sorting static functions which may have the same name in
  // multiple .o files, _orderFiles maps the symbol name to a vector
  // of OrderFileNode each of which can specify a file prefix.
  OrderFileNode info;
  if (!filename.empty())
    info.fileFilter = copy(filename);
  info.order = _orderFileEntries++;
  _orderFiles[symbol].push_back(info);
}

bool
MachOLinkingContext::findOrderOrdinal(const std::vector<OrderFileNode> &nodes,
                                      const DefinedAtom *atom,
                                      unsigned &ordinal) {
  const File *objFile = &atom->file();
  assert(objFile);
  StringRef objName = objFile->path();
  std::pair<StringRef, StringRef> dirAndLeaf = objName.rsplit('/');
  if (!dirAndLeaf.second.empty())
    objName = dirAndLeaf.second;
  for (const OrderFileNode &info : nodes) {
    if (info.fileFilter.empty()) {
      // Have unprefixed symbol name in order file that matches this atom.
      ordinal = info.order;
      return true;
    }
    if (info.fileFilter.equals(objName)) {
      // Have prefixed symbol name in order file that matches atom's path.
      ordinal = info.order;
      return true;
    }
  }
  return false;
}

bool MachOLinkingContext::customAtomOrderer(const DefinedAtom *left,
                                            const DefinedAtom *right,
                                            bool &leftBeforeRight) const {
  // No custom sorting if no order file entries.
  if (!_orderFileEntries)
    return false;

  // Order files can only order named atoms.
  StringRef leftName = left->name();
  StringRef rightName = right->name();
  if (leftName.empty() || rightName.empty())
    return false;

  // If neither is in order file list, no custom sorter.
  auto leftPos = _orderFiles.find(leftName);
  auto rightPos = _orderFiles.find(rightName);
  bool leftIsOrdered = (leftPos != _orderFiles.end());
  bool rightIsOrdered = (rightPos != _orderFiles.end());
  if (!leftIsOrdered && !rightIsOrdered)
    return false;

  // There could be multiple symbols with same name but different file prefixes.
  unsigned leftOrder;
  unsigned rightOrder;
  bool foundLeft =
      leftIsOrdered && findOrderOrdinal(leftPos->getValue(), left, leftOrder);
  bool foundRight = rightIsOrdered &&
                    findOrderOrdinal(rightPos->getValue(), right, rightOrder);
  if (!foundLeft && !foundRight)
    return false;

  // If only one is in order file list, ordered one goes first.
  if (foundLeft != foundRight)
    leftBeforeRight = foundLeft;
  else
    leftBeforeRight = (leftOrder < rightOrder);

  return true;
}

static bool isLibrary(const std::unique_ptr<Node> &elem) {
  if (FileNode *node = dyn_cast<FileNode>(const_cast<Node *>(elem.get()))) {
    File *file = node->getFile();
    return isa<SharedLibraryFile>(file) || isa<ArchiveLibraryFile>(file);
  }
  return false;
}

// The darwin linker processes input files in two phases.  The first phase
// links in all object (.o) files in command line order. The second phase
// links in libraries in command line order.
// In this function we reorder the input files so that all the object files
// comes before any library file. We also make a group for the library files
// so that the Resolver will reiterate over the libraries as long as we find
// new undefines from libraries.
void MachOLinkingContext::maybeSortInputFiles() {
  std::vector<std::unique_ptr<Node>> &elements = getNodes();
  std::stable_sort(elements.begin(), elements.end(),
                   [](const std::unique_ptr<Node> &a,
                      const std::unique_ptr<Node> &b) {
                     return !isLibrary(a) && isLibrary(b);
                   });
  size_t numLibs = std::count_if(elements.begin(), elements.end(), isLibrary);
  elements.push_back(llvm::make_unique<GroupEnd>(numLibs));
}

} // end namespace lld
