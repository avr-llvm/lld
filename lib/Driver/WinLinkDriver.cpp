//===- lib/Driver/WinLinkDriver.cpp ---------------------------------------===//
//
//                             The LLVM Linker
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
///
/// \file
///
/// Concrete instance of the Driver for Windows link.exe.
///
//===----------------------------------------------------------------------===//

#include "lld/Driver/Driver.h"
#include "lld/Driver/WinLinkModuleDef.h"
#include "lld/ReaderWriter/PECOFFLinkingContext.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/Optional.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/SmallString.h"
#include "llvm/ADT/StringSwitch.h"
#include "llvm/Object/COFF.h"
#include "llvm/Option/Arg.h"
#include "llvm/Option/Option.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/Path.h"
#include "llvm/Support/Process.h"
#include "llvm/Support/Program.h"
#include "llvm/Support/raw_ostream.h"
#include <algorithm>
#include <cctype>
#include <map>
#include <memory>
#include <sstream>
#include <tuple>

namespace lld {

//
// Option definitions
//

// Create enum with OPT_xxx values for each option in WinLinkOptions.td
enum {
  OPT_INVALID = 0,
#define OPTION(PREFIX, NAME, ID, KIND, GROUP, ALIAS, ALIASARGS, FLAGS, PARAM, \
               HELP, META) \
          OPT_##ID,
#include "WinLinkOptions.inc"
#undef OPTION
};

// Create prefix string literals used in WinLinkOptions.td
#define PREFIX(NAME, VALUE) const char *const NAME[] = VALUE;
#include "WinLinkOptions.inc"
#undef PREFIX

// Create table mapping all options defined in WinLinkOptions.td
static const llvm::opt::OptTable::Info infoTable[] = {
#define OPTION(PREFIX, NAME, ID, KIND, GROUP, ALIAS, ALIASARGS, FLAGS, PARAM, \
               HELPTEXT, METAVAR)   \
  { PREFIX, NAME, HELPTEXT, METAVAR, OPT_##ID, llvm::opt::Option::KIND##Class, \
    PARAM, FLAGS, OPT_##GROUP, OPT_##ALIAS, ALIASARGS },
#include "WinLinkOptions.inc"
#undef OPTION
};

namespace {

// Create OptTable class for parsing actual command line arguments
class WinLinkOptTable : public llvm::opt::OptTable {
public:
  // link.exe's command line options are case insensitive, unlike
  // other driver's options for Unix.
  WinLinkOptTable()
      : OptTable(infoTable, llvm::array_lengthof(infoTable),
                 /* ignoreCase */ true) {}
};

} // anonymous namespace

//
// Functions to parse each command line option
//

// Split the given string with spaces.
static std::vector<std::string> splitArgList(const std::string &str) {
  std::stringstream stream(str);
  std::istream_iterator<std::string> begin(stream);
  std::istream_iterator<std::string> end;
  return std::vector<std::string>(begin, end);
}

// Split the given string with the path separator.
static std::vector<StringRef> splitPathList(StringRef str) {
  std::vector<StringRef> ret;
  while (!str.empty()) {
    StringRef path;
    std::tie(path, str) = str.split(';');
    ret.push_back(path);
  }
  return ret;
}

// Parse an argument for /alternatename. The expected string is
// "<string>=<string>".
static bool parseAlternateName(StringRef arg, StringRef &weak, StringRef &def,
                               raw_ostream &diag) {
  std::tie(weak, def) = arg.split('=');
  if (weak.empty() || def.empty()) {
    diag << "Error: malformed /alternatename option: " << arg << "\n";
    return false;
  }
  return true;
}

// Parse an argument for /base, /stack or /heap. The expected string
// is "<integer>[,<integer>]".
static bool parseMemoryOption(StringRef arg, uint64_t &reserve,
                              uint64_t &commit) {
  StringRef reserveStr, commitStr;
  std::tie(reserveStr, commitStr) = arg.split(',');
  if (reserveStr.getAsInteger(0, reserve))
    return false;
  if (!commitStr.empty() && commitStr.getAsInteger(0, commit))
    return false;
  return true;
}

// Parse an argument for /version or /subsystem. The expected string is
// "<integer>[.<integer>]".
static bool parseVersion(StringRef arg, uint32_t &major, uint32_t &minor) {
  StringRef majorVersion, minorVersion;
  std::tie(majorVersion, minorVersion) = arg.split('.');
  if (minorVersion.empty())
    minorVersion = "0";
  if (majorVersion.getAsInteger(0, major))
    return false;
  if (minorVersion.getAsInteger(0, minor))
    return false;
  return true;
}

// Returns subsystem type for the given string.
static llvm::COFF::WindowsSubsystem stringToWinSubsystem(StringRef str) {
  return llvm::StringSwitch<llvm::COFF::WindowsSubsystem>(str.lower())
      .Case("windows", llvm::COFF::IMAGE_SUBSYSTEM_WINDOWS_GUI)
      .Case("console", llvm::COFF::IMAGE_SUBSYSTEM_WINDOWS_CUI)
      .Case("boot_application",
            llvm::COFF::IMAGE_SUBSYSTEM_WINDOWS_BOOT_APPLICATION)
      .Case("efi_application", llvm::COFF::IMAGE_SUBSYSTEM_EFI_APPLICATION)
      .Case("efi_boot_service_driver",
            llvm::COFF::IMAGE_SUBSYSTEM_EFI_BOOT_SERVICE_DRIVER)
      .Case("efi_rom", llvm::COFF::IMAGE_SUBSYSTEM_EFI_ROM)
      .Case("efi_runtime_driver",
            llvm::COFF::IMAGE_SUBSYSTEM_EFI_RUNTIME_DRIVER)
      .Case("native", llvm::COFF::IMAGE_SUBSYSTEM_NATIVE)
      .Case("posix", llvm::COFF::IMAGE_SUBSYSTEM_POSIX_CUI)
      .Default(llvm::COFF::IMAGE_SUBSYSTEM_UNKNOWN);
}

// Parse /subsystem command line option. The form of /subsystem is
// "subsystem_name[,majorOSVersion[.minorOSVersion]]".
static bool parseSubsystem(StringRef arg,
                           llvm::COFF::WindowsSubsystem &subsystem,
                           llvm::Optional<uint32_t> &major,
                           llvm::Optional<uint32_t> &minor, raw_ostream &diag) {
  StringRef subsystemStr, osVersion;
  std::tie(subsystemStr, osVersion) = arg.split(',');
  if (!osVersion.empty()) {
    uint32_t v1, v2;
    if (!parseVersion(osVersion, v1, v2))
      return false;
    major = v1;
    minor = v2;
  }
  subsystem = stringToWinSubsystem(subsystemStr);
  if (subsystem == llvm::COFF::IMAGE_SUBSYSTEM_UNKNOWN) {
    diag << "error: unknown subsystem name: " << subsystemStr << "\n";
    return false;
  }
  return true;
}

static llvm::COFF::MachineTypes stringToMachineType(StringRef str) {
  // FIXME: we have no way to differentiate between ARM and ARMNT currently.
  // However, given that LLVM only supports ARM NT, default to that for now.
  return llvm::StringSwitch<llvm::COFF::MachineTypes>(str.lower())
      .Case("arm", llvm::COFF::IMAGE_FILE_MACHINE_ARMNT)
      .Case("x64", llvm::COFF::IMAGE_FILE_MACHINE_AMD64)
      .Case("x86", llvm::COFF::IMAGE_FILE_MACHINE_I386)
      .Default(llvm::COFF::IMAGE_FILE_MACHINE_UNKNOWN);
}

// Parse /section:name,[[!]{DEKPRSW}]
//
// /section option is to set non-default bits in the Characteristics fields of
// the section header. D, E, K, P, R, S, and W represent discardable,
// execute, not_cachable, not_pageable, read, shared, and write bits,
// respectively. You can specify multiple flags in one /section option.
//
// If the flag starts with "!", the flags represent a mask that should be turned
// off regardless of the default value. You can even create a section which is
// not readable, writable nor executable with this -- although it's probably
// useless.
static bool parseSection(StringRef option, std::string &section,
                         llvm::Optional<uint32_t> &flags,
                         llvm::Optional<uint32_t> &mask) {
  StringRef flagString;
  std::tie(section, flagString) = option.split(",");

  bool negative = false;
  if (flagString.startswith("!")) {
    negative = true;
    flagString = flagString.substr(1);
  }
  if (flagString.empty())
    return false;

  uint32_t attribs = 0;
  for (size_t i = 0, e = flagString.size(); i < e; ++i) {
    switch (tolower(flagString[i])) {
#define CASE(c, flag)                           \
    case c:                                     \
      attribs |= flag;                          \
      break
    CASE('d', llvm::COFF::IMAGE_SCN_MEM_DISCARDABLE);
    CASE('e', llvm::COFF::IMAGE_SCN_MEM_EXECUTE);
    CASE('k', llvm::COFF::IMAGE_SCN_MEM_NOT_CACHED);
    CASE('p', llvm::COFF::IMAGE_SCN_MEM_NOT_PAGED);
    CASE('r', llvm::COFF::IMAGE_SCN_MEM_READ);
    CASE('s', llvm::COFF::IMAGE_SCN_MEM_SHARED);
    CASE('w', llvm::COFF::IMAGE_SCN_MEM_WRITE);
#undef CASE
    default:
      return false;
    }
  }

  if (negative) {
    mask = attribs;
  } else {
    flags = attribs;
  }
  return true;
}

static bool readFile(PECOFFLinkingContext &ctx, StringRef path,
                     ArrayRef<uint8_t> &result) {
  ErrorOr<std::unique_ptr<MemoryBuffer>> buf = MemoryBuffer::getFile(path);
  if (!buf)
    return false;
  StringRef Data = buf.get()->getBuffer();
  result = ctx.allocate(ArrayRef<uint8_t>(
      reinterpret_cast<const uint8_t *>(Data.begin()), Data.size()));
  return true;
}

// Parse /manifest:EMBED[,ID=#]|NO.
static bool parseManifest(StringRef option, bool &enable, bool &embed,
                          int &id) {
  if (option.equals_lower("no")) {
    enable = false;
    return true;
  }
  if (!option.startswith_lower("embed"))
    return false;

  embed = true;
  option = option.substr(strlen("embed"));
  if (option.empty())
    return true;
  if (!option.startswith_lower(",id="))
    return false;
  option = option.substr(strlen(",id="));
  if (option.getAsInteger(0, id))
    return false;
  return true;
}

static bool isLibraryFile(StringRef path) {
  return path.endswith_lower(".lib") || path.endswith_lower(".imp");
}

static StringRef getObjectPath(PECOFFLinkingContext &ctx, StringRef path) {
  std::string result;
  if (isLibraryFile(path)) {
    result = ctx.searchLibraryFile(path);
  } else if (llvm::sys::path::extension(path).empty()) {
    result = path.str() + ".obj";
  } else {
    result = path;
  }
  return ctx.allocate(result);
}

static StringRef getLibraryPath(PECOFFLinkingContext &ctx, StringRef path) {
  std::string result = isLibraryFile(path)
      ? ctx.searchLibraryFile(path)
      : ctx.searchLibraryFile(path.str() + ".lib");
  return ctx.allocate(result);
}

// Returns true if the given file is a Windows resource file.
static bool isResoruceFile(StringRef path) {
  llvm::sys::fs::file_magic fileType;
  if (llvm::sys::fs::identify_magic(path, fileType)) {
    // If we cannot read the file, assume it's not a resource file.
    // The further stage will raise an error on this unreadable file.
    return false;
  }
  return fileType == llvm::sys::fs::file_magic::windows_resource;
}

// Merge Windows resource files and convert them to a single COFF file.
// The temporary file path is set to result.
static bool convertResourceFiles(PECOFFLinkingContext &ctx,
                                 std::vector<std::string> inFiles,
                                 std::string &result) {
  // Create an output file path.
  SmallString<128> outFile;
  if (llvm::sys::fs::createTemporaryFile("resource", "obj", outFile))
    return false;
  std::string outFileArg = ("/out:" + outFile).str();

  // Construct CVTRES.EXE command line and execute it.
  std::string program = "cvtres.exe";
  ErrorOr<std::string> programPathOrErr = llvm::sys::findProgramByName(program);
  if (!programPathOrErr) {
    llvm::errs() << "Unable to find " << program << " in PATH\n";
    return false;
  }
  const std::string &programPath = *programPathOrErr;

  std::vector<const char *> args;
  args.push_back(programPath.c_str());
  args.push_back(ctx.is64Bit() ? "/machine:x64" : "/machine:x86");
  args.push_back("/readonly");
  args.push_back("/nologo");
  args.push_back(outFileArg.c_str());
  for (const std::string &path : inFiles)
    args.push_back(path.c_str());
  args.push_back(nullptr);

  if (llvm::sys::ExecuteAndWait(programPath.c_str(), &args[0]) != 0) {
    llvm::errs() << program << " failed\n";
    return false;
  }
  result = outFile.str();
  return true;
}

// Parse /manifestuac:(level=<string>|uiAccess=<string>).
//
// The arguments will be embedded to the manifest XML file with no error check,
// so the values given via the command line must be valid as XML attributes.
// This may sound a bit odd, but that's how link.exe works, so we will follow.
static bool parseManifestUAC(StringRef option,
                             llvm::Optional<std::string> &level,
                             llvm::Optional<std::string> &uiAccess) {
  for (;;) {
    option = option.ltrim();
    if (option.empty())
      return true;
    if (option.startswith_lower("level=")) {
      option = option.substr(strlen("level="));
      StringRef value;
      std::tie(value, option) = option.split(" ");
      level = value.str();
      continue;
    }
    if (option.startswith_lower("uiaccess=")) {
      option = option.substr(strlen("uiaccess="));
      StringRef value;
      std::tie(value, option) = option.split(" ");
      uiAccess = value.str();
      continue;
    }
    return false;
  }
}

// Returns the machine type (e.g. x86) of the given input file.
// If the file is not COFF, returns false.
static bool getMachineType(StringRef path, llvm::COFF::MachineTypes &result) {
  llvm::sys::fs::file_magic fileType;
  if (llvm::sys::fs::identify_magic(path, fileType))
    return false;
  if (fileType != llvm::sys::fs::file_magic::coff_object)
    return false;
  ErrorOr<std::unique_ptr<MemoryBuffer>> buf = MemoryBuffer::getFile(path);
  if (!buf)
    return false;
  std::error_code ec;
  llvm::object::COFFObjectFile obj(buf.get()->getMemBufferRef(), ec);
  if (ec)
    return false;
  result = static_cast<llvm::COFF::MachineTypes>(obj.getMachine());
  return true;
}

// Parse /export:entryname[=internalname][,@ordinal[,NONAME]][,DATA][,PRIVATE].
//
// MSDN doesn't say anything about /export:foo=bar style option or PRIVATE
// attribtute, but link.exe actually accepts them.
static bool parseExport(StringRef option,
                        PECOFFLinkingContext::ExportDesc &ret) {
  StringRef name;
  StringRef rest;
  std::tie(name, rest) = option.split(",");
  if (name.empty())
    return false;
  if (name.find('=') == StringRef::npos) {
    ret.name = name;
  } else {
    std::tie(ret.externalName, ret.name) = name.split("=");
    if (ret.name.empty())
      return false;
  }

  for (;;) {
    if (rest.empty())
      return true;
    StringRef arg;
    std::tie(arg, rest) = rest.split(",");
    if (arg.equals_lower("noname")) {
      if (ret.ordinal < 0)
        return false;
      ret.noname = true;
      continue;
    }
    if (arg.equals_lower("data")) {
      ret.isData = true;
      continue;
    }
    if (arg.equals_lower("private")) {
      ret.isPrivate = true;
      continue;
    }
    if (arg.startswith("@")) {
      int ordinal;
      if (arg.substr(1).getAsInteger(0, ordinal))
        return false;
      if (ordinal <= 0 || 65535 < ordinal)
        return false;
      ret.ordinal = ordinal;
      continue;
    }
    return false;
  }
}

// Read module-definition file.
static bool parseDef(StringRef option, llvm::BumpPtrAllocator &alloc,
                     std::vector<moduledef::Directive *> &result) {
  ErrorOr<std::unique_ptr<MemoryBuffer>> buf = MemoryBuffer::getFile(option);
  if (!buf)
    return false;
  moduledef::Lexer lexer(std::move(buf.get()));
  moduledef::Parser parser(lexer, alloc);
  return parser.parse(result);
}

static StringRef replaceExtension(PECOFFLinkingContext &ctx, StringRef path,
                                  StringRef extension) {
  SmallString<128> val = path;
  llvm::sys::path::replace_extension(val, extension);
  return ctx.allocate(val.str());
}

// Create a manifest file contents.
static std::string createManifestXml(PECOFFLinkingContext &ctx) {
  std::string ret;
  llvm::raw_string_ostream out(ret);
  // Emit the XML. Note that we do *not* verify that the XML attributes are
  // syntactically correct. This is intentional for link.exe compatibility.
  out << "<?xml version=\"1.0\" standalone=\"yes\"?>\n"
         "<assembly xmlns=\"urn:schemas-microsoft-com:asm.v1\"\n"
         "          manifestVersion=\"1.0\">\n";
  if (ctx.getManifestUAC()) {
    out << "  <trustInfo>\n"
           "    <security>\n"
           "      <requestedPrivileges>\n"
           "         <requestedExecutionLevel level=" << ctx.getManifestLevel()
        << " uiAccess=" << ctx.getManifestUiAccess()
        << "/>\n"
           "      </requestedPrivileges>\n"
           "    </security>\n"
           "  </trustInfo>\n";
    const std::string &dependency = ctx.getManifestDependency();
    if (!dependency.empty()) {
      out << "  <dependency>\n"
             "    <dependentAssembly>\n"
             "      <assemblyIdentity " << dependency
          << " />\n"
             "    </dependentAssembly>\n"
             "  </dependency>\n";
    }
  }
  out << "</assembly>\n";
  out.flush();
  return ret;
}

// Convert one doublequote to two doublequotes, so that we can embed the string
// into a resource script file.
static void quoteAndPrintXml(raw_ostream &out, StringRef str) {
  for (;;) {
    if (str.empty())
      return;
    StringRef line;
    std::tie(line, str) = str.split("\n");
    if (line.empty())
      continue;
    out << '\"';
    const char *p = line.data();
    for (int i = 0, size = line.size(); i < size; ++i) {
      switch (p[i]) {
      case '\"':
        out << '\"';
        // fallthrough
      default:
        out << p[i];
      }
    }
    out << "\"\n";
  }
}

// Create a resource file (.res file) containing the manifest XML. This is done
// in two steps:
//
//  1. Create a resource script file containing the XML as a literal string.
//  2. Run RC.EXE command to compile the script file to a resource file.
//
// The temporary file created in step 1 will be deleted on exit from this
// function. The file created in step 2 will have the same lifetime as the
// PECOFFLinkingContext.
static bool createManifestResourceFile(PECOFFLinkingContext &ctx,
                                       raw_ostream &diag,
                                       std::string &resFile) {
  // Create a temporary file for the resource script file.
  SmallString<128> rcFileSmallString;
  if (llvm::sys::fs::createTemporaryFile("tmp", "rc", rcFileSmallString)) {
    diag << "Cannot create a temporary file\n";
    return false;
  }
  StringRef rcFile(rcFileSmallString.str());
  llvm::FileRemover rcFileRemover((Twine(rcFile)));

  // Open the temporary file for writing.
  std::error_code ec;
  llvm::raw_fd_ostream out(rcFileSmallString, ec, llvm::sys::fs::F_Text);
  if (ec) {
    diag << "Failed to open " << ctx.getManifestOutputPath() << ": "
         << ec.message() << "\n";
    return false;
  }

  // Write resource script to the RC file.
  out << "#define LANG_ENGLISH 9\n"
      << "#define SUBLANG_DEFAULT 1\n"
      << "#define APP_MANIFEST " << ctx.getManifestId() << "\n"
      << "#define RT_MANIFEST 24\n"
      << "LANGUAGE LANG_ENGLISH, SUBLANG_DEFAULT\n"
      << "APP_MANIFEST RT_MANIFEST {\n";
  quoteAndPrintXml(out, createManifestXml(ctx));
  out << "}\n";
  out.close();

  // Create output resource file.
  SmallString<128> resFileSmallString;
  if (llvm::sys::fs::createTemporaryFile("tmp", "res", resFileSmallString)) {
    diag << "Cannot create a temporary file";
    return false;
  }
  resFile = resFileSmallString.str();

  // Register the resource file path so that the file will be deleted when the
  // context's destructor is called.
  ctx.registerTemporaryFile(resFile);

  // Run RC.EXE /fo tmp.res tmp.rc
  std::string program = "rc.exe";
  ErrorOr<std::string> programPathOrErr = llvm::sys::findProgramByName(program);
  if (!programPathOrErr) {
    diag << "Unable to find " << program << " in PATH\n";
    return false;
  }
  const std::string &programPath = *programPathOrErr;
  std::vector<const char *> args;
  args.push_back(programPath.c_str());
  args.push_back("/fo");
  args.push_back(resFile.c_str());
  args.push_back("/nologo");
  args.push_back(rcFileSmallString.c_str());
  args.push_back(nullptr);

  if (llvm::sys::ExecuteAndWait(programPath.c_str(), &args[0]) != 0) {
    diag << program << " failed\n";
    return false;
  }
  return true;
}


// Create the a side-by-side manifest file.
//
// The manifest file will convey some information to the linker, such as whether
// the binary needs to run as Administrator or not. Instead of being placed in
// the PE/COFF header, it's in XML format for some reason -- I guess it's
// probably because it's invented in the early dot-com era.
//
// The side-by-side manifest file is a separate XML file having ".manifest"
// extension. It will be created in the same directory as the resulting
// executable.
static bool createSideBySideManifestFile(PECOFFLinkingContext &ctx,
                                         raw_ostream &diag) {
  std::string path = ctx.getManifestOutputPath();
  if (path.empty()) {
    // Default name of the manifest file is "foo.exe.manifest" where "foo.exe" is
    // the output path.
    path = ctx.outputPath();
    path.append(".manifest");
  }

  std::error_code ec;
  llvm::raw_fd_ostream out(path, ec, llvm::sys::fs::F_Text);
  if (ec) {
    diag << ec.message() << "\n";
    return false;
  }
  out << createManifestXml(ctx);
  return true;
}

// Handle /failifmismatch option.
static bool
handleFailIfMismatchOption(StringRef option,
                           std::map<StringRef, StringRef> &mustMatch,
                           raw_ostream &diag) {
  StringRef key, value;
  std::tie(key, value) = option.split('=');
  if (key.empty() || value.empty()) {
    diag << "error: malformed /failifmismatch option: " << option << "\n";
    return true;
  }
  auto it = mustMatch.find(key);
  if (it != mustMatch.end() && it->second != value) {
    diag << "error: mismatch detected: '" << it->second << "' and '" << value
         << "' for key '" << key << "'\n";
    return true;
  }
  mustMatch[key] = value;
  return false;
}

//
// Environment variable
//

// Process "LINK" environment variable. If defined, the value of the variable
// should be processed as command line arguments.
static std::vector<const char *> processLinkEnv(PECOFFLinkingContext &ctx,
                                                int argc, const char **argv) {
  std::vector<const char *> ret;
  // The first argument is the name of the command. This should stay at the head
  // of the argument list.
  assert(argc > 0);
  ret.push_back(argv[0]);

  // Add arguments specified by the LINK environment variable.
  llvm::Optional<std::string> env = llvm::sys::Process::GetEnv("LINK");
  if (env.hasValue())
    for (std::string &arg : splitArgList(*env))
      ret.push_back(ctx.allocate(arg).data());

  // Add the rest of arguments passed via the command line.
  for (int i = 1; i < argc; ++i)
    ret.push_back(argv[i]);
  ret.push_back(nullptr);
  return ret;
}

// Process "LIB" environment variable. The variable contains a list of search
// paths separated by semicolons.
static void processLibEnv(PECOFFLinkingContext &ctx) {
  llvm::Optional<std::string> env = llvm::sys::Process::GetEnv("LIB");
  if (env.hasValue())
    for (StringRef path : splitPathList(*env))
      ctx.appendInputSearchPath(ctx.allocate(path));
}

namespace {
class DriverStringSaver : public llvm::cl::StringSaver {
public:
  DriverStringSaver(PECOFFLinkingContext &ctx) : _ctx(ctx) {}

  const char *SaveString(const char *s) override {
    return _ctx.allocate(StringRef(s)).data();
  }

private:
  PECOFFLinkingContext &_ctx;
};
}

// Tokenize command line options in a given file and add them to result.
static bool readResponseFile(StringRef path, PECOFFLinkingContext &ctx,
                             std::vector<const char *> &result) {
  ArrayRef<uint8_t> contents;
  if (!readFile(ctx, path, contents))
    return false;
  StringRef contentsStr(reinterpret_cast<const char *>(contents.data()),
                        contents.size());
  DriverStringSaver saver(ctx);
  SmallVector<const char *, 0> args;
  llvm::cl::TokenizeWindowsCommandLine(contentsStr, saver, args);
  for (const char *s : args)
    result.push_back(s);
  return true;
}

// Expand arguments starting with "@". It's an error if a specified file does
// not exist. Returns true on success.
static bool expandResponseFiles(int &argc, const char **&argv,
                                PECOFFLinkingContext &ctx, raw_ostream &diag,
                                bool &expanded) {
  std::vector<const char *> newArgv;
  for (int i = 0; i < argc; ++i) {
    if (argv[i][0] != '@') {
      newArgv.push_back(argv[i]);
      continue;
    }
    StringRef filename = StringRef(argv[i] + 1);
    if (!readResponseFile(filename, ctx, newArgv)) {
      diag << "error: cannot read response file: " << filename << "\n";
      return false;
    }
    expanded = true;
  }
  if (!expanded)
    return true;
  argc = newArgv.size();
  newArgv.push_back(nullptr);
  argv = &ctx.allocateCopy(newArgv)[0];
  return true;
}

// Parses the given command line options and returns the result. Returns NULL if
// there's an error in the options.
static std::unique_ptr<llvm::opt::InputArgList>
parseArgs(int argc, const char **argv, PECOFFLinkingContext &ctx,
          raw_ostream &diag, bool isReadingDirectiveSection) {
  // Expand arguments starting with "@".
  bool expanded = false;
  if (!expandResponseFiles(argc, argv, ctx, diag, expanded))
    return nullptr;

  // Parse command line options using WinLinkOptions.td
  std::unique_ptr<llvm::opt::InputArgList> parsedArgs;
  WinLinkOptTable table;
  unsigned missingIndex;
  unsigned missingCount;
  parsedArgs.reset(table.ParseArgs(&argv[1], &argv[argc],
                                   missingIndex, missingCount));
  if (missingCount) {
    diag << "error: missing arg value for '"
         << parsedArgs->getArgString(missingIndex) << "' expected "
         << missingCount << " argument(s).\n";
    return nullptr;
  }

  // Show warning for unknown arguments. In .drectve section, unknown options
  // starting with "-?" are silently ignored. This is a COFF's feature to embed a
  // new linker option to an object file while keeping backward compatibility.
  for (auto unknownArg : parsedArgs->filtered(OPT_UNKNOWN)) {
    StringRef arg = unknownArg->getSpelling();
    if (isReadingDirectiveSection && arg.startswith("-?"))
      continue;
    diag << "warning: ignoring unknown argument: " << arg << "\n";
  }

  // Copy mllvm
  for (auto arg : parsedArgs->filtered(OPT_mllvm))
    ctx.appendLLVMOption(arg->getValue());

  // If we have expaneded response files and /verbose is given, print out the
  // final command line.
  if (!isReadingDirectiveSection && expanded &&
      parsedArgs->getLastArg(OPT_verbose)) {
    diag << "Command line:";
    for (int i = 0; i < argc; ++i)
      diag << " " << argv[i];
    diag << "\n\n";
  }

  return parsedArgs;
}

// Returns true if the given file node has already been added to the input
// graph.
static bool hasLibrary(PECOFFLinkingContext &ctx, File *file) {
  StringRef path = file->path();
  std::vector<std::unique_ptr<Node>> &nodes = ctx.getNodes();
  for (size_t i = 0; i < nodes.size(); ++i)
    if (auto *f = dyn_cast<FileNode>(nodes[i].get()))
      if (f->getFile()->path() == path)
        return true;
  return false;
}

// If the first command line argument is "/lib", link.exe acts as if it's
// "lib.exe" command. This is for backward compatibility.
// http://msdn.microsoft.com/en-us/library/h34w59b3.aspx
static bool maybeRunLibCommand(int argc, const char **argv, raw_ostream &diag) {
  if (argc <= 1)
    return false;
  if (!StringRef(argv[1]).equals_lower("/lib"))
    return false;
  ErrorOr<std::string> pathOrErr = llvm::sys::findProgramByName("lib.exe");
  if (!pathOrErr) {
    diag << "Unable to find lib.exe in PATH\n";
    return true;
  }
  const std::string &path = *pathOrErr;

  // Run lib.exe
  std::vector<const char *> vec;
  vec.push_back(path.c_str());
  for (int i = 2; i < argc; ++i)
    vec.push_back(argv[i]);
  vec.push_back(nullptr);

  if (llvm::sys::ExecuteAndWait(path.c_str(), &vec[0]) != 0)
    diag << "lib.exe failed\n";
  return true;
}

/// \brief Parse the input file to lld::File.
void addFiles(PECOFFLinkingContext &ctx, StringRef path, raw_ostream &diag,
              std::vector<std::unique_ptr<File>> &files) {
  for (std::unique_ptr<File> &file : loadFile(ctx, path, false)) {
    if (ctx.logInputFiles())
      diag << file->path() << "\n";
    files.push_back(std::move(file));
  }
}

//
// Main driver
//

bool WinLinkDriver::linkPECOFF(int argc, const char **argv, raw_ostream &diag) {
  if (maybeRunLibCommand(argc, argv, diag))
    return true;

  PECOFFLinkingContext ctx;
  ctx.setParseDirectives(parseDirectives);
  ctx.registry().addSupportCOFFObjects(ctx);
  ctx.registry().addSupportCOFFImportLibraries(ctx);
  ctx.registry().addSupportArchives(ctx.logInputFiles());
  ctx.registry().addSupportNativeObjects();
  ctx.registry().addSupportYamlFiles();

  std::vector<const char *> newargv = processLinkEnv(ctx, argc, argv);
  processLibEnv(ctx);
  if (!parse(newargv.size() - 1, &newargv[0], ctx, diag))
    return false;

  // Create the file if needed.
  if (ctx.getCreateManifest() && !ctx.getEmbedManifest())
    if (!createSideBySideManifestFile(ctx, diag))
      return false;

  return link(ctx, diag);
}

bool WinLinkDriver::parse(int argc, const char *argv[],
                          PECOFFLinkingContext &ctx, raw_ostream &diag,
                          bool isReadingDirectiveSection) {
  // Parse may be called from multiple threads simultaneously to parse .drectve
  // sections. This function is not thread-safe because it mutates the context
  // object. So acquire the lock.
  std::lock_guard<std::recursive_mutex> lock(ctx.getMutex());

  std::map<StringRef, StringRef> failIfMismatchMap;
  // Parse the options.
  std::unique_ptr<llvm::opt::InputArgList> parsedArgs =
      parseArgs(argc, argv, ctx, diag, isReadingDirectiveSection);
  if (!parsedArgs)
    return false;

  // The list of input files.
  std::vector<std::unique_ptr<File>> files;
  std::vector<std::unique_ptr<File>> libraries;

  // Handle /help
  if (parsedArgs->hasArg(OPT_help)) {
    WinLinkOptTable table;
    table.PrintHelp(llvm::outs(), argv[0], "LLVM Linker", false);
    return false;
  }

  // Handle /machine before parsing all the other options, as the target machine
  // type affects how to handle other options. For example, x86 needs the
  // leading underscore to mangle symbols, while x64 doesn't need it.
  if (llvm::opt::Arg *inputArg = parsedArgs->getLastArg(OPT_machine)) {
    StringRef arg = inputArg->getValue();
    llvm::COFF::MachineTypes type = stringToMachineType(arg);
    if (type == llvm::COFF::IMAGE_FILE_MACHINE_UNKNOWN) {
      diag << "error: unknown machine type: " << arg << "\n";
      return false;
    }
    ctx.setMachineType(type);
  } else {
    // If /machine option is missing, we need to take a look at
    // the magic byte of the first object file to infer machine type.
    std::vector<StringRef> filePaths;
    for (auto arg : *parsedArgs)
      if (arg->getOption().getID() == OPT_INPUT)
        filePaths.push_back(arg->getValue());
    if (llvm::opt::Arg *arg = parsedArgs->getLastArg(OPT_DASH_DASH))
      filePaths.insert(filePaths.end(), arg->getValues().begin(),
                   arg->getValues().end());
    for (StringRef path : filePaths) {
      llvm::COFF::MachineTypes type;
      if (!getMachineType(path, type))
        continue;
      if (type == llvm::COFF::IMAGE_FILE_MACHINE_UNKNOWN)
        continue;
      ctx.setMachineType(type);
      break;
    }
  }

  // Handle /nodefaultlib:<lib>. The same option without argument is handled in
  // the following for loop.
  for (auto *arg : parsedArgs->filtered(OPT_nodefaultlib))
    ctx.addNoDefaultLib(arg->getValue());

  // Handle /defaultlib. Argument of the option is added to the input file list
  // unless it's blacklisted by /nodefaultlib.
  std::vector<StringRef> defaultLibs;
  for (auto *arg : parsedArgs->filtered(OPT_defaultlib))
    defaultLibs.push_back(arg->getValue());

  // -alternatename:<alias>=<symbol>
  for (auto *arg : parsedArgs->filtered(OPT_alternatename)) {
    StringRef weak, def;
    if (!parseAlternateName(arg->getValue(), weak, def, diag))
      return false;
    ctx.addAlternateName(weak, def);
  }

  // Parse /base command line option. The argument for the parameter is in
  // the form of "<address>[:<size>]".
  if (auto *arg = parsedArgs->getLastArg(OPT_base)) {
      uint64_t addr, size;
      // Size should be set to SizeOfImage field in the COFF header, and if
      // it's smaller than the actual size, the linker should warn about that.
      // Currently we just ignore the value of size parameter.
      if (!parseMemoryOption(arg->getValue(), addr, size))
        return false;
      ctx.setBaseAddress(addr);
  }

  // Parse /dll command line option
  if (parsedArgs->hasArg(OPT_dll)) {
    ctx.setIsDll(true);
    // Default base address of a DLL is 0x10000000.
    if (!parsedArgs->hasArg(OPT_base))
      ctx.setBaseAddress(0x10000000);
  }

  // Parse /stack command line option
  if (auto *arg = parsedArgs->getLastArg(OPT_stack)) {
    uint64_t reserve;
    uint64_t commit = ctx.getStackCommit();
    if (!parseMemoryOption(arg->getValue(), reserve, commit))
      return false;
    ctx.setStackReserve(reserve);
    ctx.setStackCommit(commit);
  }

  // Parse /heap command line option
  if (auto *arg = parsedArgs->getLastArg(OPT_heap)) {
    uint64_t reserve;
    uint64_t commit = ctx.getHeapCommit();
    if (!parseMemoryOption(arg->getValue(), reserve, commit))
      return false;
    ctx.setHeapReserve(reserve);
    ctx.setHeapCommit(commit);
  }

  if (auto *arg = parsedArgs->getLastArg(OPT_align)) {
    uint32_t align;
    StringRef val = arg->getValue();
    if (val.getAsInteger(10, align)) {
      diag << "error: invalid value for /align: " << val << "\n";
      return false;
    }
    ctx.setSectionDefaultAlignment(align);
  }

  if (auto *arg = parsedArgs->getLastArg(OPT_version)) {
    uint32_t major, minor;
    if (!parseVersion(arg->getValue(), major, minor))
      return false;
    ctx.setImageVersion(PECOFFLinkingContext::Version(major, minor));
  }

  // Parse /merge:<from>=<to>.
  for (auto *arg : parsedArgs->filtered(OPT_merge)) {
    StringRef from, to;
    std::tie(from, to) = StringRef(arg->getValue()).split('=');
    if (from.empty() || to.empty()) {
      diag << "error: malformed /merge option: " << arg->getValue() << "\n";
      return false;
    }
    if (!ctx.addSectionRenaming(diag, from, to))
      return false;
  }

  // Parse /subsystem:<subsystem>[,<majorOSVersion>[.<minorOSVersion>]].
  if (auto *arg = parsedArgs->getLastArg(OPT_subsystem)) {
    llvm::COFF::WindowsSubsystem subsystem;
    llvm::Optional<uint32_t> major, minor;
    if (!parseSubsystem(arg->getValue(), subsystem, major, minor, diag))
      return false;
    ctx.setSubsystem(subsystem);
    if (major.hasValue())
      ctx.setMinOSVersion(PECOFFLinkingContext::Version(*major, *minor));
  }

  // Parse /section:name,[[!]{DEKPRSW}]
  for (auto *arg : parsedArgs->filtered(OPT_section)) {
    std::string section;
    llvm::Optional<uint32_t> flags, mask;
    if (!parseSection(arg->getValue(), section, flags, mask)) {
      diag << "Unknown argument for /section: " << arg->getValue() << "\n";
      return false;
    }
    if (flags.hasValue())
      ctx.setSectionSetMask(section, *flags);
    if (mask.hasValue())
      ctx.setSectionClearMask(section, *mask);
  }

  // Parse /manifest:EMBED[,ID=#]|NO.
  if (auto *arg = parsedArgs->getLastArg(OPT_manifest_colon)) {
    bool enable = true;
    bool embed = false;
    int id = 1;
    if (!parseManifest(arg->getValue(), enable, embed, id)) {
      diag << "Unknown argument for /manifest: " << arg->getValue() << "\n";
      return false;
    }
    ctx.setCreateManifest(enable);
    ctx.setEmbedManifest(embed);
    ctx.setManifestId(id);
  }

  // Parse /manifestuac.
  if (auto *arg = parsedArgs->getLastArg(OPT_manifestuac)) {
    if (StringRef(arg->getValue()).equals_lower("no")) {
      ctx.setManifestUAC(false);
    } else {
      llvm::Optional<std::string> privilegeLevel;
      llvm::Optional<std::string> uiAccess;
      if (!parseManifestUAC(arg->getValue(), privilegeLevel, uiAccess)) {
        diag << "Unknown argument for /manifestuac: " << arg->getValue()
             << "\n";
        return false;
      }
      if (privilegeLevel.hasValue())
        ctx.setManifestLevel(privilegeLevel.getValue());
      if (uiAccess.hasValue())
        ctx.setManifestUiAccess(uiAccess.getValue());
    }
  }

  if (auto *arg = parsedArgs->getLastArg(OPT_manifestfile))
    ctx.setManifestOutputPath(ctx.allocate(arg->getValue()));

  // /manifestdependency:<string> option. Note that the argument will be
  // embedded to the manifest XML file with no error check, for link.exe
  // compatibility. We do not gurantete that the resulting XML file is
  // valid.
  if (auto *arg = parsedArgs->getLastArg(OPT_manifestdependency))
    ctx.setManifestDependency(ctx.allocate(arg->getValue()));

  for (auto *arg : parsedArgs->filtered(OPT_failifmismatch))
    if (handleFailIfMismatchOption(arg->getValue(), failIfMismatchMap, diag))
      return false;

  if (auto *arg = parsedArgs->getLastArg(OPT_entry))
    ctx.setEntrySymbolName(ctx.allocate(arg->getValue()));

  for (auto *arg : parsedArgs->filtered(OPT_export)) {
    PECOFFLinkingContext::ExportDesc desc;
    if (!parseExport(arg->getValue(), desc)) {
      diag << "Error: malformed /export option: " << arg->getValue() << "\n";
      return false;
    }

    // Mangle the symbol name only if it is reading user-supplied command line
    // arguments. Because the symbol name in the .drectve section is already
    // mangled by the compiler, we shouldn't add a leading underscore in that
    // case. It's odd that the command line option has different semantics in
    // the .drectve section, but this behavior is needed for compatibility
    // with MSVC's link.exe.
    if (!isReadingDirectiveSection)
      desc.name = ctx.decorateSymbol(desc.name);
    ctx.addDllExport(desc);
  }

  for (auto *arg : parsedArgs->filtered(OPT_deffile)) {
    llvm::BumpPtrAllocator alloc;
    std::vector<moduledef::Directive *> dirs;
    if (!parseDef(arg->getValue(), alloc, dirs)) {
      diag << "Error: invalid module-definition file\n";
      return false;
    }
    for (moduledef::Directive *dir : dirs) {
      if (auto *exp = dyn_cast<moduledef::Exports>(dir)) {
        for (PECOFFLinkingContext::ExportDesc desc : exp->getExports()) {
          desc.name = ctx.decorateSymbol(desc.name);
          ctx.addDllExport(desc);
        }
      } else if (auto *hs = dyn_cast<moduledef::Heapsize>(dir)) {
        ctx.setHeapReserve(hs->getReserve());
        ctx.setHeapCommit(hs->getCommit());
      } else if (auto *lib = dyn_cast<moduledef::Library>(dir)) {
        ctx.setIsDll(true);
        ctx.setOutputPath(ctx.allocate(lib->getName()));
        if (lib->getBaseAddress() && !ctx.getBaseAddress())
          ctx.setBaseAddress(lib->getBaseAddress());
      } else if (auto *name = dyn_cast<moduledef::Name>(dir)) {
        if (!name->getOutputPath().empty() && ctx.outputPath().empty())
          ctx.setOutputPath(ctx.allocate(name->getOutputPath()));
        if (name->getBaseAddress() && ctx.getBaseAddress())
          ctx.setBaseAddress(name->getBaseAddress());
      } else if (auto *ver = dyn_cast<moduledef::Version>(dir)) {
        ctx.setImageVersion(PECOFFLinkingContext::Version(
                              ver->getMajorVersion(), ver->getMinorVersion()));
      } else {
        llvm::dbgs() << static_cast<int>(dir->getKind()) << "\n";
        llvm_unreachable("Unknown module-definition directive.\n");
      }
    }
  }

  for (auto *arg : parsedArgs->filtered(OPT_libpath))
    ctx.appendInputSearchPath(ctx.allocate(arg->getValue()));

  for (auto *arg : parsedArgs->filtered(OPT_opt)) {
    std::string val = StringRef(arg->getValue()).lower();
    if (val == "noref") {
      ctx.setDeadStripping(false);
    } else if (val != "ref" && val != "icf" && val != "noicf" &&
	       val != "lbr" && val != "nolbr" &&
               !StringRef(val).startswith("icf=")) {
      diag << "unknown option for /opt: " << val << "\n";
      return false;
    }
  }

  // LLD is not yet capable of creating a PDB file, so /debug does not have
  // any effect.
  // TODO: This should disable dead stripping. Currently we can't do that
  // because removal of associative sections depends on dead stripping.
  if (parsedArgs->hasArg(OPT_debug))
    ctx.setDebug(true);

  if (parsedArgs->hasArg(OPT_verbose))
    ctx.setLogInputFiles(true);

  // /force and /force:unresolved mean the same thing. We do not currently
  // support /force:multiple.
  if (parsedArgs->hasArg(OPT_force) ||
      parsedArgs->hasArg(OPT_force_unresolved)) {
    ctx.setAllowRemainingUndefines(true);
  }

  if (parsedArgs->hasArg(OPT_fixed)) {
    // /fixed is not compatible with /dynamicbase. Check for it.
    if (parsedArgs->hasArg(OPT_dynamicbase)) {
      diag << "/dynamicbase must not be specified with /fixed\n";
      return false;
    }
    ctx.setBaseRelocationEnabled(false);
    ctx.setDynamicBaseEnabled(false);
  }

  // /swaprun:{cd,net} options set IMAGE_FILE_{REMOVABLE,NET}_RUN_FROM_SWAP
  // bits in the COFF header, respectively. If one of the bits is on, the
  // Windows loader will copy the entire file to swap area then execute it,
  // so that the user can eject a CD or disconnect from the network.
  if (parsedArgs->hasArg(OPT_swaprun_cd))
    ctx.setSwapRunFromCD(true);

  if (parsedArgs->hasArg(OPT_swaprun_net))
    ctx.setSwapRunFromNet(true);

  if (parsedArgs->hasArg(OPT_profile)) {
    // /profile implies /opt:ref, /opt:noicf, /incremental:no and /fixed:no.
    ctx.setDeadStripping(true);
    ctx.setBaseRelocationEnabled(true);
    ctx.setDynamicBaseEnabled(true);
  }

  for (auto *arg : parsedArgs->filtered(OPT_implib))
    ctx.setOutputImportLibraryPath(arg->getValue());

  for (auto *arg : parsedArgs->filtered(OPT_delayload)) {
    ctx.addInitialUndefinedSymbol(ctx.getDelayLoadHelperName());
    ctx.addDelayLoadDLL(arg->getValue());
  }

  if (auto *arg = parsedArgs->getLastArg(OPT_stub)) {
    ArrayRef<uint8_t> contents;
    if (!readFile(ctx, arg->getValue(), contents)) {
      diag << "Failed to read DOS stub file " << arg->getValue() << "\n";
      return false;
    }
    ctx.setDosStub(contents);
  }

  for (auto *arg : parsedArgs->filtered(OPT_incl))
    ctx.addInitialUndefinedSymbol(ctx.allocate(arg->getValue()));

  if (parsedArgs->hasArg(OPT_noentry))
    ctx.setHasEntry(false);

  if (parsedArgs->hasArg(OPT_nodefaultlib_all))
    ctx.setNoDefaultLibAll(true);

  if (auto *arg = parsedArgs->getLastArg(OPT_out))
    ctx.setOutputPath(ctx.allocate(arg->getValue()));

  if (auto *arg = parsedArgs->getLastArg(OPT_pdb))
    ctx.setPDBFilePath(arg->getValue());

  if (auto *arg = parsedArgs->getLastArg(OPT_lldmoduledeffile))
    ctx.setModuleDefinitionFile(arg->getValue());

  std::vector<StringRef> inputFiles;
  for (auto *arg : parsedArgs->filtered(OPT_INPUT))
    inputFiles.push_back(ctx.allocate(arg->getValue()));

#define BOOLEAN_FLAG(name, setter) \
  if (auto *arg = parsedArgs->getLastArg(OPT_##name, OPT_##name##_no)) \
    ctx.setter(arg->getOption().matches(OPT_##name));

  BOOLEAN_FLAG(nxcompat, setNxCompat);
  BOOLEAN_FLAG(largeaddressaware, setLargeAddressAware);
  BOOLEAN_FLAG(allowbind, setAllowBind);
  BOOLEAN_FLAG(allowisolation, setAllowIsolation);
  BOOLEAN_FLAG(dynamicbase, setDynamicBaseEnabled);
  BOOLEAN_FLAG(tsaware, setTerminalServerAware);
  BOOLEAN_FLAG(highentropyva, setHighEntropyVA);
  BOOLEAN_FLAG(safeseh, setSafeSEH);
#undef BOOLEAN_FLAG

  // Arguments after "--" are interpreted as filenames even if they
  // start with a hypen or a slash. This is not compatible with link.exe
  // but useful for us to test lld on Unix.
  if (llvm::opt::Arg *dashdash = parsedArgs->getLastArg(OPT_DASH_DASH))
    for (const StringRef value : dashdash->getValues())
      inputFiles.push_back(value);

  // Compile Windows resource files to compiled resource file.
  if (ctx.getCreateManifest() && ctx.getEmbedManifest() &&
      !isReadingDirectiveSection) {
    std::string resFile;
    if (!createManifestResourceFile(ctx, diag, resFile))
      return false;
    inputFiles.push_back(ctx.allocate(resFile));
  }

  // A Windows Resource file is not an object file. It contains data,
  // such as an icon image, and is not in COFF file format. If resource
  // files are given, the linker merge them into one COFF file using
  // CVTRES.EXE and then link the resulting file.
  {
    auto it = std::partition(inputFiles.begin(), inputFiles.end(),
                             isResoruceFile);
    if (it != inputFiles.begin()) {
      std::vector<std::string> resFiles(inputFiles.begin(), it);
      std::string resObj;
      if (!convertResourceFiles(ctx, resFiles, resObj)) {
        diag << "Failed to convert resource files\n";
        return false;
      }
      inputFiles = std::vector<StringRef>(it, inputFiles.end());
      inputFiles.push_back(ctx.allocate(resObj));
      ctx.registerTemporaryFile(resObj);
    }
  }

  // Prepare objects to add them to the list of input files.
  for (StringRef path : inputFiles) {
    path = ctx.allocate(path);
    if (isLibraryFile(path)) {
      addFiles(ctx, getLibraryPath(ctx, path), diag, libraries);
    } else {
      addFiles(ctx, getObjectPath(ctx, path), diag, files);
    }
  }

  // If dead-stripping is enabled, we need to add the entry symbol and
  // symbols given by /include to the dead strip root set, so that it
  // won't be removed from the output.
  if (ctx.deadStrip())
    for (const StringRef symbolName : ctx.initialUndefinedSymbols())
      ctx.addDeadStripRoot(symbolName);

  // Add the libraries specified by /defaultlib unless they are already added
  // nor blacklisted by /nodefaultlib.
  if (!ctx.getNoDefaultLibAll())
    for (const StringRef path : defaultLibs)
      if (!ctx.hasNoDefaultLib(path))
        addFiles(ctx, getLibraryPath(ctx, path.lower()), diag, libraries);

  if (files.empty() && !isReadingDirectiveSection) {
    diag << "No input files\n";
    return false;
  }

  // If /out option was not specified, the default output file name is
  // constructed by replacing an extension of the first input file
  // with ".exe".
  if (ctx.outputPath().empty()) {
    StringRef path = files[0]->path();
    ctx.setOutputPath(replaceExtension(ctx, path, ".exe"));
  }

  // Add the input files to the linking context.
  for (std::unique_ptr<File> &file : files) {
    if (isReadingDirectiveSection)
      file.get()->parse();
    ctx.getNodes().push_back(llvm::make_unique<FileNode>(std::move(file)));
  }

  // Add the library group to the linking context.
  if (!isReadingDirectiveSection) {
    // Add a group-end marker.
    ctx.getNodes().push_back(llvm::make_unique<GroupEnd>(0));
  }

  // Add the library files to the library group.
  for (std::unique_ptr<File> &file : libraries) {
    if (!hasLibrary(ctx, file.get())) {
      if (isReadingDirectiveSection)
        file.get()->parse();
      ctx.addLibraryFile(llvm::make_unique<FileNode>(std::move(file)));
    }
  }

  // Validate the combination of options used.
  return ctx.validate(diag);
}

} // namespace lld
