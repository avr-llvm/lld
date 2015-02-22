//===- ReaderWriter/LinkerScript.cpp --------------------------------------===//
//
//                             The LLVM Linker
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
///
/// \file
/// \brief Linker script parser.
///
//===----------------------------------------------------------------------===//

#include "lld/ReaderWriter/LinkerScript.h"

namespace lld {
namespace script {
void Token::dump(raw_ostream &os) const {
  switch (_kind) {
#define CASE(name)                                                             \
  case Token::name:                                                            \
    os << #name ": ";                                                          \
    break;
    CASE(unknown)
    CASE(eof)
    CASE(exclaim)
    CASE(exclaimequal)
    CASE(amp)
    CASE(ampequal)
    CASE(l_paren)
    CASE(r_paren)
    CASE(star)
    CASE(starequal)
    CASE(plus)
    CASE(plusequal)
    CASE(comma)
    CASE(minus)
    CASE(minusequal)
    CASE(slash)
    CASE(slashequal)
    CASE(number)
    CASE(colon)
    CASE(semicolon)
    CASE(less)
    CASE(lessequal)
    CASE(lessless)
    CASE(lesslessequal)
    CASE(equal)
    CASE(equalequal)
    CASE(greater)
    CASE(greaterequal)
    CASE(greatergreater)
    CASE(greatergreaterequal)
    CASE(question)
    CASE(identifier)
    CASE(libname)
    CASE(kw_align)
    CASE(kw_align_with_input)
    CASE(kw_as_needed)
    CASE(kw_at)
    CASE(kw_discard)
    CASE(kw_entry)
    CASE(kw_exclude_file)
    CASE(kw_group)
    CASE(kw_hidden)
    CASE(kw_input)
    CASE(kw_keep)
    CASE(kw_provide)
    CASE(kw_provide_hidden)
    CASE(kw_only_if_ro)
    CASE(kw_only_if_rw)
    CASE(kw_output)
    CASE(kw_output_arch)
    CASE(kw_output_format)
    CASE(kw_overlay)
    CASE(kw_search_dir)
    CASE(kw_sections)
    CASE(kw_sort_by_alignment)
    CASE(kw_sort_by_init_priority)
    CASE(kw_sort_by_name)
    CASE(kw_sort_none)
    CASE(kw_subalign)
    CASE(l_brace)
    CASE(pipe)
    CASE(pipeequal)
    CASE(r_brace)
    CASE(tilde)
#undef CASE
  }
  os << _range << "\n";
}

static llvm::ErrorOr<uint64_t> parseDecimal(StringRef str) {
  uint64_t res = 0;
  for (auto &c : str) {
    res *= 10;
    if (c < '0' || c > '9')
      return llvm::ErrorOr<uint64_t>(std::make_error_code(std::errc::io_error));
    res += c - '0';
  }
  return res;
}

static llvm::ErrorOr<uint64_t> parseOctal(StringRef str) {
  uint64_t res = 0;
  for (auto &c : str) {
    res <<= 3;
    if (c < '0' || c > '7')
      return llvm::ErrorOr<uint64_t>(std::make_error_code(std::errc::io_error));
    res += c - '0';
  }
  return res;
}

static llvm::ErrorOr<uint64_t> parseBinary(StringRef str) {
  uint64_t res = 0;
  for (auto &c : str) {
    res <<= 1;
    if (c != '0' && c != '1')
      return llvm::ErrorOr<uint64_t>(std::make_error_code(std::errc::io_error));
    res += c - '0';
  }
  return res;
}

static llvm::ErrorOr<uint64_t> parseHex(StringRef str) {
  uint64_t res = 0;
  for (auto &c : str) {
    res <<= 4;
    if (c >= '0' && c <= '9')
      res += c - '0';
    else if (c >= 'a' && c <= 'f')
      res += c - 'a' + 10;
    else if (c >= 'A' && c <= 'F')
      res += c - 'A' + 10;
    else
      return llvm::ErrorOr<uint64_t>(std::make_error_code(std::errc::io_error));
  }
  return res;
}

static bool parseHexToByteStream(StringRef str, std::string &buf) {
  unsigned char byte = 0;
  bool dumpByte = str.size() % 2;
  for (auto &c : str) {
    byte <<= 4;
    if (c >= '0' && c <= '9')
      byte += c - '0';
    else if (c >= 'a' && c <= 'f')
      byte += c - 'a' + 10;
    else if (c >= 'A' && c <= 'F')
      byte += c - 'A' + 10;
    else
      return false;
    if (!dumpByte) {
      dumpByte = true;
      continue;
    }
    buf.push_back(byte);
    byte = 0;
    dumpByte = false;
  }
  return !dumpByte;
}

static void dumpByteStream(raw_ostream &os, StringRef stream) {
  os << "0x";
  for (auto &c : stream) {
    unsigned char firstNibble = c >> 4 & 0xF;
    if (firstNibble > 9)
      os << (char) ('A' + firstNibble - 10);
    else
      os << (char) ('0' + firstNibble);
    unsigned char secondNibble = c & 0xF;
    if (secondNibble > 9)
      os << (char) ('A' + secondNibble - 10);
    else
      os << (char) ('0' + secondNibble);
  }
}

static llvm::ErrorOr<uint64_t> parseNum(StringRef str) {
  unsigned multiplier = 1;
  enum NumKind { decimal, hex, octal, binary };
  NumKind kind = llvm::StringSwitch<NumKind>(str)
                     .StartsWith("0x", hex)
                     .StartsWith("0X", hex)
                     .StartsWith("0", octal)
                     .Default(decimal);

  // Parse scale
  if (str.endswith("K")) {
    multiplier = 1 << 10;
    str = str.drop_back();
  } else if (str.endswith("M")) {
    multiplier = 1 << 20;
    str = str.drop_back();
  }

  // Parse type
  if (str.endswith_lower("o")) {
    kind = octal;
    str = str.drop_back();
  } else if (str.endswith_lower("h")) {
    kind = hex;
    str = str.drop_back();
  } else if (str.endswith_lower("d")) {
    kind = decimal;
    str = str.drop_back();
  } else if (str.endswith_lower("b")) {
    kind = binary;
    str = str.drop_back();
  }

  llvm::ErrorOr<uint64_t> res(0);
  switch (kind) {
  case hex:
    if (str.startswith_lower("0x"))
      str = str.drop_front(2);
    res = parseHex(str);
    break;
  case octal:
    res = parseOctal(str);
    break;
  case decimal:
    res = parseDecimal(str);
    break;
  case binary:
    res = parseBinary(str);
    break;
  }
  if (res.getError())
    return res;

  *res = *res * multiplier;
  return res;
}

bool Lexer::canStartNumber(char c) const {
  return '0' <= c && c <= '9';
}

bool Lexer::canContinueNumber(char c) const {
  // [xX] = hex marker, [hHoO] = type suffix, [MK] = scale suffix.
  return strchr("0123456789ABCDEFabcdefxXhHoOMK", c);
}

bool Lexer::canStartName(char c) const {
  return strchr(
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz_.$/\\*", c);
}

bool Lexer::canContinueName(char c) const {
  return strchr("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz"
                "0123456789_.$/\\~=+[]*?-:", c);
}

/// Helper function to split a StringRef in two at the nth character.
/// The StringRef s is updated, while the function returns the n first
/// characters.
static StringRef drop(StringRef &s, int n) {
  StringRef res = s.substr(0, n);
  s = s.drop_front(n);
  return res;
}

void Lexer::lex(Token &tok) {
  skipWhitespace();
  if (_buffer.empty()) {
    tok = Token(_buffer, Token::eof);
    return;
  }
  switch (_buffer[0]) {
  case 0:
    tok = Token(drop(_buffer, 1), Token::eof);
    return;
  case '(':
    tok = Token(drop(_buffer, 1), Token::l_paren);
    return;
  case ')':
    tok = Token(drop(_buffer, 1), Token::r_paren);
    return;
  case '{':
    tok = Token(drop(_buffer, 1), Token::l_brace);
    return;
  case '}':
    tok = Token(drop(_buffer, 1), Token::r_brace);
    return;
  case '=':
    if (_buffer.startswith("==")) {
      tok = Token(drop(_buffer, 2), Token::equalequal);
      return;
    }
    tok = Token(drop(_buffer, 1), Token::equal);
    return;
  case '!':
    if (_buffer.startswith("!=")) {
      tok = Token(drop(_buffer, 2), Token::exclaimequal);
      return;
    }
    tok = Token(drop(_buffer, 1), Token::exclaim);
    return;
  case ',':
    tok = Token(drop(_buffer, 1), Token::comma);
    return;
  case ';':
    tok = Token(drop(_buffer, 1), Token::semicolon);
    return;
  case ':':
    tok = Token(drop(_buffer, 1), Token::colon);
    return;
  case '&':
    if (_buffer.startswith("&=")) {
      tok = Token(drop(_buffer, 2), Token::ampequal);
      return;
    }
    tok = Token(drop(_buffer, 1), Token::amp);
    return;
  case '|':
    if (_buffer.startswith("|=")) {
      tok = Token(drop(_buffer, 2), Token::pipeequal);
      return;
    }
    tok = Token(drop(_buffer, 1), Token::pipe);
    return;
  case '+':
    if (_buffer.startswith("+=")) {
      tok = Token(drop(_buffer, 2), Token::plusequal);
      return;
    }
    tok = Token(drop(_buffer, 1), Token::plus);
    return;
  case '-': {
    if (_buffer.startswith("-=")) {
      tok = Token(drop(_buffer, 2), Token::minusequal);
      return;
    }
    if (!_buffer.startswith("-l")) {
      tok = Token(drop(_buffer, 1), Token::minus);
      return;
    }
    // -l<lib name>
    _buffer = _buffer.drop_front(2);
    StringRef::size_type start = 0;
    if (_buffer[start] == ':')
      ++start;
    if (!canStartName(_buffer[start]))
      // Create 'unknown' token.
      break;
    auto libNameEnd = std::find_if(_buffer.begin() + start + 1, _buffer.end(),
                                   [=](char c) { return !canContinueName(c); });
    StringRef::size_type libNameLen =
        std::distance(_buffer.begin(), libNameEnd);
    tok = Token(_buffer.substr(0, libNameLen), Token::libname);
    _buffer = _buffer.drop_front(libNameLen);
    return;
  }
  case '<':
    if (_buffer.startswith("<<=")) {
      tok = Token(drop(_buffer, 3), Token::lesslessequal);
      return;
    }
    if (_buffer.startswith("<<")) {
      tok = Token(drop(_buffer, 2), Token::lessless);
      return;
    }
    if (_buffer.startswith("<=")) {
      tok = Token(drop(_buffer, 2), Token::lessequal);
      return;
    }
    tok = Token(drop(_buffer, 1), Token::less);
    return;
  case '>':
    if (_buffer.startswith(">>=")) {
      tok = Token(drop(_buffer, 3), Token::greatergreaterequal);
      return;
    }
    if (_buffer.startswith(">>")) {
      tok = Token(drop(_buffer, 2), Token::greatergreater);
      return;
    }
    if (_buffer.startswith(">=")) {
      tok = Token(drop(_buffer, 2), Token::greaterequal);
      return;
    }
    tok = Token(drop(_buffer, 1), Token::greater);
    return;
  case '~':
    tok = Token(drop(_buffer, 1), Token::tilde);
    return;
  case '\"': case '\'': {
    // Handle quoted strings. They are treated as identifiers for
    // simplicity.
    char c = _buffer[0];
    _buffer = _buffer.drop_front();
    auto quotedStringEnd = _buffer.find(c);
    if (quotedStringEnd == StringRef::npos || quotedStringEnd == 0)
      break;
    StringRef word = _buffer.substr(0, quotedStringEnd);
    tok = Token(word, Token::identifier);
    _buffer = _buffer.drop_front(quotedStringEnd + 1);
    return;
  }
  default:
    // Handle literal numbers
    if (canStartNumber(_buffer[0])) {
      auto endIter = std::find_if(_buffer.begin(), _buffer.end(), [=](char c) {
        return !canContinueNumber(c);
      });
      StringRef::size_type end = endIter == _buffer.end()
                                     ? StringRef::npos
                                     : std::distance(_buffer.begin(), endIter);
      if (end == StringRef::npos || end == 0)
        break;
      StringRef word = _buffer.substr(0, end);
      tok = Token(word, Token::number);
      _buffer = _buffer.drop_front(end);
      return;
    }
    // Handle slashes '/', which can be either an operator inside an expression
    // or the beginning of an identifier
    if (_buffer.startswith("/=")) {
      tok = Token(drop(_buffer, 2), Token::slashequal);
      return;
    }
    if (_buffer[0] == '/' && _buffer.size() > 1 &&
        !canContinueName(_buffer[1])) {
      tok = Token(drop(_buffer, 1), Token::slash);
      return;
    }
    // Handle stars '*'
    if (_buffer.startswith("*=")) {
      tok = Token(drop(_buffer, 2), Token::starequal);
      return;
    }
    if (_buffer[0] == '*' && _buffer.size() > 1 &&
        !canContinueName(_buffer[1])) {
      tok = Token(drop(_buffer, 1), Token::star);
      return;
    }
    // Handle questions '?'
    if (_buffer[0] == '?' && _buffer.size() > 1 &&
        !canContinueName(_buffer[1])) {
      tok = Token(drop(_buffer, 1), Token::question);
      return;
    }
    // keyword or identifier.
    if (!canStartName(_buffer[0]))
      break;
    auto endIter = std::find_if(_buffer.begin() + 1, _buffer.end(),
                                [=](char c) { return !canContinueName(c); });
    StringRef::size_type end = endIter == _buffer.end()
                                   ? StringRef::npos
                                   : std::distance(_buffer.begin(), endIter);
    if (end == StringRef::npos || end == 0)
      break;
    StringRef word = _buffer.substr(0, end);
    Token::Kind kind =
        llvm::StringSwitch<Token::Kind>(word)
            .Case("ALIGN", Token::kw_align)
            .Case("ALIGN_WITH_INPUT", Token::kw_align_with_input)
            .Case("AS_NEEDED", Token::kw_as_needed)
            .Case("AT", Token::kw_at)
            .Case("ENTRY", Token::kw_entry)
            .Case("EXCLUDE_FILE", Token::kw_exclude_file)
            .Case("GROUP", Token::kw_group)
            .Case("HIDDEN", Token::kw_hidden)
            .Case("INPUT", Token::kw_input)
            .Case("KEEP", Token::kw_keep)
            .Case("ONLY_IF_RO", Token::kw_only_if_ro)
            .Case("ONLY_IF_RW", Token::kw_only_if_rw)
            .Case("OUTPUT", Token::kw_output)
            .Case("OUTPUT_ARCH", Token::kw_output_arch)
            .Case("OUTPUT_FORMAT", Token::kw_output_format)
            .Case("OVERLAY", Token::kw_overlay)
            .Case("PROVIDE", Token::kw_provide)
            .Case("PROVIDE_HIDDEN", Token::kw_provide_hidden)
            .Case("SEARCH_DIR", Token::kw_search_dir)
            .Case("SECTIONS", Token::kw_sections)
            .Case("SORT", Token::kw_sort_by_name)
            .Case("SORT_BY_ALIGNMENT", Token::kw_sort_by_alignment)
            .Case("SORT_BY_INIT_PRIORITY", Token::kw_sort_by_init_priority)
            .Case("SORT_BY_NAME", Token::kw_sort_by_name)
            .Case("SORT_NONE", Token::kw_sort_none)
            .Case("SUBALIGN", Token::kw_subalign)
            .Case("/DISCARD/", Token::kw_discard)
            .Default(Token::identifier);
    tok = Token(word, kind);
    _buffer = _buffer.drop_front(end);
    return;
  }
  tok = Token(drop(_buffer, 1), Token::unknown);
}

void Lexer::skipWhitespace() {
  while (true) {
    if (_buffer.empty())
      return;
    switch (_buffer[0]) {
    case ' ':
    case '\r':
    case '\n':
    case '\t':
      _buffer = _buffer.drop_front();
      break;
    // Potential comment.
    case '/':
      if (_buffer.size() <= 1 || _buffer[1] != '*')
        return;
      // Skip starting /*
      _buffer = _buffer.drop_front(2);
      // If the next char is also a /, it's not the end.
      if (!_buffer.empty() && _buffer[0] == '/')
        _buffer = _buffer.drop_front();

      // Scan for /'s. We're done if it is preceded by a *.
      while (true) {
        if (_buffer.empty())
          break;
        _buffer = _buffer.drop_front();
        if (_buffer.data()[-1] == '/' && _buffer.data()[-2] == '*')
          break;
      }
      break;
    default:
      return;
    }
  }
}

// Constant functions
void Constant::dump(raw_ostream &os) const { os << _num; }

// Symbol functions
void Symbol::dump(raw_ostream &os) const { os << _name; }

// FunctionCall functions
void FunctionCall::dump(raw_ostream &os) const {
  os << _name << "(";
  for (unsigned i = 0, e = _args.size(); i != e; ++i) {
    if (i)
      os << ", ";
    _args[i]->dump(os);
  }
  os << ")";
}

// Unary functions
void Unary::dump(raw_ostream &os) const {
  os << "(";
  if (_op == Unary::Minus)
    os << "-";
  else
    os << "~";
  _child->dump(os);
  os << ")";
}

// BinOp functions
void BinOp::dump(raw_ostream &os) const {
  os << "(";
  _lhs->dump(os);
  os << " ";
  switch (_op) {
  case Sum:
    os << "+";
    break;
  case Sub:
    os << "-";
    break;
  case Mul:
    os << "*";
    break;
  case Div:
    os << "/";
    break;
  case Shl:
    os << "<<";
    break;
  case Shr:
    os << ">>";
    break;
  case And:
    os << "&";
    break;
  case Or:
    os << "|";
    break;
  case CompareEqual:
    os << "==";
    break;
  case CompareDifferent:
    os << "!=";
    break;
  case CompareLess:
    os << "<";
    break;
  case CompareGreater:
    os << ">";
    break;
  case CompareLessEqual:
    os << "<=";
    break;
  case CompareGreaterEqual:
    os << ">=";
    break;
  }
  os << " ";
  _rhs->dump(os);
  os << ")";
}

// TernaryConditional functions
void TernaryConditional::dump(raw_ostream &os) const {
  _conditional->dump(os);
  os << " ? ";
  _trueExpr->dump(os);
  os << " : ";
  _falseExpr->dump(os);
}

// SymbolAssignment functions
void SymbolAssignment::dump(raw_ostream &os) const {
  int numParen = 0;

  if (_assignmentVisibility != Normal) {
    switch (_assignmentVisibility) {
    case Hidden:
      os << "HIDDEN(";
      break;
    case Provide:
      os << "PROVIDE(";
      break;
    case ProvideHidden:
      os << "PROVIDE_HIDDEN(";
      break;
    default:
      llvm_unreachable("Unknown visibility");
    }
    ++numParen;
  }

  os << _symbol << " ";
  switch (_assignmentKind) {
  case Simple:
    os << "=";
    break;
  case Sum:
    os << "+=";
    break;
  case Sub:
    os << "-=";
    break;
  case Mul:
    os << "*=";
    break;
  case Div:
    os << "/=";
    break;
  case Shl:
    os << "<<=";
    break;
  case Shr:
    os << ">>=";
    break;
  case And:
    os << "&=";
    break;
  case Or:
    os << "|=";
    break;
  }

  os << " ";
  _expression->dump(os);
  if (numParen)
    os << ")";
  os << ";";
}

static int dumpSortDirectives(raw_ostream &os, WildcardSortMode sortMode) {
  switch (sortMode) {
  case WildcardSortMode::NA:
    return 0;
  case WildcardSortMode::ByName:
    os << "SORT_BY_NAME(";
    return 1;
  case WildcardSortMode::ByAlignment:
    os << "SORT_BY_ALIGNMENT(";
    return 1;
  case WildcardSortMode::ByInitPriority:
    os << "SORT_BY_INIT_PRIORITY(";
    return 1;
  case WildcardSortMode::ByNameAndAlignment:
    os << "SORT_BY_NAME(SORT_BY_ALIGNMENT(";
    return 2;
  case WildcardSortMode::ByAlignmentAndName:
    os << "SORT_BY_ALIGNMENT(SORT_BY_NAME(";
    return 2;
  case WildcardSortMode::None:
    os << "SORT_NONE(";
    return 1;
  }
  return 0;
}

// InputSectionName functions
void InputSectionName::dump(raw_ostream &os) const {
  os << _name;
}

// InputSectionSortedGroup functions
static void dumpInputSections(raw_ostream &os,
                              llvm::ArrayRef<const InputSection *> secs) {
  bool excludeFile = false;
  bool first = true;

  for (auto &secName : secs) {
    if (!first)
      os << " ";
    first = false;
    // Coalesce multiple input sections marked with EXCLUDE_FILE in the same
    // EXCLUDE_FILE() group
    if (auto inputSec = dyn_cast<InputSectionName>(secName)) {
      if (!excludeFile && inputSec->hasExcludeFile()) {
        excludeFile = true;
        os << "EXCLUDE_FILE(";
      } else if (excludeFile && !inputSec->hasExcludeFile()) {
        excludeFile = false;
        os << ") ";
      }
    }
    secName->dump(os);
  }

  if (excludeFile)
    os << ")";
}

void InputSectionSortedGroup::dump(raw_ostream &os) const {
  int numParen = dumpSortDirectives(os, _sortMode);
  dumpInputSections(os, _sections);
  for (int i = 0; i < numParen; ++i)
    os << ")";
}

// InputSectionsCmd functions
void InputSectionsCmd::dump(raw_ostream &os) const {
  if (_keep)
    os << "KEEP(";

  int numParen = dumpSortDirectives(os, _fileSortMode);
  os << _fileName;
  for (int i = 0; i < numParen; ++i)
    os << ")";

  if (_archiveName.size() > 0) {
    os << ":";
    numParen = dumpSortDirectives(os, _archiveSortMode);
    os << _archiveName;
    for (int i = 0; i < numParen; ++i)
      os << ")";
  }

  if (_sections.size() > 0) {
    os << "(";
    dumpInputSections(os, _sections);
    os << ")";
  }

  if (_keep)
    os << ")";
}

// OutputSectionDescription functions
void OutputSectionDescription::dump(raw_ostream &os) const {
  if (_discard)
    os << "/DISCARD/";
  else
    os << _sectionName;

  if (_address) {
    os << " ";
    _address->dump(os);
  }
  os << " :\n";

  if (_at) {
    os << "  AT(";
    _at->dump(os);
    os << ")\n";
  }

  if (_align) {
    os << "  ALIGN(";
    _align->dump(os);
    os << ")\n";
  } else if (_alignWithInput) {
    os << " ALIGN_WITH_INPUT\n";
  }

  if (_subAlign) {
    os << "  SUBALIGN(";
    _subAlign->dump(os);
    os << ")\n";
  }

  switch (_constraint) {
  case C_None:
    break;
  case C_OnlyIfRO:
    os << "ONLY_IF_RO";
    break;
  case C_OnlyIfRW:
    os << "ONLY_IF_RW";
    break;
  }

  os << "  {\n";
  for (auto &command : _outputSectionCommands) {
    os << "    ";
    command->dump(os);
    os << "\n";
  }
  os << "  }";

  if (_fillStream.size() > 0) {
    os << " =";
    dumpByteStream(os, _fillStream);
  } else if (_fillExpr) {
    os << " =";
    _fillExpr->dump(os);
  }
}

// Sections functions
void Sections::dump(raw_ostream &os) const {
  os << "SECTIONS\n{\n";
  for (auto &command : _sectionsCommands) {
    command->dump(os);
    os << "\n";
  }
  os << "}\n";
}

// Parser functions
std::error_code Parser::parse() {
  // Get the first token.
  _lex.lex(_tok);
  // Parse top level commands.
  while (true) {
    switch (_tok._kind) {
    case Token::eof:
      return std::error_code();
    case Token::semicolon:
      consumeToken();
      break;
    case Token::kw_output: {
      auto output = parseOutput();
      if (!output)
        return LinkerScriptReaderError::parse_error;
      _script._commands.push_back(output);
      break;
    }
    case Token::kw_output_format: {
      auto outputFormat = parseOutputFormat();
      if (!outputFormat)
        return LinkerScriptReaderError::parse_error;
      _script._commands.push_back(outputFormat);
      break;
    }
    case Token::kw_output_arch: {
      auto outputArch = parseOutputArch();
      if (!outputArch)
        return LinkerScriptReaderError::parse_error;
      _script._commands.push_back(outputArch);
      break;
    }
    case Token::kw_input: {
      Input *input = parsePathList<Input>();
      if (!input)
        return LinkerScriptReaderError::parse_error;
      _script._commands.push_back(input);
      break;
    }
    case Token::kw_group: {
      Group *group = parsePathList<Group>();
      if (!group)
        return LinkerScriptReaderError::parse_error;
      _script._commands.push_back(group);
      break;
    }
    case Token::kw_as_needed:
      // Not allowed at top level.
      error(_tok, "AS_NEEDED not allowed at top level.");
      return LinkerScriptReaderError::parse_error;
    case Token::kw_entry: {
      Entry *entry = parseEntry();
      if (!entry)
        return LinkerScriptReaderError::parse_error;
      _script._commands.push_back(entry);
      break;
    }
    case Token::kw_search_dir: {
      SearchDir *searchDir = parseSearchDir();
      if (!searchDir)
        return LinkerScriptReaderError::parse_error;
      _script._commands.push_back(searchDir);
      break;
    }
    case Token::kw_sections: {
      Sections *sections = parseSections();
      if (!sections)
        return LinkerScriptReaderError::parse_error;
      _script._commands.push_back(sections);
      break;
    }
    case Token::identifier:
    case Token::kw_hidden:
    case Token::kw_provide:
    case Token::kw_provide_hidden: {
      const Command *cmd = parseSymbolAssignment();
      if (!cmd)
        return LinkerScriptReaderError::parse_error;
      _script._commands.push_back(cmd);
      break;
    }
    default:
      // Unexpected.
      error(_tok, "expected linker script command");
      return LinkerScriptReaderError::parse_error;
    }
  }
  return LinkerScriptReaderError::parse_error;
}

const Expression *Parser::parseFunctionCall() {
  assert((_tok._kind == Token::identifier || _tok._kind == Token::kw_align) &&
         "expected function call first tokens");
  SmallVector<const Expression *, 8> params;
  StringRef name = _tok._range;

  consumeToken();
  if (!expectAndConsume(Token::l_paren, "expected ("))
    return nullptr;

  if (_tok._kind == Token::r_paren) {
    consumeToken();
    return new (_alloc) FunctionCall(*this, _tok._range, params);
  }

  if (const Expression *firstParam = parseExpression())
    params.push_back(firstParam);
  else
    return nullptr;

  while (_tok._kind == Token::comma) {
    consumeToken();
    if (const Expression *param = parseExpression())
      params.push_back(param);
    else
      return nullptr;
  }

  if (!expectAndConsume(Token::r_paren, "expected )"))
    return nullptr;
  return new (_alloc) FunctionCall(*this, name, params);
}

bool Parser::expectExprOperand() {
  if (!(_tok._kind == Token::identifier || _tok._kind == Token::number ||
        _tok._kind == Token::kw_align || _tok._kind == Token::l_paren ||
        _tok._kind == Token::minus || _tok._kind == Token::tilde)) {
    error(_tok, "expected symbol, number, minus, tilde or left parenthesis.");
    return false;
  }
  return true;
}

const Expression *Parser::parseExprOperand() {
  if (!expectExprOperand())
    return nullptr;

  switch (_tok._kind) {
  case Token::identifier: {
    if (peek()._kind== Token::l_paren)
      return parseFunctionCall();
    Symbol *sym = new (_alloc) Symbol(*this, _tok._range);
    consumeToken();
    return sym;
  }
  case Token::kw_align:
    return parseFunctionCall();
  case Token::minus:
    consumeToken();
    return new (_alloc) Unary(*this, Unary::Minus, parseExprOperand());
  case Token::tilde:
    consumeToken();
    return new (_alloc) Unary(*this, Unary::Not, parseExprOperand());
  case Token::number: {
    auto val = parseNum(_tok._range);
    if (val.getError()) {
      error(_tok, "Unrecognized number constant");
      return nullptr;
    }
    Constant *c = new (_alloc) Constant(*this, *val);
    consumeToken();
    return c;
  }
  case Token::l_paren: {
    consumeToken();
    const Expression *expr = parseExpression();
    if (!expectAndConsume(Token::r_paren, "expected )"))
      return nullptr;
    return expr;
  }
  default:
    llvm_unreachable("Unknown token");
  }
}

static bool TokenToBinOp(const Token &tok, BinOp::Operation &op,
                         unsigned &precedence) {
  switch (tok._kind) {
  case Token::star:
    op = BinOp::Mul;
    precedence = 3;
    return true;
  case Token::slash:
    op = BinOp::Div;
    precedence = 3;
    return true;
  case Token::plus:
    op = BinOp::Sum;
    precedence = 4;
    return true;
  case Token::minus:
    op = BinOp::Sub;
    precedence = 4;
    return true;
  case Token::lessless:
    op = BinOp::Shl;
    precedence = 5;
    return true;
  case Token::greatergreater:
    op = BinOp::Shr;
    precedence = 5;
    return true;
  case Token::less:
    op = BinOp::CompareLess;
    precedence = 6;
    return true;
  case Token::greater:
    op = BinOp::CompareGreater;
    precedence = 6;
    return true;
  case Token::lessequal:
    op = BinOp::CompareLessEqual;
    precedence = 6;
    return true;
  case Token::greaterequal:
    op = BinOp::CompareGreaterEqual;
    precedence = 6;
    return true;
  case Token::equalequal:
    op = BinOp::CompareEqual;
    precedence = 7;
    return true;
  case Token::exclaimequal:
    op = BinOp::CompareDifferent;
    precedence = 7;
    return true;
  case Token::amp:
    op = BinOp::And;
    precedence = 8;
    return true;
  case Token::pipe:
    op = BinOp::Or;
    precedence = 10;
    return true;
  default:
    break;
  }
  return false;
}

static bool isExpressionOperator(Token tok) {
  switch (tok._kind) {
  case Token::star:
  case Token::slash:
  case Token::plus:
  case Token::minus:
  case Token::lessless:
  case Token::greatergreater:
  case Token::less:
  case Token::greater:
  case Token::lessequal:
  case Token::greaterequal:
  case Token::equalequal:
  case Token::exclaimequal:
  case Token::amp:
  case Token::pipe:
  case Token::question:
    return true;
  default:
    return false;
  }
}

const Expression *Parser::parseExpression(unsigned precedence) {
  assert(precedence <= 13 && "Invalid precedence value");
  if (!expectExprOperand())
    return nullptr;

  const Expression *expr = parseExprOperand();
  if (!expr)
    return nullptr;

  BinOp::Operation op;
  unsigned binOpPrecedence = 0;
  if (TokenToBinOp(_tok, op, binOpPrecedence)) {
    if (precedence >= binOpPrecedence)
      return parseOperatorOperandLoop(expr, precedence);
    return expr;
  }

  // Non-binary operators
  if (_tok._kind == Token::question && precedence >= 13)
    return parseOperatorOperandLoop(expr, precedence);
  return expr;
}

const Expression *Parser::parseOperatorOperandLoop(const Expression *lhs,
                                                   unsigned highestPrecedence) {
  assert(highestPrecedence <= 13 && "Invalid precedence value");
  unsigned precedence = 0;
  const Expression *binOp = nullptr;

  while (1) {
    BinOp::Operation op;
    if (!TokenToBinOp(_tok, op, precedence)) {
      if (_tok._kind == Token::question && highestPrecedence >= 13)
        return parseTernaryCondOp(lhs);
      return binOp;
    }

    if (precedence > highestPrecedence)
      return binOp;

    consumeToken();
    const Expression *rhs = parseExpression(precedence - 1);
    if (!rhs)
      return nullptr;
    binOp = new (_alloc) BinOp(*this, lhs, op, rhs);
    lhs = binOp;
  }
}

const Expression *Parser::parseTernaryCondOp(const Expression *lhs) {
  assert(_tok._kind == Token::question && "Expected question mark");

  consumeToken();

  // The ternary conditional operator has right-to-left associativity.
  // To implement this, we allow our children to contain ternary conditional
  // operators themselves (precedence 13).
  const Expression *trueExpr = parseExpression(13);
  if (!trueExpr)
    return nullptr;

  if (!expectAndConsume(Token::colon, "expected :"))
    return nullptr;

  const Expression *falseExpr = parseExpression(13);
  if (!falseExpr)
    return nullptr;

  return new (_alloc) TernaryConditional(*this, lhs, trueExpr, falseExpr);
}

// Parse OUTPUT(ident)
Output *Parser::parseOutput() {
  assert(_tok._kind == Token::kw_output && "Expected OUTPUT");
  consumeToken();
  if (!expectAndConsume(Token::l_paren, "expected ("))
    return nullptr;

  if (_tok._kind != Token::identifier) {
    error(_tok, "Expected identifier in OUTPUT.");
    return nullptr;
  }

  auto ret = new (_alloc) Output(*this, _tok._range);
  consumeToken();

  if (!expectAndConsume(Token::r_paren, "expected )"))
    return nullptr;

  return ret;
}

// Parse OUTPUT_FORMAT(ident)
OutputFormat *Parser::parseOutputFormat() {
  assert(_tok._kind == Token::kw_output_format && "Expected OUTPUT_FORMAT!");
  consumeToken();
  if (!expectAndConsume(Token::l_paren, "expected ("))
    return nullptr;

  if (_tok._kind != Token::identifier) {
    error(_tok, "Expected identifier in OUTPUT_FORMAT.");
    return nullptr;
  }

  SmallVector<StringRef, 8> formats;
  formats.push_back(_tok._range);

  consumeToken();

  do {
    if (isNextToken(Token::comma))
      consumeToken();
    else
      break;
    if (_tok._kind != Token::identifier) {
      error(_tok, "Expected identifier in OUTPUT_FORMAT.");
      return nullptr;
    }
    formats.push_back(_tok._range);
    consumeToken();
  } while (isNextToken(Token::comma));

  if (!expectAndConsume(Token::r_paren, "expected )"))
    return nullptr;

  return new (_alloc) OutputFormat(*this, formats);
}

// Parse OUTPUT_ARCH(ident)
OutputArch *Parser::parseOutputArch() {
  assert(_tok._kind == Token::kw_output_arch && "Expected OUTPUT_ARCH!");
  consumeToken();
  if (!expectAndConsume(Token::l_paren, "expected ("))
    return nullptr;

  if (_tok._kind != Token::identifier) {
    error(_tok, "Expected identifier in OUTPUT_ARCH.");
    return nullptr;
  }

  auto ret = new (_alloc) OutputArch(*this, _tok._range);
  consumeToken();

  if (!expectAndConsume(Token::r_paren, "expected )"))
    return nullptr;

  return ret;
}

// Parse file list for INPUT or GROUP
template<class T> T *Parser::parsePathList() {
  consumeToken();
  if (!expectAndConsume(Token::l_paren, "expected ("))
    return nullptr;

  SmallVector<Path, 8> paths;
  while (_tok._kind == Token::identifier || _tok._kind == Token::libname ||
         _tok._kind == Token::kw_as_needed) {
    switch (_tok._kind) {
    case Token::identifier:
      paths.push_back(Path(_tok._range));
      consumeToken();
      break;
    case Token::libname:
      paths.push_back(Path(_tok._range, false, true));
      consumeToken();
      break;
    case Token::kw_as_needed:
      if (!parseAsNeeded(paths))
        return nullptr;
      break;
    default:
      llvm_unreachable("Invalid token.");
    }
  }
  if (!expectAndConsume(Token::r_paren, "expected )"))
    return nullptr;
  return new (_alloc) T(*this, paths);
}

// Parse AS_NEEDED(file ...)
bool Parser::parseAsNeeded(SmallVectorImpl<Path> &paths) {
  assert(_tok._kind == Token::kw_as_needed && "Expected AS_NEEDED!");
  consumeToken();
  if (!expectAndConsume(Token::l_paren, "expected ("))
    return false;

  while (_tok._kind == Token::identifier || _tok._kind == Token::libname) {
    switch (_tok._kind) {
    case Token::identifier:
      paths.push_back(Path(_tok._range, true, false));
      consumeToken();
      break;
    case Token::libname:
      paths.push_back(Path(_tok._range, true, true));
      consumeToken();
      break;
    default:
      llvm_unreachable("Invalid token.");
    }
  }

  if (!expectAndConsume(Token::r_paren, "expected )"))
    return false;
  return true;
}

// Parse ENTRY(ident)
Entry *Parser::parseEntry() {
  assert(_tok._kind == Token::kw_entry && "Expected ENTRY!");
  consumeToken();
  if (!expectAndConsume(Token::l_paren, "expected ("))
    return nullptr;
  if (_tok._kind != Token::identifier) {
    error(_tok, "expected identifier in ENTRY");
    return nullptr;
  }
  StringRef entryName(_tok._range);
  consumeToken();
  if (!expectAndConsume(Token::r_paren, "expected )"))
    return nullptr;
  return new (_alloc) Entry(*this, entryName);
}

// Parse SEARCH_DIR(ident)
SearchDir *Parser::parseSearchDir() {
  assert(_tok._kind == Token::kw_search_dir && "Expected SEARCH_DIR!");
  consumeToken();
  if (!expectAndConsume(Token::l_paren, "expected ("))
    return nullptr;
  if (_tok._kind != Token::identifier) {
    error(_tok, "expected identifier in SEARCH_DIR");
    return nullptr;
  }
  StringRef searchPath(_tok._range);
  consumeToken();
  if (!expectAndConsume(Token::r_paren, "expected )"))
    return nullptr;
  return new (_alloc) SearchDir(*this, searchPath);
}

const SymbolAssignment *Parser::parseSymbolAssignment() {
  assert((_tok._kind == Token::identifier || _tok._kind == Token::kw_hidden ||
          _tok._kind == Token::kw_provide ||
          _tok._kind == Token::kw_provide_hidden) &&
         "Expected identifier!");
  SymbolAssignment::AssignmentVisibility visibility = SymbolAssignment::Normal;
  SymbolAssignment::AssignmentKind kind;
  int numParen = 0;

  switch (_tok._kind) {
  case Token::kw_hidden:
    visibility = SymbolAssignment::Hidden;
    ++numParen;
    consumeToken();
    if (!expectAndConsume(Token::l_paren, "expected ("))
      return nullptr;
    break;
  case Token::kw_provide:
    visibility = SymbolAssignment::Provide;
    ++numParen;
    consumeToken();
    if (!expectAndConsume(Token::l_paren, "expected ("))
      return nullptr;
    break;
  case Token::kw_provide_hidden:
    visibility = SymbolAssignment::ProvideHidden;
    ++numParen;
    consumeToken();
    if (!expectAndConsume(Token::l_paren, "expected ("))
      return nullptr;
    break;
  default:
    break;
  }

  StringRef name = _tok._range;
  consumeToken();

  // Parse assignment operator (=, +=, -= etc.)
  switch (_tok._kind) {
  case Token::equal:
    kind = SymbolAssignment::Simple;
    break;
  case Token::plusequal:
    kind = SymbolAssignment::Sum;
    break;
  case Token::minusequal:
    kind = SymbolAssignment::Sub;
    break;
  case Token::starequal:
    kind = SymbolAssignment::Mul;
    break;
  case Token::slashequal:
    kind = SymbolAssignment::Div;
    break;
  case Token::ampequal:
    kind = SymbolAssignment::And;
    break;
  case Token::pipeequal:
    kind = SymbolAssignment::Or;
    break;
  case Token::lesslessequal:
    kind = SymbolAssignment::Shl;
    break;
  case Token::greatergreaterequal:
    kind = SymbolAssignment::Shr;
    break;
  default:
    error(_tok, "unexpected token");
    return nullptr;
  }

  consumeToken();

  const Expression *expr = nullptr;
  switch (_tok._kind) {
  case Token::number:
  case Token::kw_align:
  case Token::identifier:
  case Token::l_paren:
    expr = parseExpression();
    if (!expr)
      return nullptr;
    break;
  default:
    error(_tok, "unexpected token while parsing assignment value.");
    return nullptr;
  }

  for (int i = 0; i < numParen; ++i)
    if (!expectAndConsume(Token::r_paren, "expected )"))
      return nullptr;

  return new (_alloc) SymbolAssignment(*this, name, expr, kind, visibility);
}

llvm::ErrorOr<InputSectionsCmd::VectorTy> Parser::parseExcludeFile() {
  assert(_tok._kind == Token::kw_exclude_file && "Expected EXCLUDE_FILE!");
  InputSectionsCmd::VectorTy res;
  consumeToken();

  if (!expectAndConsume(Token::l_paren, "expected ("))
    return llvm::ErrorOr<InputSectionsCmd::VectorTy>(
        std::make_error_code(std::errc::io_error));

  while (_tok._kind == Token::identifier) {
    res.push_back(new (_alloc) InputSectionName(*this, _tok._range, true));
    consumeToken();
  }

  if (!expectAndConsume(Token::r_paren, "expected )"))
    return llvm::ErrorOr<InputSectionsCmd::VectorTy>(
        std::make_error_code(std::errc::io_error));
  return llvm::ErrorOr<InputSectionsCmd::VectorTy>(std::move(res));
}

int Parser::parseSortDirectives(WildcardSortMode &sortMode) {
  int numParsedDirectives = 0;
  sortMode = WildcardSortMode::NA;

  if (_tok._kind == Token::kw_sort_by_name) {
    consumeToken();
    if (!expectAndConsume(Token::l_paren, "expected ("))
      return -1;
    ++numParsedDirectives;
    sortMode = WildcardSortMode::ByName;
  }

  if (_tok._kind == Token::kw_sort_by_init_priority) {
    consumeToken();
    if (!expectAndConsume(Token::l_paren, "expected ("))
      return -1;
    ++numParsedDirectives;
    sortMode = WildcardSortMode::ByInitPriority;
  }

  if (_tok._kind == Token::kw_sort_by_alignment) {
    consumeToken();
    if (!expectAndConsume(Token::l_paren, "expected ("))
      return -1;
    ++numParsedDirectives;
    if (sortMode != WildcardSortMode::ByName)
      sortMode = WildcardSortMode::ByAlignment;
    else
      sortMode = WildcardSortMode::ByNameAndAlignment;
  }

  if (numParsedDirectives < 2 && _tok._kind == Token::kw_sort_by_name) {
    consumeToken();
    if (!expectAndConsume(Token::l_paren, "expected ("))
      return -1;
    ++numParsedDirectives;
    if (sortMode == WildcardSortMode::ByAlignment)
      sortMode = WildcardSortMode::ByAlignmentAndName;
  }

  if (numParsedDirectives < 2 && _tok._kind == Token::kw_sort_by_alignment) {
    consumeToken();
    if (!expectAndConsume(Token::l_paren, "expected ("))
      return -1;
    ++numParsedDirectives;
  }

  if (numParsedDirectives == 0 && _tok._kind == Token::kw_sort_none) {
    consumeToken();
    if (!expectAndConsume(Token::l_paren, "expected ("))
      return -1;
    ++numParsedDirectives;
    sortMode = WildcardSortMode::None;
  }

  return numParsedDirectives;
}

const InputSection *Parser::parseSortedInputSections() {
  assert((_tok._kind == Token::kw_sort_by_name ||
          _tok._kind == Token::kw_sort_by_alignment ||
          _tok._kind == Token::kw_sort_by_init_priority ||
          _tok._kind == Token::kw_sort_none) &&
         "Expected SORT directives!");

  WildcardSortMode sortMode = WildcardSortMode::NA;
  int numParen = parseSortDirectives(sortMode);
  if (numParen == -1)
    return nullptr;

  SmallVector<const InputSection *, 8> inputSections;

  while (_tok._kind == Token::identifier) {
    inputSections.push_back(new (_alloc)
                                InputSectionName(*this, _tok._range, false));
    consumeToken();
  }

  // Eat "numParen" rparens
  for (int i = 0, e = numParen; i != e; ++i)
    if (!expectAndConsume(Token::r_paren, "expected )"))
      return nullptr;

  return new (_alloc) InputSectionSortedGroup(*this, sortMode, inputSections);
}

const InputSectionsCmd *Parser::parseInputSectionsCmd() {
  assert((_tok._kind == Token::identifier || _tok._kind == Token::colon ||
          _tok._kind == Token::star || _tok._kind == Token::kw_keep ||
          _tok._kind == Token::kw_sort_by_name ||
          _tok._kind == Token::kw_sort_by_alignment ||
          _tok._kind == Token::kw_sort_by_init_priority ||
          _tok._kind == Token::kw_sort_none) &&
         "Expected input section first tokens!");
  int numParen = 1;
  bool keep = false;
  WildcardSortMode fileSortMode = WildcardSortMode::NA;
  WildcardSortMode archiveSortMode = WildcardSortMode::NA;
  StringRef fileName;
  StringRef archiveName;

  if (_tok._kind == Token::kw_keep) {
    consumeToken();
    if (!expectAndConsume(Token::l_paren, "expected ("))
      return nullptr;
    ++numParen;
    keep = true;
  }

  // Input name
  if (_tok._kind != Token::colon) {
    int numParen = parseSortDirectives(fileSortMode);
    if (numParen == -1)
      return nullptr;
    fileName = _tok._range;
    consumeToken();
    if (numParen) {
      while (numParen--)
        if (!expectAndConsume(Token::r_paren, "expected )"))
          return nullptr;
    }
  }
  if (_tok._kind == Token::colon) {
    consumeToken();
    if (_tok._kind == Token::identifier ||
        _tok._kind == Token::kw_sort_by_name ||
        _tok._kind == Token::kw_sort_by_alignment ||
        _tok._kind == Token::kw_sort_by_init_priority ||
        _tok._kind == Token::kw_sort_none) {
      int numParen = parseSortDirectives(archiveSortMode);
      if (numParen == -1)
        return nullptr;
      archiveName = _tok._range;
      consumeToken();
      for (int i = 0; i != numParen; ++i)
	if (!expectAndConsume(Token::r_paren, "expected )"))
	  return nullptr;
    }
  }

  SmallVector<const InputSection *, 8> inputSections;

  if (_tok._kind != Token::l_paren)
    return new (_alloc)
        InputSectionsCmd(*this, fileName, archiveName, keep, fileSortMode,
                         archiveSortMode, inputSections);
  consumeToken();

  while (_tok._kind == Token::identifier ||
         _tok._kind == Token::kw_exclude_file ||
         _tok._kind == Token::kw_sort_by_name ||
         _tok._kind == Token::kw_sort_by_alignment ||
         _tok._kind == Token::kw_sort_by_init_priority ||
         _tok._kind == Token::kw_sort_none) {
    switch (_tok._kind) {
    case Token::kw_exclude_file: {
      auto vec = parseExcludeFile();
      if (vec.getError())
        return nullptr;
      inputSections.insert(inputSections.end(), vec->begin(), vec->end());
      break;
    }
    case Token::star:
    case Token::identifier: {
      inputSections.push_back(new (_alloc)
                                  InputSectionName(*this, _tok._range, false));
      consumeToken();
      break;
    }
    case Token::kw_sort_by_name:
    case Token::kw_sort_by_alignment:
    case Token::kw_sort_by_init_priority:
    case Token::kw_sort_none: {
      const InputSection *group = parseSortedInputSections();
      if (!group)
        return nullptr;
      inputSections.push_back(group);
      break;
    }
    default:
      llvm_unreachable("Unknown token");
    }
  }

  for (int i = 0; i < numParen; ++i)
    if (!expectAndConsume(Token::r_paren, "expected )"))
      return nullptr;
  return new (_alloc)
      InputSectionsCmd(*this, fileName, archiveName, keep, fileSortMode,
                       archiveSortMode, inputSections);
}

const OutputSectionDescription *Parser::parseOutputSectionDescription() {
  assert((_tok._kind == Token::kw_discard || _tok._kind == Token::identifier) &&
         "Expected /DISCARD/ or identifier!");
  StringRef sectionName;
  const Expression *address = nullptr;
  const Expression *align = nullptr;
  const Expression *subAlign = nullptr;
  const Expression *at = nullptr;
  const Expression *fillExpr = nullptr;
  StringRef fillStream;
  bool alignWithInput = false;
  bool discard = false;
  OutputSectionDescription::Constraint constraint =
      OutputSectionDescription::C_None;
  SmallVector<const Command *, 8> outputSectionCommands;

  if (_tok._kind == Token::kw_discard)
    discard = true;
  else
    sectionName = _tok._range;
  consumeToken();

  if (_tok._kind == Token::number || _tok._kind == Token::identifier ||
      _tok._kind == Token::kw_align || _tok._kind == Token::l_paren) {
    address = parseExpression();
    if (!address)
      return nullptr;
  }

  if (!expectAndConsume(Token::colon, "expected :"))
    return nullptr;

  if (_tok._kind == Token::kw_at) {
    consumeToken();
    at = parseExpression();
    if (!at)
      return nullptr;
  }

  if (_tok._kind == Token::kw_align) {
    consumeToken();
    align = parseExpression();
    if (!align)
      return nullptr;
  }

  if (_tok._kind == Token::kw_align_with_input) {
    consumeToken();
    alignWithInput = true;
  }

  if (_tok._kind == Token::kw_subalign) {
    consumeToken();
    subAlign = parseExpression();
    if (!subAlign)
      return nullptr;
  }

  if (_tok._kind == Token::kw_only_if_ro) {
    consumeToken();
    constraint = OutputSectionDescription::C_OnlyIfRO;
  } else if (_tok._kind == Token::kw_only_if_rw) {
    consumeToken();
    constraint = OutputSectionDescription::C_OnlyIfRW;
  }

  if (!expectAndConsume(Token::l_brace, "expected {"))
    return nullptr;

  // Parse zero or more output-section-commands
  while (_tok._kind != Token::r_brace) {
    switch (_tok._kind) {
    case Token::semicolon:
      consumeToken();
      break;
    case Token::identifier:
      switch (peek()._kind) {
      case Token::equal:
      case Token::plusequal:
      case Token::minusequal:
      case Token::starequal:
      case Token::slashequal:
      case Token::ampequal:
      case Token::pipeequal:
      case Token::lesslessequal:
      case Token::greatergreaterequal:
        if (const Command *cmd = parseSymbolAssignment())
          outputSectionCommands.push_back(cmd);
        else
          return nullptr;
        break;
      default:
        if (const Command *cmd = parseInputSectionsCmd())
          outputSectionCommands.push_back(cmd);
        else
          return nullptr;
        break;
      }
      break;
    case Token::kw_keep:
    case Token::star:
    case Token::colon:
    case Token::kw_sort_by_name:
    case Token::kw_sort_by_alignment:
    case Token::kw_sort_by_init_priority:
    case Token::kw_sort_none:
      if (const Command *cmd = parseInputSectionsCmd())
        outputSectionCommands.push_back(cmd);
      else
        return nullptr;
      break;
    case Token::kw_hidden:
    case Token::kw_provide:
    case Token::kw_provide_hidden:
      if (const Command *cmd = parseSymbolAssignment())
        outputSectionCommands.push_back(cmd);
      else
        return nullptr;
      break;
    default:
      error(_tok, "expected symbol assignment or input file name.");
      return nullptr;
    }
  }

  if (!expectAndConsume(Token::r_brace, "expected }"))
    return nullptr;

  if (_tok._kind == Token::equal) {
    consumeToken();
    if (_tok._kind != Token::number || !_tok._range.startswith_lower("0x")) {
      fillExpr = parseExpression();
      if (!fillExpr)
        return nullptr;
    } else {
      std::string strBuf;
      if (isExpressionOperator(peek()) ||
          !parseHexToByteStream(_tok._range.drop_front(2), strBuf)) {
        fillExpr = parseExpression();
        if(!fillExpr)
          return nullptr;
      } else {
        char *rawBuf = (char *) _alloc.Allocate(strBuf.size(), 1);
        memcpy(rawBuf, strBuf.c_str(), strBuf.size());
        fillStream = StringRef(rawBuf, strBuf.size());
        consumeToken();
      }
    }
  }

  return new (_alloc) OutputSectionDescription(
      *this, sectionName, address, align, subAlign, at, fillExpr, fillStream,
      alignWithInput, discard, constraint, outputSectionCommands);
}

const Overlay *Parser::parseOverlay() {
  assert(_tok._kind == Token::kw_overlay && "Expected OVERLAY!");
  error(_tok, "Overlay description is not yet supported.");
  return nullptr;
}

Sections *Parser::parseSections() {
  assert(_tok._kind == Token::kw_sections && "Expected SECTIONS!");
  consumeToken();
  if (!expectAndConsume(Token::l_brace, "expected {"))
    return nullptr;
  SmallVector<const Command *, 8> sectionsCommands;

  bool unrecognizedToken = false;
  // Parse zero or more sections-commands
  while (!unrecognizedToken) {
    switch (_tok._kind) {
    case Token::semicolon:
      consumeToken();
      break;

    case Token::identifier:
      switch (peek()._kind) {
      case Token::equal:
      case Token::plusequal:
      case Token::minusequal:
      case Token::starequal:
      case Token::slashequal:
      case Token::ampequal:
      case Token::pipeequal:
      case Token::lesslessequal:
      case Token::greatergreaterequal:
        if (const Command *cmd = parseSymbolAssignment())
          sectionsCommands.push_back(cmd);
        else
          return nullptr;
        break;
      default:
        if (const Command *cmd = parseOutputSectionDescription())
          sectionsCommands.push_back(cmd);
        else
          return nullptr;
        break;
      }
      break;

    case Token::kw_discard:
    case Token::star:
      if (const Command *cmd = parseOutputSectionDescription())
        sectionsCommands.push_back(cmd);
      else
        return nullptr;
      break;

    case Token::kw_entry:
      if (const Command *cmd = parseEntry())
        sectionsCommands.push_back(cmd);
      else
        return nullptr;
      break;

    case Token::kw_hidden:
    case Token::kw_provide:
    case Token::kw_provide_hidden:
      if (const Command *cmd = parseSymbolAssignment())
        sectionsCommands.push_back(cmd);
      else
        return nullptr;
      break;

    case Token::kw_overlay:
      if (const Command *cmd = parseOverlay())
        sectionsCommands.push_back(cmd);
      else
        return nullptr;
      break;

    default:
      unrecognizedToken = true;
      break;
    }
  }

  if (!expectAndConsume(
          Token::r_brace,
          "expected symbol assignment, entry, overlay or output section name."))
    return nullptr;

  return new (_alloc) Sections(*this, sectionsCommands);
}

} // end namespace script
} // end namespace lld
