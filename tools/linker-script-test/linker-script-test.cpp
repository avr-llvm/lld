//===- utils/linker-script-test/linker-script-test.cpp --------------------===//
//
//                             The LLVM Linker
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
///
/// \file
/// \brief Tool for testing linker script parsing.
///
//===----------------------------------------------------------------------===//

#include "lld/ReaderWriter/LinkerScript.h"

#include "llvm/Support/PrettyStackTrace.h"
#include "llvm/Support/Signals.h"

using namespace llvm;
using namespace lld;
using namespace script;

int main(int argc, const char **argv) {
  llvm::sys::PrintStackTraceOnErrorSignal();
  llvm::PrettyStackTraceProgram X(argc, argv);

  {
    ErrorOr<std::unique_ptr<MemoryBuffer>> mb =
        MemoryBuffer::getFileOrSTDIN(argv[1]);
    if (std::error_code ec = mb.getError()) {
      llvm::errs() << ec.message() << "\n";
      return 1;
    }
    Lexer l(std::move(mb.get()));
    Token tok;
    while (true) {
      l.lex(tok);
      tok.dump(llvm::outs());
      if (tok._kind == Token::eof || tok._kind == Token::unknown)
        break;
    }
  }
  {
    ErrorOr<std::unique_ptr<MemoryBuffer>> mb =
        MemoryBuffer::getFileOrSTDIN(argv[1]);
    if (std::error_code ec = mb.getError()) {
      llvm::errs() << ec.message() << "\n";
      return 1;
    }
    Parser p(std::move(mb.get()));
    if (!p.parse()) {
      LinkerScript *ls = p.get();
      ls->dump(llvm::outs());
    }
  }
}
