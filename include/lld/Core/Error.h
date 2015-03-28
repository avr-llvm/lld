//===- Error.h - system_error extensions for lld ----------------*- C++ -*-===//
//
//                             The LLVM Linker
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This declares a new error_category for the lld library.
//
//===----------------------------------------------------------------------===//

#ifndef LLD_CORE_ERROR_H
#define LLD_CORE_ERROR_H

#include "lld/Core/LLVM.h"
#include <system_error>

namespace lld {

const std::error_category &native_reader_category();

enum class NativeReaderError {
  success = 0,
  unknown_file_format,
  file_too_short,
  file_malformed,
  unknown_chunk_type,
  memory_error,
  conflicting_target_machine,
};

inline std::error_code make_error_code(NativeReaderError e) {
  return std::error_code(static_cast<int>(e), native_reader_category());
}

const std::error_category &YamlReaderCategory();

enum class YamlReaderError {
  success = 0,
  unknown_keyword,
  illegal_value
};

inline std::error_code make_error_code(YamlReaderError e) {
  return std::error_code(static_cast<int>(e), YamlReaderCategory());
}

const std::error_category &LinkerScriptReaderCategory();

enum class LinkerScriptReaderError {
  success = 0,
  parse_error,
  unknown_symbol_in_expr,
  unrecognized_function_in_expr
};

inline std::error_code make_error_code(LinkerScriptReaderError e) {
  return std::error_code(static_cast<int>(e), LinkerScriptReaderCategory());
}

/// Creates an error_code object that has associated with it an arbitrary
/// error messsage.  The value() of the error_code will always be non-zero
/// but its value is meaningless. The messsage() will be (a copy of) the
/// supplied error string.
/// Note:  Once ErrorOr<> is updated to work with errors other than error_code,
/// this can be updated to return some other kind of error.
std::error_code make_dynamic_error_code(StringRef msg);
std::error_code make_dynamic_error_code(const Twine &msg);

} // end namespace lld

namespace std {
template <>
struct is_error_code_enum<lld::NativeReaderError> : std::true_type {};
template <> struct is_error_code_enum<lld::YamlReaderError> : std::true_type {};
template <>
struct is_error_code_enum<lld::LinkerScriptReaderError> : std::true_type {};
}

#endif
