// Force-included on MSVC builds (/FI).
//
// detector.cpp formats an ArmorName enum directly with fmt::format
// ("{}/{}_{}.jpg", ..., armor.name, ...).  libstdc++'s bundled fmt implicitly
// converted unscoped enums to int; vcpkg fmt v12 deliberately removed that
// implicit conversion (C2079 type_is_unformattable_for).  Provide an explicit
// formatter so the upstream 09-12 sources stay byte-identical.  It must be
// visible BEFORE fmt instantiates the call site, hence /FI.
#ifndef WIN_SHIMS__WIN_FMT_ENUM_HPP
#define WIN_SHIMS__WIN_FMT_ENUM_HPP

#include <fmt/format.h>

#include "tasks/auto_aim/armor.hpp"

template <>
struct fmt::formatter<auto_aim::ArmorName> : fmt::formatter<int>
{
  auto format(auto_aim::ArmorName value, format_context & ctx) const
  {
    return fmt::formatter<int>::format(static_cast<int>(value), ctx);
  }
};

#endif  // WIN_SHIMS__WIN_FMT_ENUM_HPP
