// Copyright (c) 2026 INRIA

#pragma once

#include "../fwd.hpp"

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

/// A nanobind call guard that emits a Python DeprecationWarning before the wrapped function
/// is called. Suitable for use with nb::call_guard<>.
/// The message must be encoded into the guard type because nb::call_guard<Ts...> assumes its type
/// template parameters are default-constructible.
///
/// In C++17, the template parameter must be a pointer to a string with static storage duratieon,
/// e.g.:
///
///   constexpr char kMsg[] = "Use bar() instead.";
///   m.def("deprecated_foo", ..., nb::call_guard<deprecated_function<kMsg>>());
///
/// No intermediate constexpr string is needed in C++20, however.
///
/// @remark This replaces eigenpy::deprecation_policy.
template<const char * msg>
struct deprecated_guard
{
  deprecated_guard() noexcept
  {
    PyErr_WarnEx(PyExc_DeprecationWarning, msg, 1);
  }
};

PINOCCHIO_PYTHON_NAMESPACE_END
