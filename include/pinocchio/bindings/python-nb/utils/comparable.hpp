// Copyright (c) 2026 INRIA

#pragma once

#include "../fwd.hpp"

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

namespace detail
{
  template<class C, class = void>
  struct has_not_equal_op : std::false_type
  {
  };

  template<class C>
  struct has_not_equal_op<C, std::void_t<decltype(std::declval<C>() != std::declval<C>())>>
  : std::true_type
  {
  };
} // namespace detail

/// \brief Set the Python methods __eq__ and __ne__ to use the overloaded operators == and !=.
/// \note __ne__ is only bound if operator!= is defined for C.
template<class C>
struct ComparableVisitor : public nb::def_visitor<ComparableVisitor<C>>
{
  template<class PyClass, class... Extra>
  void execute(PyClass & cl, const Extra &...) const
  {
    cl.def("__eq__", [](const C & a, const C & b) { return a == b; }, nb::is_operator());
    if constexpr (detail::has_not_equal_op<C>::value)
      cl.def("__ne__", [](const C & a, const C & b) { return a != b; }, nb::is_operator());
  }
};

PINOCCHIO_PYTHON_NAMESPACE_END
