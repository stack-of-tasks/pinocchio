// Copyright (c) 2026 INRIA

#pragma once

#include <type_traits>

#include "pinocchio/bindings/python-nb/fwd.hpp"

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

namespace internal
{
  template<typename T, typename = void>
  struct has_clone : std::false_type
  {
  };

  template<typename T>
  struct has_clone<T, std::void_t<decltype(std::declval<const T &>().clone())>> : std::true_type
  {
  };
} // namespace internal

/// Binds `copy()` and `__copy__` to the copy constructor of `C`, and `__deepcopy__` to
/// `C::clone()` when `C` has one (e.g. GeometryObject, which clones its geometry), to the copy
/// constructor otherwise.
template<class C>
struct CopyableVisitor : nb::def_visitor<CopyableVisitor<C>>
{
  template<class PyClass, class... Extra>
  void execute(PyClass & cl, const Extra &...) const
  {
    using namespace nb::literals;
    cl.def("copy", &copy, "Returns a copy of this object.", nb::rv_policy::take_ownership)
      .def("__copy__", &copy, "Returns a copy of this object.", nb::rv_policy::take_ownership)
      .def(
        "__deepcopy__", [](const C & self, nb::dict) -> C * { return deepcopy(self); }, "memo"_a,
        "Returns a deep copy of this object.", nb::rv_policy::take_ownership);
  }

private:
  static C * copy(const C & self)
  {
    return new C(self);
  }

  static C * deepcopy(const C & self)
  {
    if constexpr (internal::has_clone<C>::value)
    {
      using Clone = decltype(self.clone());
      if constexpr (std::is_pointer_v<Clone>)
      {
        // Catches a C that inherits clone() without overriding it: that clone() copies a base
        // class object, not a C.
        static_assert(std::is_convertible_v<Clone, C *>, "C::clone() must return a C*.");
        return self.clone();
      }
      else
      {
        return new C(self.clone());
      }
    }
    else
    {
      return new C(self);
    }
  }
};

PINOCCHIO_PYTHON_NAMESPACE_END
