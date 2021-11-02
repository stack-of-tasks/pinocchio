#ifndef __pinocchio_python_pybind11_hpp__
#define __pinocchio_python_pybind11_hpp__

/// \mainpage Pinocchio PyBind11 helpers
///
/// This package provides utilities to ease the use of Pinocchio objects when
/// using PyBind11.
/// There are two methods:
/// 1. The developer-friendly but likely less user-friendly method: \ref
/// PINOCCHIO_PYBIND11_TYPE_CASTER
/// 2. The user-friendly but less developer-friendly method: \ref
/// pinocchio::python::make_pybind11_function
///
/// Both methods can be mixed. For both cases, you may
/// \code
/// // with necessary #define. See below
/// #include <pinocchio/bindings/python/pybind11-all.hpp>
/// \endcode
/// to get some \subpage default_type_caster.
///
/// \section example Example
/// \code
/// #define SCALAR double
/// #define OPTIONS 0
/// #define JOINT_MODEL_COLLECTION ::pinocchio::JointCollectionDefaultTpl
/// #include <pinocchio/bindings/python/pybind11-all.hpp>
///
/// ...
/// // method 1
/// m.def("function", my_function);
/// // method 2
/// m.def("function", pinocchio::python::make_pybind11_function(my_function));
/// \endcode
///

#include <iostream>
#include <pinocchio/fwd.hpp>

// This lines forces clang-format to keep the include split here
#include <pybind11/pybind11.h>

#include <boost/python.hpp>

namespace pinocchio {
namespace python {
namespace bp = boost::python;
namespace py = pybind11;

template <typename T>
inline py::object to(T& t) {
  // Create PyObject using boost Python
  bp::object obj = bp::api::object(t);
  PyObject* pyobj = obj.ptr();
  return pybind11::reinterpret_borrow<py::object>(pyobj);
}
template <typename T>
inline py::object to(T* t) {
  // Create PyObject using boost Python
  typename bp::manage_new_object::apply<T*>::type converter;
  PyObject* pyobj = converter(t);
  // Create the Pybind11 object
  return py::reinterpret_borrow<py::object>(pyobj);
}

template <typename ReturnType>
inline ReturnType& from(py::handle model) {
  return bp::extract<ReturnType&>(model.ptr());
}

template <typename T>
struct convert_type {
  typedef T type;
  static inline T _to(T t) { return t; }
  static inline type _from(type t) { return t; }
};
template <>
struct convert_type<void> {
  // typedef void type;
  // static inline void _to() {}
};

template <typename T>
struct convert_boost_python_object {
  typedef py::object type;
  static inline type _to(T t) {
    return to<typename std::remove_pointer<typename std::remove_reference<
        typename std::remove_cv<T>::type>::type>::type>(t);
  }
  static inline T _from(type t) {
    return from<
        typename std::remove_cv<typename std::remove_reference<T>::type>::type>(
        t);
  }
};

/// \brief Defines a conversion used by \ref make_pybind11_function
#define PINOCCHIO_PYBIND11_ADD_CONVERT_TYPE(CLASS)                    \
  namespace pinocchio {                                               \
  namespace python {                                                  \
  template <>                                                         \
  struct convert_type<CLASS> : convert_boost_python_object<CLASS> {}; \
  }                                                                   \
  }

/// \brief Defines a set of conversion used by \ref make_pybind11_function
#define _SINGLE_ARG(...) __VA_ARGS__
#define PINOCCHIO_PYBIND11_ADD_ALL_CONVERT_TYPE(CLASS)           \
  PINOCCHIO_PYBIND11_ADD_CONVERT_TYPE(_SINGLE_ARG(CLASS))        \
  PINOCCHIO_PYBIND11_ADD_CONVERT_TYPE(_SINGLE_ARG(CLASS const))  \
  PINOCCHIO_PYBIND11_ADD_CONVERT_TYPE(_SINGLE_ARG(CLASS&))       \
  PINOCCHIO_PYBIND11_ADD_CONVERT_TYPE(_SINGLE_ARG(CLASS const&)) \
  PINOCCHIO_PYBIND11_ADD_CONVERT_TYPE(_SINGLE_ARG(CLASS*))       \
  PINOCCHIO_PYBIND11_ADD_CONVERT_TYPE(_SINGLE_ARG(CLASS const*))

namespace internal {

template <typename R, typename... Args>
auto call(R (*f)(Args...), typename convert_type<Args>::type... args) {
  return convert_type<R>::_to(f(convert_type<Args>::_from(args)...));
}
template <typename... Args>
void call(void (*f)(Args...), typename convert_type<Args>::type... args) {
  f(convert_type<Args>::_from(args)...);
}

template <typename T>
struct function_wrapper;

template <typename R, typename... Args>
struct function_wrapper<R (*)(Args...)> {
  static const size_t nargs = sizeof...(Args);

  typedef R result_type;

  template <size_t i>
  struct arg {
    typedef typename std::tuple_element<i, std::tuple<Args...>>::type type;
  };

  typedef R (*func_type)(Args...);

  func_type f;

  // typename convert_type<result_type>::type
  auto operator()(typename convert_type<Args>::type... args) {
    return call(f, args...);
  }
};
}  // namespace internal

/// \brief Creates a function wrapper.
///
/// Using function wrapper has the advantage of being copy-less when possible
/// but the disadvantage of requiring to wrap the exposed function.
///
/// The wrapper does:
/// - converts the argument if a conversion has been previously declared,
/// - call the wrapped function
/// - converts the result if a conversion has been previously declared.
template <typename R, typename... Args>
internal::function_wrapper<R (*)(Args...)> make_pybind11_function(
    R (*func)(Args...)) {
  internal::function_wrapper<R (*)(Args...)> wrapper;
  wrapper.f = func;
  return wrapper;
}

template <typename T>
py::object default_arg(T t) {
  py::object obj = to<T>(t);
  //obj.inc_ref();
  return obj;
}

/// \brief Add a PyBind11 type caster.
///
/// Using type caster has the advantage of not requiring to wrap the exposed
/// functions but the disadvantage of systematically requiring a copy.
///
/// See \ref https://pybind11.readthedocs.io/en/stable/advanced/cast/custom.html
/// "PyBind11 documentation"
#define PINOCCHIO_PYBIND11_TYPE_CASTER(native_type, boost_python_name)     \
  namespace pybind11 {                                                     \
  namespace detail {                                                       \
  template <>                                                              \
  struct type_caster<native_type> {                                        \
    PYBIND11_TYPE_CASTER(_SINGLE_ARG(native_type), boost_python_name);     \
                                                                           \
    /* Python -> C++ */                                                    \
    bool load(pybind11::handle src, bool) {                                \
      PyObject* source = src.ptr();                                        \
      value = boost::python::extract<native_type>(source);                 \
      return !PyErr_Occurred();                                            \
    }                                                                      \
    /* C++ -> Python */                                                    \
    static pybind11::handle cast(native_type src,                          \
                                 pybind11::return_value_policy /*policy*/, \
                                 pybind11::handle /*parent*/) {            \
      return boost::python::api::object(src).ptr();                        \
    }                                                                      \
  };                                                                       \
  } /* namespace detail */                                                 \
  } /* namespace pybind11 */

}  // namespace python
}  // namespace pinocchio

#undef _SINGLE_ARG

#endif  // #ifndef __pinocchio_python_pybind11_hpp__
