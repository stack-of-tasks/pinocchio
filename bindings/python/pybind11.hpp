#include <pybind11/pybind11.h>

#include <pinocchio/multibody/model.hpp>

namespace pinocchio {
namespace cpp2pybind11 {
namespace bp = boost::python;

template<typename T> pybind11::object to(T& t)
{
  // Create PyObject using boost Python
  typename bp::reference_existing_object::apply<T*>::type converter;
  PyObject* pyobj = converter(t);
  // Create the Pybind11 object
  return pybind11::reinterpret_borrow<pybind11::object>(pyobj);
}
template <typename T>
auto to(T* t) {
  // Create PyObject using boost Python
  typename bp::manage_new_object::apply<T*>::type converter;
  PyObject* pyobj = converter(t);
  // Create the Pybind11 object
  return pybind11::reinterpret_steal<pybind11::object>(pyobj);
}

template <typename ReturnType>
ReturnType& from(pybind11::object model) {
  return boost::python::extract<ReturnType&>(model.ptr());
}

template <typename T>
struct convert_type {
  typedef T type;
  static inline auto _to(T t) { return t; }
  static inline auto _from(type t) { return t; }
};
template <>
struct convert_type<void> {
  static inline void _to() {}
};

template <typename T>
struct convert_boost_python_object {
  typedef pybind11::object type;
  static inline auto _to(T t) {
    return to<typename std::remove_pointer<typename std::remove_reference<
                  typename std::remove_cv<T>::type>::type>::type>(t);
  }
  static inline T _from(type t) {
    return from<
        typename std::remove_cv<typename std::remove_reference<T>::type>::type>(
        t);
  }
};

#define ADD_CONVERT_TYPE(CLASS)         \
  template <>                           \
  struct convert_type<pinocchio::CLASS> \
      : convert_boost_python_object<pinocchio::CLASS> {}

ADD_CONVERT_TYPE(Model*);
ADD_CONVERT_TYPE(Model&);
ADD_CONVERT_TYPE(Model const&);

namespace internal {

template <typename R, typename... Args>
auto call(R (*f)(Args...), typename convert_type<Args>::type... args) {
  return convert_type<R>::_to(f(convert_type<Args>::_from(args)...));
}
template <typename... Args>
void call(void (*f)(Args...), typename convert_type<Args>::type... args) {
  return f(convert_type<Args>::_from(args)...);
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

  auto operator()(typename convert_type<Args>::type... args) {
    // return convert_type<R>::_to(f(convert_type<Args>::_from(args)...));
    return call(f, args...);
  }
};
}  // namespace internal

template <typename R, typename... Args>
internal::function_wrapper<R (*)(Args...)> make_function(R (*func)(Args...)) {
  internal::function_wrapper<R (*)(Args...)> wrapper;
  wrapper.f = func;
  return wrapper;
}

}  // namespace cpp2pybind11
}  // namespace pinocchio
