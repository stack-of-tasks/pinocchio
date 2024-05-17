//
// Copyright (c) 2021-2022 INRIA
//

#ifndef __pinocchio_python_math_multiprecision_boost_number_hpp__
#define __pinocchio_python_math_multiprecision_boost_number_hpp__

#include "pinocchio/math/multiprecision.hpp"

#include <boost/python.hpp>
#include <boost/python/return_value_policy.hpp>
#include <eigenpy/user-type.hpp>
#include <eigenpy/ufunc.hpp>
#include <sstream>

namespace
{

  template<class Backend>
  struct get_backend_precision
  {
  };

  template<unsigned Digits10, ::boost::multiprecision::mpfr_allocation_type AllocateType>
  struct get_backend_precision<::boost::multiprecision::mpfr_float_backend<Digits10, AllocateType>>
  {
    enum
    {
      value = Digits10
    };
  };

} // namespace

namespace eigenpy
{
  namespace internal
  {
    template<class Backend, ::boost::multiprecision::expression_template_option ExpressionTemplates>
    struct getitem<::boost::multiprecision::number<Backend, ExpressionTemplates>>
    {

      typedef ::boost::multiprecision::number<Backend, ExpressionTemplates> Scalar;

      static PyObject * run(void * data, void * /* arr */)
      {
        Scalar & mpfr_scalar = *static_cast<Scalar *>(data);
        Backend & backend = mpfr_scalar.backend();

        if (backend.data()[0]._mpfr_d == 0) // If the mpfr_scalar is not initialized, we have to
                                            // init it.
        {
          mpfr_scalar = Scalar(0);
          //          unsigned int precision = get_backend_precision<Backend>::value ?
          //          get_backend_precision<Backend>::value : backend.default_precision();
          //          mpfr_init2(backend.data(),
          //          ::boost::multiprecision::detail::digits10_2_2(precision));
        }
        bp::object m(boost::ref(mpfr_scalar));
        Py_INCREF(m.ptr());
        return m.ptr();
      }
    };
  } // namespace internal

} // namespace eigenpy

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    template<typename BoostNumber>
    struct BoostNumberPythonVisitor
    : public boost::python::def_visitor<BoostNumberPythonVisitor<BoostNumber>>
    {

    public:
      template<class PyClass>
      void visit(PyClass & cl) const
      {
        cl.def(bp::init<>("Default constructor.", bp::arg("self")))
          .def(bp::init<BoostNumber>("Copy constructor.", bp::args("self", "value")))
          //        .def(bp::init<bool>("Copy constructor.",bp::args("self","value")))
          //        .def(bp::init<float>("Copy constructor.",bp::args("self","value")))
          //        .def(bp::init<double>("Copy constructor.",bp::args("self","value")))
          //        .def(bp::init<int>("Copy constructor.",bp::args("self","value")))
          //        .def(bp::init<long int>("Copy constructor.",bp::args("self","value")))
          //        .def(bp::init<unsigned int>("Copy constructor.",bp::args("self","value")))
          //        .def(bp::init<unsigned long int>("Copy constructor.",bp::args("self","value")))
          .def(bp::init<std::string>("Constructor from a string.", bp::args("self", "str_value")));

        PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
        PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_SELF_ASSIGN_OVERLOADED
        cl.def(bp::self + bp::self)
          .def(bp::self += bp::self)
          .def(bp::self - bp::self)
          .def(bp::self -= bp::self)
          .def(bp::self * bp::self)
          .def(bp::self *= bp::self)
          .def(bp::self / bp::self)
          .def(bp::self /= bp::self)

          .def(bp::self < bp::self)
          .def(bp::self <= bp::self)
          .def(bp::self > bp::self)
          .def(bp::self >= bp::self)
          .def(bp::self == bp::self)
          .def(bp::self != bp::self)
          .def(bp::self_ns::pow(bp::self_ns::self, long()));
        PINOCCHIO_COMPILER_DIAGNOSTIC_POP

        cl.def("str", &BoostNumber::str, bp::args("self", "precision", "scientific"))

          .def(
            "default_precision", static_cast<unsigned (*)()>(BoostNumber::default_precision),
            "Get the default precision of the class.")
          .def(
            "default_precision", static_cast<void (*)(unsigned)>(BoostNumber::default_precision),
            bp::arg("digits10"), "Set the default precision of the class.")
          .staticmethod("default_precision")

          .def(
            "precision", static_cast<unsigned (BoostNumber::*)() const>(&BoostNumber::precision),
            bp::arg("self"), "Get the precision of this.")
          .def(
            "precision", static_cast<void (BoostNumber::*)(unsigned)>(&BoostNumber::precision),
            bp::args("self", "digits10"), "Set the precision of this.")

          .def("__float__", &cast<double>, bp::arg("self"), "Cast to float.")
          .def("__int__", &cast<int64_t>, bp::arg("self"), "Cast to int.")

          .def("__str__", &print, bp::arg("self"))
          .def("__repr__", &print, bp::arg("self"))

          .def(
            "set_display_precision", &set_display_precision, bp::arg("digit"),
            "Set the precision when printing values.")
          .staticmethod("set_display_precision")

          .def(
            "get_display_precision", &get_display_precision,
            "Get the precision when printing values.",
            bp::return_value_policy<bp::copy_non_const_reference>())
          .staticmethod("get_display_precision")

          // #ifndef PINOCCHIO_PYTHON_NO_SERIALIZATION
          //         .def_pickle(Pickle())
          // #endif
          ;
      }

      static void expose(const std::string & type_name)
      {
        bp::class_<BoostNumber>(type_name.c_str(), "", bp::no_init)
          .def(BoostNumberPythonVisitor<BoostNumber>());

        eigenpy::registerNewType<BoostNumber>();
        eigenpy::registerCommonUfunc<BoostNumber>();

#define IMPLICITLY_CONVERTIBLE(T1, T2) bp::implicitly_convertible<T1, T2>();
        //  bp::implicitly_convertible<T2,T1>();

        IMPLICITLY_CONVERTIBLE(double, BoostNumber);
        IMPLICITLY_CONVERTIBLE(float, BoostNumber);
        IMPLICITLY_CONVERTIBLE(long int, BoostNumber);
        IMPLICITLY_CONVERTIBLE(int, BoostNumber);
        IMPLICITLY_CONVERTIBLE(long, BoostNumber);
        IMPLICITLY_CONVERTIBLE(unsigned int, BoostNumber);
        IMPLICITLY_CONVERTIBLE(unsigned long int, BoostNumber);
        IMPLICITLY_CONVERTIBLE(bool, BoostNumber);

#undef IMPLICITLY_CONVERTIBLE

        eigenpy::registerCast<BoostNumber, double>(false);
        eigenpy::registerCast<double, BoostNumber>(true);
        eigenpy::registerCast<BoostNumber, float>(false);
        eigenpy::registerCast<float, BoostNumber>(true);
        eigenpy::registerCast<BoostNumber, long>(false);
        eigenpy::registerCast<long, BoostNumber>(true);
        eigenpy::registerCast<BoostNumber, int>(false);
        eigenpy::registerCast<int, BoostNumber>(true);
        ;
        eigenpy::registerCast<BoostNumber, int64_t>(false);
        eigenpy::registerCast<int64_t, BoostNumber>(true);
      }

    private:
      template<typename T>
      static T cast(const BoostNumber & self)
      {
        return static_cast<T>(self);
      }

      static std::string print(const BoostNumber & self)
      {
        return self.str(get_display_precision(), std::ios_base::dec);
      }

      static void set_display_precision(const int digit)
      {
        get_display_precision() = digit;
      }

      static int & get_display_precision()
      {
        static int precision = BoostNumber::default_precision();
        return precision;
      }

      //      struct Pickle : bp::pickle_suite
      //      {
      //        static
      //        boost::python::tuple
      //        getinitargs(const SE3 & M)
      //        { return bp::make_tuple((Matrix3)M.rotation(),(Vector3)M.translation()); }
      //      };
    };

  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_math_multiprecision_boost_number_hpp__
