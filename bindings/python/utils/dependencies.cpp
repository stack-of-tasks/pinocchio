//
// Copyright (c) 2020-2021 INRIA
//

#include <eigenpy/deprecation-policy.hpp>
#include <eigenpy/registration.hpp>

#include <boost/python.hpp>

#ifdef PINOCCHIO_WITH_COLLISION
  #define WITH_COLLISION true
#else
  #define WITH_COLLISION false
#endif

#ifdef PINOCCHIO_WITH_URDFDOM
  #define WITH_URDF true
#else
  #define WITH_URDF false
#endif

#ifdef PINOCCHIO_WITH_CPPAD
  #define WITH_CPPAD true
#else
  #define WITH_CPPAD false
#endif

#ifdef PINOCCHIO_PYTHON_INTERFACE_WITH_OPENMP
  #define WITH_OPENMP true
#else
  #define WITH_OPENMP false
#endif

#ifdef PINOCCHIO_WITH_SDFORMAT
  #define WITH_SDFORMAT true
#else
  #define WITH_SDFORMAT false
#endif

namespace pinocchio
{
  namespace python
  {

    namespace bp = boost::python;

    struct DeprecatedBool
    {
      DeprecatedBool(bool value, const std::string & warning_msg)
      : value(value)
      , warning_msg(warning_msg)
      {
      }

      bool __bool__() const
      {
        PyErr_WarnEx(
          eigenpy::detail::deprecationTypeToPyObj(eigenpy::DeprecationType::DEPRECATION),
          warning_msg.c_str(), 1);
        return value;
      }

      const bool value;
      const std::string warning_msg;
    };

    void exposeDependencies()
    {
      if (!eigenpy::register_symbolic_link_to_registered_type<DeprecatedBool>())
      {
        bp::class_<DeprecatedBool>("DeprecatedBool", bp::no_init)
          .def("__bool__", &DeprecatedBool::__bool__);
      }

      bp::scope().attr("WITH_COLLISION") = WITH_COLLISION;
      // To conserve back compatibility
      bp::scope().attr("WITH_HPP_FCL") = DeprecatedBool(
        WITH_COLLISION, "This attriubte has been marked as deprecated"
                        ", and will be removed in the future.");
      bp::scope().attr("WITH_HPP_FCL_BINDINGS") = DeprecatedBool(
        WITH_COLLISION, "This attriubte has been marked as deprecated"
                        ", and will be removed in the future.");
      bp::scope().attr("WITH_URDFDOM") = WITH_URDF;
      bp::scope().attr("WITH_CPPAD") = WITH_CPPAD;
      bp::scope().attr("WITH_OPENMP") = WITH_OPENMP;
      bp::scope().attr("WITH_SDFORMAT") = WITH_SDFORMAT;
    }

  } // namespace python
} // namespace pinocchio
