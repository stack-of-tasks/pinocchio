//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/bindings/python/context/mpfr.hpp"

#include <eigenpy/eigenpy.hpp>
#include "pinocchio/bindings/python/math/multiprecision/boost/number.hpp"

namespace pinocchio
{
  namespace python
  {
    void exposeSpecificTypeFeatures()
    {
      BoostNumberPythonVisitor<PINOCCHIO_PYTHON_SCALAR_TYPE>::expose("mpfr");
    };

    boost::python::object getScalarType()
    {
      namespace bp = boost::python;
      PyTypeObject * pytype = const_cast<PyTypeObject *>(
        bp::converter::registered_pytype_direct<PINOCCHIO_PYTHON_SCALAR_TYPE>::get_pytype());
      return bp::object(bp::handle<>(bp::borrowed(reinterpret_cast<PyObject *>(pytype))));
    }
  } // namespace python
} // namespace pinocchio
