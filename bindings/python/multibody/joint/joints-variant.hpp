//
// Copyright (c) 2015-2016 CNRS
//

#ifndef __pinocchio_python_joints_variant_hpp__
#define __pinocchio_python_joints_variant_hpp__

#include <eigenpy/exception.hpp>
#include <eigenpy/eigenpy.hpp>
#include "pinocchio/multibody/joint/joint-collection.hpp"
#include "pinocchio/bindings/python/multibody/joint/joints-models.hpp"

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    struct jointModelVariantVisitor : boost::static_visitor<PyObject *>
    {
      static result_type convert(JointModelVariant const & jv)
      {
        return apply_visitor(jointModelVariantVisitor(), jv);
      }

      template<typename T>
      result_type operator()(T const & t) const
      {
        return boost::python::incref(boost::python::object(t).ptr());
      }
    };

    struct exposer
    {
      template<class T>
      void operator()(T)
      {
        expose_joint_model<T>(bp::class_<T>(T::classname().c_str(),bp::init<>()).def(JointPythonVisitor<T>()));
        bp::implicitly_convertible<T,pinocchio::JointModelVariant>();
      }
    };

  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_joints_variant_hpp__
