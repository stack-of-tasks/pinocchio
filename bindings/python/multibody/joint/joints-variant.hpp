//
// Copyright (c) 2015-2016 CNRS
//

#ifndef __pinocchio_python_joints_variant_hpp__
#define __pinocchio_python_joints_variant_hpp__

#include <eigenpy/exception.hpp>
#include <eigenpy/eigenpy.hpp>
#include "pinocchio/multibody/joint/joint-collection.hpp"
#include "pinocchio/bindings/python/multibody/joint/joints-models.hpp"
#include "pinocchio/bindings/python/multibody/joint/joints-datas.hpp"
#include "pinocchio/bindings/python/utils/printable.hpp"

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    template<typename VariantType>
    struct JointVariantVisitor : boost::static_visitor<PyObject *>
    {
      static result_type convert(VariantType const & jv)
      {
        return apply_visitor(JointVariantVisitor<VariantType>(), jv);
      }

      template<typename T>
      result_type operator()(T const & t) const
      {
        return boost::python::incref(boost::python::object(t).ptr());
      }
    };

    struct DataExposer
    {
      template<class T>
      void operator()(T)
      {
        expose_joint_data<T>(
            bp::class_<T>(T::classname().c_str(),
                          T::classname().c_str(),
                          bp::init<>())
            .def(JointDataDerivedPythonVisitor<T>())
            .def(PrintableVisitor<T>())
        );
        bp::implicitly_convertible<T,pinocchio::JointDataVariant>();
      }
    };

    struct ModelExposer
    {
      template<class T>
      void operator()(T)
      {
        expose_joint_model<T>(
            bp::class_<T>(T::classname().c_str(),
                          T::classname().c_str(),
                          bp::init<>())
            .def(JointModelDerivedPythonVisitor<T>())
            .def(PrintableVisitor<T>())
        );
        bp::implicitly_convertible<T,pinocchio::JointModelVariant>();
      }
    };
    

  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_joints_variant_hpp__
