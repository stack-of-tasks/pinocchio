//
// Copyright (c) 2015-2021 CNRS INRIA
//

#ifndef __pinocchio_python_joints_variant_hpp__
#define __pinocchio_python_joints_variant_hpp__

#include <boost/algorithm/string/replace.hpp>

#include <boost/python.hpp>

#include "pinocchio/multibody/joint/joint-collection.hpp"
#include "pinocchio/bindings/python/multibody/joint/joints-models.hpp"
#include "pinocchio/bindings/python/multibody/joint/joints-datas.hpp"
#include "pinocchio/bindings/python/utils/printable.hpp"

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    template<typename T>
    std::string sanitizedClassname()
    {
        std::string className = boost::replace_all_copy(T::classname(), "<", "_");
        boost::replace_all(className, ">", "");
        return className;
    }

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

    struct JointDataExposer
    {
      template<class T>
      void operator()(T)
      {
        expose_joint_data<T>(
            bp::class_<T>(sanitizedClassname<T>().c_str(),
                          sanitizedClassname<T>().c_str(),
                          bp::init<>())
            .def(JointDataDerivedPythonVisitor<T>())
            .def(PrintableVisitor<T>())
        );
        bp::implicitly_convertible<T,pinocchio::JointData>();
      }
    };

    struct JointModelExposer
    {
      template<class T>
      void operator()(T)
      {
        expose_joint_model<T>(
            bp::class_<T>(sanitizedClassname<T>().c_str(),
                          sanitizedClassname<T>().c_str(),
                          bp::no_init)
            .def(JointModelDerivedPythonVisitor<T>())
            .def(PrintableVisitor<T>())
        );
        bp::implicitly_convertible<T,pinocchio::JointModel>();
      }
    };
    

  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_joints_variant_hpp__
