//
// Copyright (c) 2015-2021 CNRS INRIA
//

#ifndef __pinocchio_python_multibody_joint_joint_model_hpp__
#define __pinocchio_python_multibody_joint_joint_model_hpp__

#include <boost/python.hpp>

#include "pinocchio/multibody/joint/joint-generic.hpp"
#include "pinocchio/bindings/python/multibody/joint/joint-derived.hpp"
#include "pinocchio/bindings/python/utils/printable.hpp"

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;
    
    template<typename JointModel>
    struct JointModelPythonVisitor
    {
      
      static void expose()
      {
        bp::class_<JointModel>("JointModel",
                               "Generic Joint Model",
                               bp::no_init)
        .def(bp::init<JointModel>(bp::args("self","other")))
        .def(JointModelBasePythonVisitor<JointModel>())
        .def(PrintableVisitor<JointModel>())
        ;
      }

    }; 
    
}} // namespace pinocchio::python

#endif // ifndef __pinocchio_python_multibody_joint_joint_model_hpp__
