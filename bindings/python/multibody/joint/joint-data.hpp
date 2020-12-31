//
// Copyright (c) 2020 INRIA
//

#ifndef __pinocchio_python_multibody_joint_joint_data_hpp__
#define __pinocchio_python_multibody_joint_joint_data_hpp__

#include <boost/python.hpp>

#include "pinocchio/multibody/joint/joint-generic.hpp"
#include "pinocchio/bindings/python/multibody/joint/joint-derived.hpp"
#include "pinocchio/bindings/python/utils/printable.hpp"

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;
    
    template<typename JointData>
    struct JointDataPythonVisitor
    {
      
      static void expose()
      {
        bp::class_<JointData>("JointData",
                              "Generic Joint Data",
                              bp::no_init)
        .def(bp::init<typename JointData::JointDataVariant>(bp::args("self","joint_data")))
        .def(JointDataBasePythonVisitor<JointData>())
        .def(PrintableVisitor<JointData>())
        ;
      }

    }; 
    
}} // namespace pinocchio::python

#endif // ifndef __pinocchio_python_multibody_joint_joint_data_hpp__
