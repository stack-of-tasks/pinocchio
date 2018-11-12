//
// Copyright (c) 2015 CNRS
//

#ifndef __pinocchio_python_joints_models_hpp__
#define __pinocchio_python_joints_models_hpp__

#include <eigenpy/exception.hpp>
#include <eigenpy/eigenpy.hpp>
#include "pinocchio/multibody/joint/joint-collection.hpp"

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;


    // generic expose_model : do nothing special
    template <class T>
    inline bp::class_<T>& expose_model(bp::class_<T>& cl)
    {
      return cl;
    }

    template<>
    inline bp::class_<JointModelRevoluteUnaligned>& expose_model<JointModelRevoluteUnaligned> (bp::class_<JointModelRevoluteUnaligned> & cl)
    {
      return cl
               .def(bp::init<double, double, double> (bp::args("x", "y", "z"), "Init JointModelRevoluteUnaligned from the components x, y, z of the axis"))
               .def(bp::init<Eigen::Vector3d> (bp::args("axis"), "Init JointModelRevoluteUnaligned from an axis with x-y-z components"))
               .add_property("axis",
                             make_getter(&JointModelRevoluteUnaligned::axis, bp::return_value_policy<bp::return_by_value>()),
                             make_setter(&JointModelRevoluteUnaligned::axis, bp::return_value_policy<bp::return_by_value>()),
                             "Rotation axis of the JointModelRevoluteUnaligned.")
               ;
    }

    template<>
    inline bp::class_<JointModelPrismaticUnaligned>& expose_model<JointModelPrismaticUnaligned> (bp::class_<JointModelPrismaticUnaligned> & cl)
    {
      return cl
               .def(bp::init<double, double, double> (bp::args("x", "y", "z"), "Init JointModelPrismaticUnaligned from the components x, y, z of the axis"))
               .def(bp::init<Eigen::Vector3d> (bp::args("axis"), "Init JointModelPrismaticUnaligned from an axis with x-y-z components"))
               .add_property("axis",
                             make_getter(&JointModelPrismaticUnaligned::axis, bp::return_value_policy<bp::return_by_value>()),
                             make_setter(&JointModelPrismaticUnaligned::axis, bp::return_value_policy<bp::return_by_value>()),
                             "Translation axis of the JointModelPrismaticUnaligned.")
               ;
    }
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_joint_models_hpp__
