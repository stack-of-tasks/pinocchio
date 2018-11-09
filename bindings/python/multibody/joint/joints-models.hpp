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


    // generic expose_constructor : do nothing special
    template <class T>
    inline bp::class_<T>& expose_constructors(bp::class_<T>& cl)
    {
      return cl;
    }

    template<>
    inline bp::class_<JointModelRevoluteUnaligned>& expose_constructors<JointModelRevoluteUnaligned> (bp::class_<JointModelRevoluteUnaligned> & cl)
    {
      return cl.def(bp::init<double, double, double> ((bp::args("x"), bp::args("y"), bp::args("z")), "Init JointRevoluteUnaligned from the components x, y, z of the axis"));
    }
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_joint_models_hpp__
