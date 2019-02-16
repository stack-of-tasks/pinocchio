//
// Copyright (c) 2019 CNRS
//

#ifndef __pinocchio_python_joints_datas_hpp__
#define __pinocchio_python_joints_datas_hpp__

#include <eigenpy/exception.hpp>
#include <eigenpy/eigenpy.hpp>
//#include "pinocchio/multibody/joint/joint-collection.hpp"
//#include "pinocchio/multibody/joint/joint-composite.hpp"

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;


    // generic expose_joint_data : do nothing special
    template <class T>
    inline bp::class_<T>& expose_joint_data(bp::class_<T>& cl)
    {
      return cl;
    }

    template<>
    inline bp::class_<JointDataComposite>& expose_joint_data<JointDataComposite> (bp::class_<JointDataComposite> & cl)
    {
      return cl
        .def(bp::init<const typename JointDataComposite::JointDataVector&, const int, const int>
           (bp::args("Vector of joints", "nq", "nv"),
            "Init JointDataComposite with a defined size"))
        ;
    }
    
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_joint_datas_hpp__
