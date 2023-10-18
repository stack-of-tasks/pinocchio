//
// Copyright (c) 2019-2020 CNRS INRIA
//

#ifndef __pinocchio_python_joints_datas_hpp__
#define __pinocchio_python_joints_datas_hpp__

#include <boost/python.hpp>

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

    // specialization for JointDataRevoluteUnaligned
    template<>
    inline bp::class_<JointDataRevoluteUnaligned>& expose_joint_data<JointDataRevoluteUnaligned> (bp::class_<JointDataRevoluteUnaligned> & cl)
    {
      return cl
        .def(bp::init<Eigen::Vector3d> (bp::args("axis"), "Init JointDataRevoluteUnaligned from an axis with x-y-z components"))
        ;
    }

    // specialization for JointDataPrismaticUnaligned
    template<>
    inline bp::class_<JointDataPrismaticUnaligned>& expose_joint_data<JointDataPrismaticUnaligned> (bp::class_<JointDataPrismaticUnaligned> & cl)
    {
      return cl
        .def(bp::init<Eigen::Vector3d> (bp::args("axis"), "Init JointDataPrismaticUnaligned from an axis with x-y-z components"))
               ;
    }

    template<>
    inline bp::class_<JointDataPlanar>& expose_joint_data<JointDataPlanar> (bp::class_<JointDataPlanar> & cl)
    {
      return cl
        .add_property("StU",&JointDataPlanar::StU)
        ;
    }


    template<>
    inline bp::class_<JointDataSphericalZYX>& expose_joint_data<JointDataSphericalZYX> (bp::class_<JointDataSphericalZYX> & cl)
    {
      return cl
        .add_property("StU",&JointDataSphericalZYX::StU)
        ;
    }
    
    template<>
    inline bp::class_<JointDataComposite>& expose_joint_data<JointDataComposite> (bp::class_<JointDataComposite> & cl)
    {
      return cl
        .def(bp::init<const JointDataComposite::JointDataVector&, const int, const int>
           (bp::args("joint_data_vectors", "nq", "nv"),
            "Init JointDataComposite from a given collection of joint data"))
        .add_property("joints",&JointDataComposite::joints)
        .add_property("iMlast",&JointDataComposite::iMlast)
        .add_property("pjMi",&JointDataComposite::pjMi)
        .add_property("StU",&JointDataComposite::StU)
        ;
    }
    
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_joint_datas_hpp__
