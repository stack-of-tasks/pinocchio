//
// Copyright (c) 2022 INRIA
//

#include <boost/python.hpp>
#include <eigenpy/eigen-to-python.hpp>
#include <iostream>

#include "pinocchio/algorithm/broadphase-callbacks.hpp"

namespace pinocchio { namespace python {

void exposeBroadphaseCallbacks()
{
  namespace bp = boost::python;
  
  bp::class_<CollisionCallBackBase, bp::bases<hpp::fcl::CollisionCallBackBase>, boost::noncopyable>("CollisionCallBackBase",bp::no_init)
  .def("getGeometryModel",&CollisionCallBackDefault::getGeometryModel,
       bp::return_value_policy<bp::copy_const_reference>())
  .def("getGeometryData",(GeometryData & (CollisionCallBackDefault::*)())&CollisionCallBackDefault::getGeometryData,
       bp::return_internal_reference<>())
  .def_readonly("collision",
                &CollisionCallBackDefault::collision,
                "Whether there is a collision or not")
  ;
  
  bp::class_<CollisionCallBackDefault, bp::bases<CollisionCallBackBase> >("CollisionCallBackDefault",bp::no_init)
  .def(bp::init<const GeometryModel &,GeometryData &,bp::optional<bool> >
       (bp::args("self","geometry_model","geometry_data","stopAtFirstCollision"),
        "Default constructor from a given GeometryModel and a GeometryData")[bp::with_custodian_and_ward<1,2>(),bp::with_custodian_and_ward<1,3>()])

  .def_readwrite("stopAtFirstCollision",
                 &CollisionCallBackDefault::stopAtFirstCollision,
                 "Whether to stop or not when localizing a first collision")
  .def_readonly("collisionPairIndex",
                &CollisionCallBackDefault::collisionPairIndex,
                "The collision index of the first pair in collision")
  .def_readonly("count",
                &CollisionCallBackDefault::count,
                "Number of visits of the collide method")
  ;
}
    
}} // namespace pinocchio::python
