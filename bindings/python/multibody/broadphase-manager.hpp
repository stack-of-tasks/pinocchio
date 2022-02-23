//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_python_algorithm_broadphase_manager_hpp__
#define __pinocchio_python_algorithm_broadphase_manager_hpp__

#include "pinocchio/multibody/broadphase-manager.hpp"

#include <eigenpy/eigen-to-python.hpp>
#include <boost/algorithm/string/replace.hpp>

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;
    
    template<typename Derived>
    struct BroadPhaseManagerPythonVisitor
    : public bp::def_visitor< BroadPhaseManagerPythonVisitor<Derived> >
    {
    public:
      
      typedef BroadPhaseManagerTpl<Derived> Self;
      typedef typename Self::CollisionObjectVector CollisionObjectVector;

      /* --- Exposing C++ API to python through the handler ----------------- */
      template<class PyClass>
      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<const GeometryModel *, GeometryData *>(bp::args("self","geometry_model","geometry_data"),
                                                             "Default constructor")[bp::with_custodian_and_ward<1,2>(),bp::with_custodian_and_ward<1,3>()])
        .def(bp::init<const Self &>(bp::args("self","other"),
                                                             "Copy constructor")[bp::with_custodian_and_ward<1,2>()])
        
        .def("getGeometryModel",&Self::getGeometryModel,
             bp::arg("self"),
             bp::return_value_policy<bp::copy_const_reference>())
        .def("getGeometryData",(GeometryData & (Self::*)())&Self::getGeometryData,
             bp::arg("self"),
             bp::return_internal_reference<>())
        .def("getCollisionObjects",(CollisionObjectVector & (Self::*)())&Self::getCollisionObjects,
             bp::arg("self"),
             bp::return_internal_reference<>())
        
        .def("getCollisionObjectInflation",&Self::getCollisionObjectInflation,
             bp::arg("self"),
             "Returns the inflation value related to each collision object.",
             bp::return_internal_reference<>())
        
        .def("check", (bool (Self::*)() const)&Self::check,
             bp::arg("self"),
             "Check whether the base broad phase manager is aligned with the current collision_objects.")
        .def("check", (bool (Self::*)(CollisionCallBackBase *) const)&Self::check,
             bp::args("self","callback"),
             "Check whether the callback is inline with *this.")
        
        .def("collide",
             (bool (Self::*)(CollisionObject &, CollisionCallBackBase *) const)&Self::collide,
             bp::args("self","collision_object","callback"),
             "Performs collision test between one object and all the objects belonging to the manager.")
        .def("collide",
             (bool (Self::*)(CollisionCallBackBase *) const)&Self::collide,
             bp::args("self","callback"),
             "Performs collision test for the objects belonging to the manager.")
        .def("collide",
             (bool (Self::*)(Self &, CollisionCallBackBase *) const)&Self::collide,
             bp::args("self","other_manager","callback"),
             "Performs collision test with objects belonging to another manager.")
        
        .def("update",
             (void (Self::*)(const bool))&Self::update,
             (bp::arg("self"),bp::arg("compute_local_aabb") = false),
             "Update the manager from the current geometry positions and update the underlying FCL broad phase manager.")
        .def("update",
             (void (Self::*)(GeometryData * geom_data_new))&Self::update,
             (bp::arg("self"),bp::arg("geom_data_new")),
             "Update the manager with a new geometry data.",
             bp::with_custodian_and_ward<1,2>())
             
        .def("manager",(Derived &(Self::*)())&Self::getManager,
             bp::arg("self"),
             "Returns the internal FCL manager",
             bp::return_internal_reference<>())
        ;
      }
     
      /* --- Expose --------------------------------------------------------- */
      static void expose()
      {
        std::string derived_name = boost::typeindex::type_id<Derived>().pretty_name();
        boost::algorithm::replace_all(derived_name, "hpp::fcl::", "");
        const std::string class_name = "BroadPhaseManager_" + derived_name;
        
        const std::string class_doc = "Broad phase manager associated to hpp::fcl::" + derived_name;
        bp::class_<Self> registered_class(class_name.c_str(),
                                          class_doc.c_str(),
                                          bp::no_init);
        registered_class.def(BroadPhaseManagerPythonVisitor());
          
      }
      
    };
    
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_algorithm_broadphase_manager_hpp__
