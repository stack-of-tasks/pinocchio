//
// Copyright (c) 2017 CNRS
//

#ifndef __pinocchio_python_geometry_object_hpp__
#define __pinocchio_python_geometry_object_hpp__

#include <boost/python.hpp>
#include <eigenpy/memory.hpp>

#include "pinocchio/multibody/geometry.hpp"

EIGENPY_DEFINE_STRUCT_ALLOCATOR_SPECIALIZATION(pinocchio::GeometryObject)

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    struct GeometryObjectPythonVisitor
      : public boost::python::def_visitor< GeometryObjectPythonVisitor >
    {

      template<class PyClass>
      void visit(PyClass& cl) const 
      {
        cl
        .add_property("meshScale",
                      bp::make_getter(&GeometryObject::meshScale, bp::return_value_policy<bp::return_by_value>()),
                      bp::make_setter(&GeometryObject::meshScale),
                      "Scaling parameter for the mesh")
        .add_property("meshColor",
                      bp::make_getter(&GeometryObject::meshColor, bp::return_value_policy<bp::return_by_value>()),
                      bp::make_setter(&GeometryObject::meshColor),
                      "Color rgba for the mesh")
        .def_readwrite("name", &GeometryObject::name, "Name of the GeometryObject")
        .def_readwrite("parentJoint", &GeometryObject::parentJoint, "Index of the parent joint")
        .def_readwrite("parentFrame", &GeometryObject::parentFrame, "Index of the parent frame")
        .def_readwrite("placement",&GeometryObject::placement,
                       "Position of geometry object in parent joint's frame")
        .def_readonly("meshPath", &GeometryObject::meshPath, "Absolute path to the mesh file")
        .def_readonly("overrideMaterial", &GeometryObject::overrideMaterial, "Boolean that tells whether material information is stored in Geometry object")
        .def_readonly("meshTexturePath", &GeometryObject::meshTexturePath, "Absolute path to the mesh texture file")

#ifdef PINOCCHIO_WITH_HPP_FCL
          .def("CreateCapsule", &GeometryObjectPythonVisitor::maker_capsule)
          .staticmethod("CreateCapsule")
#endif // PINOCCHIO_WITH_HPP_FCL
        ;
      }

#ifdef PINOCCHIO_WITH_HPP_FCL
      static GeometryObject maker_capsule( const double radius , const double length)
      {
        return GeometryObject("",FrameIndex(0),JointIndex(0),
                              boost::shared_ptr<fcl::CollisionGeometry>(new fcl::Capsule (radius, length)),
                              SE3::Identity());

      }
#endif // PINOCCHIO_WITH_HPP_FCL

      static void expose()
      {
        bp::class_<GeometryObject>("GeometryObject",
                                   "A wrapper on a collision geometry including its parent joint, parent frame, placement in parent joint's frame.\n\n",
                                   bp::no_init
                                   )
        .def(GeometryObjectPythonVisitor())
        ;
        
        bp::enum_<GeometryType>("GeometryType")
        .value("VISUAL",VISUAL)
        .value("COLLISION",COLLISION)
        ;
      }

    };
    

  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_geometry_object_hpp__
