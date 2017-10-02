//
// Copyright (c) 2017 CNRS
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.

#ifndef __se3_python_geometry_object_hpp__
#define __se3_python_geometry_object_hpp__

#include <boost/python.hpp>
#include <eigenpy/memory.hpp>

#include "pinocchio/multibody/geometry.hpp"

EIGENPY_DEFINE_STRUCT_ALLOCATOR_SPECIALIZATION(se3::GeometryObject)

namespace se3
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

#ifdef WITH_HPP_FCL
          .def("CreateCapsule", &GeometryObjectPythonVisitor::maker_capsule)
          .staticmethod("CreateCapsule")
#endif // WITH_HPP_FCL
        ;
      }

#ifdef WITH_HPP_FCL
      static GeometryObject maker_capsule( const double radius , const double length)
      {
        return GeometryObject("",FrameIndex(0),JointIndex(0),
                              boost::shared_ptr<fcl::CollisionGeometry>(new fcl::Capsule (radius, length)),
                              SE3::Identity());

      }
#endif // WITH_HPP_FCL

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
} // namespace se3

#endif // ifndef __se3_python_geometry_object_hpp__
