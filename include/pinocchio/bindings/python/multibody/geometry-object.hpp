//
// Copyright (c) 2017-2021 CNRS INRIA
//

#ifndef __pinocchio_python_geometry_object_hpp__
#define __pinocchio_python_geometry_object_hpp__

#include <boost/python.hpp>
#include <eigenpy/memory.hpp>
#include <eigenpy/eigen-to-python.hpp>

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
      typedef GeometryObject::CollisionGeometryPtr CollisionGeometryPtr;

      template<class PyClass>
      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<std::string,FrameIndex,JointIndex,CollisionGeometryPtr,SE3,
                      bp::optional<std::string,Eigen::Vector3d,bool,Eigen::Vector4d,std::string> >
             (
             bp::args("self","name","parent_frame","parent_joint","collision_geometry",
                      "placement", "mesh_path", "mesh_scale", "override_material", "mesh_color", "mesh_texture_path"),
             "Full constructor of a GeometryObject."))
        .def(bp::init<std::string,JointIndex,CollisionGeometryPtr,SE3,
                      bp::optional<std::string,Eigen::Vector3d,bool,Eigen::Vector4d,std::string> >
             (
              bp::args("self","name","parent_joint","collision_geometry",
                       "placement", "mesh_path", "mesh_scale", "override_material", "mesh_color", "mesh_texture_path"),
              "Reduced constructor of a GeometryObject. This constructor does not require to specify the parent frame index."
              ))
        .def(bp::init<const GeometryObject&>
             (
              bp::args("self","otherGeometryObject"),
              "Copy constructor"
              ))
        .add_property("meshScale",
                      bp::make_getter(&GeometryObject::meshScale,
                                      bp::return_internal_reference<>()),
                      bp::make_setter(&GeometryObject::meshScale),
                       "Scaling parameter of the mesh.")
        .add_property("meshColor",
                      bp::make_getter(&GeometryObject::meshColor,
                                      bp::return_internal_reference<>()),
                      bp::make_setter(&GeometryObject::meshColor),
                      "Color rgba of the mesh.")
        .def_readwrite("geometry", &GeometryObject::geometry,
                       "The FCL CollisionGeometry associated to the given GeometryObject.")
        .def_readwrite("name", &GeometryObject::name,
                       "Name associated to the given GeometryObject.")
        .def_readwrite("parentJoint", &GeometryObject::parentJoint,
                       "Index of the parent joint.")
        .def_readwrite("parentFrame", &GeometryObject::parentFrame,
                       "Index of the parent frame.")
        .def_readwrite("placement",&GeometryObject::placement,
                       "Position of geometry object in parent joint's frame.")
        .def_readwrite("meshPath", &GeometryObject::meshPath,
                       "Path to the mesh file.")
        .def_readwrite("overrideMaterial", &GeometryObject::overrideMaterial,
                       "Boolean that tells whether material information is stored inside the given GeometryObject.")
        .def_readwrite("meshTexturePath", &GeometryObject::meshTexturePath,
                       "Path to the mesh texture file.")
        .def_readwrite("disableCollision", &GeometryObject::disableCollision,
                       "If true, no collision or distance check will be done between the Geometry and any other geometry.")

        .def(bp::self == bp::self)
        .def(bp::self != bp::self)

#ifdef PINOCCHIO_WITH_HPP_FCL
          .def("CreateCapsule", &GeometryObjectPythonVisitor::maker_capsule)
          .staticmethod("CreateCapsule")
#endif // PINOCCHIO_WITH_HPP_FCL
        ;

        // Check registration
        {
          const bp::type_info info = bp::type_id<CollisionGeometryPtr>();
          const bp::converter::registration* reg = bp::converter::registry::query(info);
          // We just need to check if the type shared_ptr<CollisionGeometry> exist in the registry
          if(!reg)
            bp::register_ptr_to_python<CollisionGeometryPtr>();
        }
      }

#ifdef PINOCCHIO_WITH_HPP_FCL
      static GeometryObject maker_capsule(const double radius, const double length)
      {
        return GeometryObject("",FrameIndex(0),JointIndex(0),
                              pinocchio::shared_ptr<fcl::CollisionGeometry>(new fcl::Capsule(radius, length)),
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
        .export_values()
        ;
      }

    };


  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_geometry_object_hpp__
