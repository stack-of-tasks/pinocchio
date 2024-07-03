//
// Copyright (c) 2017-2023 CNRS INRIA
//

#ifndef __pinocchio_python_geometry_object_hpp__
#define __pinocchio_python_geometry_object_hpp__

#include <eigenpy/memory.hpp>
#include <eigenpy/eigen-from-python.hpp>
#include <eigenpy/eigen-to-python.hpp>
#include <eigenpy/variant.hpp>

#include "pinocchio/bindings/python/utils/address.hpp"
#include "pinocchio/bindings/python/utils/copyable.hpp"
#include "pinocchio/bindings/python/utils/registration.hpp"
#include "pinocchio/bindings/python/utils/deprecation.hpp"
#include "pinocchio/bindings/python/utils/pickle.hpp"
#include "pinocchio/bindings/python/serialization/serializable.hpp"

#include "pinocchio/multibody/geometry.hpp"

#if EIGENPY_VERSION_AT_MOST(2, 8, 1)
EIGENPY_DEFINE_STRUCT_ALLOCATOR_SPECIALIZATION(pinocchio::GeometryObject)
#endif

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    struct GeometryObjectPythonVisitor
    : public boost::python::def_visitor<GeometryObjectPythonVisitor>
    {

      typedef GeometryObject::CollisionGeometryPtr CollisionGeometryPtr;

      template<class PyClass>
      void visit(PyClass & cl) const
      {
        typedef eigenpy::VariantConverter<GeometryMaterial> Converter;
        Converter::registration();

        cl.def(bp::init<
                 std::string, JointIndex, FrameIndex, const SE3 &, CollisionGeometryPtr,
                 bp::optional<
                   std::string, const Eigen::Vector3d &, bool, const Eigen::Vector4d &, std::string,
                   GeometryMaterial>>(
                 bp::args(
                   "self", "name", "parent_joint", "parent_frame", "placement",
                   "collision_geometry", "mesh_path", "mesh_scale", "override_material",
                   "mesh_color", "mesh_texture_path", "mesh_material"),
                 "Full constructor of a GeometryObject."))
          .def(bp::init<
               std::string, JointIndex, const SE3 &, CollisionGeometryPtr,
               bp::optional<
                 std::string, const Eigen::Vector3d &, bool, const Eigen::Vector4d &, std::string,
                 GeometryMaterial>>(
            bp::args(
              "self", "name", "parent_joint", "placement", "collision_geometry", "mesh_path",
              "mesh_scale", "override_material", "mesh_color", "mesh_texture_path",
              "mesh_material"),
            "Reduced constructor of a GeometryObject. This constructor does not require to specify "
            "the parent frame index."))
          .def(bp::init<
               std::string, FrameIndex, JointIndex, CollisionGeometryPtr, const SE3 &,
               bp::optional<
                 std::string, const Eigen::Vector3d &, bool, const Eigen::Vector4d &, std::string,
                 GeometryMaterial>>(
            bp::args(
              "self", "name", "parent_frame", "parent_joint", "collision_geometry", "placement",
              "mesh_path", "mesh_scale", "override_material", "mesh_color",
              "mesh_texture_path"
              "mesh_material"),
            "Deprecated. Full constructor of a GeometryObject.")[deprecated_function<>()])
          .def(bp::init<
               std::string, JointIndex, CollisionGeometryPtr, const SE3 &,
               bp::optional<
                 std::string, const Eigen::Vector3d &, bool, const Eigen::Vector4d &, std::string,
                 GeometryMaterial>>(
            bp::args(
              "self", "name", "parent_joint", "collision_geometry", "placement", "mesh_path",
              "mesh_scale", "override_material", "mesh_color", "mesh_texture_path",
              "mesh_material"),
            "Deprecated. Reduced constructor of a GeometryObject. This constructor does not "
            "require to specify the parent frame index.")[deprecated_function<>()])
          .def(bp::init<const GeometryObject &>(
            bp::args("self", "otherGeometryObject"), "Copy constructor"))
          .add_property(
            "meshScale",
            bp::make_getter(&GeometryObject::meshScale, bp::return_internal_reference<>()),
            bp::make_setter(&GeometryObject::meshScale), "Scaling parameter of the mesh.")
          .add_property(
            "meshColor",
            bp::make_getter(&GeometryObject::meshColor, bp::return_internal_reference<>()),
            bp::make_setter(&GeometryObject::meshColor), "Color rgba of the mesh.")
          .def_readwrite(
            "geometry", &GeometryObject::geometry,
            "The FCL CollisionGeometry associated to the given GeometryObject.")
          .def_readwrite(
            "name", &GeometryObject::name, "Name associated to the given GeometryObject.")
          .def_readwrite("parentJoint", &GeometryObject::parentJoint, "Index of the parent joint.")
          .def_readwrite("parentFrame", &GeometryObject::parentFrame, "Index of the parent frame.")
          .def_readwrite(
            "placement", &GeometryObject::placement,
            "Position of geometry object in parent joint's frame.")
          .def_readwrite("meshPath", &GeometryObject::meshPath, "Path to the mesh file.")
          .def_readwrite(
            "overrideMaterial", &GeometryObject::overrideMaterial,
            "Boolean that tells whether material information is stored inside the "
            "given GeometryObject.")
          .def_readwrite(
            "meshTexturePath", &GeometryObject::meshTexturePath, "Path to the mesh texture file.")
          .def_readwrite(
            "disableCollision", &GeometryObject::disableCollision,
            "If true, no collision or distance check will be done between the "
            "Geometry and any other geometry.")
          .def(
            "clone", &GeometryObject::clone, bp::arg("self"),
            "Perform a deep copy of this. It will create a copy of the underlying FCL geometry.")
          .add_property(
            "meshMaterial",
            bp::make_getter(&GeometryObject::meshMaterial, Converter::return_internal_reference()),
            bp::make_setter(&GeometryObject::meshMaterial),
            "Material associated to the mesh (applied only if overrideMaterial is True)")

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
          const bp::converter::registration * reg = bp::converter::registry::query(info);
          // We just need to check if the type shared_ptr<CollisionGeometry> exist in the registry
          if (!reg)
            bp::register_ptr_to_python<CollisionGeometryPtr>();
        }
      }

#ifdef PINOCCHIO_WITH_HPP_FCL
      static GeometryObject maker_capsule(const double radius, const double length)
      {
        return GeometryObject(
          "", JointIndex(0), FrameIndex(0), SE3::Identity(),
          std::shared_ptr<fcl::CollisionGeometry>(new fcl::Capsule(radius, length)));
      }
#endif // PINOCCHIO_WITH_HPP_FCL

      static void expose()
      {
        if (!register_symbolic_link_to_registered_type<GeometryObject>())
        {
          bp::class_<GeometryObject>(
            "GeometryObject",
            "A wrapper on a collision geometry including its parent "
            "joint, parent frame, placement in parent joint's frame.\n\n",
            bp::no_init)
            .def(GeometryObjectPythonVisitor())
            .def(CopyableVisitor<GeometryObject>())
            .def(AddressVisitor<GeometryObject>())
            .def(SerializableVisitor<GeometryObject>())
#ifndef PINOCCHIO_PYTHON_NO_SERIALIZATION
            .def_pickle(PickleFromStringSerialization<GeometryObject>())
#endif
            ;
        }

#ifdef PINOCCHIO_WITH_HPP_FCL
        if (!register_symbolic_link_to_registered_type<CollisionObject>())
        {
          bp::class_<CollisionObject, bp::bases<::hpp::fcl::CollisionObject>>(
            "CollisionObject", "A Pinocchio collision object derived from FCL CollisionObject.",
            bp::no_init)
            .def(bp::init<
                 const std::shared_ptr<::hpp::fcl::CollisionGeometry> &,
                 bp::optional<const size_t, bool>>(
              (bp::arg("self"), bp::arg("collision_geometry"),
               bp::arg("geometryObjectIndex") = (std::numeric_limits<size_t>::max)(),
               bp::arg("compute_local_aabb") = true),
              "Constructor"))
            .def(bp::init<
                 const std::shared_ptr<::hpp::fcl::CollisionGeometry> &, SE3,
                 bp::optional<const size_t, bool>>(
              (bp::arg("self"), bp::arg("collision_geometry"), bp::arg("placement"),
               bp::arg("geometryObjectIndex") = (std::numeric_limits<size_t>::max)(),
               bp::arg("compute_local_aabb") = true),
              "Constructor"));
        }
#endif

        if (!register_symbolic_link_to_registered_type<GeometryNoMaterial>())
        {
          /// Define material types
          bp::class_<GeometryNoMaterial>("GeometryNoMaterial", bp::init<>())
            .def(bp::init<GeometryNoMaterial>());
        }

        if (!register_symbolic_link_to_registered_type<GeometryPhongMaterial>())
        {
          bp::class_<GeometryPhongMaterial>("GeometryPhongMaterial", bp::init<>())
            .def(bp::init<GeometryPhongMaterial>())
            .def(bp::init<Eigen::Vector4d, Eigen::Vector4d, double>())
            .add_property(
              "meshEmissionColor",
              bp::make_getter(
                &GeometryPhongMaterial::meshEmissionColor, bp::return_internal_reference<>()),
              bp::make_setter(&GeometryPhongMaterial::meshEmissionColor),
              "RGBA emission (ambient) color value of the mesh")
            .add_property(
              "meshSpecularColor",
              bp::make_getter(
                &GeometryPhongMaterial::meshSpecularColor, bp::return_internal_reference<>()),
              bp::make_setter(&GeometryPhongMaterial::meshSpecularColor),
              "RGBA specular value of the mesh")
            .def_readwrite(
              "meshShininess", &GeometryPhongMaterial::meshShininess,
              "Shininess associated to the specular lighting model (between 0 and 1)");
        }
        if (!register_symbolic_link_to_registered_type<GeometryType>())
        {
          bp::enum_<GeometryType>("GeometryType")
            .value("VISUAL", VISUAL)
            .value("COLLISION", COLLISION)
            .export_values();
        }
      }
    };

  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_geometry_object_hpp__
