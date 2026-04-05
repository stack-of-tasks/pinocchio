// Copyright (c) 2026 INRIA

#pragma once

#include "../fwd.hpp"
#include "../utils/boost-variant.hpp"
#include "../utils/comparable.hpp"
#include "../utils/printable.hpp"

#include "pinocchio/geometry.hpp"

#include <nanobind/stl/bind_vector.h>
#include <nanobind/stl/string.h>

#include <limits>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

namespace details
{

  inline void exposeGeometryType(nb::module_ m)
  {
    nb::enum_<GeometryType>(m, "GeometryType", "Geometry type: visual or collision.")
      .value("VISUAL", VISUAL, "Visual geometry.")
      .value("COLLISION", COLLISION, "Collision geometry.")
      .export_values();
  }

  inline void exposeGeometryMaterials(nb::module_ m)
  {
    using namespace nb::literals;

    nb::class_<GeometryNoMaterial>(
      m, "GeometryNoMaterial", "Tag type indicating that a geometry has no associated material.")
      .def(nb::init<>(), "Default constructor.")
      .def(nb::init<const GeometryNoMaterial &>(), "other"_a, "Copy constructor.")
      .def(ComparableVisitor<GeometryNoMaterial>());

    nb::class_<GeometryPhongMaterial>(
      m, "GeometryPhongMaterial",
      "Mesh material based on the Phong lighting model.\n"
      "Diffuse color is stored in GeometryObject.meshColor.")
      .def(nb::init<>(), "Default constructor.")
      .def(nb::init<const GeometryPhongMaterial &>(), "other"_a, "Copy constructor.")
      .def(
        nb::init<const Eigen::Vector4d &, const Eigen::Vector4d &, double>(),
        "mesh_emission_color"_a, "mesh_specular_color"_a, "mesh_shininess"_a,
        "Construct a Phong material with the given emission color, specular color and shininess.")
      .def_rw(
        "meshEmissionColor", &GeometryPhongMaterial::meshEmissionColor,
        "RGBA emission (ambient) color value of the mesh.")
      .def_rw(
        "meshSpecularColor", &GeometryPhongMaterial::meshSpecularColor,
        "RGBA specular color value of the mesh.")
      .def_rw(
        "meshShininess", &GeometryPhongMaterial::meshShininess,
        "Shininess factor for the specular lighting model (normalized between 0 and 1).")
      .def(ComparableVisitor<GeometryPhongMaterial>());
  }

  inline void exposePhysicsMaterialType(nb::module_ m)
  {
    nb::enum_<PhysicsMaterialType>(
      m, "PhysicsMaterialType",
      "Type of physics material, used to look up friction coefficients between collision pairs.")
      .value("ICE", ICE)
      .value("METAL", METAL)
      .value("CONCRETE", CONCRETE)
      .value("PLASTIC", PLASTIC)
      .value("WOOD", WOOD)
      .export_values();
  }

  inline void exposePhysicsMaterial(nb::module_ m)
  {
    using namespace nb::literals;

    nb::class_<PhysicsMaterial>(
      m, "PhysicsMaterial", "Physics material associated to a geometry object.")
      .def(
        nb::init<PhysicsMaterialType, double, double>(), "material_type"_a = PLASTIC,
        "compliance"_a = 0.0, "elasticity"_a = 0.0,
        "Construct a physics material with the given type, compliance and elasticity.")
      .def(nb::init<const PhysicsMaterial &>(), "other"_a, "Copy constructor.")
      .def_rw("materialType", &PhysicsMaterial::materialType, "Type of the material.")
      .def_rw("compliance", &PhysicsMaterial::compliance, "Compliance of the material.")
      .def_rw("elasticity", &PhysicsMaterial::elasticity, "Elasticity of the material.")
      .def(ComparableVisitor<PhysicsMaterial>());
  }

  inline void exposeFrictionCoefficientMatrix(nb::module_ m)
  {
    using namespace nb::literals;

    nb::class_<FrictionCoefficientMatrix>(
      m, "FrictionCoefficientMatrix",
      "Symmetric matrix of static friction coefficients between pairs of physics materials.\n"
      "Use getFrictionCoefficientMatrix() to access the global singleton.")
      .def_rw(
        "friction_coefficient_matrix", &FrictionCoefficientMatrix::friction_coefficient_matrix,
        "The underlying Eigen matrix of friction coefficients.")
      .def(
        "getFrictionFromMaterialPair", &FrictionCoefficientMatrix::getFrictionFromMaterialPair,
        "type1"_a, "type2"_a,
        "Return the static friction coefficient for the given pair of material types.");

    m.def(
      "getFrictionCoefficientMatrix", &getFrictionCoefficientMatrix, nb::rv_policy::reference,
      "Return a reference to the global friction coefficient matrix singleton.");
  }

#ifdef PINOCCHIO_WITH_COLLISION
  inline void exposeCollisionObject(nb::module_ m)
  {
    using namespace nb::literals;

    nb::class_<CollisionObject, coal::CollisionObject>(
      m, "CollisionObject",
      "A Pinocchio collision object associating a coal::CollisionGeometry with a geometry "
      "object index.")
      .def(
        nb::init<const std::shared_ptr<::coal::CollisionGeometry> &, size_t, bool>(),
        "collision_geometry"_a, "geometry_object_index"_a = (std::numeric_limits<size_t>::max)(),
        "compute_local_aabb"_a = true, "Construct from a collision geometry.")
      .def(
        nb::init<const std::shared_ptr<::coal::CollisionGeometry> &, const SE3 &, size_t, bool>(),
        "collision_geometry"_a, "placement"_a,
        "geometry_object_index"_a = (std::numeric_limits<size_t>::max)(),
        "compute_local_aabb"_a = true, "Construct from a collision geometry and a placement.")
      .def_rw(
        "geometryObjectIndex", &CollisionObject::geometryObjectIndex,
        "Index of the GeometryObject this collision object is associated with.")
      .def(ComparableVisitor<CollisionObject>());
  }
#endif

} // namespace details

inline void exposeGeometryObject(nb::module_ m)
{
  using namespace nb::literals;
  using Self = GeometryObject;
  using CollisionGeometryPtr = Self::CollisionGeometryPtr;
  using Eigen::Vector3d;
  using Eigen::Vector4d;

  details::exposeGeometryType(m);
  details::exposeGeometryMaterials(m);
  details::exposePhysicsMaterialType(m);
  details::exposePhysicsMaterial(m);
  details::exposeFrictionCoefficientMatrix(m);
#ifdef PINOCCHIO_WITH_COLLISION
  details::exposeCollisionObject(m);
#endif

  nb::class_<GeometryObject>(
    m, "GeometryObject",
    "A wrapper on a collision geometry including its parent joint, parent frame, placement "
    "in parent joint's frame.")
    .def(
      nb::init<
        const std::string &, JointIndex, FrameIndex, const SE3 &, CollisionGeometryPtr,
        const std::string &, const Vector3d &, bool, const Vector4d &, const std::string &,
        GeometryMaterial, PhysicsMaterial>(),
      "name"_a, "parent_joint"_a, "parent_frame"_a, "placement"_a, "collision_geometry"_a,
      "mesh_path"_a = "", "mesh_scale"_a = Vector3d::Ones().eval(), "override_material"_a = false,
      "mesh_color"_a = Vector4d::UnitW(), "mesh_texture_path"_a = "",
      "mesh_material"_a = GeometryNoMaterial(), "physics_material"_a = PhysicsMaterial(),
      "Full constructor of a GeometryObject.")
    .def(nb::init<const GeometryObject &>(), "other"_a, "Copy constructor.")
    //
    .def_rw("name", &Self::name, "Name of the GeometryObject.")
    .def_rw("parentJoint", &Self::parentJoint, "Index of the parent joint.")
    .def_rw("parentFrame", &Self::parentFrame, "Index of the parent frame.")
    .def_rw(
      "placement", &Self::placement,
      "Placement of the geometry object in the parent joint's frame.")
    .def_rw(
      "geometry", &Self::geometry,
      "The coal::CollisionGeometry associated to the given GeometryObject.")
    .def_rw("meshPath", &Self::meshPath, "Path to the mesh file.")
    .def_rw("meshScale", &Self::meshScale, "Scaling parameter of the mesh.")
    .def_rw("meshColor", &Self::meshColor, "RGBA color vector of the mesh.")
    .def_rw(
      "overrideMaterial", &Self::overrideMaterial,
      "If true, the material stored in this GeometryObject overrides the mesh default material.")
    .def_rw("meshTexturePath", &Self::meshTexturePath, "Path to the mesh texture file.")
    .def_rw(
      "meshMaterial", &Self::meshMaterial,
      "Material associated with the mesh (applied only if overrideMaterial is True).")
    .def_rw(
      "disableCollision", &Self::disableCollision,
      "If true, no collision or distance check will be done between the GeometryObject and any "
      "other.")
    .def_rw("physicsMaterial", &Self::physicsMaterial, "Physics material of the GeometryObject.")
    .def(
      "clone", &Self::clone,
      "Perform a deep copy of this GeometryObject, including the underlying coal geometry.")
    .def(ComparableVisitor<GeometryObject>())
    .def(PrintableVisitor<Self>());

  nb::bind_vector<std::vector<GeometryObject>, nb::rv_policy::reference_internal>(
    m, "GeometryObjectStdVec");
}

PINOCCHIO_PYTHON_NAMESPACE_END
