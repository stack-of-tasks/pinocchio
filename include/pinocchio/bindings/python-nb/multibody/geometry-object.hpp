// Copyright (c) 2026 INRIA

#pragma once

#include "../fwd.hpp"
#include "../utils/printable.hpp"

#include "pinocchio/geometry.hpp"

#include <nanobind/stl/bind_vector.h>
#include <nanobind/stl/string.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
inline void exposeGeometryObject(nb::module_ m)
{
  using namespace nb::literals;
  using Self = GeometryObject;
  using CollisionGeometryPtr = Self::CollisionGeometryPtr;
  using Eigen::Vector3d;
  using Eigen::Vector4d;

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
      // doc
      "Full constructor of a GeometryObject")
    .def(nb::init<const GeometryObject &>(), "other"_a, "Copy constructor.")
    //
    .def_rw("meshScale", &Self::meshScale, "Scaling parameter of the mesh.")
    .def_rw("meshColor", &Self::meshColor, "RGBA color vector of the mesh.")
    .def_rw(
      "geometry", &Self::geometry,
      "The coal::CollisionGeometry associated to the given GeometryObject.")
    .def_rw(
      "placement", &Self::placement,
      "Placement of the geometry object in the parent joint's frame.")
    .def_rw("meshPath", &Self::meshPath, "Path to the mesh file.")
    .def_rw(
      "overrideMaterial", &Self::overrideMaterial,
      "Boolean flag whether information on the geometry material is stored inside the given "
      "GeometryObject.")
    .def_rw("meshTexturePath", &Self::meshTexturePath, "Path to the mesh texture file.")
    .def_rw(
      "disableCollision", &Self::disableCollision,
      "If true, no collision or distance check will be done between the GeometryObject and any "
      "other.")
    .def(
      "clone", &Self::clone,
      "Perform a deep copy of this GeometryObject. It will a copy of the underlying coal "
      "geometries.")
    .def_rw(
      "meshMaterial", &Self::meshMaterial,
      "Material associated with the mesh (applied only if self.overrideMaterial is True).")
    .def_rw("physicsMaterial", &Self::physicsMaterial, "Physics matertial of the GeometryMaterial.")
    // Repr and str
    .def(PrintableVisitor<Self>());

  nb::bind_vector<std::vector<GeometryObject>, nb::rv_policy::reference_internal>(
    m, "GeometryObjectStdVec");
}
PINOCCHIO_PYTHON_NAMESPACE_END
