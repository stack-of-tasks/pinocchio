// Copyright (c) 2026 INRIA

#pragma once

#include "pinocchio/bindings/python-nb/fwd.hpp"
#include "pinocchio/bindings/python-nb/utils/comparable.hpp"
#include "pinocchio/bindings/python-nb/utils/copyable.hpp"
#include "pinocchio/bindings/python-nb/utils/printable.hpp"
#include "pinocchio/bindings/python-nb/serialization/serializable.hpp"

#include "pinocchio/geometry.hpp"

#include <nanobind/stl/bind_vector.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
inline void exposeGeometryModel(nb::module_ m)
{
  using namespace nb::literals;
  using Self = GeometryModel;

  nb::class_<GeometryModel>(
    m, "GeometryModel",
    "Geometry model containing the collision or visual geometries associated to a model.")
    .def(nb::init<>())
    .def(nb::init<const Self &>(), "other"_a, "Copy constructor.")
    //
    .def_ro(
      "ngeoms", &GeometryModel::ngeoms, "Number of geometries contained in the GeometryModel.")
    .def_ro("geometryObjects", &GeometryModel::geometryObjects, "Vector of geometry objects.")
    //
    .def(
      "addGeometryObject",
      static_cast<Self::GeomIndex (Self::*)(const GeometryObject &)>(&Self::addGeometryObject),
      "geometry_object"_a,
      "Add a GeometryObject to a GeometryModel.\n"
      "Parameters:\n"
      "\tgeometry_object : a GeometryObject")
    .def(
      "addGeometryObject",
      static_cast<Self::GeomIndex (Self::*)(const GeometryObject &, const Model &)>(
        &Self::addGeometryObject),
      "geometry_object"_a, "model"_a,
      "Add a GeometryObject to a GeometryModel and set its parent joint by reading its value "
      "in the model.\n"
      "Parameters\n"
      "\tgeometry_object : a GeometryObject\n"
      "\tmodel : a Model of the system")
    .def(
      "removeGeometryObject", &Self::removeGeometryObject, "name"_a,
      "Remove a GeometryObject. Remove also the collision pairs that contain the object.")
    .def(
      "getGeometryId", &Self::getGeometryId, "name"_a,
      "Returns the index of a GeometryObject given by its name.")
    .def(
      "existGeometryName", &Self::existGeometryName, "name"_a,
      "Checks if a GeometryObject  given by its name exists.")
    .def(
      "createData", [](const Self & self) { return GeometryData(self); },
      "Create a GeometryData associated to the current model.")
    .def("clone", &Self::clone, "Create a deep copy of this GeometryModel.")
    // Collision pairs
    .def_ro("collisionPairs", &Self::collisionPairs, "Vector of collision pairs.")
    .def_ro(
      "collisionPairMapping", &Self::collisionPairMapping,
      "Matrix relating the collision pair ID to a pair of two GeometryObject indexes.")
    .def(
      "addCollisionPair", &Self::addCollisionPair, "collision_pair"_a,
      "Add a collision pair given by the index of the two collision objects.")
    .def(
      "addAllCollisionPairs", &Self::addAllCollisionPairs,
      "Add all collision pairs.\n"
      "note : collision pairs between geometries having the same parent joint are not added.")
    .def(
      "setCollisionPairs", &Self::setCollisionPairs, "collision_map"_a, "upper"_a = true,
      "Set the collision pairs from a given input array.\n"
      "Each entry of the input matrix defines the activation of a given collision pair"
      "(map[i,j] == True means that the pair (i,j) is active).")
    .def(
      "removeCollisionPair", &Self::removeCollisionPair, "collision_pair"_a,
      "Remove a collision pair.")
    .def("removeAllCollisionPairs", &Self::removeAllCollisionPairs, "Remove all collision pairs.")
    .def(
      "existCollisionPair", &Self::existCollisionPair, "collision_pair"_a,
      "Check if a collision pair exists.")
    .def(
      "findCollisionPair", &Self::findCollisionPair, "collision_pair"_a,
      "Return the index of a collision pair.")

    .def(ComparableVisitor<GeometryModel>())
    .def(CopyableVisitor<GeometryModel>())
    // Repr and str
    .def(PrintableVisitor<GeometryModel>())
    .def(SerializableVisitor<GeometryModel>());
}
PINOCCHIO_PYTHON_NAMESPACE_END
