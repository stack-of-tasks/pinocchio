// Copyright (c) 2026 INRIA

#pragma once

#include "../fwd.hpp"
#include "../utils/comparable.hpp"
#include "../utils/printable.hpp"

#include "pinocchio/geometry.hpp"

#include <nanobind/stl/bind_vector.h>
#include <nanobind/stl/string.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
inline void exposeGeometryData(nb::module_ m)
{
  using namespace nb::literals;

  nb::class_<CollisionPair>(
    m, "CollisionPair", "Pair of ordered index defining a pair of collisions")
    .def(nb::init<>(), "Default constructor")
    .def(
      nb::init<const GeomIndex &, const GeomIndex &>(), "index1"_a, "index2"_a,
      "Initializer of collision pair.")
    .def_rw("first", &CollisionPair::first)
    .def_rw("second", &CollisionPair::second)
    .def(ComparableVisitor<CollisionPair>())
    .def(PrintableVisitor<CollisionPair>());

  nb::bind_vector<std::vector<CollisionPair>, nb::rv_policy::reference_internal>(
    m, "StdVec_CollisionPair");

  nb::class_<GeometryData>(
    m, "GeometryData", "Geometry data linked to a GeometryModel and a Data struct.")
    .def(
      nb::init<const GeometryModel &>(), "geometry_model"_a,
      "Constructor from a given GeometryModel.")
    .def_rw(
      "oMg", &GeometryData::oMg,
      "Vector of collision objects placement relative to the world frame.\n"
      "note: These quantities have to be updated by calling updateGeometryPlacements.")
    .def_rw(
      "activeCollisionPairs", &GeometryData::activeCollisionPairs,
      "Vector of active CollisionPairs")
#ifdef PINOCCHIO_WITH_COLLISION
    .def_rw(
      "distanceRequests", &GeometryData::distanceRequests,
      "Defines which information should be computed by coal for distance computations.")
    .def_rw("distanceResults", &GeometryData::distanceRequests, "Vector of distance results.")
    .def_rw(
      "collisionRequests", &GeometryData::collisionRequests,
      "Defines which information should be computed by coal for collision computations.\n\n"
      "Note: it is possible to define a security_margin and a break_distance for a collision "
      "request.\n"
      "Most likely, for robotics application, these thresholds will be different for each "
      "collision pairs\n"
      "(e.g. the two hands can have a large security margin while the two hips cannot.)")
    .def_rw("collisionResults", &GeometryData::collisionResults, "Vector of collision results.")
    .def_rw(
      "contactPatchRequests", &GeometryData::contactPatchRequests,
      "Defines which information should be computed by coal for contact patch requests.\n")
    .def_rw(
      "contactPatchResults", &GeometryData::contactPatchResults, "Vector of contact patch results.")
    .def_rw(
      "collision_functors", &GeometryData::collision_functors, "Vector of collision functors.")
    .def_rw(
      "contact_patch_functors", &GeometryData::contact_patch_functors,
      "Vector of contact patch functors.")
    .def_rw("distance_functors", &GeometryData::distance_functors, "Vector of distance functors.")
    .def_rw(
      "radius", &GeometryData::radius,
      "Vector of radius of bodies, i.e. the distance between the further point of the "
      "geometry object from the joint center.\n"
      "note: This radius information might be usuful in continuous collision checking")
#endif
    .def(
      "fillInnerOuterObjectMaps", &GeometryData::fillInnerOuterObjectMaps, "geometry_model"_a,
      "Fill inner and outer objects maps")
    .def(
      "activateCollisionPair",
      static_cast<void (GeometryData::*)(const PairIndex)>(&GeometryData::activateCollisionPair),
      "pair_id"_a,
      "Activate the collsion pair pair_id in geomModel.collisionPairs if it exists.\n"
      "note: Only active pairs are check for collision and distance computations.")
    .def(
      "setGeometryCollisionStatus", &GeometryData::setGeometryCollisionStatus, "geom_model"_a,
      "geom_id"_a, "enable_collision"_a,
      "Enable or disable collision for the given geometry given by its geometry id with all "
      "the other geometries registered in the list of collision pairs.")
    .def(
      "setActiveCollisionPairs", &GeometryData::setActiveCollisionPairs, "geometry_model"_a,
      "collision_map"_a, "upper"_a = true,
      "Set the collision pair association from a given input array.\n"
      "Each entry of the input matrix defines the activation of a given collision pair.")
    .def(
      "deactivateCollisionPair", &GeometryData::deactivateCollisionPair, "pair_id"_a,
      "Deactivate the collsion pair pair_id in geomModel.collisionPairs if it exists.")
    .def(
      "deactivateAllCollisionPairs", &GeometryData::deactivateAllCollisionPairs,
      "Deactivate all collision pairs.")
#ifdef PINOCCHIO_WITH_COLLISION
    .def(
      "setSecurityMargins", &GeometryData::setSecurityMargins, "geometry_model"_a,
      "security_margin_map"_a, "upper"_a = true, "sync_distance_upper_bound"_a = true,
      "Set the security margin of all the collision request in a row, according to the "
      "values stored in the associative map.")
#endif // PINOCCHIO_WITH_COLLISION
    .def(ComparableVisitor<GeometryData>())
    // Repr and str
    .def(PrintableVisitor<GeometryData>());
}
PINOCCHIO_PYTHON_NAMESPACE_END
