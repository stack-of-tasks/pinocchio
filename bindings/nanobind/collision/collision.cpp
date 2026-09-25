// Copyright (c) 2026 INRIA

#include <iostream>

#include "pinocchio/bindings/python-nb/fwd.hpp"

#include "pinocchio/collision/collision.hpp"
#include "pinocchio/collision/distance.hpp"

#include <nanobind/stl/shared_ptr.h>
#include <nanobind/eigen/dense.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
using namespace nb::literals;

void exposeCollision(nb::module_ m)
{
  using GeometryObject = pinocchio::GeometryObject;

  nb::class_<ComputeCollision>(
    m, "ComputeCollision", "Collision function between two geometry objects.")
    .def(
      nb::init<const GeometryObject &, const GeometryObject &>(), "geometry_object1"_a,
      "geometry_object2"_a, "Constructor of a ComputeCollision")
    .def(
      "run", &ComputeCollision::run, "tf1"_a, "tf2"_a, "request"_a, "result"_a,
      "Call the function and return the result")
    .def(
      "getGeometryObject1", &ComputeCollision::getGeometryObject1,
      nb::rv_policy::reference_internal)
    .def(
      "getGeometryObject2", &ComputeCollision::getGeometryObject2,
      nb::rv_policy::reference_internal)
    .def("print", [](const ComputeCollision & self) {
      std::cout << "address #1: " << &self.getGeometryObject1() << std::endl;
      std::cout << "address #2: " << &self.getGeometryObject2() << std::endl;
    });

  nb::class_<ComputeDistance>(
    m, "ComputeDistance", "Distance function between two geometry objects.")
    .def(
      nb::init<const GeometryObject &, const GeometryObject &>(), "geometry_object1"_a,
      "geometry_object2"_a, "Constructor of a ComputeDistance")
    .def(
      "run", &ComputeDistance::run, "tf1"_a, "tf2"_a, "request"_a, "result"_a,
      "Call the function and return the result")
    .def(
      "getGeometryObject1", &ComputeDistance::getGeometryObject1, nb::rv_policy::reference_internal)
    .def(
      "getGeometryObject2", &ComputeDistance::getGeometryObject2, nb::rv_policy::reference_internal)
    .def("print", [](const ComputeDistance & self) {
      std::cout << "address #1: " << &self.getGeometryObject1() << std::endl;
      std::cout << "address #2: " << &self.getGeometryObject2() << std::endl;
    });

  m.def(
    "computeCollision",
    static_cast<bool (*)(
      const GeometryModel &, GeometryData &, const PairIndex, coal::CollisionRequest &)>(
      computeCollision),
    "geometry_model"_a, "geometry_data"_a, "pair_index"_a, "collision_request"_a,
    "Check if the collision objects of a collision pair for a given Geometry Model and Data "
    "are in collision.\n"
    "The collision pair is given by the two index of the collision objects.");

  m.def(
    "computeCollision",
    static_cast<bool (*)(const GeometryModel &, GeometryData &, const PairIndex)>(computeCollision),
    "geometry_model"_a, "geometry_data"_a, "pair_index"_a,
    "Check if the collision objects of a collision pair for a given Geometry Model and Data "
    "are in collision.\n"
    "The collision pair is given by the two index of the collision objects.");

  m.def(
    "computeCollisions",
    static_cast<bool (*)(const GeometryModel &, GeometryData &, const bool)>(computeCollisions),
    "geometry_model"_a, "geometry_data"_a, "stop_at_first_collision"_a = false,
    "Determine if all collision pairs are effectively in collision or not.");

  m.def(
    "computeCollisions",
    [](
      const Model & model, Data & data, const GeometryModel & geom_model, GeometryData & geom_data,
      ConstVectorRef q, bool stop_at_first_collision) -> bool {
      return computeCollisions(model, data, geom_model, geom_data, q, stop_at_first_collision);
    },
    "model"_a, "data"_a, "geometry_model"_a, "geometry_data"_a, "q"_a,
    "stop_at_first_collision"_a = false,
    "Update the geometry for a given configuration and "
    "determine if all collision pairs are effectively in collision or not.");

  m.def(
    "computeContactPatch",
    static_cast<void (*)(const GeometryModel &, GeometryData &, const PairIndex)>(
      computeContactPatch),
    "geometry_model"_a, "geometry_data"_a, "pair_index"_a,
    "Compute the contact patch info associated with the collision pair given by pair_id. Note "
    "that an actual computation will only occur if the collision pair is indeed in collision "
    "(i.e. only if geom_data.collisionResult[pair_id].isCollision() is true).");

  m.def(
    "computeContactPatches",
    static_cast<void (*)(const GeometryModel &, GeometryData &)>(computeContactPatches),
    "geometry_model"_a, "geometry_data"_a, "Calls computeContactPatch for every collision pair.");

  m.def(
    "computeDistance",
    [](const GeometryModel & geom_model, GeometryData & geom_data, const PairIndex pair_id)
      -> coal::DistanceResult & { return computeDistance(geom_model, geom_data, pair_id); },
    "geometry_model"_a, "geometry_data"_a, "pair_index"_a, nb::rv_policy::reference,
    "Compute the distance between the two geometry objects of a given collision pair for a "
    "GeometryModel and associated GeometryData.");

  m.def(
    "computeDistances",
    static_cast<std::size_t (*)(const GeometryModel &, GeometryData &)>(computeDistances),
    "geometry_model"_a, "geometry_data"_a,
    "Compute the distance between each collision pair for a given GeometryModel and associated "
    "GeometryData.");

  m.def(
    "computeDistances",
    [](
      const Model & model, Data & data, const GeometryModel & geom_model, GeometryData & geom_data,
      ConstVectorRef q) -> std::size_t {
      return computeDistances(model, data, geom_model, geom_data, q);
    },
    "model"_a, "data"_a, "geometry_model"_a, "geometry_data"_a, "q"_a,
    "Update the geometry for a given configuration and "
    "compute the distance between each collision pair.");

  m.def(
    "computeBodyRadius",
    [](const Model & model, const GeometryModel & geom_model, GeometryData & geom_data) {
      computeBodyRadius(model, geom_model, geom_data);
    },
    "model"_a, "geometry_model"_a, "geometry_data"_a,
    "Compute the radius of the geometry volumes attached to every joints.");
}

PINOCCHIO_PYTHON_NAMESPACE_END
