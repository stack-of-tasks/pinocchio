// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"

#include "pinocchio/collision/broadphase.hpp"
#include "pinocchio/collision/tree-broadphase-manager.hpp"

#include <coal/broadphase/broadphase_dynamic_AABB_tree.h>
#include <coal/broadphase/broadphase_dynamic_AABB_tree_array.h>
#include <coal/broadphase/broadphase_SSaP.h>
#include <coal/broadphase/broadphase_SaP.h>
#include <coal/broadphase/broadphase_bruteforce.h>
#include <coal/broadphase/broadphase_interval_tree.h>

#include <nanobind/eigen/dense.h>
#include <nanobind/stl/bind_vector.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
using namespace nb::literals;

template<typename BroadPhaseManager>
static void exposeOneBroadphaseManager(nb::module_ m, const std::string & name)
{
  using Manager = BroadPhaseManagerTpl<BroadPhaseManager>;
  using CollisionObjectVector = typename Manager::CollisionObjectVector;

  const std::string class_name = "BroadPhaseManager_" + name;
  const std::string class_doc = "Broad phase manager associated to coal::" + name;

  nb::class_<Manager>(m, class_name.c_str(), class_doc.c_str())
    .def(
      nb::init<const Model *, const GeometryModel *, GeometryData *>(), "model"_a,
      "geometry_model"_a, "geometry_data"_a, "Default constructor", nb::keep_alive<2, 1>(),
      nb::keep_alive<3, 1>(), nb::keep_alive<4, 1>())
    .def(nb::init<const Manager &>(), "other"_a, "Copy constructor", nb::keep_alive<2, 1>())
    .def(
      "getModel", [](Manager & self) -> Model & { return const_cast<Model &>(self.getModel()); },
      nb::rv_policy::reference_internal, "Returns the related model.")
    .def(
      "getGeometryModel",
      [](Manager & self) -> GeometryModel & {
        return const_cast<GeometryModel &>(self.getGeometryModel());
      },
      nb::rv_policy::reference_internal, "Returns the related geometry model.")
    .def(
      "getGeometryData", static_cast<GeometryData & (Manager::*)()>(&Manager::getGeometryData),
      nb::rv_policy::reference_internal, "Returns the related geometry data.")
    .def(
      "getCollisionObjects",
      static_cast<CollisionObjectVector & (Manager::*)()>(&Manager::getCollisionObjects),
      nb::rv_policy::reference_internal,
      "Returns the vector of collision objects associated to the manager.")
    .def(
      "getCollisionObjectInflation", &Manager::getCollisionObjectInflation,
      nb::rv_policy::reference_internal,
      "Returns the inflation value related to each collision object.")
    .def(
      "getManager", static_cast<BroadPhaseManager & (Manager::*)()>(&Manager::getManager),
      nb::rv_policy::reference_internal, "Returns the internal coal manager.")
    .def(
      "getCollisionObjectStatus", &Manager::getCollisionObjectStatus, nb::rv_policy::copy,
      "Returns the status of the collision object.")
    .def(
      "check", static_cast<bool (Manager::*)() const>(&Manager::check),
      "Check whether the base broad phase manager is aligned with the current "
      "collision_objects.")
    .def(
      "check", static_cast<bool (Manager::*)(CollisionCallBackBase *) const>(&Manager::check),
      "callback"_a, "Check whether the callback is inline with this manager.")
    .def(
      "update", static_cast<void (Manager::*)(const bool)>(&Manager::update),
      "compute_local_aabb"_a = false,
      "Update the manager from the current geometry positions and update the underlying "
      "coal broad phase manager.")
    .def(
      "update", static_cast<void (Manager::*)(GeometryData *)>(&Manager::update), "geom_data_new"_a,
      "Update the manager with a new geometry data.", nb::keep_alive<2, 1>())
    .def(
      "collide",
      static_cast<bool (Manager::*)(CollisionObject &, CollisionCallBackBase *) const>(
        &Manager::collide),
      "collision_object"_a, "callback"_a,
      "Performs collision test between one object and all the objects belonging to the manager.")
    .def(
      "collide", static_cast<bool (Manager::*)(CollisionCallBackBase *) const>(&Manager::collide),
      "callback"_a, "Performs collision test for the objects belonging to the manager.")
    .def(
      "collide",
      static_cast<bool (Manager::*)(Manager &, CollisionCallBackBase *) const>(&Manager::collide),
      "other_manager"_a, "callback"_a,
      "Performs collision test with objects belonging to another manager.");

  m.def(
    "computeCollisions",
    [](Manager & manager, CollisionCallBackBase * callback) -> bool {
      return computeCollisions(manager, callback);
    },
    "manager"_a, "callback"_a,
    "Determine if all collision pairs are effectively in collision or not.\n"
    "This function assumes that updateGeometryPlacements and broadphase_manager.update() have "
    "been called first.");
  m.def(
    "computeCollisions",
    [](Manager & manager, bool stop_at_first_collision) -> bool {
      return computeCollisions(manager, stop_at_first_collision);
    },
    "manager"_a, "stop_at_first_collision"_a = false,
    "Determine if all collision pairs are effectively in collision or not.\n"
    "This function assumes that updateGeometryPlacements and broadphase_manager.update() have "
    "been called first.");
  m.def(
    "computeCollisions",
    [](
      const Model & model, Data & data, Manager & manager, ConstVectorRef q,
      bool stop_at_first_collision) -> bool {
      return computeCollisions(model, data, manager, q, stop_at_first_collision);
    },
    "model"_a, "data"_a, "broadphase_manager"_a, "q"_a, "stop_at_first_collision"_a = false,
    "Compute the forward kinematics, update the geometry placements and run the collision "
    "detection using the broadphase manager.");
  m.def(
    "computeCollisions",
    [](
      const Model & model, Data & data, Manager & manager, CollisionCallBackBase * callback,
      ConstVectorRef q) -> bool { return computeCollisions(model, data, manager, callback, q); },
    "model"_a, "data"_a, "broadphase_manager"_a, "callback"_a, "q"_a,
    "Compute the forward kinematics, update the geometry placements and run the collision "
    "detection using the broadphase manager.");
}

template<typename BroadPhaseManager>
static void exposeOneTreeBroadphaseManager(nb::module_ m, const std::string & name)
{
  using Manager = TreeBroadPhaseManagerTpl<BroadPhaseManager>;
  using BroadPhaseManagerVector = typename Manager::BroadPhaseManagerVector;

  const std::string class_name = "TreeBroadPhaseManager_" + name;
  const std::string class_doc = "Tree-based broad phase manager associated to coal::" + name;

  nb::class_<Manager>(m, class_name.c_str(), class_doc.c_str())
    .def(
      nb::init<const Model *, const GeometryModel *, GeometryData *>(), "model"_a,
      "geometry_model"_a, "geometry_data"_a, "Default constructor", nb::keep_alive<2, 1>(),
      nb::keep_alive<3, 1>(), nb::keep_alive<4, 1>())
    .def(nb::init<const Manager &>(), "other"_a, "Copy constructor", nb::keep_alive<2, 1>())
    .def(
      "getModel", [](Manager & self) -> Model & { return const_cast<Model &>(self.getModel()); },
      nb::rv_policy::reference_internal, "Returns the related model.")
    .def(
      "getGeometryModel",
      [](Manager & self) -> GeometryModel & {
        return const_cast<GeometryModel &>(self.getGeometryModel());
      },
      nb::rv_policy::reference_internal, "Returns the related geometry model.")
    .def(
      "getGeometryData", static_cast<GeometryData & (Manager::*)()>(&Manager::getGeometryData),
      nb::rv_policy::reference_internal, "Returns the related geometry data.")
    .def(
      "getBroadPhaseManagers",
      static_cast<BroadPhaseManagerVector & (Manager::*)()>(&Manager::getBroadPhaseManagers),
      nb::rv_policy::reference_internal, "Returns the internal broad phase managers.")
    .def(
      "check", static_cast<bool (Manager::*)() const>(&Manager::check),
      "Check whether the base broad phase manager is aligned with the current "
      "collision_objects.")
    .def(
      "check", static_cast<bool (Manager::*)(CollisionCallBackBase *) const>(&Manager::check),
      "callback"_a, "Check whether the callback is inline with this manager.")
    .def(
      "update", static_cast<void (Manager::*)(const bool)>(&Manager::update),
      "compute_local_aabb"_a = false,
      "Update the manager from the current geometry positions and update the underlying "
      "coal broad phase manager.")
    .def(
      "update", static_cast<void (Manager::*)(GeometryData *)>(&Manager::update), "geom_data_new"_a,
      "Update the manager with a new geometry data.", nb::keep_alive<2, 1>())
    .def(
      "collide",
      static_cast<bool (Manager::*)(CollisionObject &, CollisionCallBackBase *) const>(
        &Manager::collide),
      "collision_object"_a, "callback"_a,
      "Performs collision test between one object and all the objects belonging to the manager.")
    .def(
      "collide", static_cast<bool (Manager::*)(CollisionCallBackBase *) const>(&Manager::collide),
      "callback"_a, "Performs collision test for the objects belonging to the manager.")
    .def(
      "collide",
      static_cast<bool (Manager::*)(Manager &, CollisionCallBackBase *) const>(&Manager::collide),
      "other_manager"_a, "callback"_a,
      "Performs collision test with objects belonging to another manager.");

  m.def(
    "computeCollisions",
    [](Manager & manager, CollisionCallBackBase * callback) -> bool {
      return computeCollisions(manager, callback);
    },
    "manager"_a, "callback"_a,
    "Determine if all collision pairs are effectively in collision or not.\n"
    "This function assumes that updateGeometryPlacements and broadphase_manager.update() have "
    "been called first.");
  m.def(
    "computeCollisions",
    [](Manager & manager, bool stop_at_first_collision) -> bool {
      return computeCollisions(manager, stop_at_first_collision);
    },
    "manager"_a, "stop_at_first_collision"_a = false,
    "Determine if all collision pairs are effectively in collision or not.\n"
    "This function assumes that updateGeometryPlacements and broadphase_manager.update() have "
    "been called first.");
  m.def(
    "computeCollisions",
    [](
      const Model & model, Data & data, Manager & manager, ConstVectorRef q,
      bool stop_at_first_collision) -> bool {
      return computeCollisions(model, data, manager, q, stop_at_first_collision);
    },
    "model"_a, "data"_a, "broadphase_manager"_a, "q"_a, "stop_at_first_collision"_a = false,
    "Compute the forward kinematics, update the geometry placements and run the collision "
    "detection using the broadphase manager.");
  m.def(
    "computeCollisions",
    [](
      const Model & model, Data & data, Manager & manager, CollisionCallBackBase * callback,
      ConstVectorRef q) -> bool { return computeCollisions(model, data, manager, callback, q); },
    "model"_a, "data"_a, "broadphase_manager"_a, "callback"_a, "q"_a,
    "Compute the forward kinematics, update the geometry placements and run the collision "
    "detection using the broadphase manager.");
}

template<typename BroadPhaseManager>
static void exposeOneBroadphaseAlgo(nb::module_ m, const std::string & name)
{
  exposeOneBroadphaseManager<BroadPhaseManager>(m, name);

  // Bind the BroadPhaseManagerTpl vector before registering TreeBroadPhaseManagerTpl,
  // since TreeBroadPhaseManagerTpl::getBroadPhaseManagers() returns a reference to it.
  using FlatManager = BroadPhaseManagerTpl<BroadPhaseManager>;
  nb::bind_vector<std::vector<FlatManager>>(m, ("StdVec_BroadPhaseManager_" + name).c_str());

  exposeOneTreeBroadphaseManager<BroadPhaseManager>(m, name);
}

void exposeBroadphase(nb::module_ m)
{
  nb::bind_vector<std::vector<CollisionObject *>>(m, "StdVec_CollisionObjectPointer");
  nb::bind_vector<std::vector<CollisionObject>>(m, "StdVec_CollisionObject");

  exposeOneBroadphaseAlgo<coal::DynamicAABBTreeCollisionManager>(
    m, "DynamicAABBTreeCollisionManager");
  exposeOneBroadphaseAlgo<coal::DynamicAABBTreeArrayCollisionManager>(
    m, "DynamicAABBTreeArrayCollisionManager");
  exposeOneBroadphaseAlgo<coal::SSaPCollisionManager>(m, "SSaPCollisionManager");
  exposeOneBroadphaseAlgo<coal::SaPCollisionManager>(m, "SaPCollisionManager");
  exposeOneBroadphaseAlgo<coal::NaiveCollisionManager>(m, "NaiveCollisionManager");
  exposeOneBroadphaseAlgo<coal::IntervalTreeCollisionManager>(m, "IntervalTreeCollisionManager");
  //  exposeOneBroadphaseAlgo<coal::SpatialHashingCollisionManager<>>(
  //    m, "SpatialHashingCollisionManager");
}

PINOCCHIO_PYTHON_NAMESPACE_END
