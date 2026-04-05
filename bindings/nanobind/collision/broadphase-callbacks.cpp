// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"
#include "pinocchio/collision/broadphase-callbacks.hpp"

#include <nanobind/trampoline.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
struct PyCollisionCallback : CollisionCallBackBase
{
  NB_TRAMPOLINE(CollisionCallBackBase, 2);

  bool stop() const override
  {
    NB_OVERRIDE_PURE(stop, );
  }

  void done() override
  {
    NB_OVERRIDE(done, );
  }
};

void exposeBroadphaseCallbacks(nb::module_ m)
{
  using namespace nb::literals;
  // Base class
  nb::class_<CollisionCallBackBase, PyCollisionCallback>(m, "CollisionCallBackBase")
    .def(
      "getGeometryModel", &CollisionCallBackBase::getGeometryModel,
      nb::rv_policy::reference_internal)
    .def(
      "getGeometryData",
      [](CollisionCallBackBase & self) -> GeometryData & { return self.getGeometryData(); },
      nb::rv_policy::reference_internal)
    .def_ro("collision", &CollisionCallBackBase::collision, "Whether there is a collision or not.")
    .def_ro(
      "accumulate", &CollisionCallBackBase::accumulate,
      "Whether the callback is used in an accumulate mode where several collide methods are "
      "called successively.")
    .def(
      "stop", &CollisionCallBackBase::stop,
      "If true, the stopping criteria related to the collision callback has been met and one "
      "can stop.")
    .def(
      "done", &CollisionCallBackBase::done,
      "Callback method called after the termination of a collisition detection algorithm.");

  // Collect
  nb::class_<CollisionCallBackCollect, CollisionCallBackBase>(m, "CollisionCallBackCollect")
    .def(
      nb::init<const GeometryModel &, GeometryData &>(), "geometry_model"_a, "geometry_data"_a,
      nb::keep_alive<2, 1>(), nb::keep_alive<3, 1>())
    .def(
      nb::init<const GeometryModel &, GeometryData &, int>(), "geometry_model"_a, "geometry_data"_a,
      "max_num_pairs"_a, nb::keep_alive<2, 1>(), nb::keep_alive<3, 1>())
    .def_ro("pair_indexes", &CollisionCallBackCollect::pair_indexes);

  // Default
  nb::class_<CollisionCallBackDefault, CollisionCallBackBase>(m, "CollisionCallBackDefault")
    .def(
      nb::init<const GeometryModel &, GeometryData &, bool>(), "geometry_model"_a,
      "geometry_data"_a, "stop_at_first_collision"_a = false, nb::keep_alive<2, 1>(),
      nb::keep_alive<3, 1>())
    .def_rw(
      "stopAtFirstCollision", &CollisionCallBackDefault::stopAtFirstCollision,
      "Whether to stop or not when localizing a first collision.")
    .def_ro(
      "collisionPairIndex", &CollisionCallBackDefault::collisionPairIndex,
      "The collision index of the first pair in collision.")
    .def_ro("count", &CollisionCallBackDefault::count, "Number of visits of the collide() method.");
}

PINOCCHIO_PYTHON_NAMESPACE_END
