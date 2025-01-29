//
// Copyright (c) 2022 INRIA
//

#include <eigenpy/eigenpy.hpp>

#include "pinocchio/collision/broadphase-callbacks.hpp"

namespace bp = boost::python;

namespace pinocchio
{
  namespace python
  {

    struct CollisionCallBackBaseWrapper
    : CollisionCallBackBase
    , bp::wrapper<CollisionCallBackBase>
    {
      typedef CollisionCallBackBase Base;

      bool stop() const
      {
        return this->get_override("stop")();
      }
      void done()
      {
        if (bp::override done = this->get_override("done"))
          done();
        Base::done();
      }

      void done_default()
      {
        return this->Base::done();
      }

      static void expose()
      {
        bp::class_<
          CollisionCallBackBaseWrapper, bp::bases<coal::CollisionCallBackBase>,
          boost::noncopyable>("CollisionCallBackBase", bp::no_init)
          .def(
            "getGeometryModel", &CollisionCallBackDefault::getGeometryModel, bp::arg("self"),
            bp::return_value_policy<bp::copy_const_reference>())
          .def(
            "getGeometryData",
            (GeometryData & (CollisionCallBackDefault::*)())
              & CollisionCallBackDefault::getGeometryData,
            bp::arg("self"), bp::return_internal_reference<>())

          .def_readonly(
            "collision", &CollisionCallBackDefault::collision,
            "Whether there is a collision or not.")
          .def_readonly(
            "accumulate", &CollisionCallBackDefault::accumulate,
            "Whether the callback is used in an accumulate mode where several collide "
            "methods are called successively.")

          .def(
            "stop", bp::pure_virtual(&Base::stop), bp::arg("self"),
            "If true, the stopping criteria related to the collision callback has been met and "
            "one can stop.")
          .def(
            "done", &Base::done, &CollisionCallBackBaseWrapper::done_default,
            "Callback method called after the termination of a collisition detection algorithm.");
      }
    };

    void exposeBroadphaseCallbacks()
    {
      CollisionCallBackBaseWrapper::expose();

      bp::class_<CollisionCallBackDefault, bp::bases<CollisionCallBackBase>>(
        "CollisionCallBackDefault", bp::no_init)
        .def(bp::init<const GeometryModel &, GeometryData &, bp::optional<bool>>(
          bp::args("self", "geometry_model", "geometry_data", "stopAtFirstCollision"),
          "Default constructor from a given GeometryModel and a GeometryData")
               [bp::with_custodian_and_ward<1, 2>(), bp::with_custodian_and_ward<1, 3>()])

        .def_readwrite(
          "stopAtFirstCollision", &CollisionCallBackDefault::stopAtFirstCollision,
          "Whether to stop or not when localizing a first collision")
        .def_readonly(
          "collisionPairIndex", &CollisionCallBackDefault::collisionPairIndex,
          "The collision index of the first pair in collision")
        .def_readonly(
          "count", &CollisionCallBackDefault::count, "Number of visits of the collide method");
    }

  } // namespace python
} // namespace pinocchio
