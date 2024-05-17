//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_python_collision_broadphase_manager_base_hpp__
#define __pinocchio_python_collision_broadphase_manager_base_hpp__

#include "pinocchio/collision/broadphase-manager-base.hpp"

#include <eigenpy/eigen-to-python.hpp>

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    template<typename Derived>
    struct BroadPhaseManagerBasePythonVisitor
    : public bp::def_visitor<BroadPhaseManagerBasePythonVisitor<Derived>>
    {
      typedef Derived Self;

      typedef typename Derived::GeometryModel GeometryModel;
      typedef typename Derived::Model Model;

      static Model & getModel(const Self & self)
      {
        return const_cast<Model &>(self.getModel());
      }

      static GeometryModel & getGeometryModel(const Self & self)
      {
        return const_cast<GeometryModel &>(self.getGeometryModel());
      }

      /* --- Exposing C++ API to python through the handler ----------------- */
      template<class PyClass>
      void visit(PyClass & cl) const
      {

        cl.def(
            "getModel", getModel, bp::arg("self"), "Returns the related model.",
            bp::return_internal_reference<>())
          .def(
            "getGeometryModel", getGeometryModel, bp::arg("self"),
            "Returns the related geometry model.", bp::return_internal_reference<>())
          .def(
            "getGeometryData", (GeometryData & (Self::*)()) & Self::getGeometryData,
            bp::arg("self"), "Returns the related geometry data.",
            bp::return_internal_reference<>())

          .def(
            "check", (bool(Self::*)() const) & Self::check, bp::arg("self"),
            "Check whether the base broad phase manager is aligned with the current "
            "collision_objects.")
          .def(
            "check", (bool(Self::*)(CollisionCallBackBase *) const) & Self::check,
            bp::args("self", "callback"), "Check whether the callback is inline with *this.")

          .def(
            "update", (void(Self::*)(const bool)) & Self::update,
            (bp::arg("self"), bp::arg("compute_local_aabb") = false),
            "Update the manager from the current geometry positions and update the underlying "
            "FCL broad phase manager.")
          .def(
            "update", (void(Self::*)(GeometryData * geom_data_new)) & Self::update,
            (bp::arg("self"), bp::arg("geom_data_new")),
            "Update the manager with a new geometry data.", bp::with_custodian_and_ward<1, 2>())

          .def(
            "collide",
            (bool(Self::*)(CollisionObject &, CollisionCallBackBase *) const) & Self::collide,
            bp::args("self", "collision_object", "callback"),
            "Performs collision test between one object and all the objects belonging to the "
            "manager.")
          .def(
            "collide", (bool(Self::*)(CollisionCallBackBase *) const) & Self::collide,
            bp::args("self", "callback"),
            "Performs collision test for the objects belonging to the manager.")
          .def(
            "collide", (bool(Self::*)(Self &, CollisionCallBackBase *) const) & Self::collide,
            bp::args("self", "other_manager", "callback"),
            "Performs collision test with objects belonging to another manager.");
      }
    };

  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_collision_broadphase_manager_base_hpp__
