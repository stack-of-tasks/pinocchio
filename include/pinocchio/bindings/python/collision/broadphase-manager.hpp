//
// Copyright (c) 2022-2024 INRIA
//

#ifndef __pinocchio_python_collision_broadphase_manager_hpp__
#define __pinocchio_python_collision_broadphase_manager_hpp__

#include "pinocchio/collision/broadphase-manager.hpp"
#include "pinocchio/bindings/python/collision/broadphase-manager-base.hpp"

#include <eigenpy/eigen-to-python.hpp>
#include <boost/algorithm/string/replace.hpp>

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    template<typename Derived>
    struct BroadPhaseManagerPythonVisitor
    : public bp::def_visitor<BroadPhaseManagerPythonVisitor<Derived>>
    {
    public:
      typedef BroadPhaseManagerTpl<Derived> Self;
      typedef typename Self::CollisionObjectVector CollisionObjectVector;

      /* --- Exposing C++ API to python through the handler ----------------- */
      template<class PyClass>
      void visit(PyClass & cl) const
      {
        cl
          .def(bp::init<const Model *, const GeometryModel *, GeometryData *>(
            bp::args("self", "model", "geometry_model", "geometry_data"), "Default constructor")
                 [bp::with_custodian_and_ward<1, 2>(), bp::with_custodian_and_ward<1, 3>(),
                  bp::with_custodian_and_ward<1, 4>()])
          .def(bp::init<const Self &>(
            bp::args("self", "other"), "Copy constructor")[bp::with_custodian_and_ward<1, 2>()])

          .def(
            "getCollisionObjectInflation", &Self::getCollisionObjectInflation, bp::arg("self"),
            "Returns the inflation value related to each collision object.",
            bp::return_internal_reference<>())
          .def(
            "getCollisionObjects",
            (CollisionObjectVector & (Self::*)()) & Self::getCollisionObjects, bp::arg("self"),
            bp::return_internal_reference<>())

          .def(
            "getManager", (Derived & (Self::*)()) & Self::getManager, bp::arg("self"),
            "Returns the internal FCL manager", bp::return_internal_reference<>())

          .def(
            "getCollisionObjectStatus", &Self::getCollisionObjectStatus, bp::arg("self"),
            "Returns the status of the collision object.",
            bp::return_value_policy<bp::copy_const_reference>())

          .def(BroadPhaseManagerBasePythonVisitor<Self>());
      }

      /* --- Expose --------------------------------------------------------- */
      static void expose()
      {
        std::string derived_name = boost::typeindex::type_id<Derived>().pretty_name();
        boost::algorithm::replace_all(derived_name, "coal::", "");
        const std::string class_name = "BroadPhaseManager_" + derived_name;

        const std::string class_doc = "Broad phase manager associated to coal::" + derived_name;
        bp::class_<Self> registered_class(class_name.c_str(), class_doc.c_str(), bp::no_init);
        registered_class.def(BroadPhaseManagerPythonVisitor());

        typedef BroadPhaseManagerBase<Self> Base;
        bp::objects::register_dynamic_id<Base>();
        bp::objects::register_conversion<Self, Base>(false);
      }
    };

  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_collision_broadphase_manager_hpp__
