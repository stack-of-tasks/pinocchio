//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_python_collision_pool_broadphase_manager_hpp__
#define __pinocchio_python_collision_pool_broadphase_manager_hpp__

#include <eigenpy/eigen-to-python.hpp>

#include "pinocchio/collision/pool/broadphase-manager.hpp"

#include <boost/python/overloads.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <eigenpy/memory.hpp>
#include <eigenpy/exception.hpp>

#include "pinocchio/algorithm/check.hpp"
#include "pinocchio/bindings/python/utils/copyable.hpp"
#include "pinocchio/bindings/python/utils/std-vector.hpp"

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    namespace helper
    {
      template<typename BroadPhaseManager>
      struct base_class_name
      {
        static std::string get();
      };

      template<typename Manager>
      struct base_class_name<BroadPhaseManagerTpl<Manager>>
      {
        static std::string get()
        {
          return "BroadPhaseManager";
        }
      };

      template<typename Manager>
      struct base_class_name<TreeBroadPhaseManagerTpl<Manager>>
      {
        static std::string get()
        {
          return "TreeBroadPhaseManager";
        }
      };
    } // namespace helper

    template<typename BroadPhaseManagerPool>
    struct BroadPhaseManagerPoolPythonVisitor
    : public bp::def_visitor<BroadPhaseManagerPoolPythonVisitor<BroadPhaseManagerPool>>
    {

      typedef typename BroadPhaseManagerPool::Base Base;
      typedef typename BroadPhaseManagerPool::Model Model;
      typedef typename BroadPhaseManagerPool::GeometryModel GeometryModel;
      typedef typename BroadPhaseManagerPool::GeometryData GeometryData;
      typedef typename BroadPhaseManagerPool::BroadPhaseManagerVector BroadPhaseManagerVector;
      typedef typename BroadPhaseManagerPool::BroadPhaseManager BroadPhaseManager;
      typedef typename BroadPhaseManager::Manager Manager;

      /* --- Exposing C++ API to python through the handler ----------------- */
      template<class PyClass>
      void visit(PyClass & cl) const
      {
        cl.def(bp::init<const Model &, const GeometryModel &, bp::optional<size_t>>(
                 bp::args("self", "model", "geometry_model", "size"), "Default constructor."))
          .def(
            bp::init<const BroadPhaseManagerPool &>(bp::args("self", "other"), "Copy constructor."))

          .def(
            "getBroadPhaseManager",
            (BroadPhaseManager & (BroadPhaseManagerPool::*)(const size_t))
              & BroadPhaseManagerPool::getBroadPhaseManager,
            bp::args("self", "index"), "Return a specific broadphase manager.",
            bp::return_internal_reference<>())
          .def(
            "getBroadPhaseManagers",
            (BroadPhaseManagerVector & (BroadPhaseManagerPool::*)())
              & BroadPhaseManagerPool::getBroadPhaseManagers,
            bp::arg("self"), "Returns the vector of broadphase managers.",
            bp::return_internal_reference<>())

          .def(
            "update",
            (void(BroadPhaseManagerPool::*)(const GeometryData &)) & BroadPhaseManagerPool::update,
            bp::args("self", "geometry_data"),
            "Update all the geometry datas with the input geometry data value.")

          .def(
            "check", &BroadPhaseManagerPool::check, bp::arg("self"),
            "Check whether the current pool is valid.")

          .def(
            "asGeometryPool", downcast, bp::arg("self"),
            "Cast the object as GeometryPool (equivalent to a C++ static_cast).",
            bp::return_self<>());
      }

      static PyObject * downcast(BroadPhaseManagerPool & self)
      {
        bp::type_info type = bp::type_id<Base>();
        const bp::converter::registration * registration = bp::converter::registry::query(type);

        return registration->to_python(static_cast<Base *>(&self));
      }

      static void expose()
      {
        std::string manager_name = boost::typeindex::type_id<Manager>().pretty_name();
        boost::algorithm::replace_all(manager_name, "hpp::fcl::", "");
        const std::string broadphase_prefix = helper::base_class_name<BroadPhaseManager>::get();
        const std::string class_name = broadphase_prefix + "Pool" + "_" + manager_name;

        const std::string doc = "Pool containing a bunch of " + broadphase_prefix + ".";

        bp::class_<BroadPhaseManagerPool, bp::bases<Base>>(
          class_name.c_str(), doc.c_str(), bp::no_init)
          .def(BroadPhaseManagerPoolPythonVisitor())
          .def(CopyableVisitor<BroadPhaseManagerPool>());

        const std::string vector_name = "StdVec_" + broadphase_prefix + "_" + manager_name;
        StdVectorPythonVisitor<BroadPhaseManagerVector>::expose(vector_name);
      }
    };
  } // namespace python
} // namespace pinocchio

#endif // ifnded __pinocchio_python_collision_pool_broadphase_manager_hpp__
