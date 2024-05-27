//
// Copyright (c) 2021-2022 INRIA
//

#ifndef __pinocchio_python_collision_pool_geometry_hpp__
#define __pinocchio_python_collision_pool_geometry_hpp__

#include <boost/python/overloads.hpp>

#include <eigenpy/eigen-to-python.hpp>
#include <eigenpy/memory.hpp>
#include <eigenpy/exception.hpp>

#include "pinocchio/bindings/python/utils/copyable.hpp"
#include "pinocchio/bindings/python/utils/std-vector.hpp"

#include "pinocchio/algorithm/check.hpp"
#include "pinocchio/multibody/pool/geometry.hpp"

#if EIGENPY_VERSION_AT_MOST(2, 8, 1)
EIGENPY_DEFINE_STRUCT_ALLOCATOR_SPECIALIZATION(pinocchio::GeometryPool)
#endif

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    template<typename GeometryPool>
    struct GeometryPoolPythonVisitor
    : public bp::def_visitor<GeometryPoolPythonVisitor<GeometryPool>>
    {

      typedef typename GeometryPool::Base Base;
      typedef typename GeometryPool::Model Model;
      typedef typename GeometryPool::GeometryModel GeometryModel;
      typedef typename GeometryPool::GeometryData GeometryData;
      typedef typename GeometryPool::GeometryModelVector GeometryModelVector;
      typedef typename GeometryPool::GeometryDataVector GeometryDataVector;

      /* --- Exposing C++ API to python through the handler ----------------- */
      template<class PyClass>
      void visit(PyClass & cl) const
      {
        cl.def(bp::init<const Model &, const GeometryModel &, bp::optional<size_t>>(
                 bp::args("self", "model", "geometry_model", "size"), "Default constructor."))
          .def(bp::init<const GeometryPool &>(bp::args("self", "other"), "Copy constructor."))

          .def(
            "getGeometryModel",
            (GeometryModel & (GeometryPool::*)(const size_t)) & GeometryPool::getGeometryModel,
            bp::args("self", "index"), "Return a specific geometry model.",
            bp::return_internal_reference<>())
          .def(
            "getGeometryModels",
            (GeometryModelVector & (GeometryPool::*)()) & GeometryPool::getGeometryModels,
            bp::arg("self"), "Returns the geometry model vector.",
            bp::return_internal_reference<>())

          .def(
            "getGeometryData",
            (GeometryData & (GeometryPool::*)(const size_t)) & GeometryPool::getGeometryData,
            bp::args("self", "index"), "Return a specific geometry data.",
            bp::return_internal_reference<>())
          .def(
            "getGeometryDatas",
            (GeometryDataVector & (GeometryPool::*)()) & GeometryPool::getGeometryDatas,
            bp::arg("self"), "Returns the geometry data vector.", bp::return_internal_reference<>())

          .def(
            "sync", &GeometryPool::sync, bp::args("geometry_model", "geometry_indexes"),
            "Synchronize the internal geometry models with the input geometry for all given "
            "geometry indexes.")

          .def(
            "update", (void(GeometryPool::*)(const GeometryData &)) & GeometryPool::update,
            bp::args("self", "geometry_data"),
            "Update all the geometry datas with the input geometry data value.");
      }

      static void expose()
      {

        bp::class_<GeometryPool, bp::bases<Base>>(
          "GeometryPool",
          "Pool containing a model + a geometry_model and several datas for parallel computations",
          bp::no_init)
          .def(GeometryPoolPythonVisitor())
          .def(CopyableVisitor<GeometryPool>());

        StdVectorPythonVisitor<GeometryModelVector>::expose("StdVec_GeometryModel");
        StdVectorPythonVisitor<GeometryDataVector>::expose("StdVec_GeometryData");
      }
    };
  } // namespace python
} // namespace pinocchio

#endif // ifnded __pinocchio_python_collision_pool_geometry_hpp__
