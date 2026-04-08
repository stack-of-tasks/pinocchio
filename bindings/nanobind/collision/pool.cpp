// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"
#include "pinocchio/bindings/python-nb/utils/copyable.hpp"

#include "pinocchio/multibody/pool.hpp"

#include "pinocchio/collision/broadphase.hpp"
#include "pinocchio/collision/tree-broadphase-manager.hpp"
#include "pinocchio/collision/pool/broadphase-manager.hpp"

#include <coal/broadphase/broadphase_dynamic_AABB_tree.h>

#include <nanobind/stl/bind_vector.h>
#include <nanobind/eigen/dense.h>

#include <boost/algorithm/string/replace.hpp>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

using namespace nb::literals;

using GeometryPool = GeometryPoolTpl<Scalar, Options>;
using GeometryModelVector = GeometryPool::GeometryModelVector;
using GeometryDataVector = GeometryPool::GeometryDataVector;

template<template<typename, typename> class PoolTpl, typename Manager>
static void exposeOneBroadPhasePool(nb::module_ m, const char * base_name, const char * doc)
{
  using Pool = PoolTpl<Manager, Scalar>;
  using BroadPhaseManagerVector = typename Pool::BroadPhaseManagerVector;
  using BroadPhaseManagerType = typename Pool::BroadPhaseManager;

  std::string manager_name = boost::typeindex::type_id<Manager>().pretty_name();
  boost::algorithm::replace_all(manager_name, "coal::", "");
  std::string class_name = base_name;
  class_name += "_" + manager_name;

  nb::bind_vector<BroadPhaseManagerVector>(m, ("StdVec_" + class_name).c_str());

  nb::class_<Pool, GeometryPool>(m, class_name.c_str(), doc)
    .def(
      nb::init<const Model &, const GeometryModel &, size_t>(), "model"_a, "geometry_model"_a,
      "size"_a = (size_t)omp_get_max_threads(), "Default constructor.")
    .def(nb::init<const Pool &>(), "other"_a, "Copy constructor.")
    .def(
      "getBroadPhaseManager",
      (BroadPhaseManagerType & (Pool::*)(const size_t)) & Pool::getBroadPhaseManager, "index"_a,
      "Return a specific broadphase manager.", nb::rv_policy::reference_internal)
    .def(
      "getBroadPhaseManagers",
      (BroadPhaseManagerVector & (Pool::*)()) & Pool::getBroadPhaseManagers,
      "Returns the vector of broadphase managers.", nb::rv_policy::reference_internal)
    .def(
      "update", (void (Pool::*)(const GeometryData &))&Pool::update, "geometry_data"_a,
      "Update all the geometry datas with the input geometry data value.")
    .def("check", &Pool::check, "Check whether the current pool is valid.")
    .def(CopyableVisitor<Pool>());
}

void exposePoolCollision(nb::module_ m)
{
  nb::bind_vector<GeometryModelVector, nb::rv_policy::reference_internal>(
    m, "StdVec_GeometryModel");
  nb::bind_vector<GeometryDataVector, nb::rv_policy::reference_internal>(m, "StdVec_GeometryData");

  nb::class_<GeometryPool, ModelPoolTpl<Scalar, Options>>(
    m, "GeometryPool",
    "Pool containing a model + a geometry_model and several datas for parallel computations")
    .def(
      nb::init<const Model &, const GeometryModel &, size_t>(), "model"_a, "geometry_model"_a,
      "size"_a = (size_t)omp_get_max_threads())
    .def(nb::init<const GeometryPool &>(), "other"_a, "Copy constructor.")
    .def(
      "getGeometryModel",
      (GeometryModel & (GeometryPool::*)(const size_t)) & GeometryPool::getGeometryModel, "index"_a,
      "Return a specific GeometryModel.")
    .def(
      "getGeometryModels",
      (GeometryModelVector & (GeometryPool::*)()) & GeometryPool::getGeometryModels,
      "Returns the vector of geometry models.")
    .def(
      "getGeometryData",
      (GeometryData & (GeometryPool::*)(const size_t)) & GeometryPool::getGeometryData,
      nb::rv_policy::reference_internal, "index"_a, "Return a specific geometry data.")
    .def(
      "getGeometryDatas",
      (GeometryDataVector & (GeometryPool::*)()) & GeometryPool::getGeometryDatas,
      nb::rv_policy::reference_internal, "Returns the geometry data vector.")
    .def(
      "sync", &GeometryPool::sync, "geometry_model"_a, "geometry_indexes"_a,
      "Synchronize the internal geometry models with the input geometry for all given "
      "geometry indexes.")
    .def(
      "update", (void (GeometryPool::*)(const GeometryData &))&GeometryPool::update,
      "geometry_data"_a, "Update all the geometry datas with the input geometry data value.")
    .def(CopyableVisitor<GeometryPool>());

  exposeOneBroadPhasePool<BroadPhaseManagerPool, coal::DynamicAABBTreeCollisionManager>(
    m, "BroadPhaseManagerPool",
    "Pool containing a bunch of BroadPhaseManagers for coal::DynamicAABBTreeCollisionManager.");
  exposeOneBroadPhasePool<TreeBroadPhaseManagerPool, coal::DynamicAABBTreeCollisionManager>(
    m, "TreeBroadPhaseManagerPool",
    "Pool containing a bunch of TreeBroadPhaseManagers for coal::DynamicAABBTreeCollisionManager.");
}

PINOCCHIO_PYTHON_NAMESPACE_END
