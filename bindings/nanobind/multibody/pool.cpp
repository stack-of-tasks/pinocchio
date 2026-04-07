// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"

#include "pinocchio/multibody/pool.hpp"

#include <nanobind/stl/bind_vector.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
namespace nb = nanobind;

using namespace nb::literals;

void exposePool(nb::module_ m)
{
  using ModelPool = pinocchio::ModelPool;
  using ModelVector = ModelPool::ModelVector;
  using DataVector = ModelPool::DataVector;

  nb::bind_vector<ModelVector, nb::rv_policy::reference_internal>(m, "StdVec_Model");
  nb::bind_vector<DataVector, nb::rv_policy::reference_internal>(m, "StdVec_Data");

  nb::class_<ModelPool>(
    m, "ModelPool", "Pool containing a model and several datas for parallel computations.")
    .def(
      nb::init<const Model &, size_t>(), "model"_a, "size"_a = (size_t)omp_get_max_threads(),
      "Default constructor.")
    .def(nb::init<const ModelPool &>(), "other"_a, "Copy constructor.")

    .def(
      "getModel", (Model & (ModelPool::*)(const size_t)) & ModelPool::getModel, "index"_a,
      "Return a specific model.", nb::rv_policy::reference_internal)
    .def(
      "getModels", (ModelVector & (ModelPool::*)()) & ModelPool::getModels,
      "Returns the model vectors.", nb::rv_policy::reference_internal)

    .def(
      "getData", (Data & (ModelPool::*)(const size_t)) & ModelPool::getData, "index"_a,
      "Return a specific data.", nb::rv_policy::reference_internal)
    .def(
      "getDatas", (DataVector & (ModelPool::*)()) & ModelPool::getDatas,
      "Returns the data vectors.", nb::rv_policy::reference_internal)

    .def("size", &ModelPool::size, "Returns the size of the pool.")
    .def("resize", &ModelPool::resize, "new_size"_a, "Resize the pool.")

    .def(
      "update", (void (ModelPool::*)(const Data &))&ModelPool::update, "data"_a,
      "Update all the datas with the input data value.");
}

PINOCCHIO_PYTHON_NAMESPACE_END
