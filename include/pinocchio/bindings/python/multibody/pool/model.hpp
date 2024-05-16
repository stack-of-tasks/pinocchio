//
// Copyright (c) 2021-2022 INRIA
//

#ifndef __pinocchio_python_multibody_pool_model_hpp__
#define __pinocchio_python_multibody_pool_model_hpp__

#include <eigenpy/eigen-to-python.hpp>

#include "pinocchio/multibody/pool/model.hpp"

#include <boost/python/overloads.hpp>
#include <eigenpy/memory.hpp>
#include <eigenpy/exception.hpp>

#include "pinocchio/algorithm/check.hpp"
#include "pinocchio/bindings/python/utils/copyable.hpp"
#include "pinocchio/bindings/python/utils/std-vector.hpp"

#if EIGENPY_VERSION_AT_MOST(2, 8, 1)
EIGENPY_DEFINE_STRUCT_ALLOCATOR_SPECIALIZATION(pinocchio::ModelPool)
#endif

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    template<typename ModelPool>
    struct ModelPoolPythonVisitor : public bp::def_visitor<ModelPoolPythonVisitor<ModelPool>>
    {

      typedef typename ModelPool::Model Model;
      typedef typename ModelPool::Data Data;
      typedef typename ModelPool::ModelVector ModelVector;
      typedef typename ModelPool::DataVector DataVector;

      /* --- Exposing C++ API to python through the handler ----------------- */
      template<class PyClass>
      void visit(PyClass & cl) const
      {
        cl.def(bp::init<const Model &, bp::optional<size_t>>(
                 bp::args("self", "model", "size"), "Default constructor."))
          .def(bp::init<const ModelPool &>(bp::args("self", "other"), "Copy constructor."))

          .def(
            "getModel", (Model & (ModelPool::*)(const size_t)) & ModelPool::getModel,
            bp::args("self", "index"), "Return a specific model.",
            bp::return_internal_reference<>())
          .def(
            "getModels", (ModelVector & (ModelPool::*)()) & ModelPool::getModels, bp::arg("self"),
            "Returns the model vectors.", bp::return_internal_reference<>())

          .def(
            "getData", (Data & (ModelPool::*)(const size_t)) & ModelPool::getData,
            bp::args("self", "index"), "Return a specific data.", bp::return_internal_reference<>())
          .def(
            "getDatas", (DataVector & (ModelPool::*)()) & ModelPool::getDatas, bp::arg("self"),
            "Returns the data vectors.", bp::return_internal_reference<>())

          .def("size", &ModelPool::size, bp::arg("self"), "Returns the size of the pool.")
          .def("resize", &ModelPool::resize, bp::args("self", "new_size"), "Resize the pool.")

          .def(
            "update", (void(ModelPool::*)(const Data &)) & ModelPool::update,
            bp::args("self", "data"), "Update all the datas with the input data value.");
      }

      static void expose()
      {

        bp::class_<ModelPool>(
          "ModelPool", "Pool containing a model and several datas for parallel computations",
          bp::no_init)
          .def(ModelPoolPythonVisitor())
          .def(CopyableVisitor<ModelPool>());

        StdVectorPythonVisitor<ModelVector>::expose("StdVec_Model");
        StdVectorPythonVisitor<DataVector>::expose("StdVec_Data");
      }
    };
  } // namespace python
} // namespace pinocchio

#endif // ifnded __pinocchio_python_multibody_pool_model_hpp__
