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

EIGENPY_DEFINE_STRUCT_ALLOCATOR_SPECIALIZATION(pinocchio::ModelPool)

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    template<typename ModelPool>
    struct ModelPoolPythonVisitor
    : public bp::def_visitor< ModelPoolPythonVisitor<ModelPool> >
    {
      
      typedef typename ModelPool::Model Model;
      typedef typename ModelPool::Data Data;
      typedef typename ModelPool::DataVector DataVector;

      /* --- Exposing C++ API to python through the handler ----------------- */
      template<class PyClass>
      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<const Model *,bp::optional<int> >(bp::args("self","model","size"),"Default constructor.")
             [bp::with_custodian_and_ward<1,2>()])
        .def(bp::init<ModelPool>(bp::args("self","other"),
                                  "Copy constructor."))
        
        .def("getModel",(Model & (ModelPool::*)())&ModelPool::getModel,
             bp::arg("self"),"Model contained in the pool.",
             bp::return_internal_reference<>())
        .def("getData",(Data & (ModelPool::*)(const size_t))&ModelPool::getData,
             bp::args("self","index"),"Return a specific data.",
             bp::return_internal_reference<>())
        .def("getDatas",(DataVector & (ModelPool::*)())&ModelPool::getDatas,
             bp::arg("self"),"Returns the data vectors.",
             bp::return_internal_reference<>())
        
        .def("size",&ModelPool::size,bp::arg("self"),
             "Returns the size of the pool.")
        .def("resize",&ModelPool::resize,bp::args("self","new_size"),
             "Resize the pool.")
             
        .def("update",(void (ModelPool::*)(const Data &))&ModelPool::update,
             bp::args("self","data"),"Update all the datas with the input data value.")
        ;
      }
      
      static void expose()
      {

        bp::class_<ModelPool>("ModelPool",
                              "Pool containing a model and several datas for parallel computations",
                              bp::no_init)
        .def(ModelPoolPythonVisitor())
        .def(CopyableVisitor<ModelPool>())
        ;
        
        StdVectorPythonVisitor<DataVector>::expose("StdVec_Data");
      }
    };
  }
}

#endif // ifnded __pinocchio_python_multibody_pool_model_hpp__
