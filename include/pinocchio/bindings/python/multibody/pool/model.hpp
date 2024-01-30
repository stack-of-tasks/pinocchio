//
// Copyright (c) 2021 INRIA
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

#if EIGENPY_VERSION_AT_MOST(2,8,1)
EIGENPY_DEFINE_STRUCT_ALLOCATOR_SPECIALIZATION(pinocchio::ModelPool)
#endif

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
        .def(bp::init<Model,bp::optional<int> >(bp::args("self","model","size"),
                                                "Default constructor."))
        .def(bp::init<ModelPool>(bp::args("self","other"),
                                  "Copy constructor."))
        
        .def("model",(Model & (ModelPool::*)())&ModelPool::model,
             bp::arg("self"),"Model contained in the pool.",
             bp::return_internal_reference<>())
        .def("data",(Data & (ModelPool::*)(const size_t))&ModelPool::data,
             bp::args("self","index"),"Return a specific data.",
             bp::return_internal_reference<>())
        .def("datas",(DataVector & (ModelPool::*)())&ModelPool::datas,
             bp::arg("self"),"Returns the data vectors.",
             bp::return_internal_reference<>())
        
        .def("size",&ModelPool::size,bp::arg("self"),
             "Returns the size of the pool.")
        .def("resize",&ModelPool::resize,bp::args("self","new_size"),
             "Resize the pool.")
        
        .def("update",(void (ModelPool::*)(const Model &))&ModelPool::update,
             bp::args("self","model"),
             "Update the model, meaning that all the datas will be refreshed accordingly.")
        .def("update",(void (ModelPool::*)(const Data &))&ModelPool::update,
             bp::args("self","data"),"Update all the datas with the input data value.")
        .def("update",(void (ModelPool::*)(const Model &, const Data &))&ModelPool::update,
             bp::args("self","model","data"),"Update the model and data together.")
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
        
        StdVectorPythonVisitor<Data,typename DataVector::allocator_type>::expose("StdVec_Data");
      }
    };
  }
}

#endif // ifnded __pinocchio_python_multibody_pool_model_hpp__
