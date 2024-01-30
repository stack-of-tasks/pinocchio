//
// Copyright (c) 2021 INRIA
//

#ifndef __pinocchio_python_multibody_pool_geometry_hpp__
#define __pinocchio_python_multibody_pool_geometry_hpp__

#include <eigenpy/eigen-to-python.hpp>

#include "pinocchio/multibody/pool/geometry.hpp"

#include <boost/python/overloads.hpp>
#include <eigenpy/memory.hpp>
#include <eigenpy/exception.hpp>

#include "pinocchio/algorithm/check.hpp"
#include "pinocchio/bindings/python/utils/copyable.hpp"
#include "pinocchio/bindings/python/utils/std-vector.hpp"

#if EIGENPY_VERSION_AT_MOST(2,8,1)
EIGENPY_DEFINE_STRUCT_ALLOCATOR_SPECIALIZATION(pinocchio::GeometryPool)
#endif

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    template<typename GeometryPool>
    struct GeometryPoolPythonVisitor
    : public bp::def_visitor< GeometryPoolPythonVisitor<GeometryPool> >
    {
      
      typedef typename GeometryPool::Base Base;
      typedef typename GeometryPool::Model Model;
      typedef typename GeometryPool::GeometryModel GeometryModel;
      typedef typename GeometryPool::GeometryData GeometryData;
      typedef typename GeometryPool::GeometryDataVector GeometryDataVector;

      /* --- Exposing C++ API to python through the handler ----------------- */
      template<class PyClass>
      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<Model,GeometryModel,bp::optional<int> >(bp::args("self","model","geometry_model","size"),
                                                              "Default constructor."))
        .def(bp::init<GeometryPool>(bp::args("self","other"),
                                    "Copy constructor."))
        
        .def("geometry_model",(GeometryModel & (GeometryPool::*)())&GeometryPool::geometry_model,
             bp::arg("self"),"Geometry model contained in the pool.",
             bp::return_internal_reference<>())
        .def("geometry_data",(GeometryData & (GeometryPool::*)(const size_t))&GeometryPool::geometry_data,
             bp::args("self","index"),"Return a specific geometry_data data.",
             bp::return_internal_reference<>())
        .def("geometry_datas",(GeometryDataVector & (GeometryPool::*)())&GeometryPool::geometry_datas,
             bp::arg("self"),"Returns the geometry data vector.",
             bp::return_internal_reference<>())
        
        .def("update",(void (GeometryPool::*)(const GeometryModel &))&GeometryPool::update,
             bp::args("self","geometry_model"),
             "Update the geometry model, meaning that all the datas will be refreshed accordingly.")
        .def("update",(void (GeometryPool::*)(const GeometryData &))&GeometryPool::update,
             bp::args("self","geometry_data"),"Update all the geometry datas with the input geometry data value.")
        .def("update",(void (GeometryPool::*)(const GeometryModel &, const GeometryData &))&GeometryPool::update,
             bp::args("self","geometry_model","geometry_data"),
             "Update the geometry model and data together.")
        ;
      }
      
      static void expose()
      {

        bp::class_<GeometryPool,bp::bases<Base> >("GeometryPool",
                                                  "Pool containing a model + a geometry_model and several datas for parallel computations",
                                                  bp::no_init)
        .def(GeometryPoolPythonVisitor())
        .def(CopyableVisitor<GeometryPool>())
        ;
        
        StdVectorPythonVisitor<GeometryData,typename GeometryDataVector::allocator_type>::expose("StdVec_GeometryData");
      }
    };
  }
}

#endif // ifnded __pinocchio_python_multibody_pool_geometry_hpp__
