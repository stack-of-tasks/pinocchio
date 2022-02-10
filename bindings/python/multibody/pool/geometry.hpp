//
// Copyright (c) 2021-2022 INRIA
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

EIGENPY_DEFINE_STRUCT_ALLOCATOR_SPECIALIZATION(pinocchio::GeometryPool)

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
        .def(bp::init<const Model *, const GeometryModel *,bp::optional<int> >(bp::args("self","model","geometry_model","size"),
                                                              "Default constructor.")
             [bp::with_custodian_and_ward<1,2>(),bp::with_custodian_and_ward<1,3>()])
        .def(bp::init<GeometryPool>(bp::args("self","other"),
                                    "Copy constructor."))
        
        .def("getGeometryModel",(GeometryModel & (GeometryPool::*)())&GeometryPool::getGeometryModel,
             bp::arg("self"),"Geometry model contained in the pool.",
             bp::return_internal_reference<>())
        .def("getGeometryData",(GeometryData & (GeometryPool::*)(const size_t))&GeometryPool::getGeometryData,
             bp::args("self","index"),"Return a specific geometry_data data.",
             bp::return_internal_reference<>())
        .def("getGeometryDatas",(GeometryDataVector & (GeometryPool::*)())&GeometryPool::getGeometryDatas,
             bp::arg("self"),"Returns the geometry data vector.",
             bp::return_internal_reference<>())
        
        .def("update",(void (GeometryPool::*)(const GeometryData &))&GeometryPool::update,
             bp::args("self","geometry_data"),"Update all the geometry datas with the input geometry data value.")
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
        
        StdVectorPythonVisitor<GeometryDataVector>::expose("StdVec_GeometryData");
      }
    };
  }
}

#endif // ifnded __pinocchio_python_multibody_pool_geometry_hpp__
