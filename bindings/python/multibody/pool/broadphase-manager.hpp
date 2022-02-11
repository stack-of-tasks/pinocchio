//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_python_multibody_pool_broadphase_manager_hpp__
#define __pinocchio_python_multibody_pool_broadphase_manager_hpp__

#include <eigenpy/eigen-to-python.hpp>

#include "pinocchio/multibody/pool/broadphase-manager.hpp"

#include <boost/python/overloads.hpp>
#include <eigenpy/memory.hpp>
#include <eigenpy/exception.hpp>

#include "pinocchio/algorithm/check.hpp"
#include "pinocchio/bindings/python/utils/copyable.hpp"
#include "pinocchio/bindings/python/utils/std-vector.hpp"

//EIGENPY_DEFINE_STRUCT_ALLOCATOR_SPECIALIZATION(pinocchio::BroadPhaseManagerPool)

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    template<typename BroadPhaseManagerPool>
    struct BroadPhaseManagerPoolPythonVisitor
    : public bp::def_visitor< BroadPhaseManagerPoolPythonVisitor<BroadPhaseManagerPool> >
    {
      
      typedef typename BroadPhaseManagerPool::Base Base;
      typedef typename BroadPhaseManagerPool::Model Model;
      typedef typename BroadPhaseManagerPool::GeometryModel GeometryModel;
      typedef typename BroadPhaseManagerPool::GeometryData GeometryData;
      typedef typename BroadPhaseManagerPool::BroadPhaseManagerVector BroadPhaseManagerVector;
      typedef typename BroadPhaseManagerPool::BroadPhaseManager BroadPhaseManager;

      /* --- Exposing C++ API to python through the handler ----------------- */
      template<class PyClass>
      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<const Model *, const GeometryModel *,bp::optional<size_t> >(bp::args("self","model","geometry_model","size"),
                                                              "Default constructor.")
             [bp::with_custodian_and_ward<1,2>(),bp::with_custodian_and_ward<1,3>()])
        .def(bp::init<BroadPhaseManagerPool>(bp::args("self","other"),
                                    "Copy constructor."))
        
        .def("getBroadPhaseManager",(BroadPhaseManager & (BroadPhaseManagerPool::*)(const size_t))&BroadPhaseManagerPool::getBroadPhaseManager,
             bp::args("self","index"),"Return a specific broadphase manager.",
             bp::return_internal_reference<>())
        .def("getBroadPhaseManagers",(BroadPhaseManagerVector & (BroadPhaseManagerPool::*)())&BroadPhaseManagerPool::getBroadPhaseManagers,
             bp::arg("self"),"Returns the vector of broadphase managers.",
             bp::return_internal_reference<>())
        .def("update",(void (BroadPhaseManagerPool::*)(const GeometryData &))&BroadPhaseManagerPool::update,
             bp::args("self","geometry_data"),"Update all the geometry datas with the input geometry data value.")
        
        .def("check",&BroadPhaseManagerPool::check,
             bp::arg("self"),
             "Check whether the current pool is valid.")
        ;
      }
      
      static void expose()
      {

        bp::class_<BroadPhaseManagerPool,bp::bases<Base> >("BroadPhaseManagerPool",
                                                  "Pool containing a bunch of BroadPhaseManager",
                                                  bp::no_init)
        .def(BroadPhaseManagerPoolPythonVisitor())
        .def(CopyableVisitor<BroadPhaseManagerPool>())
        ;
        
        StdVectorPythonVisitor<BroadPhaseManagerVector>::expose("StdVec_BroadPhaseManager_");
      }
    };
  }
}

#endif // ifnded __pinocchio_python_multibody_pool_broadphase_manager_hpp__
