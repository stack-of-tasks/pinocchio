//
// Copyright (c) 2017 CNRS
//

#ifndef __pinocchio_python_fcl_mesh_loader_hpp__
#define __pinocchio_python_fcl_mesh_loader_hpp__

#include "pinocchio/spatial/fcl-pinocchio-conversions.hpp"
#include <hpp/fcl/mesh_loader/loader.h>

#include <boost/python/copy_const_reference.hpp>
#include <boost/python/return_internal_reference.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

namespace pinocchio
{
  namespace python
  {
    namespace fcl
    {
      
      namespace bp = boost::python;
      
      struct MeshLoaderPythonVisitor : public bp::def_visitor<MeshLoaderPythonVisitor>
      {
        typedef ::hpp::fcl::MeshLoader MeshLoader;
        typedef boost::shared_ptr<MeshLoader> MeshLoaderPtr_t;

        typedef ::hpp::fcl::CachedMeshLoader CachedMeshLoader;
        typedef boost::shared_ptr<CachedMeshLoader> CachedMeshLoaderPtr_t;

        template <typename T>
        static boost::shared_ptr<T> create ()
        {
          return boost::shared_ptr<T> (new T);
        }

        template<class PyClass>
        void visit(PyClass& cl) const
        {
          cl
          .def("create",&MeshLoaderPythonVisitor::create<MeshLoader>,"Create a new object.")
          .staticmethod ("create")
          ;
        }
        
        static void expose()
        {
          bp::class_<MeshLoader, MeshLoaderPtr_t>
            ("MeshLoader",
             "Class to create CollisionGeometry from mesh files.",
             bp::init<>())
            .def(MeshLoaderPythonVisitor())
          ;

          bp::class_<CachedMeshLoader, CachedMeshLoaderPtr_t, bp::bases<MeshLoader> >
            ("CachedMeshLoader",
             "Class to create CollisionGeometry from mesh files with cache mechanism.",
             bp::init<>())
          .def("create",&MeshLoaderPythonVisitor::create<CachedMeshLoader>,"Create a new object.")
          .staticmethod ("create")
          ;
        }
        
      private:
        
        
      };
      
    } // namespace fcl
    
  } // namespace python
} // namespace pinocchio

#endif // namespace __pinocchio_python_fcl_mesh_loader_hpp__
