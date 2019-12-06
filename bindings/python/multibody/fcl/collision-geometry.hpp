//
// Copyright (c) 2017-2019 CNRS INRIA
//

#ifndef __pinocchio_python_fcl_collision_geometry_hpp__
#define __pinocchio_python_fcl_collision_geometry_hpp__

#include "pinocchio/spatial/fcl-pinocchio-conversions.hpp"
#include <hpp/fcl/collision_object.h>

#include <boost/python/copy_const_reference.hpp>
#include <boost/python/return_internal_reference.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include <eigenpy/registration.hpp>

namespace pinocchio
{
  namespace python
  {
    namespace fcl
    {
      
      namespace bp = boost::python;
      
      struct CollisionGeometryPythonVisitor : public bp::def_visitor<CollisionGeometryPythonVisitor>
      {
        typedef ::hpp::fcl::CollisionGeometry CollisionGeometry;
        
        template<class PyClass>
        void visit(PyClass& cl) const
        {
          cl
          .def("getObjectType",&CollisionGeometry::getObjectType,"Get the type of the object.")
          .def("getNodeType",&CollisionGeometry::getNodeType,"Get the node type.")
          
          .def("computeLocalAABB",&CollisionGeometry::computeLocalAABB)

          .def_readwrite("aabb_radius",&CollisionGeometry::aabb_radius,"AABB radius")
          ;
        }
        
        static void expose()
        {
          bp::class_<CollisionGeometry,boost::noncopyable>("CollisionGeometry",
                                                           "The geometry for the object for collision or distance computation.",
                                                           bp::no_init)
          .def(CollisionGeometryPythonVisitor())
          ;
        }
        
      };
      
    } // namespace fcl
    
  } // namespace python
} // namespace pinocchio

#endif // namespace __pinocchio_python_fcl_collision_geometry_hpp__
