//
// Copyright (c) 2017 CNRS
//

#ifndef __pinocchio_python_fcl_collision_geometry_hpp__
#define __pinocchio_python_fcl_collision_geometry_hpp__

#include "pinocchio/spatial/fcl-pinocchio-conversions.hpp"
#include <hpp/fcl/collision_object.h>

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
      
      struct CollisionGeometryPythonVisitor : public bp::def_visitor<CollisionGeometryPythonVisitor>
      {
        typedef ::fcl::CollisionGeometry CollisionGeometry;
        
        template<class PyClass>
        void visit(PyClass& cl) const
        {
          cl
          
          .def("getObjectType",&CollisionGeometry::getObjectType,"Get the type of the object.")
          .def("getNodeType",&CollisionGeometry::getNodeType,"Get the node type.")
          
          .def("computeLocalAABB",&CollisionGeometry::computeLocalAABB)
          .def("isOccupied",&CollisionGeometry::isOccupied)
          .def("isFree",&CollisionGeometry::isFree)
          .def("isUncertain",&CollisionGeometry::isUncertain)
          
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
          
          bp::enum_< ::fcl::OBJECT_TYPE >("OBJECT_TYPE")
          .value("OT_UNKNOWN",::fcl::OT_UNKNOWN)
          .value("OT_BVH",::fcl::OT_BVH)
          .value("OT_GEOM",::fcl::OT_GEOM)
          .value("OT_OCTREE",::fcl::OT_OCTREE)
          .value("OT_COUNT",::fcl::OT_COUNT)
          ;
          
          bp::enum_< ::fcl::NODE_TYPE >("NODE_TYPE")
          .value("BV_UNKNOWN",::fcl::BV_UNKNOWN)
          .value("BV_AABB",::fcl::BV_AABB)
          .value("BV_OBB",::fcl::BV_OBB)
          .value("BV_RSS",::fcl::BV_RSS)
          .value("BV_kIOS",::fcl::BV_kIOS)
          .value("BV_OBBRSS",::fcl::BV_OBBRSS)
          .value("BV_KDOP16",::fcl::BV_KDOP16)
          .value("BV_KDOP18",::fcl::BV_KDOP18)
          .value("BV_KDOP24",::fcl::BV_KDOP24)
          .value("GEOM_BOX",::fcl::GEOM_BOX)
          .value("GEOM_SPHERE",::fcl::GEOM_SPHERE)
          .value("GEOM_CAPSULE",::fcl::GEOM_CAPSULE)
          .value("GEOM_CONE",::fcl::GEOM_CONE)
          .value("GEOM_CYLINDER",::fcl::GEOM_CYLINDER)
          .value("GEOM_CONVEX",::fcl::GEOM_CONVEX)
          .value("GEOM_PLANE",::fcl::GEOM_PLANE)
          .value("GEOM_HALFSPACE",::fcl::GEOM_HALFSPACE)
          .value("GEOM_TRIANGLE",::fcl::GEOM_TRIANGLE)
          .value("GEOM_OCTREE",::fcl::GEOM_OCTREE)
          .value("NODE_COUNT",::fcl::NODE_COUNT)
          ;
        }
        
      private:
        
        
      };
      
    } // namespace fcl
    
  } // namespace python
} // namespace pinocchio

#endif // namespace __pinocchio_python_fcl_collision_geometry_hpp__
