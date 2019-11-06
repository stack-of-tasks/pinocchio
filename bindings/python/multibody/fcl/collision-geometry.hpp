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
          
          bp::enum_< ::hpp::fcl::OBJECT_TYPE >("OBJECT_TYPE")
          .value("OT_UNKNOWN",::hpp::fcl::OT_UNKNOWN)
          .value("OT_BVH",::hpp::fcl::OT_BVH)
          .value("OT_GEOM",::hpp::fcl::OT_GEOM)
          .value("OT_OCTREE",::hpp::fcl::OT_OCTREE)
          .value("OT_COUNT",::hpp::fcl::OT_COUNT)
          ;
          
          bp::enum_< ::hpp::fcl::NODE_TYPE >("NODE_TYPE")
          .value("BV_UNKNOWN",::hpp::fcl::BV_UNKNOWN)
          .value("BV_AABB",::hpp::fcl::BV_AABB)
          .value("BV_OBB",::hpp::fcl::BV_OBB)
          .value("BV_RSS",::hpp::fcl::BV_RSS)
          .value("BV_kIOS",::hpp::fcl::BV_kIOS)
          .value("BV_OBBRSS",::hpp::fcl::BV_OBBRSS)
          .value("BV_KDOP16",::hpp::fcl::BV_KDOP16)
          .value("BV_KDOP18",::hpp::fcl::BV_KDOP18)
          .value("BV_KDOP24",::hpp::fcl::BV_KDOP24)
          .value("GEOM_BOX",::hpp::fcl::GEOM_BOX)
          .value("GEOM_SPHERE",::hpp::fcl::GEOM_SPHERE)
          .value("GEOM_CAPSULE",::hpp::fcl::GEOM_CAPSULE)
          .value("GEOM_CONE",::hpp::fcl::GEOM_CONE)
          .value("GEOM_CYLINDER",::hpp::fcl::GEOM_CYLINDER)
          .value("GEOM_CONVEX",::hpp::fcl::GEOM_CONVEX)
          .value("GEOM_PLANE",::hpp::fcl::GEOM_PLANE)
          .value("GEOM_HALFSPACE",::hpp::fcl::GEOM_HALFSPACE)
          .value("GEOM_TRIANGLE",::hpp::fcl::GEOM_TRIANGLE)
          .value("GEOM_OCTREE",::hpp::fcl::GEOM_OCTREE)
          .value("NODE_COUNT",::hpp::fcl::NODE_COUNT)
          ;
        }
        
      private:
        
        
      };
      
    } // namespace fcl
    
  } // namespace python
} // namespace pinocchio

#endif // namespace __pinocchio_python_fcl_collision_geometry_hpp__
