//
// Copyright (c) 2017-2020 CNRS INRIA
//

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/multibody/fcl/contact.hpp"
#include "pinocchio/bindings/python/multibody/fcl/collision-result.hpp"
#include "pinocchio/bindings/python/multibody/fcl/distance-result.hpp"
#include "pinocchio/bindings/python/multibody/fcl/collision-geometry.hpp"
#include "pinocchio/bindings/python/multibody/fcl/mesh-loader.hpp"
#include "pinocchio/bindings/python/multibody/fcl/transform.hpp"

#include "pinocchio/bindings/python/utils/std-vector.hpp"

namespace pinocchio
{
  namespace python
  {
    void exposeFCL()
    {
      using namespace pinocchio::python::fcl;
      namespace bp = boost::python;
      
      if(!eigenpy::register_symbolic_link_to_registered_type< ::hpp::fcl::OBJECT_TYPE>())
      {
        bp::enum_< ::hpp::fcl::OBJECT_TYPE >("OBJECT_TYPE")
        .value("OT_UNKNOWN",::hpp::fcl::OT_UNKNOWN)
        .value("OT_BVH",::hpp::fcl::OT_BVH)
        .value("OT_GEOM",::hpp::fcl::OT_GEOM)
        .value("OT_OCTREE",::hpp::fcl::OT_OCTREE)
        .value("OT_COUNT",::hpp::fcl::OT_COUNT)
        ;
      }
        
      if(!eigenpy::register_symbolic_link_to_registered_type< ::hpp::fcl::NODE_TYPE>())
      {
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
      
      if(!eigenpy::register_symbolic_link_to_registered_type< ::hpp::fcl::Contact>())
      {
        ContactPythonVisitor::expose();
      }
      if(!eigenpy::register_symbolic_link_to_registered_type< std::vector< ::hpp::fcl::Contact> >())
      {
        StdVectorPythonVisitor<ContactPythonVisitor::Contact>::expose("StdVec_Contact");
      }
      
      if(!eigenpy::register_symbolic_link_to_registered_type< ::hpp::fcl::CollisionResult>())
      {
        CollisionResultPythonVisitor::expose();
      }
      if(!eigenpy::register_symbolic_link_to_registered_type< std::vector< ::hpp::fcl::CollisionResult> >())
      {
        StdVectorPythonVisitor<CollisionResultPythonVisitor::CollisionResult>::expose("StdVec_CollisionResult");
      }
      
      if(!eigenpy::register_symbolic_link_to_registered_type< ::hpp::fcl::DistanceResult>())
      {
        DistanceResultPythonVisitor::expose();
      }
      if(!eigenpy::register_symbolic_link_to_registered_type< std::vector< ::hpp::fcl::DistanceResult> >())
      {
        StdVectorPythonVisitor<DistanceResultPythonVisitor::DistanceResult>::expose("StdVec_DistanceResult");
      }
      
      if(!eigenpy::register_symbolic_link_to_registered_type< ::hpp::fcl::CollisionGeometry>())
      {
        CollisionGeometryPythonVisitor::expose();
      }
      
      typedef ::hpp::fcl::MeshLoader MeshLoader;
      typedef ::hpp::fcl::CachedMeshLoader CachedMeshLoader;

      if(!eigenpy::register_symbolic_link_to_registered_type<MeshLoader>())
        MeshLoaderPythonVisitor<MeshLoader>::
        expose("Class to create CollisionGeometry from mesh files.");

      if(!eigenpy::register_symbolic_link_to_registered_type<CachedMeshLoader>())
        MeshLoaderPythonVisitor<CachedMeshLoader>::
        expose("Class to create CollisionGeometry from mesh files with cache mechanism.");
      
      typedef ::hpp::fcl::Transform3f Transform3f;
      
      // Register implicit conversion SE3 <=> ::hpp::fcl::Transform3f
      bp::implicitly_convertible< SE3,Transform3f >();
      bp::implicitly_convertible< Transform3f,SE3 >();
    }
    
  } // namespace python
} // namespace pinocchio
