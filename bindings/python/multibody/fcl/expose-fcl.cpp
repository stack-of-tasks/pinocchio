//
// Copyright (c) 2017-2018 CNRS
//

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/multibody/fcl/contact.hpp"
#include "pinocchio/bindings/python/multibody/fcl/collision-result.hpp"
#include "pinocchio/bindings/python/multibody/fcl/distance-result.hpp"
#include "pinocchio/bindings/python/multibody/fcl/collision-geometry.hpp"
#include "pinocchio/bindings/python/multibody/fcl/mesh-loader.hpp"

#include "pinocchio/bindings/python/utils/std-vector.hpp"

namespace pinocchio
{
  namespace python
  {
    void exposeFCL()
    {
      using namespace pinocchio::python::fcl;
      ContactPythonVisitor::expose();
      StdVectorPythonVisitor<ContactPythonVisitor::Contact>::expose("StdVec_Contact");
      
      CollisionResultPythonVisitor::expose();
      StdVectorPythonVisitor<CollisionResultPythonVisitor::CollisionResult>::expose("StdVec_CollisionResult");
      
      DistanceResultPythonVisitor::expose();
      StdVectorPythonVisitor<DistanceResultPythonVisitor::DistanceResult>::expose("StdVec_DistanceResult");
      
      CollisionGeometryPythonVisitor::expose();

      MeshLoaderPythonVisitor::expose();
    }
    
  } // namespace python
} // namespace pinocchio
