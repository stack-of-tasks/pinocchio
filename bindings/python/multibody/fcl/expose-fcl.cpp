//
// Copyright (c) 2017-2018 CNRS
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/multibody/fcl/contact.hpp"
#include "pinocchio/bindings/python/multibody/fcl/collision-result.hpp"
#include "pinocchio/bindings/python/multibody/fcl/distance-result.hpp"
#include "pinocchio/bindings/python/multibody/fcl/collision-geometry.hpp"

#include "pinocchio/bindings/python/utils/std-vector.hpp"

namespace se3
{
  namespace python
  {
    void exposeFCL()
    {
      using namespace se3::python::fcl;
      ContactPythonVisitor::expose();
      StdVectorPythonVisitor<ContactPythonVisitor::Contact>::expose("StdVect_Contact");
      
      CollisionResultPythonVisitor::expose();
      StdVectorPythonVisitor<CollisionResultPythonVisitor::CollisionResult>::expose("StdVect_CollisionResult");
      
      DistanceResultPythonVisitor::expose();
      StdVectorPythonVisitor<DistanceResultPythonVisitor::DistanceResult>::expose("StdVect_DistanceResult");
      
      CollisionGeometryPythonVisitor::expose();
    }
    
  } // namespace python
} // namespace se3
