//
// Copyright (c) 2015 CNRS
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
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

#include "pinocchio/python/python.hpp"
#include "pinocchio/python/se3.hpp"
#include "pinocchio/python/force.hpp"
#include "pinocchio/python/motion.hpp"
#include "pinocchio/python/inertia.hpp"
#include "pinocchio/python/joint-dense.hpp"
#include "pinocchio/python/joints-variant.hpp"

#include "pinocchio/python/model.hpp"
#include "pinocchio/python/data.hpp"
#include "pinocchio/python/algorithms.hpp"
#include "pinocchio/python/parsers.hpp"
#include "pinocchio/python/explog.hpp"

#ifdef WITH_HPP_FCL
#include "pinocchio/python/geometry-model.hpp"
#include "pinocchio/python/geometry-data.hpp"
#endif

namespace se3
{
  namespace python
  {
    void exposeSE3()
    {
      SE3PythonVisitor<SE3>::expose();
      PyWraperForAlignedStdVector<SE3>::expose("StdVect_SE3");
    }
    void exposeForce()
    {
      ForcePythonVisitor<Force>::expose();
      PyWraperForAlignedStdVector<Force>::expose("StdVec_Force");
    }
    void exposeMotion()
    {
      MotionPythonVisitor<Motion>::expose();
      PyWraperForAlignedStdVector<Motion>::expose("StdVec_Motion");
    }
    void exposeInertia()
    {
      InertiaPythonVisitor<Inertia>::expose();
      PyWraperForAlignedStdVector<Inertia>::expose("StdVec_Inertia");
    }
    void exposeJoints()
    {
      exposeVariants();
    }
    void exposeModel()
    {
      ModelPythonVisitor::expose();
      DataPythonVisitor::expose();
#ifdef WITH_HPP_FCL      
      CollisionPairPythonVisitor::expose();
      GeometryModelPythonVisitor::expose();
      GeometryDataPythonVisitor::expose();
#endif      
    }
    void exposeAlgorithms()
    {
      AlgorithmsPythonVisitor::expose();
    }
    void exposeParsers()
    {
      ParsersPythonVisitor::expose();
    }
    void exposeExplog()
    {
      ExplogPythonVisitor::expose();
    }
  }} // namespace se3::python
