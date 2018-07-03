//
// Copyright (c) 2015-2016,2018 CNRS
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

#ifndef __se3_python_algorithm_hpp__
#define __se3_python_algorithm_hpp__

#include <boost/python.hpp>
#include "pinocchio/bindings/python/fwd.hpp"

namespace se3
{
  namespace python
  {
    namespace bp = boost::python;
    
    void exposeJointsAlgo();
    void exposeABA();
    void exposeCRBA();
    void exposeCentroidal();
    void exposeRNEA();
    void exposeCOM();
    void exposeFramesAlgo();
    void exposeEnergy();
    void exposeKinematics();
    void exposeDynamics();
    void exposeCAT();
    void exposeJacobian();
    void exposeGeometryAlgo();
    void exposeRegressor();
    
    void exposeRNEADerivatives();
    void exposeABADerivatives();
    void exposeKinematicsDerivatives();

    void exposeAlgorithms();
    
  } // namespace python
} // namespace se3

#endif // ifndef __se3_python_algorithm_hpp__

