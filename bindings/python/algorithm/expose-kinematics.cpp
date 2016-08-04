//
// Copyright (c) 2015-2016 CNRS
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

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

namespace se3
{
  namespace python
  {
    
    static void fk_0_proxy(const ModelHandler & model,
                           DataHandler & data,
                           const VectorXd_fx & q)
    {
      forwardKinematics(*model,*data,q);
    }
    
    static void fk_1_proxy(const ModelHandler& model,
                           DataHandler & data,
                           const VectorXd_fx & q,
                           const VectorXd_fx & qdot )
    {
      forwardKinematics(*model,*data,q,qdot);
    }

    
    static void fk_2_proxy(const ModelHandler& model,
                           DataHandler & data,
                           const VectorXd_fx & q,
                           const VectorXd_fx & v,
                           const VectorXd_fx & a)
    {
      forwardKinematics(*model,*data,q,v,a);
    }
    
    void exposeKinematics()
    {
      
      bp::def("forwardKinematics",fk_0_proxy,
              bp::args("Model","Data",
                       "Configuration q (size Model::nq)"),
              "Compute the placements of all the frames of the kinematic "
              "tree and put the results in data.");
      
      bp::def("forwardKinematics",fk_1_proxy,
              bp::args("Model","Data",
                       "Configuration q (size Model::nq)",
                       "Velocity v (size Model::nv)"),
              "Compute the placements and spatial velocities of all the frames of the kinematic "
              "tree and put the results in data.");
      
      bp::def("forwardKinematics",fk_2_proxy,
              bp::args("Model","Data",
                       "Configuration q (size Model::nq)",
                       "Velocity v (size Model::nv)",
                       "Acceleration a (size Model::nv)"),
              "Compute the placements, spatial velocities and spatial accelerations of all the frames of the kinematic "
              "tree and put the results in data.");
    }
    
  } // namespace python
} // namespace se3
