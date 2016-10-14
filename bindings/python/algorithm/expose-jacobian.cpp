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
#include "pinocchio/algorithm/jacobian.hpp"

namespace se3
{
  namespace python
  {
    
    static Data::Matrix6x
    jacobian_proxy(const Model & model,
                   Data & data,
                   const Eigen::VectorXd & q,
                   Model::JointIndex jointId,
                   bool local,
                   bool update_geometry)
    {
      Data::Matrix6x J(6,model.nv); J.setZero();
      
      if (update_geometry)
        computeJacobians(model,data,q);
      
      if(local) getJacobian<true> (model,data,jointId,J);
      else getJacobian<false> (model,data,jointId,J);
      
      return J;
    }
  
    void exposeJacobian()
    {
      bp::def("computeJacobians",computeJacobians,
              bp::args("Model","Data",
                       "Joint configuration q (size Model::nq)"),
              "Calling computeJacobians",
              bp::return_value_policy<bp::return_by_value>());
      
      bp::def("jacobian",jacobian_proxy,
              bp::args("Model, the model of the kinematic tree",
                       "Data, the data associated to the model where the results are stored",
                       "Joint configuration q (size Model::nq)",
                       "Joint ID, the index of the joint.",
                       "frame (true = local, false = world)",
                       "update_geometry (true = update the value of the total jacobian)"),
              "Computes the jacobian of a given given joint according to the given input configuration."
              "If local is set to true, it returns the jacobian associated to the joint frame. Otherwise, it returns the jacobian of the frame coinciding with the world frame.");
    }
    
  } // namespace python
} // namespace se3
