//
// Copyright (c) 2015-2017 CNRS
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
                   bool update_kinematics)
    {
      Data::Matrix6x J(6,model.nv); J.setZero();
      
      if (update_kinematics)
        computeJacobians(model,data,q);
      
      if(local) getJacobian<LOCAL> (model,data,jointId,J);
      else getJacobian<WORLD> (model,data,jointId,J);
      
      return J;
    }
    
    static Data::Matrix6x
    get_jacobian_time_variation_proxy(const Model & model,
                                  Data & data,
                                  Model::JointIndex jointId,
                                  bool local)
    {
      Data::Matrix6x dJ(6,model.nv); dJ.setZero();
      
      
      if(local) getJacobianTimeVariation<LOCAL> (model,data,jointId,dJ);
      else getJacobianTimeVariation<WORLD> (model,data,jointId,dJ);
      
      return dJ;
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
                       "update_kinematics (true = update the value of the total jacobian)"),
              "Computes the jacobian of a given given joint according to the given input configuration."
              "If local is set to true, it returns the jacobian associated to the joint frame. Otherwise, it returns the jacobian of the frame coinciding with the world frame.");
      
      bp::def("computeJacobiansTimeVariation",computeJacobiansTimeVariation,
              bp::args("Model","Data",
                       "Joint configuration q (size Model::nq)",
                       "Joint velocity v (size Model::nv)"),
              "Calling computeJacobiansTimeVariation",
              bp::return_value_policy<bp::return_by_value>());
      
      bp::def("getJacobianTimeVariation",get_jacobian_time_variation_proxy,
              bp::args("Model, the model of the kinematic tree",
                       "Data, the data associated to the model where the results are stored",
                       "Joint ID, the index of the joint.",
                       "frame (true = local, false = world)"),
              "Computes the Jacobian time variation of a specific joint frame expressed either in the world frame or in the local frame of the joint."
              "You have to run computeJacobiansTimeVariation first."
              "If local is set to true, it returns the jacobian time variation associated to the joint frame. Otherwise, it returns the jacobian time variation of the frame coinciding with the world frame.");
    }
    
  } // namespace python
} // namespace se3
