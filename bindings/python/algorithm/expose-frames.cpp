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
#include "pinocchio/algorithm/frames.hpp"

namespace se3
{
  namespace python
  {
    
    static Data::Matrix6x frame_jacobian_proxy(const Model & model,
                                               Data & data,
                                               const Model::FrameIndex frame_id)
    {
      Data::Matrix6x J(6,model.nv); J.setZero();
      getFrameJacobian(model, data, frame_id, J);
      
      return J;
    }
    
    static Data::Matrix6x frame_jacobian_proxy(const Model & model,
                                               Data & data,
                                               const Model::FrameIndex frame_id,
                                               const Eigen::VectorXd & q)
    {
      computeJacobians(model,data,q);
      framesForwardKinematics(model,data);
  
      return frame_jacobian_proxy(model, data, frame_id);
    }
    
    void exposeFramesAlgo()
    {
      bp::def("framesKinematics",
              (void (*)(const Model &, Data &))&framesForwardKinematics,
              bp::args("Model","Data"),
              "Computes the placements of all the operational frames according to the current joint placement stored in data"
              "and put the results in data.");
      
      bp::def("framesKinematics",
              (void (*)(const Model &, Data &, const Eigen::VectorXd &))&framesForwardKinematics,
              bp::args("Model","Data",
                       "Configuration q (size Model::nq)"),
              "Update first the placement of the joints according to the given configuration value."
              "And computes the placements of all the operational frames"
              "and put the results in data.");
      
      bp::def("frameJacobian",
              (Data::Matrix6x (*)(const Model &, Data &, const Model::FrameIndex, const Eigen::VectorXd &))&frame_jacobian_proxy,
              bp::args("Model","Data",
                       "Operational frame ID (int)",
                       "Configuration q (size Model::nq)"),
              "Compute the Jacobian of the frame given by its ID."
              "The columns of the Jacobian are expressed in the frame coordinates.\n"
              "In other words, the velocity of the frame vF expressed in the local coordinate is given by J*v,"
              "where v is the time derivative of the configuration q.");
      
      bp::def("frameJacobian",
              (Data::Matrix6x (*)(const Model &, Data &, const Model::FrameIndex))&frame_jacobian_proxy,
              bp::args("Model","Data",
                       "Operational frame ID (int)"),
              "Compute the Jacobian of the frame given by its ID."
              "The columns of the Jacobian are expressed in the frame coordinates.\n"
              "In other words, the velocity of the frame vF expressed in the local coordinate is given by J*v,"
              "where v is the time derivative of the configuration q.\n"
              "Be aware that computeJacobians and framesKinematics must have been called first.");
      
    }
  } // namespace python
} // namespace se3
