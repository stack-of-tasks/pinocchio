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
#include "pinocchio/algorithm/frames.hpp"

namespace se3
{
  namespace python
  {
    
    static Data::Matrix6x frame_jacobian_proxy(const Model & model,
                                               Data & data,
                                               const Eigen::VectorXd & q,
                                               const Model::FrameIndex frame_id,
                                               bool local,
                                               bool update_geometry
                                               )
    {
      Data::Matrix6x J(6,model.nv); J.setZero();
      
      if (update_geometry)
        computeJacobians(model,data,q);
      
      if(local) getFrameJacobian<true> (model, data, frame_id, J);
      else getFrameJacobian<false> (model, data, frame_id, J);
      
      return J;
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
      
      bp::def("frameJacobian",frame_jacobian_proxy,
              bp::args("Model","Data",
                       "Configuration q (size Model::nq)",
                       "Operational frame ID (int)",
                       "frame (true = local, false = world)",
                       "update_geometry (true = recompute the kinematics)"),
              "Call computeJacobians if update_geometry is true. If not, user should call computeJacobians first."
              "Then call getJacobian and return the resulted jacobian matrix. Attention: if update_geometry is true, the "
              "function computes all the jacobians of the model. It is therefore outrageously costly wrt a dedicated "
              "call. Use only with update_geometry for prototyping.");
      
    }
  } // namespace python
} // namespace se3
