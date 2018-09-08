//
// Copyright (c) 2015-2018 CNRS
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
    
    static Data::Matrix6x get_frame_jacobian_proxy(const Model & model,
                                                   Data & data,
                                                   const Model::FrameIndex frame_id,
                                                   ReferenceFrame rf)
    {
      Data::Matrix6x J(6,model.nv); J.setZero();
      if(rf == LOCAL)
        getFrameJacobian<LOCAL>(model, data, frame_id, J);
      else
        getFrameJacobian<WORLD>(model, data, frame_id, J);
      
      return J;
    }
    
    static Data::Matrix6x frame_jacobian_proxy(const Model & model,
                                               Data & data,
                                               const Eigen::VectorXd & q,
                                               const Model::FrameIndex frame_id,
                                               ReferenceFrame rf
                                               )
    {
      computeJointJacobians(model,data,q);
      framesForwardKinematics(model,data);
  
      return get_frame_jacobian_proxy(model, data, frame_id, rf);
    }


    static Data::Matrix6x
    get_frame_jacobian_time_variation_proxy(const Model & model,
                                            Data & data,
                                            Model::FrameIndex jointId,
                                            ReferenceFrame rf)
    {
      Data::Matrix6x dJ(6,model.nv); dJ.setZero();
      
      if(rf == LOCAL) getFrameJacobianTimeVariation<LOCAL>(model,data,jointId,dJ);
      else getFrameJacobianTimeVariation<WORLD>(model,data,jointId,dJ);
      
      return dJ;
    }

    static Data::Matrix6x frame_jacobian_time_variation_proxy(const Model & model,
                                                              Data & data,
                                                              const Eigen::VectorXd & q,
                                                              const Eigen::VectorXd & v,
                                                              const Model::FrameIndex frame_id,
                                                              ReferenceFrame rf
                                                              )
    {
      computeJointJacobiansTimeVariation(model,data,q,v);
      framesForwardKinematics(model,data);
  
      return get_frame_jacobian_time_variation_proxy(model, data, frame_id, rf);
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
              (Data::Matrix6x (*)(const Model &, Data &, const Eigen::VectorXd &, const Model::FrameIndex, ReferenceFrame))&frame_jacobian_proxy,
              bp::args("Model","Data",
                       "Configuration q (size Model::nq)",
                       "Operational frame ID (int)",
                       "Reference frame rf (either ReferenceFrame.LOCAL or ReferenceFrame.WORLD)"),
              "Computes the Jacobian of the frame given by its ID either in the local or the world frames."
              "The columns of the Jacobian are expressed in the frame coordinates.\n"
              "In other words, the velocity of the frame vF expressed in the local coordinate is given by J*v,"
              "where v is the time derivative of the configuration q.");
      
      bp::def("getFrameJacobian",
              (Data::Matrix6x (*)(const Model &, Data &, const Model::FrameIndex, ReferenceFrame))&get_frame_jacobian_proxy,
              bp::args("Model","Data",
                       "Operational frame ID (int)",
                       "Reference frame rf (either ReferenceFrame.LOCAL or ReferenceFrame.WORLD)"),
              "Computes the Jacobian of the frame given by its ID either in the local or the world frames."
              "The columns of the Jacobian are expressed in the frame coordinates.\n"
              "In other words, the velocity of the frame vF expressed in the local coordinate is given by J*v,"
              "where v is the time derivative of the configuration q.\n"
              "Be aware that computeJointJacobians and framesKinematics must have been called first.");

      bp::def("frameJacobianTimeVariation",
              (Data::Matrix6x (*)(const Model &, Data &, const Eigen::VectorXd &,const Eigen::VectorXd &, const Model::FrameIndex, ReferenceFrame))&frame_jacobian_time_variation_proxy,
              bp::args("Model","Data",
                       "Configuration q (size Model::nq)",
                       "Joint velocity v (size Model::nv)",                       
                       "Operational frame ID (int)",
                       "Reference frame rf (either ReferenceFrame.LOCAL or ReferenceFrame.WORLD)"),
              "Computes the Jacobian Time Variation of the frame given by its ID either in the local or the world frames."
              "The columns of the Jacobian time variation are expressed in the frame coordinates.\n"
              "In other words, the velocity of the frame vF expressed in the local coordinate is given by J*v,"
              "where v is the time derivative of the configuration q.");      

      bp::def("getFrameJacobianTimeVariation",get_frame_jacobian_time_variation_proxy,
              bp::args("Model, the model of the kinematic tree",
                       "Data, the data associated to the model where the results are stored",
                       "Frame ID, the index of the frame.",
                       "Reference frame rf (either ReferenceFrame.LOCAL or ReferenceFrame.WORLD)"),
              "Returns the Jacobian time variation of a specific frame (specified by Frame ID) expressed either in the world or the local frame."
              "You have to call computeJointJacobiansTimeVariation and framesKinematics first."
              "If rf is set to LOCAL, it returns the jacobian time variation associated to the frame index. Otherwise, it returns the jacobian time variation of the frame coinciding with the world frame.");

    }
  } // namespace python
} // namespace se3
