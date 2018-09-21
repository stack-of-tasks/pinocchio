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
      updateFramePlacements(model,data);
  
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
      updateFramePlacements(model,data);
  
      return get_frame_jacobian_time_variation_proxy(model, data, frame_id, rf);
    }        

    static Motion get_frame_velocity_proxy(const Model & model,
                                           Data & data,
                                           const Model::FrameIndex frame_id
                                           )
    {
      Motion v;
      getFrameVelocity(model,data,frame_id,v);
      return v;
    }

    static Motion get_frame_acceleration_proxy(const Model & model,
                                               Data & data,
                                               const Model::FrameIndex frame_id
                                               )
    {
      Motion a;
      getFrameAcceleration(model,data,frame_id,a);
      return a;
    }
    
    void exposeFramesAlgo()
    {
      bp::def("updateFramePlacements",
              (void (*)(const Model &, Data &))&updateFramePlacements,
              bp::args("Model","Data"),
              "Computes the placements of all the operational frames according to the current joint placement stored in data"
              "and puts the results in data.");

      bp::def("updateFramePlacement",
              (const SE3 & (*)(const Model &, Data &, const Model::FrameIndex))&updateFramePlacement,
              bp::args("Model","Data","Operational frame ID (int)"),
              "Computes the placement of the given operational frames according to the current joint placement stored in data,"
              "puts the results in data and returns it.",
              bp::return_value_policy<bp::return_by_value>());

      bp::def("getFrameVelocity",
              (Motion (*)(const Model &, Data &, const Model::FrameIndex))&get_frame_velocity_proxy,
              bp::args("Model","Data","Operational frame ID (int)"),
              "Returns the spatial velocity of the frame expressed in the LOCAL frame coordinate system."
              "Fist or second order forwardKinematics should be called first.");

      bp::def("getFrameAcceleration",
              (Motion (*)(const Model &, Data &, const Model::FrameIndex))&get_frame_acceleration_proxy,
              bp::args("Model","Data","Operational frame ID (int)"),
              "Returns the spatial velocity of the frame expressed in the LOCAL frame coordinate system."
              "Second order forwardKinematics should be called first.");
      
      bp::def("framesForwardKinematics",
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
