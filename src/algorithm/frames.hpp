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

#ifndef __se3_frames_hpp__
#define __se3_frames_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

namespace se3
{

  /**
   * @brief      Updates the placement of each frame contained in the model
   *
   * @param[in]  model  The kinematic model.
   * @param      data   Data associated to model.
   *
   * @warning    One of the algorithms forwardKinematics should have been called first
   */
  inline void updateFramePlacements(const Model & model,
                                    Data & data);

  /**
   * @brief      Updates the placement of the given frame.
   *
   * @param[in]  model        The kinematic model.
   * @param      data         Data associated to model.
   * @param[in]  frame_id     Id of the operational Frame.
   *
   * @return     A reference to the frame placement stored in data.oMf[frame_id]
   *
   * @warning    One of the algorithms forwardKinematics should have been called first
   */
  inline const SE3 & updateFramePlacement(const Model & model,
                                          Data & data,
                                          const Model::FrameIndex frame_id);

  /**
   * @brief      Updates the placement of each frame contained in the model
   *
   * @deprecated This function is now deprecated. Please call se3::updateFramePlacements for same functionality
   *
   * @param[in]  model  The kinematic model.
   * @param      data   Data associated to model.
   *
   * @warning    One of the algorithms forwardKinematics should have been called first
   */
  PINOCCHIO_DEPRECATED
  inline void framesForwardKinematics(const Model & model,
                                      Data & data);

  /**
   * @brief      First calls the forwardKinematics on the model, then computes the placement of each frame.
   *             /sa se3::forwardKinematics
   *
   * @param[in]  model                    The kinematic model.
   * @param      data                     Data associated to model.
   * @param[in]  q                        Configuration vector.
   */
  inline void framesForwardKinematics(const Model & model,
                                      Data & data,
                                      const Eigen::VectorXd & q);

  /**
   * @brief      Returns the spatial velocity of the frame expressed in the LOCAL frame coordinate system.
   *             You must first call se3::forwardKinematics to update placement and velocity values in data structure.
   *
   * @param[in]  model       The kinematic model
   * @param[in]  data        Data associated to model
   * @param[in]  frame_id    Id of the operational Frame
   * @param[out] frame_v     The spatial velocity of the Frame expressed in the coordinates Frame.
   *
   * @warning    Fist or second order forwardKinematics should have been called first
   */
  inline void getFrameVelocity(const Model & model,
                               const Data & data,
                               const Model::FrameIndex frame_id,
                               Motion & frame_v);

  /**
   * @brief      Returns the spatial acceleration of the frame expressed in the LOCAL frame coordinate system.
   *             You must first call se3::forwardKinematics to update placement values in data structure.
   *
   * @param[in]  model       The kinematic model
   * @param[in]  data        Data associated to model
   * @param[in]  frame_id    Id of the operational Frame
   * @param[out] frame_a     The spatial acceleration of the Frame expressed in the coordinates Frame.
   *
   * @warning    Second order forwardKinematics should have been called first
   */
  inline void getFrameAcceleration(const Model & model,
                                   const Data & data,
                                   const Model::FrameIndex frame_id,
                                   Motion & frame_a);


  /**
   * @brief      Returns the jacobian of the frame expressed either in the LOCAL frame coordinate system or in the WORLD coordinate system,
   *             depending on the value of the template parameter rf.
   *             You must first call se3::computeJointJacobians followed by se3::framesForwardKinematics to update placement values in data structure.
   *
   * @tparam     rf Reference frame in which the columns of the Jacobian are expressed.
   *
   * @remark     Similarly to se3::getJointJacobian with LOCAL or WORLD templated parameters, if rf == LOCAL, this function returns the Jacobian of the frame expressed
   *             in the local coordinates of the frame, or if rl == WORLD, it returns the Jacobian expressed of the point coincident with the origin
   *             and expressed in a coordinate system aligned with the WORLD.
   *
   * @param[in]  model       The kinematic model
   * @param[in]  data        Data associated to model
   * @param[in]  frame_id    Id of the operational Frame
   * @param[out] J           The Jacobian of the Frame expressed in the coordinates Frame.
   *
   * @warning    The functions se3::computeJointJacobians and se3::framesForwardKinematics should have been called first.
   */
  template<ReferenceFrame rf>
  void getFrameJacobian(const Model & model,
                        const Data & data,
                        const Model::FrameIndex frame_id,
                        Data::Matrix6x & J);
  
  /**
   * @brief      Returns the jacobian of the frame expresssed the LOCAL frame coordinate system.
   *             You must first call se3::computeJointJacobians followed by se3::framesForwardKinematics to update placement values in data structure.
   \deprecated This function is now deprecated. Please call se3::getFrameJacobian<ReferenceFrame> for same functionality
   *
   * @param[in]  model       The kinematic model
   * @param[in]  data        Data associated to model
   * @param[in]  frame_id    Id of the operational Frame
   * @param[out] J           The Jacobian of the Frame expressed in the coordinates Frame.
   *
   * @warning    The function se3::computeJointJacobians and se3::framesForwardKinematics should have been called first.
   */
  PINOCCHIO_DEPRECATED   
  inline void getFrameJacobian(const Model & model,
                               const Data & data,
                               const Model::FrameIndex frame_id,
                               Data::Matrix6x & J);

  ///
  /// \brief Computes the Jacobian time variation of a specific frame (given by frame_id) expressed either in the world frame (rf = WORLD) or in the local frame (rf = LOCAL).
  /// \note This jacobian is extracted from data.dJ. You have to run se3::computeJacobiansTimeVariation before calling it.
  ///
  /// \tparam rf Reference frame in which the Jacobian is expressed.
  ///
  /// \param[in] localFrame Expressed the Jacobian in the local frame or world frame coordinates system.
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] frameId The index of the frame.
  /// \param[out] dJ A reference on the Jacobian matrix where the results will be stored in (dim 6 x model.nv). You must fill dJ with zero elements, e.g. dJ.fill(0.).
  ///
  template<ReferenceFrame rf>
  void getFrameJacobianTimeVariation(const Model & model,
                                     const Data & data,
                                     const Model::FrameIndex frameId,
                                     Data::Matrix6x & dJ);
  
 
} // namespace se3

/* --- Details -------------------------------------------------------------------- */
#include "pinocchio/algorithm/frames.hxx"

#endif // ifndef __se3_frames_hpp__
