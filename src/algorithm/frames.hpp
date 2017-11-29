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

#ifndef __se3_frames_hpp__
#define __se3_frames_hpp__

#include "pinocchio/multibody/model.hpp"

#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/check.hpp"

namespace se3
{

  /**
   * @brief      Updates the position of each frame contained in the model
   *
   * @param[in]  model  The kinematic model.
   * @param      data   Data associated to model.
   *
   * @warning    One of the algorithms forwardKinematics should have been called first
   */
  inline void framesForwardKinematics(const Model & model,
                                      Data & data
                                      );

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
                                      const Eigen::VectorXd & q
                                      );

  /**
   * @brief      Returns the jacobian of the frame expresssed in the local frame depending on the template argument.
   *             You must first call se3::framesForwardKinematics and se3::computeJacobians to update values in data structure.
   
   * @remark     Expressed in the local frame, the jacobian maps the joint velocity vector to the spatial velocity of the center of the frame, expressed in the frame coordinates system.
   *
   * @param[in]  model       The kinematic model
   * @param[in]  data        Data associated to model
   * @param[in]  frame_id    Id of the operational Frame
   * @param[out] J           The Jacobian of the Frame expressed in the coordinates Frame.
   *
   * @warning    The function se3::computeJacobians and se3::framesForwardKinematics should have been called first.
   */
  inline void getFrameJacobian(const Model & model,
                               const Data& data,
                               const Model::FrameIndex frame_id,
                               Data::Matrix6x & J
                               );
 
} // namespace se3

/* --- Details -------------------------------------------------------------------- */
namespace se3 
{
  
  
  inline void framesForwardKinematics(const Model & model,
                                      Data & data
                                      )
  {
    assert(model.check(data) && "data is not consistent with model.");
    
    // The following for loop starts by index 1 because the first frame is fixed
    // and corresponds to the universe.s
    for (Model::FrameIndex i=1; i < (Model::FrameIndex) model.nframes; ++i)
    {
      const Frame & frame = model.frames[i];
      const Model::JointIndex & parent = frame.parent;
      if (frame.placement.isIdentity())
        data.oMf[i] = data.oMi[parent];
      else
        data.oMf[i] = data.oMi[parent]*frame.placement;
    }
  }
  
  inline void framesForwardKinematics(const Model & model,
                                      Data & data,
                                      const Eigen::VectorXd & q
                                      )
  {
    assert(model.check(data) && "data is not consistent with model.");
    
    forwardKinematics(model, data, q);
    framesForwardKinematics(model, data);
  }
  
  inline void getFrameJacobian(const Model & model,
                               const Data & data,
                               const Model::FrameIndex frame_id,
                               Data::Matrix6x & J)
  {
    assert(J.cols() == model.nv);
    assert(data.J.cols() == model.nv);
    assert(model.check(data) && "data is not consistent with model.");
    
    const Frame & frame = model.frames[frame_id];
    const Model::JointIndex & parent = frame.parent;
    const SE3 & oMframe = data.oMf[frame_id];
    
    const int colRef = nv(model.joints[parent])+idx_v(model.joints[parent])-1;
    
    for(int j=colRef;j>=0;j=data.parents_fromRow[(size_t) j])
    {
      J.col(j) = oMframe.actInv(Motion(data.J.col(j))).toVector();
    }
  }

} // namespace se3

#endif // ifndef __se3_frames_hpp__

