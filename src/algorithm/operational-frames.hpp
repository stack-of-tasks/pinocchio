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

#ifndef __se3_operational_frames_hpp__
#define __se3_operational_frames_hpp__

#include "pinocchio/multibody/model.hpp"

#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"

namespace se3
{

  /**
   * @brief      Update the position of each extra frame
   *
   * @param[in]  model  The kinematic model
   * @param      data   Data associated to model
   * @warning    One of the algorithms forwardKinematics should have been called first
   */
  inline void framesForwardKinematics(const Model & model,
                                      Data & data
                                      );

  /**
   * @brief      Compute Kinematics of full model, then the position of each operational frame
   *
   * @param[in]  model                    The kinematic model
   * @param      data                     Data associated to model
   * @param[in]  q                        Configuration vector
   */
  inline void framesForwardKinematics(const Model & model,
                                      Data & data,
                                      const Eigen::VectorXd & q
                                      );

  /**
   * @brief      Return the jacobian of the operational frame in the world frame or
     in the local frame depending on the template argument.
   *
   * @param[in]  model       The kinematic model
   * @param[in]  data        Data associated to model
   * @param[in]  frame_id    Id of the operational frame we want to compute the jacobian
   * @param      J           The filled Jacobian Matrix
   *
   * @tparam     localFrame  Express the jacobian in the local or global frame
   * 
   * @warning    The function computeJacobians should have been called first
   */
  template<bool localFrame>
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
    for (Model::FrameIndex i=0; i < (Model::FrameIndex) model.nOperationalFrames; ++i)
    {
      const Model::JointIndex & parent = model.operational_frames[i].parent;
      data.oMof[i] = (data.oMi[parent] * model.operational_frames[i].placement);
    }
  }
  
  inline void framesForwardKinematics(const Model & model,
                                      Data & data,
                                      const Eigen::VectorXd & q
                                      )
  {
    forwardKinematics(model, data, q);
    framesForwardKinematics(model, data);
  }
  
  
  
  template<bool localFrame>
  inline void getFrameJacobian(const Model & model,
                               const Data & data,
                               const Model::FrameIndex frame_id,
                               Data::Matrix6x & J)
  {
    assert( J.rows() == data.J.rows() );
    assert( J.cols() == data.J.cols() );
    
    const Model::JointIndex & parent = model.operational_frames[frame_id].parent;
    const SE3 & oMframe = data.oMof[frame_id];
    const Frame & frame = model.operational_frames[frame_id];
    
    const int colRef = nv(model.joints[parent])+idx_v(model.joints[parent])-1;
    
    // Lever between the joint center and the frame center expressed in the global frame
    const SE3::Vector3 lever(data.oMi[parent].rotation() * (frame.placement.translation()));
    
    getJacobian<localFrame>(model, data, parent, J);
    for(int j=colRef;j>=0;j=data.parents_fromRow[(size_t) j])
    {
      if(! localFrame )
        J.col(j).topRows<3>() -= lever.cross(J.col(j).bottomRows<3>());
      else
        J.col(j) = oMframe.actInv(Motion(data.J.col(j))).toVector();
    }
  }

} // namespace se3

#endif // ifndef __se3_operational_frames_hpp__

