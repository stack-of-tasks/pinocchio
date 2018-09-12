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

#ifndef __se3_frames_hxx__
#define __se3_frames_hxx__

#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/check.hpp"

namespace se3 
{
  
  
  inline void updateFramePlacements(const Model & model,
                                    Data & data)
  {
    assert(model.check(data) && "data is not consistent with model.");
    
    // The following for loop starts by index 1 because the first frame is fixed
    // and corresponds to the universe
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

  inline const SE3 & updateFramePlacement(const Model & model,
                                          Data & data,
                                          const Model::FrameIndex frame_id)
  {
    const Frame & frame = model.frames[frame_id];
    const Model::JointIndex & parent = frame.parent;
    if (frame.placement.isIdentity())
      data.oMf[frame_id] = data.oMi[parent];
    else
      data.oMf[frame_id] = data.oMi[parent]*frame.placement;
    return data.oMf[frame_id];
  }

  inline void framesForwardKinematics(const Model & model,
                                      Data & data)
  {
    updateFramePlacements(model,data);
  }

  inline void framesForwardKinematics(const Model & model,
                                      Data & data,
                                      const Eigen::VectorXd & q)
  {
    assert(model.check(data) && "data is not consistent with model.");
    
    forwardKinematics(model, data, q);
    updateFramePlacements(model, data);
  }

  void getFrameVelocity(const Model & model,
                        const Data & data,
                        const Model::FrameIndex frame_id,
                        Motion & frame_v)
  {
    assert(model.check(data) && "data is not consistent with model.");

    const Frame & frame = model.frames[frame_id];
    const Model::JointIndex & parent = frame.parent;
    frame_v = frame.placement.actInv(data.v[parent]);
  }

  void getFrameAcceleration(const Model & model,
                            const Data & data,
                            const Model::FrameIndex frame_id,
                            Motion & frame_a)
  {
    assert(model.check(data) && "data is not consistent with model.");

    const Frame & frame = model.frames[frame_id];
    const Model::JointIndex & parent = frame.parent;
    frame_a = frame.placement.actInv(data.a[parent]);
  }
  
  template<ReferenceFrame rf>
  inline void getFrameJacobian(const Model & model,
                               const Data & data,
                               const Model::FrameIndex frame_id,
                               Data::Matrix6x & J)
  {
    assert(J.cols() == model.nv);
    assert(data.J.cols() == model.nv);
    assert(model.check(data) && "data is not consistent with model.");
    
    const Frame & frame = model.frames[frame_id];
    const Model::JointIndex & joint_id = frame.parent;
    if (rf == WORLD)
    {
      getJointJacobian<WORLD>(model,data,joint_id,J);
      return;
    }
    
    if (rf == LOCAL)
    {
      const SE3 & oMframe = data.oMf[frame_id];
      const int colRef = nv(model.joints[joint_id])+idx_v(model.joints[joint_id])-1;
      
      for(int j=colRef;j>=0;j=data.parents_fromRow[(size_t) j])
      {
        J.col(j) = oMframe.actInv(Motion(data.J.col(j))).toVector();
      }
      return;
    }
  }
  
  inline void getFrameJacobian(const Model & model,
                               const Data & data,
                               const Model::FrameIndex frame_id,
                               Data::Matrix6x & J)
  {
    getFrameJacobian<LOCAL>(model,data,frame_id,J);
  }

  template<ReferenceFrame rf>
  void getFrameJacobianTimeVariation(const Model & model,
                                     const Data & data,
                                     const Model::FrameIndex frameId,
                                     Data::Matrix6x & dJ)
  {
    assert( dJ.rows() == data.dJ.rows() );
    assert( dJ.cols() == data.dJ.cols() );    
    assert(model.check(data) && "data is not consistent with model.");
    
    const Frame & frame = model.frames[frameId];
    const Model::JointIndex & joint_id = frame.parent;
    if (rf == WORLD)
    {
      getJointJacobianTimeVariation<WORLD>(model,data,joint_id,dJ);
      return;
    }
    
    if (rf == LOCAL)
    {
      const SE3 & oMframe = data.oMf[frameId];
      const int colRef = nv(model.joints[joint_id])+idx_v(model.joints[joint_id])-1;
      
      for(int j=colRef;j>=0;j=data.parents_fromRow[(size_t) j])
      {
        dJ.col(j) = oMframe.actInv(Motion(data.dJ.col(j))).toVector();
      }
      return;
    }    
  }
  

} // namespace se3

#endif // ifndef __se3_frames_hxx__
