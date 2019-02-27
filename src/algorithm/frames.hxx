//
// Copyright (c) 2015-2018 CNRS
//

#ifndef __pinocchio_frames_hxx__
#define __pinocchio_frames_hxx__

#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/check.hpp"

namespace pinocchio 
{
  
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline void updateFramePlacements(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                    DataTpl<Scalar,Options,JointCollectionTpl> & data)
  {
    assert(model.check(data) && "data is not consistent with model.");
    
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef typename Model::FrameIndex FrameIndex;
    typedef typename Model::JointIndex JointIndex;
    
    // The following for loop starts by index 1 because the first frame is fixed
    // and corresponds to the universe.s
    for (FrameIndex i=1; i < (FrameIndex) model.nframes; ++i)
    {
      const Frame & frame = model.frames[i];
      const JointIndex & parent = frame.parent;
      if (frame.placement.isIdentity())
        data.oMf[i] = data.oMi[parent];
      else
        data.oMf[i] = data.oMi[parent]*frame.placement;
    }
  }
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::SE3 &
  updateFramePlacement(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                       DataTpl<Scalar,Options,JointCollectionTpl> & data,
                       const typename ModelTpl<Scalar,Options,JointCollectionTpl>::FrameIndex frame_id)
  {
    assert(model.check(data) && "data is not consistent with model.");
    
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    const typename Model::Frame & frame = model.frames[frame_id];
    const typename Model::JointIndex & parent = frame.parent;
    
    if (frame.placement.isIdentity())
      data.oMf[frame_id] = data.oMi[parent];
    else
      data.oMf[frame_id] = data.oMi[parent]*frame.placement;
    return data.oMf[frame_id];
  }


  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline void framesForwardKinematics(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                      DataTpl<Scalar,Options,JointCollectionTpl> & data)
  {
    updateFramePlacements(model,data);
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
  inline void framesForwardKinematics(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                      DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                      const Eigen::MatrixBase<ConfigVectorType> & q)
  {
    assert(model.check(data) && "data is not consistent with model.");
    
    forwardKinematics(model, data, q);
    updateFramePlacements(model, data);
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline MotionTpl<Scalar, Options>
  getFrameVelocity(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                   const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                   const typename ModelTpl<Scalar,Options,JointCollectionTpl>::FrameIndex frame_id)
  {
    assert(model.check(data) && "data is not consistent with model.");
    
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;

    const typename Model::Frame & frame = model.frames[frame_id];
    const typename Model::JointIndex & parent = frame.parent;
    return frame.placement.actInv(data.v[parent]);
  }  

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline MotionTpl<Scalar, Options>
  getFrameAcceleration(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                       const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                       const typename ModelTpl<Scalar,Options,JointCollectionTpl>::FrameIndex frame_id)
  {
    assert(model.check(data) && "data is not consistent with model.");

    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    
    const typename Model::Frame & frame = model.frames[frame_id];
    const typename Model::JointIndex & parent = frame.parent;
    return frame.placement.actInv(data.a[parent]);
  }
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename Matrix6xLike>
  inline void getFrameJacobian(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                               const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                               const typename ModelTpl<Scalar,Options,JointCollectionTpl>::FrameIndex frame_id,
                               const ReferenceFrame rf,
                               const Eigen::MatrixBase<Matrix6xLike> & J)
  {
    assert(J.rows() == 6);
    assert(J.cols() == model.nv);
    assert(data.J.cols() == model.nv);
    assert(model.check(data) && "data is not consistent with model.");
    
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    typedef typename Model::JointIndex JointIndex;
    
    const Frame & frame = model.frames[frame_id];
    const JointIndex & joint_id = frame.parent;
    if(rf == WORLD)
    {
      getJointJacobian(model,data,joint_id,WORLD,PINOCCHIO_EIGEN_CONST_CAST(Matrix6xLike,J));
      return;
    }
    
    if(rf == LOCAL)
    {
      Matrix6xLike & J_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix6xLike,J);
      const typename Data::SE3 & oMframe = data.oMf[frame_id];
      const int colRef = nv(model.joints[joint_id])+idx_v(model.joints[joint_id])-1;
      
      for(Eigen::DenseIndex j=colRef;j>=0;j=data.parents_fromRow[(size_t) j])
      {
        J_.col(j) = oMframe.actInv(Motion(data.J.col(j))).toVector(); // TODO: use MotionRef
      }
      return;
    }
  }
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename Matrix6Like>
  inline void frameJacobian(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                            DataTpl<Scalar,Options,JointCollectionTpl> & data,
                            const Eigen::MatrixBase<ConfigVectorType> & q,
                            const FrameIndex frameId,
                            const Eigen::MatrixBase<Matrix6Like> & J)
  {
    assert(model.check(data) && "data is not consistent with model.");
    assert(q.size() == model.nq && "The configuration vector is not of right size");

    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef typename Model::JointIndex JointIndex;

    const Frame & frame = model.frames[frameId];
    const JointIndex & jointId = frame.parent;
    data.iMf[jointId] = frame.placement;
    typedef JointJacobianForwardStep<Scalar,Options,JointCollectionTpl,ConfigVectorType,Matrix6Like> Pass;
    for(JointIndex i=jointId; i>0; i=model.parents[i])
    {
      Pass::run(model.joints[i],data.joints[i],
                typename Pass::ArgsType(model,data,q.derived(),PINOCCHIO_EIGEN_CONST_CAST(Matrix6Like,J)));
    }
  }
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename Matrix6xLike>
  inline void getFrameJacobian(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                               const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                               const typename ModelTpl<Scalar,Options,JointCollectionTpl>::FrameIndex frame_id,
                               const Eigen::MatrixBase<Matrix6xLike> & J)
  {
    getFrameJacobian(model,data,frame_id,LOCAL,PINOCCHIO_EIGEN_CONST_CAST(Matrix6xLike,J));
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename Matrix6xLike>
  void getFrameJacobianTimeVariation(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                     const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                     const typename ModelTpl<Scalar,Options,JointCollectionTpl>::FrameIndex frame_id,
                                     const ReferenceFrame rf,
                                     const Eigen::MatrixBase<Matrix6xLike> & dJ)
  {
    assert( dJ.rows() == data.dJ.rows() );
    assert( dJ.cols() == data.dJ.cols() );    
    assert(model.check(data) && "data is not consistent with model.");
    
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef typename Model::Frame Frame;
    typedef typename Model::SE3 SE3;
    
    const Frame & frame = model.frames[frame_id];
    const typename Model::JointIndex & joint_id = frame.parent;
    if(rf == WORLD)
    {
      getJointJacobianTimeVariation(model,data,joint_id,WORLD,PINOCCHIO_EIGEN_CONST_CAST(Matrix6xLike,dJ));
      return;
    }
    
    if (rf == LOCAL)
    {
      Matrix6xLike & dJ_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix6xLike,dJ);
      const SE3 & oMframe = data.oMf[frame_id];
      const int colRef = nv(model.joints[joint_id])+idx_v(model.joints[joint_id])-1;
      
      for(int j=colRef;j>=0;j=data.parents_fromRow[(size_t) j])
      {
        dJ_.col(j) = oMframe.actInv(Motion(data.dJ.col(j))).toVector();
      }
      return;
    }    
  }
  

} // namespace pinocchio

#endif // ifndef __pinocchio_frames_hxx__
