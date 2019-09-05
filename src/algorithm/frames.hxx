//
// Copyright (c) 2015-2020 CNRS INRIA
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
    typedef typename Model::Frame Frame;
    typedef typename Model::FrameIndex FrameIndex;
    typedef typename Model::JointIndex JointIndex;
    
    // The following for loop starts by index 1 because the first frame is fixed
    // and corresponds to the universe.
    for(FrameIndex i=1; i < (FrameIndex) model.nframes; ++i)
    {
      const Frame & frame = model.frames[i];
      const JointIndex & parent = frame.parent;
      data.oMf[i] = data.oMi[parent]*frame.placement;
    }
  }
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::SE3 &
  updateFramePlacement(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                       DataTpl<Scalar,Options,JointCollectionTpl> & data,
                       const typename ModelTpl<Scalar,Options,JointCollectionTpl>::FrameIndex & frame_id)
  {
    assert(model.check(data) && "data is not consistent with model.");
    
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    const typename Model::Frame & frame = model.frames[frame_id];
    const typename Model::JointIndex & parent = frame.parent;
    
    data.oMf[frame_id] = data.oMi[parent]*frame.placement;
    
    return data.oMf[frame_id];
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

    PINOCCHIO_CHECK_INPUT_ARGUMENT(J.rows() == 6);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(J.cols() == model.nv);
    assert(model.check(data) && "data is not consistent with model.");
    
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef typename Model::Frame Frame;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    typedef typename Model::JointIndex JointIndex;
    
    const Frame & frame = model.frames[frame_id];
    const JointIndex & joint_id = frame.parent;
    
    if(rf == WORLD)
    {
      getJointJacobian(model,data,joint_id,WORLD,PINOCCHIO_EIGEN_CONST_CAST(Matrix6xLike,J));
      return;
    } 
    else if(rf == LOCAL || rf == LOCAL_WORLD_ALIGNED) 
    {
      Matrix6xLike & J_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix6xLike,J);
      const typename Data::SE3 & oMframe = data.oMf[frame_id];
      const int colRef = nv(model.joints[joint_id])+idx_v(model.joints[joint_id])-1;
      
      for(Eigen::DenseIndex j=colRef;j>=0;j=data.parents_fromRow[(size_t) j])
      {
        typedef typename Data::Matrix6x::ConstColXpr ConstColXprIn;
        const MotionRef<ConstColXprIn> v_in(data.J.col(j));
        
        typedef typename Matrix6xLike::ColXpr ColXprOut;
        MotionRef<ColXprOut> v_out(J_.col(j));
        
        if (rf == LOCAL) 
          v_out = oMframe.actInv(v_in);
        else
        {
          v_out = v_in;
          v_out.linear() -= oMframe.translation().cross(v_in.angular());
        }
      }
    }
  }
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename Matrix6xLike>
  inline void computeFrameJacobian(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                   DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                   const Eigen::MatrixBase<ConfigVectorType> & q,
                                   const FrameIndex frameId,
                                   const ReferenceFrame reference_frame,
                                   const Eigen::MatrixBase<Matrix6xLike> & J)
  {
    assert(model.check(data) && "data is not consistent with model.");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(q.size() == model.nq, "The configuration vector is not of right size");

    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    typedef typename Model::Frame Frame;
    typedef typename Model::JointIndex JointIndex;
    typedef typename Model::IndexVector IndexVector;

    const Frame & frame = model.frames[frameId];
    const JointIndex & joint_id = frame.parent;
    
    const IndexVector & joint_support = model.supports[joint_id];
    
    switch(reference_frame)
    {
      case WORLD:
      case LOCAL_WORLD_ALIGNED:
      {
        typedef JointJacobiansForwardStep<Scalar,Options,JointCollectionTpl,ConfigVectorType,Matrix6xLike> Pass;
        for(size_t k = 1; k < joint_support.size(); k++)
        {
          JointIndex parent = joint_support[k];
          Pass::run(model.joints[parent],data.joints[parent],
                    typename Pass::ArgsType(model,data,q.derived(),
                                            PINOCCHIO_EIGEN_CONST_CAST(Matrix6xLike,J)));
        }
        
        if(reference_frame == LOCAL_WORLD_ALIGNED)
        {
          typename Data::SE3 & oMframe = data.oMf[frameId];
          oMframe = data.oMi[joint_id] * frame.placement;
          
          Matrix6xLike & J_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix6xLike,J);
          
          const int colRef = nv(model.joints[joint_id])+idx_v(model.joints[joint_id])-1;
          for(Eigen::DenseIndex j=colRef;j>=0;j=data.parents_fromRow[(size_t) j])
          {
            typedef typename Matrix6xLike::ColXpr ColXprOut;
            MotionRef<ColXprOut> J_col(J_.col(j));
            
            J_col.linear() -= oMframe.translation().cross(J_col.angular());
          }
        }
        break;
      }
      case LOCAL:
      {
        data.iMf[joint_id] = frame.placement;
        
        typedef JointJacobianForwardStep<Scalar,Options,JointCollectionTpl,ConfigVectorType,Matrix6xLike> Pass;
        for(JointIndex i=joint_id; i>0; i=model.parents[i])
        {
          Pass::run(model.joints[i],data.joints[i],
                    typename Pass::ArgsType(model,data,q.derived(),
                                            PINOCCHIO_EIGEN_CONST_CAST(Matrix6xLike,J)));
        }
        break;
      }
      default:
      {
        assert(false && "must never happened");
      }
    }
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename Matrix6xLike>
  void getFrameJacobianTimeVariation(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                     const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                     const typename ModelTpl<Scalar,Options,JointCollectionTpl>::FrameIndex frame_id,
                                     const ReferenceFrame rf,
                                     const Eigen::MatrixBase<Matrix6xLike> & dJ)
  {
    PINOCCHIO_CHECK_INPUT_ARGUMENT( dJ.rows() == data.dJ.rows() );
    PINOCCHIO_CHECK_INPUT_ARGUMENT( dJ.cols() == data.dJ.cols() );    
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
    
    if(rf == LOCAL)
    {
      Matrix6xLike & dJ_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix6xLike,dJ);
      const SE3 & oMframe = data.oMf[frame_id];
      const int colRef = nv(model.joints[joint_id])+idx_v(model.joints[joint_id])-1;
      
      for(Eigen::DenseIndex j=colRef;j>=0;j=data.parents_fromRow[(size_t) j])
      {
        typedef typename Data::Matrix6x::ConstColXpr ConstColXprIn;
        const MotionRef<ConstColXprIn> v_in(data.dJ.col(j));
        
        typedef typename Matrix6xLike::ColXpr ColXprOut;
        MotionRef<ColXprOut> v_out(dJ_.col(j));
        
        v_out = oMframe.actInv(v_in);
      }
      
      return;
    }    
  }

} // namespace pinocchio

#endif // ifndef __pinocchio_frames_hxx__
