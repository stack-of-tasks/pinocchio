//
// Copyright (c) 2015-2020 CNRS INRIA
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_multibody_model_hxx__
#define __pinocchio_multibody_model_hxx__

#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/utils/string-generator.hpp"
#include "pinocchio/multibody/liegroup/liegroup-algo.hpp"

#include <boost/bind.hpp>
#include <boost/utility.hpp>

/// @cond DEV

namespace pinocchio
{
  namespace details
  {
    struct FilterFrame
    {
      const std::string & name;
      const FrameType & typeMask;
      
      FilterFrame(const std::string& name, const FrameType & typeMask)
      : name(name), typeMask(typeMask)
      {}
      
      template<typename Scalar, int Options>
      bool operator()(const FrameTpl<Scalar,Options> & frame) const
      { return (typeMask & frame.type) && (name == frame.name); }
      
    };
  }
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  const typename ModelTpl<Scalar,Options,JointCollectionTpl>::
  Vector3 ModelTpl<Scalar,Options,JointCollectionTpl>::gravity981((Scalar)0,(Scalar)0,(Scalar)-9.81);

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline std::ostream& operator<<(std::ostream & os,
                                  const ModelTpl<Scalar,Options,JointCollectionTpl> & model)
  {
    typedef typename ModelTpl<Scalar,Options,JointCollectionTpl>::Index Index;
    
    os << "Nb joints = " << model.njoints << " (nq="<< model.nq<<",nv="<<model.nv<<")" << std::endl;
    for(Index i=0;i<(Index)(model.njoints);++i)
    {
      os << "  Joint " << i << " " << model.names[i] << ": parent=" << model.parents[i]  << std::endl;
    }
    
    return os;
  }
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  typename ModelTpl<Scalar,Options,JointCollectionTpl>::JointIndex
  ModelTpl<Scalar,Options,JointCollectionTpl>::addJoint(const JointIndex parent,
                                                        const JointModel & joint_model,
                                                        const SE3 & joint_placement,
                                                        const std::string & joint_name,
                                                        const VectorXs & max_effort,
                                                        const VectorXs & max_velocity,
                                                        const VectorXs & min_config,
                                                        const VectorXs & max_config
                                                        )
  {
    assert( (njoints==(int)joints.size())&&(njoints==(int)inertias.size())
           &&(njoints==(int)parents.size())&&(njoints==(int)jointPlacements.size()) );
    assert((joint_model.nq()>=0) && (joint_model.nv()>=0));
    assert(joint_model.nq() >= joint_model.nv());

    assert(max_effort.size() == joint_model.nv()
           && max_velocity.size() == joint_model.nv()
           && min_config.size() == joint_model.nq()
           && max_config.size() == joint_model.nq());
    
    JointIndex idx = (JointIndex)(njoints++);
    
    joints         .push_back(JointModel(joint_model.derived()));
//    JointModelDerived & jmodel = boost::get<JointModelDerived>(joints.back());
    JointModel & jmodel = joints.back();
    jmodel.setIndexes(idx,nq,nv);
    
    const int joint_nq = jmodel.nq();
    const int joint_idx_q = jmodel.idx_q();
    const int joint_nv = jmodel.nv();
    const int joint_idx_v = jmodel.idx_v();
    
    assert(joint_idx_q >= 0);
    assert(joint_idx_v >= 0);

    inertias       .push_back(Inertia::Zero());
    parents        .push_back(parent);
    jointPlacements.push_back(joint_placement);
    names          .push_back(joint_name);
    
    nq += joint_nq; nqs.push_back(joint_nq); idx_qs.push_back(joint_idx_q);
    nv += joint_nv; nvs.push_back(joint_nv); idx_vs.push_back(joint_idx_v);

    if(joint_nq > 0 && joint_nv > 0)
    {
      effortLimit.conservativeResize(nv);
      jmodel.jointVelocitySelector(effortLimit) = max_effort;
      velocityLimit.conservativeResize(nv);
      jmodel.jointVelocitySelector(velocityLimit) = max_velocity;
      lowerPositionLimit.conservativeResize(nq);
      jmodel.jointConfigSelector(lowerPositionLimit) = min_config;
      upperPositionLimit.conservativeResize(nq);
      jmodel.jointConfigSelector(upperPositionLimit) = max_config;
      
      rotorInertia.conservativeResize(nv);
      jmodel.jointVelocitySelector(rotorInertia).setZero();
      rotorGearRatio.conservativeResize(nv);
      jmodel.jointVelocitySelector(rotorGearRatio).setOnes();
    }
    
    // Init and add joint index to its parent subtrees.
    subtrees.push_back(IndexVector(1));
    subtrees[idx][0] = idx;
    addJointIndexToParentSubtrees(idx);
    
    // Init and add joint index to the supports
    supports.push_back(supports[parent]);
    supports[idx].push_back(idx);
    
    return idx;
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  typename ModelTpl<Scalar,Options,JointCollectionTpl>::JointIndex
  ModelTpl<Scalar,Options,JointCollectionTpl>::addJoint(const JointIndex parent,
                                                        const JointModel & joint_model,
                                                        const SE3 & joint_placement,
                                                        const std::string & joint_name)
  {
    VectorXs max_effort, max_velocity, min_config, max_config;

    max_effort = VectorXs::Constant(joint_model.nv(), std::numeric_limits<Scalar>::max());
    max_velocity = VectorXs::Constant(joint_model.nv(), std::numeric_limits<Scalar>::max());
    min_config = VectorXs::Constant(joint_model.nq(), -std::numeric_limits<Scalar>::max());
    max_config = VectorXs::Constant(joint_model.nq(), std::numeric_limits<Scalar>::max());

    return addJoint(parent, joint_model, joint_placement, joint_name, max_effort, max_velocity, min_config, max_config);
  }
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline typename ModelTpl<Scalar,Options,JointCollectionTpl>::FrameIndex
  ModelTpl<Scalar,Options,JointCollectionTpl>::
  addJointFrame(const JointIndex & joint_index,
                int previous_frame_index)
  {
    PINOCCHIO_CHECK_INPUT_ARGUMENT(joint_index >= 0 && joint_index < joints.size(),
                                   "The joint index is larger than the number of joints in the model.");
    if(previous_frame_index < 0)
    {
      // FIXED_JOINT is required because the parent can be the universe and its
      // type is FIXED_JOINT
      previous_frame_index = (int)getFrameId(names[parents[joint_index]], (FrameType)(JOINT | FIXED_JOINT));
    }
    assert((size_t)previous_frame_index < frames.size() && "Frame index out of bound");
    
    // Add a the joint frame attached to itself to the frame vector - redundant information but useful.
    return addFrame(Frame(names[joint_index],joint_index,(FrameIndex)previous_frame_index,SE3::Identity(),JOINT));
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline void ModelTpl<Scalar,Options,JointCollectionTpl>::
  appendBodyToJoint(const typename ModelTpl::JointIndex joint_index,
                    const Inertia & Y,
                    const SE3 & body_placement)
  {
    const Inertia & iYf = Y.se3Action(body_placement);
    inertias[joint_index] += iYf;
    nbodies++;
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline typename ModelTpl<Scalar,Options,JointCollectionTpl>::FrameIndex
  ModelTpl<Scalar,Options,JointCollectionTpl>::
  addBodyFrame(const std::string & body_name,
               const JointIndex  & parentJoint,
               const SE3         & body_placement,
               int           previousFrame)
  {
    if(previousFrame < 0) {
      // FIXED_JOINT is required because the parent can be the universe and its
      // type is FIXED_JOINT
      previousFrame = (int)getFrameId(names[parentJoint], (FrameType)(JOINT | FIXED_JOINT));
    }
    assert((size_t)previousFrame < frames.size() && "Frame index out of bound");
    return addFrame(Frame(body_name, parentJoint, (FrameIndex)previousFrame, body_placement, BODY));
  }
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline typename ModelTpl<Scalar,Options,JointCollectionTpl>::FrameIndex
  ModelTpl<Scalar,Options,JointCollectionTpl>::
  getBodyId(const std::string & name) const
  {
    return getFrameId(name, BODY);
  }
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline bool ModelTpl<Scalar,Options,JointCollectionTpl>::
  existBodyName(const std::string & name) const
  {
    return existFrame(name, BODY);
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline typename  ModelTpl<Scalar,Options,JointCollectionTpl>::JointIndex
  ModelTpl<Scalar,Options,JointCollectionTpl>::
  getJointId(const std::string & name) const
  {
    typedef std::vector<std::string>::iterator::difference_type it_diff_t;
    it_diff_t res = std::find(names.begin(),names.end(),name) - names.begin();
    assert((res<INT_MAX) && "Id superior to int range. Should never happen.");
    return ModelTpl::JointIndex(res);
  }
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline bool ModelTpl<Scalar,Options,JointCollectionTpl>::
  existJointName(const std::string & name) const
  {
    return !name.empty() && (names.end() != std::find(names.begin(),names.end(),name));
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline typename ModelTpl<Scalar,Options,JointCollectionTpl>::FrameIndex
  ModelTpl<Scalar,Options,JointCollectionTpl>::
  getFrameId(const std::string & name, const FrameType & type) const
  {
    typename PINOCCHIO_ALIGNED_STD_VECTOR(Frame)::const_iterator it
    = std::find_if(frames.begin()
                   ,frames.end()
                   ,details::FilterFrame(name, type));
    assert(((it == frames.end()) ||
            (std::find_if( boost::next(it), frames.end(), details::FilterFrame(name, type)) == frames.end()))
        && "Several frames match the filter");
    return FrameIndex(it - frames.begin());
  }
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline bool ModelTpl<Scalar,Options,JointCollectionTpl>::
  existFrame(const std::string & name, const FrameType & type) const
  {
    return !name.empty() && std::find_if(frames.begin(), frames.end(),
                        details::FilterFrame(name, type)) != frames.end();
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline typename ModelTpl<Scalar,Options,JointCollectionTpl>::FrameIndex
  ModelTpl<Scalar,Options,JointCollectionTpl>::
  addFrame(const Frame & frame)
  {
    // Check if the frame.name exists with the same type
    if(existFrame(frame.name,frame.type))
    {
      return getFrameId(frame.name,frame.type);
    }
    // else: we must add a new frames to the current stack
    frames.push_back(frame);
    nframes++;
    return FrameIndex(nframes - 1);
  }
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline void ModelTpl<Scalar,Options,JointCollectionTpl>::
  addJointIndexToParentSubtrees(const JointIndex joint_id)
  {
    for(JointIndex parent = parents[joint_id]; parent>0; parent = parents[parent])
      subtrees[parent].push_back(joint_id);
    
    // Also add joint_id to the universe
    subtrees[0].push_back(joint_id);
  }

} // namespace pinocchio

/// @endcond

#endif // ifndef __pinocchio_multibody_model_hxx__
