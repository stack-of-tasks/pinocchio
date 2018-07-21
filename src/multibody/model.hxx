//
// Copyright (c) 2015-2018 CNRS
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
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

#ifndef __se3_model_hxx__
#define __se3_model_hxx__

#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/utils/string-generator.hpp"
#include "pinocchio/multibody/liegroup/liegroup-algo.hpp"

#include <boost/bind.hpp>
#include <boost/utility.hpp>

/// @cond DEV

namespace se3
{
  namespace details
  {
    struct FilterFrame {
      const std::string& name;
      const FrameType & typeMask;
      FilterFrame(const std::string& name, const FrameType& typeMask)
        : name(name), typeMask(typeMask) {}
      bool operator()(const Frame& frame) const
      { return (typeMask & frame.type) && (name == frame.name); }
    };
  }
  template<typename JointCollection>
  const typename ModelTpl<JointCollection>::Vector3 ModelTpl<JointCollection>::gravity981(0,0,-9.81);

  template<typename JointCollection>
  inline std::ostream& operator<<(std::ostream & os, const ModelTpl<JointCollection> & model)
  {
    typedef typename ModelTpl<JointCollection>::Index Index;
    
    os << "Nb joints = " << model.njoints << " (nq="<< model.nq<<",nv="<<model.nv<<")" << std::endl;
    for(Index i=0;i<(Index)(model.njoints);++i)
    {
      os << "  Joint "<< model.names[i] << ": parent=" << model.parents[i]  << std::endl;
    }
    
    return os;
  }
  
  template<typename JointCollection>
  template<typename JointModelDerived>
  typename ModelTpl<JointCollection>::JointIndex
  ModelTpl<JointCollection>::addJoint(const ModelTpl::JointIndex parent,
                                      const JointModelBase<JointModelDerived> & joint_model,
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

    assert(max_effort.size() == joint_model.nv()
           && max_velocity.size() == joint_model.nv()
           && min_config.size() == joint_model.nq()
           && max_config.size() == joint_model.nq());
    
    JointIndex idx = (JointIndex)(njoints++);
    
    joints         .push_back(JointModel(joint_model.derived()));
    JointModelDerived & jmodel = boost::get<JointModelDerived>(joints.back());
    jmodel.setIndexes(idx,nq,nv);
    
    inertias       .push_back(Inertia::Zero());
    parents        .push_back(parent);
    jointPlacements.push_back(joint_placement);
    names          .push_back(joint_name);
    nq += joint_model.nq();
    nv += joint_model.nv();

    // Optimal efficiency here would be using the static-dim bottomRows, while specifying the dimension in argument in the case where D::NV is Eigen::Dynamic.
    // However, this option is not compiling in Travis (why?).
    // As efficiency of ModelTpl::addJoint is not critical, the dynamic bottomRows is used here.
    effortLimit.conservativeResize(nv);
    jmodel.jointVelocitySelector(effortLimit) = max_effort;
    velocityLimit.conservativeResize(nv);
    jmodel.jointVelocitySelector(velocityLimit) = max_velocity;
    lowerPositionLimit.conservativeResize(nq);
    jmodel.jointConfigSelector(lowerPositionLimit) = min_config;
    upperPositionLimit.conservativeResize(nq);
    jmodel.jointConfigSelector(upperPositionLimit) = max_config;
    
    neutralConfiguration.conservativeResize(nq);
    typedef NeutralStep<LieGroupMap,ConfigVectorType> NeutralVisitor;
    NeutralStepAlgo<NeutralVisitor,JointModelDerived>::run(jmodel,neutralConfiguration);

    rotorInertia.conservativeResize(nv);
    jmodel.jointVelocitySelector(rotorInertia).setZero();
    rotorGearRatio.conservativeResize(nv);
    jmodel.jointVelocitySelector(rotorGearRatio).setZero();
    
    // Init and add joint index to its parent subtrees.
    subtrees.push_back(IndexVector(1));
    subtrees[idx][0] = idx;
    addJointIndexToParentSubtrees(idx);
    return idx;
  }

  template<typename JointCollection>
  template<typename JointModelDerived>
  typename ModelTpl<JointCollection>::JointIndex
  ModelTpl<JointCollection>::addJoint(const JointIndex parent,
                                      const JointModelBase<JointModelDerived> & joint_model,
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
  
  template<typename JointCollection>
  inline int ModelTpl<JointCollection>::addJointFrame(const JointIndex & jidx,
                                                      int         fidx)
  {
    if(fidx < 0) {
      // FIXED_JOINT is required because the parent can be the universe and its
      // type is FIXED_JOINT
      fidx = (int)getFrameId(names[parents[jidx]], (FrameType)(JOINT | FIXED_JOINT));
    }
    assert((size_t)fidx < frames.size() && "Frame index out of bound");
    // Add a the joint frame attached to itself to the frame vector - redundant information but useful.
    return addFrame(Frame(names[jidx],jidx,(FrameIndex)fidx,SE3::Identity(),JOINT));
  }

  template<typename JointCollection>
  inline void ModelTpl<JointCollection>::appendBodyToJoint(const ModelTpl::JointIndex joint_index,
                                                           const Inertia & Y,
                                                           const SE3 & body_placement)
  {
    const Inertia & iYf = Y.se3Action(body_placement);
    inertias[joint_index] += iYf;
    nbodies++;
  }

  template<typename JointCollection>
  inline int ModelTpl<JointCollection>::addBodyFrame(const std::string & body_name,
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
  
  template<typename JointCollection>
  inline typename ModelTpl<JointCollection>::JointIndex
  ModelTpl<JointCollection>::getBodyId(const std::string & name) const
  {
    return getFrameId(name, BODY);
  }
  
  template<typename JointCollection>
  inline bool ModelTpl<JointCollection>::existBodyName(const std::string & name) const
  {
    return existFrame(name, BODY);
  }

  template<typename JointCollection>
  inline typename  ModelTpl<JointCollection>::JointIndex
  ModelTpl<JointCollection>::getJointId(const std::string & name) const
  {
    typedef std::vector<std::string>::iterator::difference_type it_diff_t;
    it_diff_t res = std::find(names.begin(),names.end(),name) - names.begin();
    assert((res<INT_MAX) && "Id superior to int range. Should never happen.");
    return ModelTpl::JointIndex(res);
  }
  
  template<typename JointCollection>
  inline bool ModelTpl<JointCollection>::existJointName(const std::string & name) const
  {
    return (names.end() != std::find(names.begin(),names.end(),name));
  }

  template<typename JointCollection>
  inline const std::string &
  ModelTpl<JointCollection>::getJointName(const JointIndex index) const
  {
    assert( index < (ModelTpl::JointIndex)joints.size() );
    return names[index];
  }

  template<typename JointCollection>
  inline typename ModelTpl<JointCollection>::FrameIndex
  ModelTpl<JointCollection>::getFrameId(const std::string & name, const FrameType & type) const
  {
    typename container::aligned_vector<Frame>::const_iterator it
    = std::find_if(frames.begin()
                   ,frames.end()
                   ,details::FilterFrame(name, type));
    assert(it != frames.end() && "Frame not found");
    assert((std::find_if( boost::next(it), frames.end(), details::FilterFrame(name, type)) == frames.end())
        && "Several frames match the filter");
    return FrameIndex(it - frames.begin());
  }
  
  template<typename JointCollection>
  inline bool ModelTpl<JointCollection>::existFrame(const std::string & name, const FrameType & type) const
  {
    return std::find_if(frames.begin(), frames.end(),
                        details::FilterFrame(name, type)) != frames.end();
  }

  template<typename JointCollection>
  inline int ModelTpl<JointCollection>::addFrame(const Frame & frame)
  {
    if(!existFrame(frame.name, frame.type))
    {
      frames.push_back(frame);
      nframes++;
      return nframes - 1;
    }
    else
    {
      return -1;
    }
  }
  
  template<typename JointCollection>
  inline void ModelTpl<JointCollection>::addJointIndexToParentSubtrees(const JointIndex joint_id)
  {
    for(JointIndex parent = parents[joint_id]; parent>0; parent = parents[parent])
      subtrees[parent].push_back(joint_id);
  }

} // namespace se3

/// @endcond

#endif // ifndef __se3_model_hxx__
