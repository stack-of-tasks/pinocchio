//
// Copyright (c) 2015-2021 CNRS INRIA
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_multibody_model_hxx__
#define __pinocchio_multibody_model_hxx__

#include "pinocchio/utils/string-generator.hpp"
#include "pinocchio/multibody/liegroup/liegroup-algo.hpp"

/// @cond DEV

namespace pinocchio
{
  namespace details
  {
    struct FilterFrame
    {
      const std::string & name;
      const FrameType & typeMask;

      FilterFrame(const std::string & name, const FrameType & typeMask)
      : name(name)
      , typeMask(typeMask)
      {
      }

      template<typename Scalar, int Options>
      bool operator()(const FrameTpl<Scalar, Options> & frame) const
      {
        return (typeMask & frame.type) && (name == frame.name);
      }
    };
  } // namespace details

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  const typename ModelTpl<Scalar, Options, JointCollectionTpl>::Vector3
    ModelTpl<Scalar, Options, JointCollectionTpl>::gravity981((Scalar)0, (Scalar)0, (Scalar)-9.81);

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  inline std::ostream &
  operator<<(std::ostream & os, const ModelTpl<Scalar, Options, JointCollectionTpl> & model)
  {
    typedef typename ModelTpl<Scalar, Options, JointCollectionTpl>::Index Index;

    os << "Nb joints = " << model.njoints << " (nq=" << model.nq << ",nv=" << model.nv << ")"
       << std::endl;
    for (Index i = 0; i < (Index)(model.njoints); ++i)
    {
      os << "  Joint " << i << " " << model.names[i] << ": parent=" << model.parents[i]
         << std::endl;
    }

    return os;
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointIndex
  ModelTpl<Scalar, Options, JointCollectionTpl>::addJoint(
    const JointIndex parent,
    const JointModel & joint_model,
    const SE3 & joint_placement,
    const std::string & joint_name,
    const VectorXs & max_effort,
    const VectorXs & max_velocity,
    const VectorXs & min_config,
    const VectorXs & max_config,
    const VectorXs & joint_friction,
    const VectorXs & joint_damping)
  {
    assert(
      (njoints == (int)joints.size()) && (njoints == (int)inertias.size())
      && (njoints == (int)parents.size()) && (njoints == (int)jointPlacements.size()));
    assert((joint_model.nq() >= 0) && (joint_model.nv() >= 0));
    assert(joint_model.nq() >= joint_model.nv());

    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      max_effort.size(), joint_model.nv(), "The joint maximum effort vector is not of right size");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      max_velocity.size(), joint_model.nv(),
      "The joint maximum velocity vector is not of right size");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      min_config.size(), joint_model.nq(),
      "The joint lower configuration bound is not of right size");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      max_config.size(), joint_model.nq(),
      "The joint upper configuration bound is not of right size");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      joint_friction.size(), joint_model.nv(), "The joint friction vector is not of right size");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      joint_damping.size(), joint_model.nv(), "The joint damping vector is not of right size");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      parent < (JointIndex)njoints, "The index of the parent joint is not valid.");

    JointIndex joint_id = (JointIndex)(njoints++);

    joints.push_back(JointModel(joint_model.derived()));
    JointModel & jmodel = joints.back();
    jmodel.setIndexes(joint_id, nq, nv);

    const int joint_nq = jmodel.nq();
    const int joint_idx_q = jmodel.idx_q();
    const int joint_nv = jmodel.nv();
    const int joint_idx_v = jmodel.idx_v();

    assert(joint_idx_q >= 0);
    assert(joint_idx_v >= 0);

    inertias.push_back(Inertia::Zero());
    parents.push_back(parent);
    children.push_back(IndexVector());
    children[parent].push_back(joint_id);
    jointPlacements.push_back(joint_placement);
    names.push_back(joint_name);

    nq += joint_nq;
    nqs.push_back(joint_nq);
    idx_qs.push_back(joint_idx_q);
    nv += joint_nv;
    nvs.push_back(joint_nv);
    idx_vs.push_back(joint_idx_v);

    if (joint_nq > 0 && joint_nv > 0)
    {
      effortLimit.conservativeResize(nv);
      jmodel.jointVelocitySelector(effortLimit) = max_effort;
      velocityLimit.conservativeResize(nv);
      jmodel.jointVelocitySelector(velocityLimit) = max_velocity;
      lowerPositionLimit.conservativeResize(nq);
      jmodel.jointConfigSelector(lowerPositionLimit) = min_config;
      upperPositionLimit.conservativeResize(nq);
      jmodel.jointConfigSelector(upperPositionLimit) = max_config;

      armature.conservativeResize(nv);
      jmodel.jointVelocitySelector(armature).setZero();
      rotorInertia.conservativeResize(nv);
      jmodel.jointVelocitySelector(rotorInertia).setZero();
      rotorGearRatio.conservativeResize(nv);
      jmodel.jointVelocitySelector(rotorGearRatio).setOnes();
      friction.conservativeResize(nv);
      jmodel.jointVelocitySelector(friction) = joint_friction;
      damping.conservativeResize(nv);
      jmodel.jointVelocitySelector(damping) = joint_damping;
    }

    // Init and add joint index to its parent subtrees.
    subtrees.push_back(IndexVector(1));
    subtrees[joint_id][0] = joint_id;
    addJointIndexToParentSubtrees(joint_id);

    // Init and add joint index to the supports
    supports.push_back(supports[parent]);
    supports[joint_id].push_back(joint_id);

    return joint_id;
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointIndex
  ModelTpl<Scalar, Options, JointCollectionTpl>::addJoint(
    const JointIndex parent,
    const JointModel & joint_model,
    const SE3 & joint_placement,
    const std::string & joint_name,
    const VectorXs & max_effort,
    const VectorXs & max_velocity,
    const VectorXs & min_config,
    const VectorXs & max_config)
  {
    const VectorXs friction = VectorXs::Constant(joint_model.nv(), static_cast<Scalar>(0));
    const VectorXs damping = VectorXs::Constant(joint_model.nv(), static_cast<Scalar>(0));

    return addJoint(
      parent, joint_model, joint_placement, joint_name, max_effort, max_velocity, min_config,
      max_config, friction, damping);
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointIndex
  ModelTpl<Scalar, Options, JointCollectionTpl>::addJoint(
    const JointIndex parent,
    const JointModel & joint_model,
    const SE3 & joint_placement,
    const std::string & joint_name)
  {
    const VectorXs max_effort =
      VectorXs::Constant(joint_model.nv(), std::numeric_limits<Scalar>::max());
    const VectorXs max_velocity =
      VectorXs::Constant(joint_model.nv(), std::numeric_limits<Scalar>::max());
    const VectorXs min_config =
      VectorXs::Constant(joint_model.nq(), -std::numeric_limits<Scalar>::max());
    const VectorXs max_config =
      VectorXs::Constant(joint_model.nq(), std::numeric_limits<Scalar>::max());

    return addJoint(
      parent, joint_model, joint_placement, joint_name, max_effort, max_velocity, min_config,
      max_config);
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  FrameIndex ModelTpl<Scalar, Options, JointCollectionTpl>::addJointFrame(
    const JointIndex & joint_index, int previous_frame_index)
  {
    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      joint_index < joints.size(),
      "The joint index is larger than the number of joints in the model.");
    if (previous_frame_index < 0)
    {
      // FIXED_JOINT is required because the parent can be the universe and its
      // type is FIXED_JOINT
      previous_frame_index =
        (int)getFrameId(names[parents[joint_index]], (FrameType)(JOINT | FIXED_JOINT));
    }
    assert((size_t)previous_frame_index < frames.size() && "Frame index out of bound");

    // Add a the joint frame attached to itself to the frame vector - redundant information but
    // useful.
    return addFrame(Frame(
      names[joint_index], joint_index, (FrameIndex)previous_frame_index, SE3::Identity(), JOINT));
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  template<typename NewScalar>
  typename CastType<NewScalar, ModelTpl<Scalar, Options, JointCollectionTpl>>::type
  ModelTpl<Scalar, Options, JointCollectionTpl>::cast() const
  {
    typedef ModelTpl<NewScalar, Options, JointCollectionTpl> ReturnType;

    ReturnType res;
    res.nq = nq;
    res.nv = nv;
    res.njoints = njoints;
    res.nbodies = nbodies;
    res.nframes = nframes;
    res.parents = parents;
    res.children = children;
    res.names = names;
    res.subtrees = subtrees;
    res.supports = supports;
    res.gravity = gravity.template cast<NewScalar>();
    res.name = name;

    res.idx_qs = idx_qs;
    res.nqs = nqs;
    res.idx_vs = idx_vs;
    res.nvs = nvs;

    // Eigen Vectors
    res.armature = armature.template cast<NewScalar>();
    res.friction = friction.template cast<NewScalar>();
    res.damping = damping.template cast<NewScalar>();
    res.rotorInertia = rotorInertia.template cast<NewScalar>();
    res.rotorGearRatio = rotorGearRatio.template cast<NewScalar>();
    res.effortLimit = effortLimit.template cast<NewScalar>();
    res.velocityLimit = velocityLimit.template cast<NewScalar>();
    res.lowerPositionLimit = lowerPositionLimit.template cast<NewScalar>();
    res.upperPositionLimit = upperPositionLimit.template cast<NewScalar>();

    typename ConfigVectorMap::const_iterator it;
    for (it = referenceConfigurations.begin(); it != referenceConfigurations.end(); it++)
    {
      res.referenceConfigurations.insert(
        std::make_pair(it->first, it->second.template cast<NewScalar>()));
    }

    // reserve vectors
    res.inertias.resize(inertias.size());
    res.jointPlacements.resize(jointPlacements.size());
    res.joints.resize(joints.size());

    // copy into vectors
    for (size_t k = 0; k < joints.size(); ++k)
    {
      res.inertias[k] = inertias[k].template cast<NewScalar>();
      res.jointPlacements[k] = jointPlacements[k].template cast<NewScalar>();
      res.joints[k] = joints[k].template cast<NewScalar>();
    }

    res.frames.resize(frames.size());
    for (size_t k = 0; k < frames.size(); ++k)
    {
      res.frames[k] = frames[k].template cast<NewScalar>();
    }

    return res;
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  bool ModelTpl<Scalar, Options, JointCollectionTpl>::operator==(const ModelTpl & other) const
  {
    bool res = other.nq == nq && other.nv == nv && other.njoints == njoints
               && other.nbodies == nbodies && other.nframes == nframes && other.parents == parents
               && other.children == children && other.names == names && other.subtrees == subtrees
               && other.gravity == gravity && other.name == name;

    res &= other.idx_qs == idx_qs && other.nqs == nqs && other.idx_vs == idx_vs && other.nvs == nvs;

    if (other.referenceConfigurations.size() != referenceConfigurations.size())
      return false;

    typename ConfigVectorMap::const_iterator it = referenceConfigurations.begin();
    typename ConfigVectorMap::const_iterator it_other = other.referenceConfigurations.begin();
    for (long k = 0; k < (long)referenceConfigurations.size(); ++k)
    {
      if (it->second.size() != it_other->second.size())
        return false;
      if (it->second != it_other->second)
        return false;
      std::advance(it, 1);
      std::advance(it_other, 1);
    }
    if (other.armature.size() != armature.size())
      return false;
    res &= other.armature == armature;
    if (!res)
      return res;

    if (other.friction.size() != friction.size())
      return false;
    res &= other.friction == friction;
    if (!res)
      return res;

    if (other.damping.size() != damping.size())
      return false;
    res &= other.damping == damping;
    if (!res)
      return res;

    if (other.rotorInertia.size() != rotorInertia.size())
      return false;
    res &= other.rotorInertia == rotorInertia;
    if (!res)
      return res;

    if (other.rotorGearRatio.size() != rotorGearRatio.size())
      return false;
    res &= other.rotorGearRatio == rotorGearRatio;
    if (!res)
      return res;

    if (other.effortLimit.size() != effortLimit.size())
      return false;
    res &= other.effortLimit == effortLimit;
    if (!res)
      return res;

    if (other.velocityLimit.size() != velocityLimit.size())
      return false;
    res &= other.velocityLimit == velocityLimit;
    if (!res)
      return res;

    if (other.lowerPositionLimit.size() != lowerPositionLimit.size())
      return false;
    res &= other.lowerPositionLimit == lowerPositionLimit;
    if (!res)
      return res;

    if (other.upperPositionLimit.size() != upperPositionLimit.size())
      return false;
    res &= other.upperPositionLimit == upperPositionLimit;
    if (!res)
      return res;

    for (size_t k = 1; k < inertias.size(); ++k)
    {
      res &= other.inertias[k] == inertias[k];
      if (!res)
        return res;
    }

    for (size_t k = 1; k < other.jointPlacements.size(); ++k)
    {
      res &= other.jointPlacements[k] == jointPlacements[k];
      if (!res)
        return res;
    }

    res &= other.joints == joints && other.frames == frames;

    return res;
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  void ModelTpl<Scalar, Options, JointCollectionTpl>::appendBodyToJoint(
    const typename ModelTpl::JointIndex joint_index, const Inertia & Y, const SE3 & body_placement)
  {
    const Inertia & iYf = Y.se3Action(body_placement);
    inertias[joint_index] += iYf;
    nbodies++;
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  typename ModelTpl<Scalar, Options, JointCollectionTpl>::FrameIndex
  ModelTpl<Scalar, Options, JointCollectionTpl>::addBodyFrame(
    const std::string & body_name,
    const JointIndex & parentJoint,
    const SE3 & body_placement,
    int parentFrame)
  {
    if (parentFrame < 0)
    {
      // FIXED_JOINT is required because the parent can be the universe and its
      // type is FIXED_JOINT
      parentFrame = (int)getFrameId(names[parentJoint], (FrameType)(JOINT | FIXED_JOINT));
    }
    PINOCCHIO_CHECK_INPUT_ARGUMENT((size_t)parentFrame < frames.size(), "Frame index out of bound");
    return addFrame(Frame(body_name, parentJoint, (FrameIndex)parentFrame, body_placement, BODY));
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  inline typename ModelTpl<Scalar, Options, JointCollectionTpl>::FrameIndex
  ModelTpl<Scalar, Options, JointCollectionTpl>::getBodyId(const std::string & name) const
  {
    return getFrameId(name, BODY);
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  inline bool
  ModelTpl<Scalar, Options, JointCollectionTpl>::existBodyName(const std::string & name) const
  {
    return existFrame(name, BODY);
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  inline typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointIndex
  ModelTpl<Scalar, Options, JointCollectionTpl>::getJointId(const std::string & name) const
  {
    typedef std::vector<std::string>::iterator::difference_type it_diff_t;
    it_diff_t res = std::find(names.begin(), names.end(), name) - names.begin();
    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      (res < INT_MAX), "Id superior to int range. Should never happen.");
    return ModelTpl::JointIndex(res);
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  inline bool
  ModelTpl<Scalar, Options, JointCollectionTpl>::existJointName(const std::string & name) const
  {
    return (names.end() != std::find(names.begin(), names.end(), name));
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  inline typename ModelTpl<Scalar, Options, JointCollectionTpl>::FrameIndex
  ModelTpl<Scalar, Options, JointCollectionTpl>::getFrameId(
    const std::string & name, const FrameType & type) const
  {
    typename PINOCCHIO_ALIGNED_STD_VECTOR(Frame)::const_iterator it =
      std::find_if(frames.begin(), frames.end(), details::FilterFrame(name, type));
    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      ((it == frames.end()
        || (std::find_if(boost::next(it), frames.end(), details::FilterFrame(name, type)) == frames.end()))),
      "Several frames match the filter - please specify the FrameType");
    return FrameIndex(it - frames.begin());
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  inline bool ModelTpl<Scalar, Options, JointCollectionTpl>::existFrame(
    const std::string & name, const FrameType & type) const
  {
    return std::find_if(frames.begin(), frames.end(), details::FilterFrame(name, type))
           != frames.end();
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  typename ModelTpl<Scalar, Options, JointCollectionTpl>::FrameIndex
  ModelTpl<Scalar, Options, JointCollectionTpl>::addFrame(
    const Frame & frame, const bool append_inertia)
  {
    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      frame.parentJoint < (JointIndex)njoints, "The index of the parent joint is not valid.");

    //    TODO: fix it
    //    PINOCCHIO_CHECK_INPUT_ARGUMENT(frame.inertia.isValid(),
    //                                   "The input inertia is not valid.")

    // Check if the frame.name exists with the same type
    if (existFrame(frame.name, frame.type))
    {
      return getFrameId(frame.name, frame.type);
    }
    // else: we must add a new frames to the current stack
    frames.push_back(frame);
    if (append_inertia)
      inertias[frame.parentJoint] += frame.placement.act(frame.inertia);
    nframes++;
    return FrameIndex(nframes - 1);
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  void ModelTpl<Scalar, Options, JointCollectionTpl>::addJointIndexToParentSubtrees(
    const JointIndex joint_id)
  {
    for (JointIndex parent = parents[joint_id]; parent > 0; parent = parents[parent])
      subtrees[parent].push_back(joint_id);

    // Also add joint_id to the universe
    subtrees[0].push_back(joint_id);
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  std::vector<bool> ModelTpl<Scalar, Options, JointCollectionTpl>::hasConfigurationLimit()
  {
    std::vector<bool> vec;
    for (Index i = 1; i < (Index)(njoints); ++i)
    {
      const std::vector<bool> & cf_limits = joints[i].hasConfigurationLimit();
      vec.insert(vec.end(), cf_limits.begin(), cf_limits.end());
    }
    return vec;
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  std::vector<bool> ModelTpl<Scalar, Options, JointCollectionTpl>::hasConfigurationLimitInTangent()
  {
    std::vector<bool> vec;
    for (Index i = 1; i < (Index)(njoints); ++i)
    {
      const std::vector<bool> & cf_limits = joints[i].hasConfigurationLimitInTangent();
      vec.insert(vec.end(), cf_limits.begin(), cf_limits.end());
    }
    return vec;
  }

} // namespace pinocchio

/// @endcond

#endif // ifndef __pinocchio_multibody_model_hxx__
