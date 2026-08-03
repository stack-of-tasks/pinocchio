//
// Copyright (c) 2015-2024 CNRS INRIA
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#pragma once

// IWYU pragma: private, include "pinocchio/multibody.hpp"

#ifdef PINOCCHIO_LSP
  #undef PINOCCHIO_LSP
  #include "pinocchio/multibody.hpp"
#endif // PINOCCHIO_LSP

namespace pinocchio
{
  template<
    typename NewScalar,
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl>
  struct CastType<NewScalar, ModelTpl<Scalar, Options, JointCollectionTpl>>
  {
    typedef ModelTpl<NewScalar, Options, JointCollectionTpl> type;
  };

  template<typename _Scalar, int _Options, template<typename, int> class JointCollectionTpl>
  struct traits<ModelTpl<_Scalar, _Options, JointCollectionTpl>>
  {
    typedef _Scalar Scalar;
    static constexpr int Options = _Options;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;
    typedef JointCollectionTpl<Scalar, Options> JointCollection;
  };

  template<typename _Scalar, int _Options, template<typename, int> class JointCollectionTpl>
  struct ModelTpl
  : serialization::Serializable<ModelTpl<_Scalar, _Options, JointCollectionTpl>>
  , NumericalBase<ModelTpl<_Scalar, _Options, JointCollectionTpl>>
  , ModelEntity<ModelTpl<_Scalar, _Options, JointCollectionTpl>>
  {

    typedef typename traits<ModelTpl>::Scalar Scalar;
    static constexpr int Options = traits<ModelTpl>::Options;

    typedef typename traits<ModelTpl>::JointCollection JointCollection;
    typedef typename traits<ModelTpl>::Data Data;

    typedef SE3Tpl<Scalar, Options> SE3;
    typedef MotionTpl<Scalar, Options> Motion;
    typedef ForceTpl<Scalar, Options> Force;
    typedef InertiaTpl<Scalar, Options> Inertia;
    typedef FrameTpl<Scalar, Options> Frame;

    typedef pinocchio::Index Index;
    typedef pinocchio::JointIndex JointIndex;
    typedef pinocchio::GeomIndex GeomIndex;
    typedef pinocchio::FrameIndex FrameIndex;
    typedef std::vector<Index> IndexVector;

    typedef JointModelTpl<Scalar, Options, JointCollectionTpl> JointModel;
    typedef JointDataTpl<Scalar, Options, JointCollectionTpl> JointData;

    typedef std::vector<JointModel> JointModelVector;
    typedef std::vector<JointData> JointDataVector;

    typedef std::vector<Frame> FrameVector;

    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1, Options> VectorXs;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Options> MatrixXs;
    typedef Eigen::Matrix<Scalar, 3, 1, Options> Vector3;

    typedef std::vector<Inertia> InertiaVector;
    typedef std::vector<SE3> SE3Vector;

    typedef Eigen::Matrix<bool, Eigen::Dynamic, 1, Options> BooleanVector;
    typedef std::vector<Eigen::Index> EigenIndexVector;
    typedef std::vector<BooleanVector> VectorOfBooleanVector;
    typedef std::vector<EigenIndexVector> VectorOfEigenIndexVector;

    /// \brief Dense vectorized version of a joint configuration vector.
    typedef VectorXs ConfigVectorType;

    /// \brief Map between a string (key) and a configuration vector
    typedef std::map<std::string, ConfigVectorType> ConfigVectorMap;

    /// \brief Dense vectorized version of a joint tangent vector (e.g. velocity, acceleration,
    /// etc).
    ///        It also handles the notion of co-tangent vector (e.g. torque, etc).
    typedef VectorXs TangentVectorType;

    /// \brief Dimension of the configuration vector representation.
    int nq;

    /// \brief Dimension of the velocity vector space.
    int nv;

    /// \brief Dimension of the jacobian space.
    int nvExtended;

    /// \brief Number of joints.
    int njoints;

    /// \brief Number of bodies.
    int nbodies;

    /// \brief Number of operational frames.
    int nframes;

    /// \brief Vector of spatial inertias supported by each joint.
    InertiaVector inertias;

    /// \brief Vector of joint placements: placement of a joint *i* wrt its parent joint frame.
    SE3Vector jointPlacements;

    /// \brief Vector of joint models.
    JointModelVector joints;

    /// \brief Vector of starting index of the *i*th  joint in the configuration space.
    std::vector<int> idx_qs;

    /// \brief Vector of dimension of the  joint configuration subspace.
    std::vector<int> nqs;

    /// \brief Starting index of the *i*th joint in the tangent configuration space.
    std::vector<int> idx_vs;

    /// \brief Dimension of the *i*th joint tangent subspace.
    std::vector<int> nvs;

    /// \brief Starting index of the *i*th joint in the jacobian space.
    std::vector<int> idx_vExtendeds;

    /// \brief Dimension of the *i*th joint jacobian subspace.
    std::vector<int> nvExtendeds;

    /// \brief Vector of parent joint indexes. The parent of joint *i*, denoted *li*, corresponds to
    /// li==parents[i].
    std::vector<JointIndex> parents;

    /// \brief Vector of children index. Chidren of the *i*th joint, denoted *mu(i)* corresponds to
    /// the set (i==parents[k] for k in mu(i)).
    std::vector<IndexVector> children;

    /// \brief Vector of mimicking joints in the tree (with type MimicTpl)
    std::vector<JointIndex> mimicking_joints;

    /// \brief Vector of mimicked joints in the tree (can be any joint type)
    /// The i-th element of this vector correspond to the mimicked joint of the i-th mimicking
    /// vector in mimicking_joints
    std::vector<JointIndex> mimicked_joints;

    /// \brief Name of the joints.
    std::vector<std::string> names;

    /// \brief Map of reference configurations, indexed by user given names.
    ConfigVectorMap referenceConfigurations;

    /// \brief Vector of armature values expressed at the joint level
    /// This vector may contain the contribution of rotor inertia effects for instance.
    VectorXs armature;

    /// \brief Vector of rotor inertia parameters
    TangentVectorType rotorInertia;

    /// \brief Vector of rotor gear ratio parameters
    TangentVectorType rotorGearRatio;

    /// \brief Vector of joint friction parameters
    /// Deprecated in favor of lowerDryFrictionLimit and upperDryFrictionLimit
    PINOCCHIO_DEPRECATED TangentVectorType & friction;

    /// \brief Vector of joint friction parameters
    TangentVectorType lowerDryFrictionLimit;

    /// \brief Vector of joint friction parameters
    TangentVectorType upperDryFrictionLimit;

    /// \brief Vector of joint damping parameters
    TangentVectorType damping;

    /// \brief Vector of minimal joint torques
    TangentVectorType lowerEffortLimit;

    /// \brief Vector of maximal joint torques
    TangentVectorType upperEffortLimit;

    /// \brief Vector of maximal joint torques
    /// Deprecated in favor of lowerEffortLimit and upperEffortLimit
    PINOCCHIO_DEPRECATED TangentVectorType & effortLimit;

    /// \brief Vector of minimal joint velocities
    TangentVectorType lowerVelocityLimit;

    /// \brief Vector of maximal joint velocities
    TangentVectorType upperVelocityLimit;

    /// \brief Vector of maximal joint velocities
    /// Deprecated in favor of lowerVelocityLimit and upperVelocityLimit
    PINOCCHIO_DEPRECATED TangentVectorType & velocityLimit;

    /// \brief Lower joint configuration limit
    ConfigVectorType lowerPositionLimit;

    /// \brief Upper joint configuration limit
    ConfigVectorType upperPositionLimit;

    /// \brief Joint configuration limit margin
    ConfigVectorType positionLimitMargin;

    /// \brief Joint acceleration lower limit
    TangentVectorType lowerAccelerationLimit;

    /// \brief Joint acceleration upper limit
    TangentVectorType upperAccelerationLimit;

    /// \brief Joint jerk lower limit
    TangentVectorType lowerJerkLimit;

    /// \brief Joint jerk upper limit
    TangentVectorType upperJerkLimit;

    /// \brief Vector of operational frames registered on the model.
    FrameVector frames;

    /// \brief Vector of joint supports.
    /// supports[j] corresponds to the vector of indices of the joints located on the path between
    /// joint *j*  and "universe".
    /// The first element of supports[j] is "universe", the last one is the index of joint *j*
    /// itself.
    std::vector<IndexVector> supports;

    /// \brief Vector of mimic supports joints.
    /// mimic_joint_supports[j] corresponds to the vector of mimic joints indices located on the
    /// path between joint *j*  and "universe". The first element of mimic_joint_supports[j] is
    /// "universe". If *j* is a mimic, the last element is the index of joint *j* itself.
    std::vector<IndexVector> mimic_joint_supports;

    /// \brief Vector of joint subtrees.
    /// subtree[j] corresponds to the subtree supported by the joint *j*.
    /// The first element of subtree[j] is the index of the joint *j* itself.
    std::vector<IndexVector> subtrees;

    /// \brief Sparsity pattern for each joint.
    /// sparsity_pattern_vector[i] is a boolean vector of size nv indicating which columns
    /// of the Jacobian are nonzero for joint i.
    VectorOfBooleanVector sparsity_pattern_vector;

    /// \brief Colwise span indexes for each joints.
    /// span_indexes_vector[i] lists the column indexes of nonzero entries for joint i.
    VectorOfEigenIndexVector span_indexes_vector;

    /// \brief Spatial gravity of the model.
    Motion gravity;

    /// \brief Default 3D gravity vector (=(0,0,-9.81)).
    static const Vector3 gravity981;

    /// \brief Model name.
    std::string name;

    /// \brief Default constructor. Builds an empty model with no joints.
    PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
    PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
    ModelTpl()
    : nq(0)
    , nv(0)
    , nvExtended(0)
    , njoints(1)
    , nbodies(1)
    , nframes(0)
    , inertias(1, Inertia::Zero())
    , jointPlacements(1, SE3::Identity())
    , joints(1)
    , idx_qs(1, 0)
    , nqs(1, 0)
    , idx_vs(1, 0)
    , nvs(1, 0)
    , idx_vExtendeds(1, 0)
    , nvExtendeds(1, 0)
    , parents(1, 0)
    , children(1)
    , names(1)
    , friction(upperDryFrictionLimit)
    , effortLimit(upperEffortLimit)
    , velocityLimit(upperVelocityLimit)
    , supports(1, IndexVector(1, 0))
    , mimic_joint_supports(1, IndexVector(1, 0))
    , subtrees(1)
    , sparsity_pattern_vector(1)
    , span_indexes_vector(1)
    , gravity(gravity981, Vector3::Zero())
    {
      names[0] = "universe"; // Should be "universe joint (trivial)"
      // FIXME Should the universe joint be a FIXED_JOINT even if it is
      // in the list of joints ? See comment in definition of
      // Model::addJointFrame and Model::addBodyFrame
      addFrame(Frame("universe", 0, 0, SE3::Identity(), FIXED_JOINT));
    }
    PINOCCHIO_COMPILER_DIAGNOSTIC_POP

    ///
    /// \brief Copy constructor by casting
    ///
    /// \param[in] other model to copy to *this
    ///
    PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
    PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
    template<typename S2, int O2>
    explicit ModelTpl(const ModelTpl<S2, O2> & other)
    : friction(upperDryFrictionLimit)
    , effortLimit(upperEffortLimit)
    , velocityLimit(upperVelocityLimit)
    {
      *this = other.template cast<Scalar>();
    }
    PINOCCHIO_COMPILER_DIAGNOSTIC_POP

    ///
    /// \brief Copy constructor from another collection
    ///
    /// \param[in] other model to copy to *this
    ///
    PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
    PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
    template<template<typename, int> class OtherJointCollectionTpl>
    ModelTpl(const ModelTpl<Scalar, Options, OtherJointCollectionTpl> & other)
    : friction(upperDryFrictionLimit)
    , effortLimit(upperEffortLimit)
    , velocityLimit(upperVelocityLimit)
    {
      *this = other;
    }
    PINOCCHIO_COMPILER_DIAGNOSTIC_POP

    ///
    /// \brief Copy constructor.
    ///
    /// \param[in] other model to copy to *this
    ///
    PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
    PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
    ModelTpl(const ModelTpl & other)
    : friction(upperDryFrictionLimit)
    , effortLimit(upperEffortLimit)
    , velocityLimit(upperVelocityLimit)
    {
      *this = other;
    }
    PINOCCHIO_COMPILER_DIAGNOSTIC_POP

    /// \returns A new copy of *this with the Scalar type casted to NewScalar.
    template<typename NewScalar>
    typename CastType<NewScalar, ModelTpl>::type cast() const;

    ///
    /// \brief Equality comparison operator.
    ///
    /// \returns true if *this is equal to other.
    ///
    bool operator==(const ModelTpl & other) const;

    ///
    /// \brief Assignment operator from another collection.
    ///
    ///
    template<template<typename, int> class OtherJointCollectionTpl>
    ModelTpl & operator=(const ModelTpl<Scalar, Options, OtherJointCollectionTpl> & other);

    ///
    /// \brief Assignment operator.
    ///
    ///
    ModelTpl & operator=(const ModelTpl & other)
    {
      (*this).template operator= <JointCollectionTpl>(other);
      return *this;
    }

    ///
    /// \returns true if *this is NOT equal to other.
    ///
    bool operator!=(const ModelTpl & other) const
    {
      return !(*this == other);
    }

    ///
    /// \brief Add a joint to the kinematic tree with infinite bounds.
    ///
    /// \remarks This method does not add a Frame of same name to the vector of frames.
    ///         Use Model::addJointFrame.
    /// \remarks The inertia supported by the joint is set to Zero.
    /// \remark Joints need to be added to the tree in a depth-first order.
    ///
    /// \tparam JointModelDerived The type of the joint model.
    ///
    /// \param[in] parent Index of the parent joint.
    /// \param[in] joint_model The joint model.
    /// \param[in] joint_placement Placement of the joint inside its parent joint.
    /// \param[in] joint_name Name of the joint. If empty, the name is random.
    ///
    /// \return The index of the new joint.
    ///
    /// \sa Model::appendBodyToJoint
    ///
    JointIndex addJoint(
      const JointIndex parent,
      const JointModel & joint_model,
      const SE3 & joint_placement,
      const std::string & joint_name);

    ///
    /// \copydoc ModelTpl::addJoint(const JointIndex,const JointModel &,const SE3 &,const
    /// std::string &)
    /// Deprecated in favor of the constructor using min and max effort/velocity
    ///
    /// \param[in] max_effort Maximal joint torque.
    /// \param[in] max_velocity Maximal joint velocity.
    /// \param[in] min_config Lower joint configuration.
    /// \param[in] max_config Upper joint configuration.
    ///
    JointIndex addJoint(
      const JointIndex parent,
      const JointModel & joint_model,
      const SE3 & joint_placement,
      const std::string & joint_name,
      const VectorXs & max_effort,
      const VectorXs & max_velocity,
      const VectorXs & min_config,
      const VectorXs & max_config);

    ///
    /// \copydoc ModelTpl::addJoint(const JointIndex,const JointModel &,const SE3 &,const
    /// std::string &)
    /// Deprecated in favor of the constructor using min and max effort/velocity
    ///
    /// \param[in] max_effort Maximal joint torque.
    /// \param[in] max_velocity Maximal joint velocity.
    /// \param[in] min_config Lower joint configuration.
    /// \param[in] max_config Upper joint configuration.
    /// \param[in] config_limit_margin Joint configuration limit margin.
    ///
    JointIndex addJoint(
      const JointIndex parent,
      const JointModel & joint_model,
      const SE3 & joint_placement,
      const std::string & joint_name,
      const VectorXs & max_effort,
      const VectorXs & max_velocity,
      const VectorXs & min_config,
      const VectorXs & max_config,
      const VectorXs & config_limit_margin);

    ///
    /// \copydoc ModelTpl::addJoint(const JointIndex,const JointModel &,const SE3 &,const
    /// std::string &,const VectorXs &,const VectorXs &,const VectorXs &,const VectorXs &)
    /// Deprecated in favor of the constructor using min and max effort/velocity
    ///
    /// \param[in] min_effort Minimal joint torque.
    /// \param[in] min_velocity Minimal joint velocity.
    /// \param[in] min_friction Minimal joint friction parameters.
    /// \param[in] max_friction Maximal joint friction parameters.
    /// \param[in] damping Joint damping parameters.
    ///
    JointIndex addJoint(
      const JointIndex parent,
      const JointModel & joint_model,
      const SE3 & joint_placement,
      const std::string & joint_name,
      const VectorXs & min_effort,
      const VectorXs & max_effort,
      const VectorXs & min_velocity,
      const VectorXs & max_velocity,
      const VectorXs & min_config,
      const VectorXs & max_config,
      const VectorXs & min_friction,
      const VectorXs & max_friction,
      const VectorXs & damping);

    ///
    /// \copydoc ModelTpl::addJoint(const JointIndex,const JointModel &,const SE3 &,const
    /// std::string &,const VectorXs &,const VectorXs &,const VectorXs &,const VectorXs &)
    /// Deprecated in favor of the constructor using min and max effort/velocity
    ///
    /// \param[in] config_limit_margin Joint configuration limit margin.
    /// \param[in] min_effort Minimal joint torque.
    /// \param[in] min_velocity Minimal joint velocity.
    /// \param[in] min_friction Minimal joint friction parameters.
    /// \param[in] max_friction Maximal joint friction parameters.
    /// \param[in] damping Joint damping parameters.
    ///
    JointIndex addJoint(
      const JointIndex parent,
      const JointModel & joint_model,
      const SE3 & joint_placement,
      const std::string & joint_name,
      const VectorXs & min_effort,
      const VectorXs & max_effort,
      const VectorXs & min_velocity,
      const VectorXs & max_velocity,
      const VectorXs & min_config,
      const VectorXs & max_config,
      const VectorXs & config_limit_margin,
      const VectorXs & min_friction,
      const VectorXs & max_friction,
      const VectorXs & damping);

    JointIndex addJoint(
      const JointIndex parent,
      const JointModel & joint_model,
      const SE3 & joint_placement,
      const std::string & joint_name,
      const VectorXs & min_effort,
      const VectorXs & max_effort,
      const VectorXs & min_velocity,
      const VectorXs & max_velocity,
      const VectorXs & min_config,
      const VectorXs & max_config,
      const VectorXs & config_limit_margin,
      const VectorXs & min_friction,
      const VectorXs & max_friction,
      const VectorXs & damping,
      const VectorXs & min_acceleration,
      const VectorXs & max_acceleration,
      const VectorXs & min_jerk,
      const VectorXs & max_jerk);

    ///
    /// \copydoc ModelTpl::addJoint(const JointIndex,const JointModel &,const SE3 &,const
    /// std::string &,const VectorXs &,const VectorXs &,const VectorXs &,const VectorXs &)
    ///
    /// \param[in] config_limit_margin Joint configuration limit margin.
    /// \param[in] friction Joint friction parameters.
    /// \param[in] damping Joint damping parameters.
    ///
    JointIndex addJoint(
      const JointIndex parent,
      const JointModel & joint_model,
      const SE3 & joint_placement,
      const std::string & joint_name,
      const VectorXs & max_effort,
      const VectorXs & max_velocity,
      const VectorXs & min_config,
      const VectorXs & max_config,
      const VectorXs & config_limit_margin,
      const VectorXs & friction,
      const VectorXs & damping);

    ///
    /// \copydoc ModelTpl::addJoint(const JointIndex,const JointModel &,const SE3 &,const
    /// std::string &,const VectorXs &,const VectorXs &,const VectorXs &,const VectorXs &)
    ///
    /// \param[in] friction Joint friction parameters.
    /// \param[in] damping Joint damping parameters.
    ///
    JointIndex addJoint(
      const JointIndex parent,
      const JointModel & joint_model,
      const SE3 & joint_placement,
      const std::string & joint_name,
      const VectorXs & max_effort,
      const VectorXs & max_velocity,
      const VectorXs & min_config,
      const VectorXs & max_config,
      const VectorXs & friction,
      const VectorXs & damping);

    ///
    /// \brief Add a joint to the frame tree.
    ///
    /// \param[in] jointIndex Index of the joint.
    /// \param[in] frameIndex Index of the parent frame. If negative,
    ///            the parent frame is the frame of the parent joint.
    ///
    /// \return The index of the new frame
    ///
    FrameIndex addJointFrame(const JointIndex joint_index, int previous_frame_index = -1);

    ///
    /// \brief Append a body to a given joint of the kinematic tree.
    ///
    /// \param[in] joint_index Index of the supporting joint.
    /// \param[in] Y Spatial inertia of the body.
    /// \param[in] body_placement The relative placement of the body regarding to the parent joint.
    /// Set default to the Identity placement.
    ///
    /// \sa Model::addJoint
    ///
    void appendBodyToJoint(
      const JointIndex joint_index,
      const Inertia & Y,
      const SE3 & body_placement = SE3::Identity());

    ///
    /// \brief Add a body to the frame tree.
    ///
    /// \param[in] body_name Name of the body.
    /// \param[in] parentJoint Index of the parent joint.
    /// \param[in] body_placement The relative placement of the body regarding to the parent joint.
    /// Set default to the Identity placement. \param[in] parentFrame Index of the parent frame. If
    /// negative,
    ///            the parent frame is the frame of the parent joint.
    ///
    /// \return The index of the new frame
    ///
    FrameIndex addBodyFrame(
      const std::string & body_name,
      const JointIndex & parentJoint,
      const SE3 & body_placement = SE3::Identity(),
      int parentFrame = -1);

    ///
    /// \brief Return the index of a body given by its name.
    ///
    /// \warning If no body is found, return the number of elements at time T.
    /// This can lead to errors if the model is expanded after this method is called
    /// (for example to get the id of a parent body)
    ///
    /// \param[in] name Name of the body.
    ///
    /// \return Index of the body.
    ///
    FrameIndex getBodyId(const std::string & name) const;

    ///
    /// \brief Check if a body given by its name exists.
    ///
    /// \param[in] name Name of the body.
    ///
    /// \return True if the body exists in the kinematic tree.
    ///
    bool existBodyName(const std::string & name) const;

    ///
    /// \brief Return the index of a joint given by its name.
    ///
    /// \warning If no joint is found, return the number of elements at time T.
    /// This can lead to errors if the model is expanded after this method is called
    /// (for example to get the id of a parent joint)
    /// \param[in] name Name of the joint.
    ///
    /// \return Index of the joint.
    ///
    JointIndex getJointId(const std::string & name) const;

    ///
    /// \brief Check if a joint given by its name exists.
    ///
    /// \param[in] name Name of the joint.
    ///
    /// \return True if the joint exists in the kinematic tree.
    ///
    bool existJointName(const std::string & name) const;

    ///
    /// \brief Returns the index of a frame given by its name.
    ///        \sa Model::existFrame to check if the frame exists or not.
    ///
    /// \warning If no frame is found, returns the size of the vector of Model::frames.
    /// This can lead to errors if the model is expanded after this method is called
    /// (for example to get the id of a parent frame).
    ///
    /// \param[in] name Name of the frame.
    /// \param[in] type Type of the frame.
    ///
    /// \return Index of the frame.
    ///
    FrameIndex getFrameId(
      const std::string & name,
      const FrameType & type = (FrameType)(JOINT | FIXED_JOINT | BODY | OP_FRAME | SENSOR)) const;

    ///
    /// \brief Checks if a frame given by its name exists.
    ///
    /// \param[in] name Name of the frame.
    /// \param[in] type Type of the frame.
    ///
    /// \return Returns true if the frame exists.
    ///
    bool existFrame(
      const std::string & name,
      const FrameType & type = (FrameType)(JOINT | FIXED_JOINT | BODY | OP_FRAME | SENSOR)) const;

    ///
    /// \brief Adds a frame to the kinematic tree.
    ///        The inertia stored within the frame will be happened to the inertia supported by the
    ///        joint (frame.parentJoint).
    ///
    /// \param[in] frame The frame to add to the kinematic tree.
    /// \param[in] append_inertia Append the inertia contained in the Frame to the inertia supported
    /// by the joint.
    ///
    /// \return Returns the index of the frame if it has been successfully added or if it already
    /// exists in the kinematic tree.
    ///
    FrameIndex addFrame(const Frame & frame, const bool append_inertia = true);

    ///
    /// \brief Check the validity of the attributes of Model with respect to the specification of
    /// some algorithms.
    ///
    /// The method is a template so that the checkers can be defined in each algorithms.
    /// \param[in] checker a class, typically defined in the algorithm module, that
    /// validates the attributes of model.
    ///
    /// \return true if the Model is valid, false otherwise.
    ///
    template<typename D>
    bool check(const AlgorithmCheckerBase<D> & checker) const
    {
      return checker.checkModel(*this);
    }

    ///
    /// \brief Check if joints have configuration limits
    ///
    /// \return Returns list of boolean of size model.nq.
    ///
    std::vector<bool> hasConfigurationLimit() const;

    ///
    /// \brief Check if joints have configuration limits
    ///
    /// \return Returns list of boolean of size model.nq.
    ///
    std::vector<bool> hasConfigurationLimitInTangent() const;

    /// Run check(fusion::list) with DEFAULT_CHECKERS as argument.
    bool check() const;

    ///
    /// \brief Run checkData on data and current model.
    ///
    /// \param[in] data to be checked wrt *this.
    ///
    /// \return true if the data is valid, false otherwise.
    ///
    bool check(const Data & data) const;

    ///
    /// \brief Create a Data structure associated with the current model
    ///
    Data createData() const;

    /// Returns a vector of the children joints of the kinematic tree.
    /// \remark: a child joint is a node without any child joint.
    std::vector<JointIndex> getChildJoints() const;

  protected:
    ///
    /// \brief Add the joint_id to its parent subtrees.
    ///
    /// \param[in] joint_id The id of the joint to add to the subtrees
    ///
    void addJointIndexToParentSubtrees(const JointIndex joint_id);
  };

} // namespace pinocchio

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */

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
    const VectorXs & min_effort,
    const VectorXs & max_effort,
    const VectorXs & min_velocity,
    const VectorXs & max_velocity,
    const VectorXs & min_config,
    const VectorXs & max_config,
    const VectorXs & min_joint_friction,
    const VectorXs & max_joint_friction,
    const VectorXs & joint_damping)
  {
    const VectorXs config_limit_margin =
      VectorXs::Constant(joint_model.nq(), static_cast<Scalar>(0));
    return addJoint(
      parent, joint_model, joint_placement, joint_name, min_effort, max_effort, min_velocity,
      max_velocity, min_config, max_config, config_limit_margin, min_joint_friction,
      max_joint_friction, joint_damping);
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointIndex
  ModelTpl<Scalar, Options, JointCollectionTpl>::addJoint(
    const JointIndex parent,
    const JointModel & joint_model,
    const SE3 & joint_placement,
    const std::string & joint_name,
    const VectorXs & min_effort,
    const VectorXs & max_effort,
    const VectorXs & min_velocity,
    const VectorXs & max_velocity,
    const VectorXs & min_config,
    const VectorXs & max_config,
    const VectorXs & config_limit_margin,
    const VectorXs & min_joint_friction,
    const VectorXs & max_joint_friction,
    const VectorXs & joint_damping)
  {
    const Scalar inf = std::numeric_limits<Scalar>::infinity();

    const VectorXs min_acceleration = VectorXs::Constant(joint_model.nv(), -inf);
    const VectorXs max_acceleration = VectorXs::Constant(joint_model.nv(), inf);
    const VectorXs min_jerk = VectorXs::Constant(joint_model.nv(), -inf);
    const VectorXs max_jerk = VectorXs::Constant(joint_model.nv(), inf);

    return addJoint(
      parent, joint_model, joint_placement, joint_name, min_effort, max_effort, min_velocity,
      max_velocity, min_config, max_config, config_limit_margin, min_joint_friction,
      max_joint_friction, joint_damping, min_acceleration, max_acceleration, min_jerk, max_jerk);
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointIndex
  ModelTpl<Scalar, Options, JointCollectionTpl>::addJoint(
    const JointIndex parent,
    const JointModel & joint_model,
    const SE3 & joint_placement,
    const std::string & joint_name,
    const VectorXs & min_effort,
    const VectorXs & max_effort,
    const VectorXs & min_velocity,
    const VectorXs & max_velocity,
    const VectorXs & min_config,
    const VectorXs & max_config,
    const VectorXs & config_limit_margin,
    const VectorXs & min_joint_friction,
    const VectorXs & max_joint_friction,
    const VectorXs & joint_damping,
    const VectorXs & min_acceleration,
    const VectorXs & max_acceleration,
    const VectorXs & min_jerk,
    const VectorXs & max_jerk)
  {
    assert(
      (njoints == (int)joints.size()) && (njoints == (int)inertias.size())
      && (njoints == (int)parents.size()) && (njoints == (int)jointPlacements.size()));
    assert((joint_model.nq() >= 0) && (joint_model.nv() >= 0) && (joint_model.nvExtended() >= 0));
    assert(joint_model.nq() >= joint_model.nv());

    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      min_effort.size(), joint_model.nv(), "The joint minimal effort vector is not of right size");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      min_joint_friction.size(), joint_model.nv(),
      "The joint minimal dry friction vector is not of right size");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      min_velocity.size(), joint_model.nv(),
      "The joint minimal velocity vector is not of right size");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      max_effort.size(), joint_model.nv(), "The joint maximum effort vector is not of right size");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      max_joint_friction.size(), joint_model.nv(),
      "The joint maximum dry friction vector is not of right size");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      max_velocity.size(), joint_model.nv(),
      "The joint maximum velocity vector is not of right size");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      compareAll(min_effort, max_effort, internal::ComparisonOperators::LE),
      "Some components of min_effort are greater than max_effort");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      compareAll(min_joint_friction, max_joint_friction, internal::ComparisonOperators::LE),
      "Some components of min_dry_friction are greater than max_dry_friction");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      compareAll(min_velocity, max_velocity, internal::ComparisonOperators::LE),
      "Some components of min_velocity are greater than max_velocity");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      min_config.size(), joint_model.nq(),
      "The joint lower configuration bound is not of right size");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      max_config.size(), joint_model.nq(),
      "The joint upper configuration bound is not of right size");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      config_limit_margin.size(), joint_model.nq(),
      "The joint config limit margin is not of right size");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      joint_damping.size(), joint_model.nv(), "The joint damping vector is not of right size");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      parent < (JointIndex)njoints, "The index of the parent joint is not valid.");

    JointIndex joint_id = (JointIndex)(njoints++);

    joints.push_back(JointModel(joint_model.derived()));
    JointModel & jmodel = joints.back();
    jmodel.setIndexes(joint_id, nq, nv, nvExtended);

    const int joint_nq = jmodel.nq();
    const int joint_idx_q = jmodel.idx_q();
    const int joint_nv = jmodel.nv();
    const int joint_idx_v = jmodel.idx_v();
    const int joint_nvExtended = jmodel.nvExtended();
    const int joint_idx_vExtended = jmodel.idx_vExtended();

    assert(joint_idx_q >= 0);
    assert(joint_idx_v >= 0);
    assert(joint_idx_vExtended >= 0);

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
    nvExtended += joint_nvExtended;
    nvExtendeds.push_back(joint_nvExtended);
    idx_vExtendeds.push_back(joint_idx_vExtended);

    if (joint_nq > 0 && joint_nv > 0)
    {
      upperEffortLimit.conservativeResize(nv);
      jmodel.jointVelocitySelector(upperEffortLimit) = max_effort;
      lowerEffortLimit.conservativeResize(nv);
      jmodel.jointVelocitySelector(lowerEffortLimit) = min_effort;
      upperVelocityLimit.conservativeResize(nv);
      jmodel.jointVelocitySelector(upperVelocityLimit) = max_velocity;
      lowerVelocityLimit.conservativeResize(nv);
      jmodel.jointVelocitySelector(lowerVelocityLimit) = max_velocity;
      lowerPositionLimit.conservativeResize(nq);
      jmodel.jointConfigSelector(lowerPositionLimit) = min_config;
      upperPositionLimit.conservativeResize(nq);
      jmodel.jointConfigSelector(upperPositionLimit) = max_config;
      positionLimitMargin.conservativeResize(nq);
      jmodel.jointConfigSelector(positionLimitMargin) = config_limit_margin;
      lowerAccelerationLimit.conservativeResize(nv);
      jmodel.jointVelocitySelector(lowerAccelerationLimit) = min_acceleration;
      upperAccelerationLimit.conservativeResize(nv);
      jmodel.jointVelocitySelector(upperAccelerationLimit) = max_acceleration;
      lowerJerkLimit.conservativeResize(nv);
      jmodel.jointVelocitySelector(lowerJerkLimit) = min_jerk;
      upperJerkLimit.conservativeResize(nv);
      jmodel.jointVelocitySelector(upperJerkLimit) = max_jerk;

      armature.conservativeResize(nv);
      jmodel.jointVelocitySelector(armature).setZero();
      rotorInertia.conservativeResize(nv);
      jmodel.jointVelocitySelector(rotorInertia).setZero();
      rotorGearRatio.conservativeResize(nv);
      jmodel.jointVelocitySelector(rotorGearRatio).setOnes();
      upperDryFrictionLimit.conservativeResize(nv);
      jmodel.jointVelocitySelector(upperDryFrictionLimit) = max_joint_friction;
      lowerDryFrictionLimit.conservativeResize(nv);
      jmodel.jointVelocitySelector(lowerDryFrictionLimit) = min_joint_friction;
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

    // Resize existing BooleanVectors to the new nv, zero-initializing the new tail entries.
    // conservativeResize alone does not initialize new elements.
    if (joint_nq > 0 && joint_nv > 0)
    {
      for (auto & sparsity : sparsity_pattern_vector)
      {
        const Eigen::Index old_size = sparsity.size();
        sparsity.conservativeResize(nv);
        sparsity.tail(nv - old_size).setZero();
      }
    }

    // Build sparsity pattern and span indexes of the new joint.
    EigenIndexVector extended_support;
    extended_support.reserve(size_t(nv));
    const auto & jsupport = supports[joint_id];
    for (size_t j = 1; j < jsupport.size() - 1; ++j)
    {
      const JointIndex jsupport_id = jsupport[j];
      const int jsupport_nv = nvs[jsupport_id];
      const int jsupport_idx_v = idx_vs[jsupport_id];
      for (int k = 0; k < jsupport_nv; ++k)
        extended_support.push_back(jsupport_idx_v + k);
    }
    for (int k = 0; k < joint_nv; ++k)
    {
      extended_support.push_back(joint_idx_v + k);
    }

    BooleanVector sparsity_pattern = BooleanVector::Zero(nv);
    for (const auto col_id : extended_support)
      sparsity_pattern[col_id] = true;

    sparsity_pattern_vector.push_back(std::move(sparsity_pattern));
    span_indexes_vector.push_back(std::move(extended_support));

    // Update mimicking.
    mimic_joint_supports.push_back(mimic_joint_supports[parent]);
    if (
      const auto & jmodel_ =
        boost::get<JointModelMimicTpl<Scalar, Options, JointCollectionTpl>>(&jmodel))
    {
      mimicking_joints.push_back(jmodel.id());
      mimicked_joints.push_back(jmodel_->jmodel().id());
      mimic_joint_supports[joint_id].push_back(joint_id);
    }
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
    const VectorXs & max_config,
    const VectorXs & config_limit_margin,
    const VectorXs & friction,
    const VectorXs & damping)
  {

    return addJoint(
      parent, joint_model, joint_placement, joint_name, -max_effort, max_effort, -max_velocity,
      max_velocity, min_config, max_config, config_limit_margin, -friction, friction, damping);
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
    const VectorXs & friction,
    const VectorXs & damping)
  {
    const VectorXs config_limit_margin =
      VectorXs::Constant(joint_model.nq(), static_cast<Scalar>(0));

    return addJoint(
      parent, joint_model, joint_placement, joint_name, -max_effort, max_effort, -max_velocity,
      max_velocity, min_config, max_config, config_limit_margin, -friction, friction, damping);
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
    const VectorXs config_limit_margin =
      VectorXs::Constant(joint_model.nq(), static_cast<Scalar>(0));
    const VectorXs friction = VectorXs::Constant(joint_model.nv(), static_cast<Scalar>(0));
    const VectorXs damping = VectorXs::Constant(joint_model.nv(), static_cast<Scalar>(0));

    return addJoint(
      parent, joint_model, joint_placement, joint_name, max_effort, max_velocity, min_config,
      max_config, config_limit_margin, friction, damping);
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
    const VectorXs & config_limit_margin)
  {
    const VectorXs friction = VectorXs::Constant(joint_model.nv(), static_cast<Scalar>(0));
    const VectorXs damping = VectorXs::Constant(joint_model.nv(), static_cast<Scalar>(0));

    return addJoint(
      parent, joint_model, joint_placement, joint_name, max_effort, max_velocity, min_config,
      max_config, config_limit_margin, friction, damping);
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
    const VectorXs config_limit_margin =
      VectorXs::Constant(joint_model.nq(), static_cast<Scalar>(0));

    return addJoint(
      parent, joint_model, joint_placement, joint_name, max_effort, max_velocity, min_config,
      max_config, config_limit_margin);
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  FrameIndex ModelTpl<Scalar, Options, JointCollectionTpl>::addJointFrame(
    const JointIndex joint_index, int previous_frame_index)
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
    res.nvExtended = nvExtended;
    res.njoints = njoints;
    res.nbodies = nbodies;
    res.nframes = nframes;
    res.parents = parents;
    res.children = children;
    res.names = names;
    res.subtrees = subtrees;
    res.supports = supports;
    res.mimic_joint_supports = mimic_joint_supports;
    res.mimicking_joints = mimicking_joints;
    res.mimicked_joints = mimicked_joints;
    res.gravity = gravity.template cast<NewScalar>();
    res.name = name;
    res.sparsity_pattern_vector = sparsity_pattern_vector;
    res.span_indexes_vector = span_indexes_vector;
    res.idx_qs = idx_qs;
    res.nqs = nqs;
    res.idx_vs = idx_vs;
    res.nvs = nvs;
    res.idx_vExtendeds = idx_vExtendeds;
    res.nvExtendeds = nvExtendeds;
    // Eigen Vectors
    res.armature = armature.template cast<NewScalar>();
    res.damping = damping.template cast<NewScalar>();
    res.rotorInertia = rotorInertia.template cast<NewScalar>();
    res.rotorGearRatio = rotorGearRatio.template cast<NewScalar>();
    res.upperEffortLimit = upperEffortLimit.template cast<NewScalar>();
    res.lowerEffortLimit = lowerEffortLimit.template cast<NewScalar>();
    res.upperDryFrictionLimit = upperDryFrictionLimit.template cast<NewScalar>();
    res.lowerDryFrictionLimit = lowerDryFrictionLimit.template cast<NewScalar>();
    res.lowerVelocityLimit = lowerVelocityLimit.template cast<NewScalar>();
    res.upperVelocityLimit = upperVelocityLimit.template cast<NewScalar>();
    res.lowerPositionLimit = lowerPositionLimit.template cast<NewScalar>();
    res.upperPositionLimit = upperPositionLimit.template cast<NewScalar>();
    res.positionLimitMargin = positionLimitMargin.template cast<NewScalar>();
    res.lowerAccelerationLimit = lowerAccelerationLimit.template cast<NewScalar>();
    res.upperAccelerationLimit = upperAccelerationLimit.template cast<NewScalar>();
    res.lowerJerkLimit = lowerJerkLimit.template cast<NewScalar>();
    res.upperJerkLimit = upperJerkLimit.template cast<NewScalar>();

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
  template<template<typename, int> class OtherJointCollectionTpl>
  ModelTpl<Scalar, Options, JointCollectionTpl> &
  ModelTpl<Scalar, Options, JointCollectionTpl>::operator=(
    const ModelTpl<Scalar, Options, OtherJointCollectionTpl> & other)
  {
    this->nq = other.nq;
    this->nv = other.nv;
    this->nvExtended = other.nvExtended;
    this->njoints = other.njoints;
    this->nbodies = other.nbodies;
    this->nframes = other.nframes;
    this->inertias = other.inertias;
    this->jointPlacements = other.jointPlacements;
    this->joints.clear();
    this->joints.reserve(other.joints.size());
    for (const auto & other_joint : other.joints)
    {
      this->joints.push_back(other_joint);
    }
    this->idx_qs = other.idx_qs;
    this->nqs = other.nqs;
    this->idx_vs = other.idx_vs;
    this->nvs = other.nvs;
    this->idx_vExtendeds = other.idx_vExtendeds;
    this->nvExtendeds = other.nvExtendeds;
    this->parents = other.parents;
    this->children = other.children;
    this->names = other.names;
    this->referenceConfigurations = other.referenceConfigurations;
    this->armature = other.armature;
    this->rotorInertia = other.rotorInertia;
    this->rotorGearRatio = other.rotorGearRatio;
    this->lowerDryFrictionLimit = other.lowerDryFrictionLimit;
    this->upperDryFrictionLimit = other.upperDryFrictionLimit;
    this->damping = other.damping;
    this->lowerEffortLimit = other.lowerEffortLimit;
    this->upperEffortLimit = other.upperEffortLimit;
    this->lowerVelocityLimit = other.lowerVelocityLimit;
    this->upperVelocityLimit = other.upperVelocityLimit;
    this->lowerPositionLimit = other.lowerPositionLimit;
    this->upperPositionLimit = other.upperPositionLimit;
    this->positionLimitMargin = other.positionLimitMargin;
    this->lowerAccelerationLimit = other.lowerAccelerationLimit;
    this->upperAccelerationLimit = other.upperAccelerationLimit;
    this->upperJerkLimit = other.upperJerkLimit;
    this->lowerJerkLimit = other.lowerJerkLimit;
    this->frames = other.frames;
    this->supports = other.supports;
    this->subtrees = other.subtrees;
    this->mimic_joint_supports = other.mimic_joint_supports;
    this->mimicking_joints = other.mimicking_joints;
    this->mimicked_joints = other.mimicked_joints;
    this->gravity = other.gravity;
    this->name = other.name;
    this->sparsity_pattern_vector = other.sparsity_pattern_vector;
    this->span_indexes_vector = other.span_indexes_vector;
    return *this;
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  bool ModelTpl<Scalar, Options, JointCollectionTpl>::operator==(const ModelTpl & other) const
  {
    bool res = other.nq == nq && other.nv == nv && other.nvExtended == nvExtended
               && other.njoints == njoints && other.nbodies == nbodies && other.nframes == nframes
               && other.parents == parents && other.children == children && other.names == names
               && other.subtrees == subtrees && other.mimic_joint_supports == mimic_joint_supports
               && other.sparsity_pattern_vector == sparsity_pattern_vector
               && other.span_indexes_vector == span_indexes_vector
               && other.mimicking_joints == mimicking_joints
               && other.mimicked_joints == mimicked_joints && other.gravity == gravity
               && other.name == name;

    res &= other.idx_qs == idx_qs && other.nqs == nqs && other.idx_vs == idx_vs && other.nvs == nvs
           && other.idx_vExtendeds == idx_vExtendeds && other.nvExtendeds == nvExtendeds;

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

    if (other.lowerEffortLimit.size() != lowerEffortLimit.size())
      return false;
    res &= other.lowerEffortLimit == lowerEffortLimit;
    if (!res)
      return res;

    if (other.upperEffortLimit.size() != upperEffortLimit.size())
      return false;
    res &= other.upperEffortLimit == upperEffortLimit;
    if (!res)
      return res;

    if (other.lowerDryFrictionLimit.size() != lowerDryFrictionLimit.size())
      return false;
    res &= other.lowerDryFrictionLimit == lowerDryFrictionLimit;
    if (!res)
      return res;

    if (other.upperDryFrictionLimit.size() != upperDryFrictionLimit.size())
      return false;
    res &= other.upperDryFrictionLimit == upperDryFrictionLimit;
    if (!res)
      return res;

    if (other.lowerVelocityLimit.size() != lowerVelocityLimit.size())
      return false;
    res &= other.lowerVelocityLimit == lowerVelocityLimit;
    if (!res)
      return res;

    if (other.upperVelocityLimit.size() != upperVelocityLimit.size())
      return false;
    res &= other.upperVelocityLimit == upperVelocityLimit;
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

    if (other.positionLimitMargin.size() != positionLimitMargin.size())
      return false;
    res &= other.positionLimitMargin == positionLimitMargin;

    if (other.lowerAccelerationLimit.size() != lowerAccelerationLimit.size())
      return false;
    res &= other.lowerAccelerationLimit == lowerAccelerationLimit;

    if (other.upperAccelerationLimit.size() != upperAccelerationLimit.size())
      return false;
    res &= other.upperAccelerationLimit == upperAccelerationLimit;

    if (other.lowerJerkLimit.size() != lowerJerkLimit.size())
      return false;
    res &= other.lowerJerkLimit == lowerJerkLimit;

    if (other.upperJerkLimit.size() != upperJerkLimit.size())
      return false;
    res &= other.upperJerkLimit == upperJerkLimit;

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
    typename std::vector<Frame>::const_iterator it =
      std::find_if(frames.begin(), frames.end(), details::FilterFrame(name, type));
    std::ostringstream os;
    os << "Several frames match the filter - please specify the FrameType (name=\"" << name
       << "\", type=\"" << type << "\")";
    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      ((it == frames.end()
        || (std::find_if(boost::next(it), frames.end(), details::FilterFrame(name, type)) == frames.end()))),
      os.str().c_str());
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
    if (append_inertia && check_expression_if_real<Scalar>(frame.inertia.mass() > Scalar(0.)))
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
  std::vector<bool> ModelTpl<Scalar, Options, JointCollectionTpl>::hasConfigurationLimit() const
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
  std::vector<bool>
  ModelTpl<Scalar, Options, JointCollectionTpl>::hasConfigurationLimitInTangent() const
  {
    std::vector<bool> vec;
    for (Index i = 1; i < (Index)(njoints); ++i)
    {
      const std::vector<bool> & cf_limits = joints[i].hasConfigurationLimitInTangent();
      vec.insert(vec.end(), cf_limits.begin(), cf_limits.end());
    }
    return vec;
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  std::vector<JointIndex> ModelTpl<Scalar, Options, JointCollectionTpl>::getChildJoints() const
  {
    std::vector<JointIndex> res;
    for (JointIndex joint_id = 1; joint_id < JointIndex(njoints); ++joint_id)
    {
      if (this->children[joint_id].empty())
        res.push_back(joint_id);
    }
    return res;
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  bool ModelTpl<Scalar, Options, JointCollectionTpl>::check() const
  {
    return this->check(makeDefaultCheckerList);
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  bool ModelTpl<Scalar, Options, JointCollectionTpl>::check(const Data & data) const
  {
    return checkData(*this, data);
  }

} // namespace pinocchio

#ifdef PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION

namespace pinocchio
{
  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI
  ModelTpl<context::Scalar, context::Options, JointCollectionDefaultTpl>::ModelTpl();

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI JointIndex
  ModelTpl<context::Scalar, context::Options, JointCollectionDefaultTpl>::addJoint(
    const JointIndex, const JointModel &, const SE3 &, const std::string &);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI JointIndex
  ModelTpl<context::Scalar, context::Options, JointCollectionDefaultTpl>::addJoint(
    const JointIndex,
    const JointModel &,
    const SE3 &,
    const std::string &,
    const context::VectorXs &,
    const context::VectorXs &,
    const context::VectorXs &,
    const context::VectorXs &);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI JointIndex
  ModelTpl<context::Scalar, context::Options, JointCollectionDefaultTpl>::addJoint(
    const JointIndex,
    const JointModel &,
    const SE3 &,
    const std::string &,
    const context::VectorXs &,
    const context::VectorXs &,
    const context::VectorXs &,
    const context::VectorXs &,
    const context::VectorXs &,
    const context::VectorXs &);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI FrameIndex
  ModelTpl<context::Scalar, context::Options, JointCollectionDefaultTpl>::addJointFrame(
    const JointIndex, int);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
  ModelTpl<context::Scalar, context::Options, JointCollectionDefaultTpl>::appendBodyToJoint(
    const JointIndex, const Inertia &, const SE3 &);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI FrameIndex
  ModelTpl<context::Scalar, context::Options, JointCollectionDefaultTpl>::addBodyFrame(
    const std::string &, const JointIndex &, const SE3 &, int);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI FrameIndex
  ModelTpl<context::Scalar, context::Options, JointCollectionDefaultTpl>::addFrame(
    const Frame &, const bool);

} // namespace pinocchio

#endif // ifdef PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
