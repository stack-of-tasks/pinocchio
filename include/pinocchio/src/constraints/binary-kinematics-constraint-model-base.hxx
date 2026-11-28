//
// Copyright (c) 2025 INRIA
//

#pragma once

// IWYU pragma: private, include "pinocchio/constraints.hpp"

#ifdef PINOCCHIO_LSP
  #undef PINOCCHIO_LSP
  #include "pinocchio/constraints.hpp"
#endif // PINOCCHIO_LSP

namespace pinocchio
{
  // --------------------------------------------------------------
  // Declaration
  // --------------------------------------------------------------
  template<typename Derived>
  struct BinaryKinematicsConstraintModelBase;

  // --------------------------------------------------------------
  // Helpers
  // --------------------------------------------------------------
  template<typename Derived>
  using enable_if_binarykinematic_model_t =
    std::enable_if_t<std::is_base_of_v<BinaryKinematicsConstraintModelBase<Derived>, Derived>>;

  // --------------------------------------------------------------
  // Struct
  // --------------------------------------------------------------
  template<typename Derived>
  struct BinaryKinematicsConstraintModelBase
  : KinematicsConstraintModelBase<Derived>
  , ConstraintModelCommonParameters<Derived>
  {
    // --------------------------------------------------------------
    // Type defs
    // --------------------------------------------------------------
    // CRTP related types -------------------------------------------
    typedef KinematicsConstraintModelBase<Derived> Base;
    typedef ConstraintModelCommonParameters<Derived> BaseCommonParameters;
    typedef ConstraintModelBase<Derived> RootBase;

    // Retrieving traits --------------------------------------------
    typedef typename traits<Derived>::ConstraintModel ConstraintModel;
    typedef typename traits<Derived>::ConstraintData ConstraintData;

    typedef typename traits<Derived>::Scalar Scalar;
    static constexpr int Options = traits<Derived>::Options;

    static constexpr int Size = traits<Derived>::Size;

    typedef typename traits<Derived>::ResidualVectorType ResidualVectorType;
    typedef typename traits<Derived>::JacobianMatrixType JacobianMatrixType;
    typedef typename traits<Derived>::BaumgarteCorrectorParameters BaumgarteCorrectorParameters;

    // Friendship ---------------------------------------------------
    template<typename OtherDerived>
    friend struct BinaryKinematicsConstraintModelBase;

    // Base usage --------------------------------------------------
    using RootBase::derived;
    using RootBase::residualSize;
    using typename RootBase::BooleanVector;
    using typename RootBase::EigenIndexVector;

    // Useful types ------------------------------------------------
    typedef SE3Tpl<Scalar, Options> SE3;
    typedef MotionTpl<Scalar, Options> Motion;
    typedef ForceTpl<Scalar, Options> Force;
    typedef Eigen::Matrix<Scalar, 3, 1, Options> Vector3;
    typedef Eigen::Matrix<Scalar, 6, 1, Options> Vector6;
    typedef Eigen::Matrix<Scalar, 3, 3, Options> Matrix3;
    typedef Eigen::Matrix<Scalar, 6, 6, Options> Matrix6;
    typedef Eigen::Matrix<Scalar, 3, 6, Options> Matrix36;
    /// \remarks Eigen only accepts a row major storage order for 1 x N matrices with N != 1, hence
    /// the storage order of MatrixSize6 depends on Size.
    typedef Eigen::
      Matrix<Scalar, Size, 6, (Size == 1) ? int(Options) | int(Eigen::RowMajor) : int(Options)>
        MatrixSize6;

    // -------------------------------
    // METHODS SPECIFIC TO CLASS
    // -------------------------------

    // CRTP related ------------------

    /// \brief Cast to Base
    Base & base()
    {
      return static_cast<Base &>(*this);
    }

    /// \brief Const cast to Base
    const Base & base() const
    {
      return static_cast<const Base &>(*this);
    }

    /// \brief Cast to BaseCommonParameters
    BaseCommonParameters & base_common_parameters()
    {
      return static_cast<BaseCommonParameters &>(*this);
    }

    /// \brief Const cast to BaseCommonParameters
    const BaseCommonParameters & base_common_parameters() const
    {
      return static_cast<const BaseCommonParameters &>(*this);
    }

    // Constructors ------------------

  protected:
    /// \brief Default constructor
    BinaryKinematicsConstraintModelBase()
    : joint1_id(0)
    , joint2_id(0)
    , joint1_placement(SE3::Identity())
    , joint2_placement(SE3::Identity())
    , desired_constraint_offset(ResidualVectorType::Zero(residualSize()))
    , desired_constraint_velocity(ResidualVectorType::Zero(residualSize()))
    , desired_constraint_acceleration(ResidualVectorType::Zero(residualSize()))
    , nv(-1)
    , depth_joint1(0)
    , depth_joint2(0)
    {
      m_compliance = ResidualVectorType::Zero(residualSize());
      m_baumgarte_parameters = BaumgarteCorrectorParameters();
    }

    /// \brief Full constructor
    template<int OtherOptions, template<typename, int> class JointCollectionTpl>
    BinaryKinematicsConstraintModelBase(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      const JointIndex joint1_id,
      const SE3 & joint1_placement,
      const JointIndex joint2_id,
      const SE3 & joint2_placement)
    : Base(model)
    , joint1_id(joint1_id)
    , joint2_id(joint2_id)
    , joint1_placement(joint1_placement)
    , joint2_placement(joint2_placement)
    , desired_constraint_offset(ResidualVectorType::Zero(residualSize()))
    , desired_constraint_velocity(ResidualVectorType::Zero(residualSize()))
    , desired_constraint_acceleration(ResidualVectorType::Zero(residualSize()))
    , nv(model.nv)
    , depth_joint1(static_cast<size_t>(model.supports[joint1_id].size()))
    , depth_joint2(static_cast<size_t>(model.supports[joint2_id].size()))
    {
      m_compliance = ResidualVectorType::Zero(residualSize());
      m_baumgarte_parameters = BaumgarteCorrectorParameters();
    }

    /// \brief Constructor with only model.
    /// Relative placements are identity and joints are 0.
    template<int OtherOptions, template<typename, int> class JointCollectionTpl>
    BinaryKinematicsConstraintModelBase(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model)
    : BinaryKinematicsConstraintModelBase(model, 0, SE3::Identity(), 0, SE3::Identity())
    {
    }

    /// \brief Constructor with only joint1, relative placement is identity.
    /// joint2 defaults to 0 with identity relative placement.
    template<int OtherOptions, template<typename, int> class JointCollectionTpl>
    BinaryKinematicsConstraintModelBase(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model, const JointIndex joint1_id)
    : BinaryKinematicsConstraintModelBase(model, joint1_id, SE3::Identity(), 0, SE3::Identity())
    {
    }

    /// \brief Constructor with only joint1 and relative placement to joint1.
    /// joint2 defaults to 0 with identity relative placement.
    template<int OtherOptions, template<typename, int> class JointCollectionTpl>
    BinaryKinematicsConstraintModelBase(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      const JointIndex joint1_id,
      const SE3 & joint1_placement)
    : BinaryKinematicsConstraintModelBase(model, joint1_id, joint1_placement, 0, SE3::Identity())
    {
    }

    /// \brief Constructor with only joint1 and joint2 ids.
    /// Relative placements are identity.
    template<int OtherOptions, template<typename, int> class JointCollectionTpl>
    BinaryKinematicsConstraintModelBase(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      const JointIndex joint1_id,
      const JointIndex joint2_id)
    : BinaryKinematicsConstraintModelBase(
        model, joint1_id, SE3::Identity(), joint2_id, SE3::Identity())
    {
    }

    // Operators ---------------------

  public:
    /// \brief Cast to NewScalar.
    template<typename NewScalar, typename OtherDerived>
    void cast(BinaryKinematicsConstraintModelBase<OtherDerived> & res) const
    {
      Base::cast(res);
      BaseCommonParameters::template cast<NewScalar>(res);

      res.joint1_id = joint1_id;
      res.joint2_id = joint2_id;
      res.joint1_placement = joint1_placement.template cast<NewScalar>();
      res.joint2_placement = joint2_placement.template cast<NewScalar>();
      res.desired_constraint_offset = desired_constraint_offset.template cast<NewScalar>();
      res.desired_constraint_velocity = desired_constraint_velocity.template cast<NewScalar>();
      res.desired_constraint_acceleration =
        desired_constraint_acceleration.template cast<NewScalar>();
      res.nv = nv;
      res.depth_joint1 = depth_joint1;
      res.depth_joint2 = depth_joint2;
    }

    /// \brief Comparison operator.
    template<typename OtherDerived>
    bool operator==(const BinaryKinematicsConstraintModelBase<OtherDerived> & other) const
    {
      if (this == &other)
        return true;

      return base() == other.base() && base_common_parameters() == other.base_common_parameters()
             && joint1_id == other.joint1_id && joint2_id == other.joint2_id
             && joint1_placement == other.joint1_placement
             && joint2_placement == other.joint2_placement
             && desired_constraint_offset == other.desired_constraint_offset
             && desired_constraint_velocity == other.desired_constraint_velocity
             && desired_constraint_acceleration == other.desired_constraint_acceleration
             && depth_joint1 == other.depth_joint1 && depth_joint2 == other.depth_joint2;
    }

    /// \brief Comparison operator.
    template<typename OtherDerived>
    bool operator!=(const BinaryKinematicsConstraintModelBase<OtherDerived> & other) const
    {
      return !(*this == other);
    }

    // Binary related ----------------

    /// \brief Returns the constraint projector associated with joint 1.
    /// This matrix transforms a spatial velocity expressed at the origin to the first component of
    /// the constraint associated with joint 1.
    template<ReferenceFrame rf>
    MatrixSize6 getA1(const ConstraintData & cdata, ReferenceFrameTag<rf> rft) const
    {
      return derived().getA1Impl(cdata, rft);
    }

    /// \brief Returns the constraint projector associated with joint 2.
    /// This matrix transforms a spatial velocity expressed at the origin to the first component of
    /// the constraint associated with joint 2.
    template<ReferenceFrame rf>
    MatrixSize6 getA2(const ConstraintData & cdata, ReferenceFrameTag<rf> rft) const
    {
      return derived().getA2Impl(cdata, rft);
    }

    // -------------------------------
    // IMPLEMENTATIONS OF BASE METHODS
    // -------------------------------

    /// \copydoc Base::getRowSparsityPattern
    template<int OtherOptions, template<typename, int> class JointCollectionTpl>
    void getRowSparsityPatternImpl(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      const DataTpl<Scalar, OtherOptions, JointCollectionTpl> & data,
      const ConstraintData & cdata,
      const Eigen::Index row_id,
      BooleanVector & result) const
    {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(row_id < residualSize());
      PINOCCHIO_UNUSED_VARIABLE(data);
      PINOCCHIO_UNUSED_VARIABLE(cdata);
      const auto & sparsity_pattern_1 = model.sparsity_pattern_vector[joint1_id];
      const auto & sparsity_pattern_2 = model.sparsity_pattern_vector[joint2_id];
      result = sparsity_pattern_1.binaryExpr(
        sparsity_pattern_2, [](bool a, bool b) -> bool { return a || b; });
    }

    /// \copydoc Base::getRowIndexes
    template<int OtherOptions, template<typename, int> class JointCollectionTpl>
    void getRowIndexesImpl(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      const DataTpl<Scalar, OtherOptions, JointCollectionTpl> & data,
      const ConstraintData & cdata,
      const Eigen::Index row_id,
      EigenIndexVector & result) const
    {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(row_id < residualSize());
      PINOCCHIO_UNUSED_VARIABLE(data);
      PINOCCHIO_UNUSED_VARIABLE(cdata);
      const auto & span_indexes_1 = model.span_indexes_vector[joint1_id];
      const auto & span_indexes_2 = model.span_indexes_vector[joint2_id];
      result.clear();
      result.reserve(static_cast<std::size_t>(model.nv));
      std::set_union(
        span_indexes_1.begin(), span_indexes_1.end(), span_indexes_2.begin(), span_indexes_2.end(),
        std::back_inserter(result));
    }

    // ------------------------------
    // MEMBERS
    // ------------------------------

    /// \brief Index of the first joint in the model tree
    JointIndex joint1_id;

    /// \brief Index of the second joint in the model tree
    JointIndex joint2_id;

    /// \brief Position of attached point with respect to the frame of joint1.
    SE3 joint1_placement;

    /// \brief Position of attached point with respect to the frame of joint2.
    SE3 joint2_placement;

    /// \brief Desired constraint shift at position level.
    ResidualVectorType desired_constraint_offset;

    /// \brief Desired constraint velocity at velocity level.
    ResidualVectorType desired_constraint_velocity;

    /// \brief Desired constraint acceleration at acceleration level.
    ResidualVectorType desired_constraint_acceleration;

    /// \brief Dimensions of the model.
    int nv;

    /// \brief Depth of the kinematic tree for joint1 and joint2.
    size_t depth_joint1, depth_joint2;

  protected:
    using BaseCommonParameters::m_baumgarte_parameters;
    using BaseCommonParameters::m_compliance;
  }; // struct BinaryKinematicsConstraintModelBase

} // namespace pinocchio
