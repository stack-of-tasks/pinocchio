//
// Copyright (c) 2026 INRIA
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
  // Cast
  // --------------------------------------------------------------
  template<typename NewScalar, typename Scalar, int Options>
  struct CastType<NewScalar, ConstantLengthConstraintModelTpl<Scalar, Options>>
  {
    typedef ConstantLengthConstraintModelTpl<NewScalar, Options> type;
  };

  // --------------------------------------------------------------
  // Traits
  // --------------------------------------------------------------
  template<typename _Scalar, int _Options>
  struct traits<ConstantLengthConstraintModelTpl<_Scalar, _Options>>
  {
    // --------------------------------------------------------------
    // Traits referencing the constraint and associated types
    // --------------------------------------------------------------
    typedef ConstantLengthConstraintModelTpl<_Scalar, _Options> ConstraintModel;
    typedef ConstantLengthConstraintDataTpl<_Scalar, _Options> ConstraintData;

    typedef ConstraintModel Model;
    typedef ConstraintData Data;

    // --------------------------------------------------------------
    // Traits characterizing the constraints
    // --------------------------------------------------------------
    typedef _Scalar Scalar;
    static constexpr int Options = _Options;

    static constexpr ConstraintSizeType constraint_size_type = ConstraintSizeType::STATIC;

    static constexpr bool has_baumgarte_corrector = true;
    static constexpr bool has_set = true;
    static constexpr bool is_inequality_constraint = false;

    // --------------------------------------------------------------
    // Traits for associated struct and sizes
    // --------------------------------------------------------------
    typedef FullSpaceConeTpl<Scalar, Options> ConstraintSet;
    typedef ZeroConeJordanOperationTpl<Scalar, Options> JordanOperation;
    typedef BaumgarteCorrectorParametersTpl<Scalar> BaumgarteCorrectorParameters;

    static constexpr int Size = 1;
    static constexpr int SymmetricConeSize = JordanOperation::ConeSize;
    static constexpr int SymmetricConeScalingSize = JordanOperation::ConeScalingSize;

    // --------------------------------------------------------------
    // Traits that are helper for Eigen types
    // --------------------------------------------------------------
    // \remarks Eigen only accepts a row major storage order for 1 x N matrices with N != 1,
    // hence the explicit Eigen::RowMajor below instead of Options.
    typedef Eigen::Matrix<Scalar, Size, 1, Options> ResidualVectorType;
    typedef Eigen::Matrix<Scalar, Size, Eigen::Dynamic, Eigen::RowMajor> JacobianMatrixType;
    typedef Eigen::Matrix<Scalar, SymmetricConeSize, 1, Options> ConeVectorType;
    typedef Eigen::Matrix<Scalar, SymmetricConeScalingSize, 1, Options> ConeScalingVectorType;

    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1, Options> VectorXs;
    typedef Eigen::Matrix<Scalar, 1, Eigen::Dynamic, Eigen::RowMajor> RowVectorXs;

    // Template to generate type
    template<typename InputMatrix>
    struct JacobianMatrixProductReturnType
    {
      typedef typename InputMatrix::Scalar Scalar;
      typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(InputMatrix) InputMatrixPlain;
      typedef Eigen::Matrix<Scalar, Size, InputMatrixPlain::ColsAtCompileTime, Eigen::RowMajor>
        type;
    };

    template<typename InputMatrix>
    struct JacobianTransposeMatrixProductReturnType
    {
      typedef typename InputMatrix::Scalar Scalar;
      typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(InputMatrix) InputMatrixPlain;
      typedef Eigen::Matrix<
        Scalar,
        Eigen::Dynamic,
        InputMatrixPlain::ColsAtCompileTime,
        InputMatrixPlain::Options>
        type;
    };
  };

  template<typename _Scalar, int _Options>
  struct traits<ConstantLengthConstraintDataTpl<_Scalar, _Options>>
  : traits<ConstantLengthConstraintModelTpl<_Scalar, _Options>>
  {
  };

  ///
  /// \brief Constraint model enforcing a constant distance between two material points.
  ///
  /// The first point is rigidly attached to joint1 (at joint1_placement), the second one to
  /// joint2 (at joint2_placement). Denoting by \f$ x \f$ the position of the second point
  /// expressed in the frame of the first one, the (scalar) constraint residual reads
  /// \f[
  ///   \varphi(q) = \| x(q) \| - L - \delta
  /// \f]
  /// where \f$ L \f$ is the constant length of the constraint and \f$ \delta \f$ is the
  /// generic desired_constraint_offset inherited from the binary kinematics constraints.
  /// The residual is homogeneous to a length (in meters), which makes it directly comparable
  /// to the other position level quantities of the model.
  ///
  /// \remarks The constraint is singular when the two points are coincident (\f$ \|x\| = 0 \f$).
  ///
  template<typename _Scalar, int _Options>
  struct ConstantLengthConstraintModelTpl
  : BinaryKinematicsConstraintModelBase<ConstantLengthConstraintModelTpl<_Scalar, _Options>>
  {
    // --------------------------------------------------------------
    // Type defs
    // --------------------------------------------------------------
    // CRTP related types -------------------------------------------
    typedef ConstantLengthConstraintModelTpl Self;
    typedef BinaryKinematicsConstraintModelBase<Self> Base;
    typedef ConstraintModelCommonParameters<Self> BaseCommonParameters;
    typedef ConstraintModelBase<Self> RootBase;

    // Retrieving traits --------------------------------------------
    typedef typename traits<Self>::ConstraintModel ConstraintModel;
    typedef typename traits<Self>::ConstraintData ConstraintData;

    typedef typename traits<Self>::Scalar Scalar;
    static constexpr int Options = traits<Self>::Options;

    static constexpr ConstraintSizeType constraint_size_type = traits<Self>::constraint_size_type;

    static constexpr bool has_baumgarte_corrector = traits<Self>::has_baumgarte_corrector;

    typedef typename traits<Self>::ConstraintSet ConstraintSet;
    typedef typename traits<Self>::JordanOperation JordanOperation;
    typedef typename traits<Self>::BaumgarteCorrectorParameters BaumgarteCorrectorParameters;

    static constexpr int Size = traits<Self>::Size;
    static constexpr int SymmetricConeSize = traits<Self>::SymmetricConeSize;
    static constexpr int SymmetricConeScalingSize = traits<Self>::SymmetricConeScalingSize;

    typedef typename traits<Self>::ResidualVectorType ResidualVectorType;
    typedef typename traits<Self>::JacobianMatrixType JacobianMatrixType;
    typedef typename traits<Self>::ConeVectorType ConeVectorType;
    typedef typename traits<Self>::ConeScalingVectorType ConeScalingVectorType;

    // Friendship ---------------------------------------------------
    template<typename NewScalar, int NewOptions>
    friend struct ConstantLengthConstraintModelTpl;

    // Base usage ---------------------------------------------------
    using Base::joint1_id;
    using Base::joint2_id;
    using Base::residualSize;
    using RootBase::classname;
    using RootBase::jacobianMatrixProduct;
    using RootBase::jacobianTransposeMatrixProduct;
    using typename Base::Matrix3;
    using typename Base::Matrix36;
    using typename Base::Matrix6;
    using typename Base::MatrixSize6;
    using typename Base::Motion;
    using typename Base::SE3;
    using typename Base::Vector3;

    // Useful types ------------------------------------------------
    typedef Eigen::Matrix<Scalar, Size, Size, Options> MatrixSize;

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

    // Constructors ------------------

    ///
    /// \brief Default constructor. The length is set to zero.
    ///
    ConstantLengthConstraintModelTpl()
    : Base()
    , m_length(Scalar(0))
    {
    }

    ///
    /// \brief Constructor from the model only. The length is set to zero.
    ///
    /// \param[in] model Kinematic tree.
    ///
    template<int OtherOptions, template<typename, int> class JointCollectionTpl>
    explicit ConstantLengthConstraintModelTpl(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model)
    : Base(model)
    , m_length(Scalar(0))
    {
    }

    ///
    /// \brief Constructor from joint indexes, placements and length.
    ///
    /// \param[in] model Model associated to the constraint.
    /// \param[in] joint1_id Index of the joint 1 in the model tree.
    /// \param[in] joint1_placement Placement of the first point w.r.t the frame of joint1.
    /// \param[in] joint2_id Index of the joint 2 in the model tree.
    /// \param[in] joint2_placement Placement of the second point w.r.t the frame of joint2.
    /// \param[in] length Constant distance enforced between the two points.
    ///
    template<int OtherOptions, template<typename, int> class JointCollectionTpl>
    ConstantLengthConstraintModelTpl(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      const JointIndex joint1_id,
      const SE3 & joint1_placement,
      const JointIndex joint2_id,
      const SE3 & joint2_placement,
      const Scalar length)
    : Base(model, joint1_id, joint1_placement, joint2_id, joint2_placement)
    , m_length(length)
    {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(
        check_expression_if_real<Scalar>(length >= Scalar(0)), "The length must be non negative.");
    }

    ///
    /// \brief Constructor from joint indexes and placements. The length is set to zero.
    ///
    /// \param[in] model Model associated to the constraint.
    /// \param[in] joint1_id Index of the joint 1 in the model tree.
    /// \param[in] joint1_placement Placement of the first point w.r.t the frame of joint1.
    /// \param[in] joint2_id Index of the joint 2 in the model tree.
    /// \param[in] joint2_placement Placement of the second point w.r.t the frame of joint2.
    ///
    template<int OtherOptions, template<typename, int> class JointCollectionTpl>
    ConstantLengthConstraintModelTpl(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      const JointIndex joint1_id,
      const SE3 & joint1_placement,
      const JointIndex joint2_id,
      const SE3 & joint2_placement)
    : Base(model, joint1_id, joint1_placement, joint2_id, joint2_placement)
    , m_length(Scalar(0))
    {
    }

    ///
    /// \brief Constructor from joint1_id and placement. The length is set to zero.
    ///
    /// \param[in] model Kinematic tree.
    /// \param[in] joint1_id Index of the joint 1 in the model tree.
    /// \param[in] joint1_placement Placement of the first point w.r.t the frame of joint1.
    ///
    /// \remarks The second joint id (joint2_id) is set to be 0 (the universe).
    ///
    template<int OtherOptions, template<typename, int> class JointCollectionTpl>
    ConstantLengthConstraintModelTpl(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      const JointIndex joint1_id,
      const SE3 & joint1_placement)
    : Base(model, joint1_id, joint1_placement)
    , m_length(Scalar(0))
    {
    }

    ///
    /// \brief Constructor from joint ids. The length is set to zero.
    ///
    /// \param[in] model Kinematic tree.
    /// \param[in] joint1_id Index of the joint 1 in the model tree.
    /// \param[in] joint2_id Index of the joint 2 in the model tree.
    ///
    template<int OtherOptions, template<typename, int> class JointCollectionTpl>
    ConstantLengthConstraintModelTpl(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      const JointIndex joint1_id,
      const JointIndex joint2_id)
    : Base(model, joint1_id, joint2_id)
    , m_length(Scalar(0))
    {
    }

    ///
    /// \brief Constructor from joint1_id. The length is set to zero.
    ///
    /// \param[in] model Kinematic tree.
    /// \param[in] joint1_id Index of the joint 1 in the model tree.
    ///
    /// \remarks The second joint id (joint2_id) is set to be 0 (the universe).
    ///
    template<int OtherOptions, template<typename, int> class JointCollectionTpl>
    ConstantLengthConstraintModelTpl(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model, const JointIndex joint1_id)
    : Base(model, joint1_id)
    , m_length(Scalar(0))
    {
    }

    // Operators ---------------------

    /// \brief Cast operator
    template<typename NewScalar>
    typename CastType<NewScalar, ConstantLengthConstraintModelTpl>::type cast() const
    {
      typedef typename CastType<NewScalar, ConstantLengthConstraintModelTpl>::type ReturnType;
      ReturnType res;
      Base::template cast<NewScalar>(res);
      res.m_length = static_cast<NewScalar>(m_length);
      return res;
    }

    ///
    /// \brief Comparison operator
    ///
    /// \param[in] other Other ConstantLengthConstraintModelTpl to compare with.
    ///
    bool operator==(const ConstantLengthConstraintModelTpl & other) const
    {
      return base() == other.base() && m_length == other.m_length;
    }

    ///
    /// \brief Opposite of the comparison operator.
    ///
    /// \param[in] other Other ConstantLengthConstraintModelTpl to compare with.
    ///
    bool operator!=(const ConstantLengthConstraintModelTpl & other) const
    {
      return !(*this == other);
    }

    // Accessors ---------------------

    /// \brief Returns the constant length enforced by the constraint.
    const Scalar & getLength() const
    {
      return m_length;
    }

    /// \brief Sets the constant length enforced by the constraint.
    void setLength(const Scalar & length)
    {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(
        check_expression_if_real<Scalar>(length >= Scalar(0)), "The length must be non negative.");
      m_length = length;
    }

    // -------------------------------
    // IMPLEMENTATIONS OF BASE METHODS
    // -------------------------------

    // General -----------------------

    /// \copydoc RootBase::classname
    static std::string classnameImpl()
    {
      return std::string("ConstantLengthConstraintModel");
    }

    /// \copydoc RootBase::shortname
    std::string shortnameImpl() const
    {
      return classname();
    }

    /// \copydoc RootBase::createData
    ConstraintData createDataImpl() const
    {
      return ConstraintData(*this);
    }

    // Methods for algorithms --------

    /// \copydoc RootBase::set
    ConstraintSet setImpl(const ConstraintData & cdata) const
    {
      PINOCCHIO_UNUSED_VARIABLE(cdata);
      return ConstraintSet();
    }

    /// \copydoc RootBase::calc
    template<int OtherOptions, template<typename, int> class JointCollectionTpl>
    void calcImpl(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      const DataTpl<Scalar, OtherOptions, JointCollectionTpl> & data,
      ConstraintData & cdata) const
    {
      PINOCCHIO_UNUSED_VARIABLE(model);

      if (this->joint1_id > 0)
        cdata.oMc1 = data.oMi[this->joint1_id] * this->joint1_placement;
      else
        cdata.oMc1 = this->joint1_placement;

      if (this->joint2_id > 0)
        cdata.oMc2 = data.oMi[this->joint2_id] * this->joint2_placement;
      else
        cdata.oMc2 = this->joint2_placement;

      // Compute relative placement
      cdata.c1Mc2 = cdata.oMc1.actInv(cdata.oMc2);
      const Matrix3 & _1R2_ = cdata.c1Mc2.rotation();

      // Position of the second point expressed in the frame of the first one, and its norm.
      cdata.relative_position = cdata.c1Mc2.translation();
      const Vector3 & relative_position = cdata.relative_position;

      cdata.distance = relative_position.norm();
      assert(
        check_expression_if_real<Scalar>(cdata.distance > Scalar(0))
        && "The two points of the constraint are coincident: the constraint is singular.");

      cdata.direction = relative_position / cdata.distance;
      const Vector3 & direction = cdata.direction;

      cdata.constraint_position_error[0] =
        cdata.distance - m_length - this->desired_constraint_offset[0];

      // First order time derivative of relative_position.
      const Motion vf1 = this->joint1_placement.actInv(data.v[this->joint1_id]);
      const Motion vf2 = this->joint2_placement.actInv(data.v[this->joint2_id]);

      const Vector3 relative_velocity_component1 = _1R2_ * vf2.linear() - vf1.linear();
      Vector3 relative_velocity = relative_velocity_component1;
      relative_velocity -= vf1.angular().cross(relative_position);

      // d/dt ||x|| = u . xdot
      const Scalar radial_velocity = direction.dot(relative_velocity);
      cdata.constraint_velocity_error[0] = radial_velocity - this->desired_constraint_velocity[0];

      // Second order time derivative of relative_position.
      const Motion af1 = this->joint1_placement.actInv(data.a[this->joint1_id]);
      const Motion af2 = this->joint2_placement.actInv(data.a[this->joint2_id]);

      Vector3 relative_acceleration = _1R2_ * (af2.linear() + vf2.angular().cross(vf2.linear()))
                                      - (af1.linear() + vf1.angular().cross(vf1.linear()));
      relative_acceleration -= af1.angular().cross(relative_position);
      relative_acceleration += vf1.angular().cross(vf1.angular().cross(relative_position));
      relative_acceleration -= Scalar(2) * vf1.angular().cross(relative_velocity_component1);

      // d^2/dt^2 ||x|| = u . xddot + (||xdot||^2 - (u . xdot)^2) / ||x||
      cdata.constraint_acceleration_error[0] =
        direction.dot(relative_acceleration)
        + (relative_velocity.squaredNorm() - radial_velocity * radial_velocity) / cdata.distance
        - this->desired_constraint_acceleration[0];

      cdata.A1_world = this->getA1(cdata, WorldFrameTag());
      cdata.A2_world = this->getA2(cdata, WorldFrameTag());
      cdata.A_world = cdata.A1_world + cdata.A2_world;

      cdata.A1_local = this->getA1(cdata, LocalFrameTag());
      cdata.A2_local = this->getA2(cdata, LocalFrameTag());
      cdata.A_local = cdata.A1_local + cdata.A2_local;
    }

    /// \copydoc RootBase::jacobian
    template<
      int OtherOptions,
      template<typename, int> class JointCollectionTpl,
      typename JacobianMatrix>
    void jacobianImpl(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      const DataTpl<Scalar, OtherOptions, JointCollectionTpl> & data,
      const ConstraintData & cdata,
      const Eigen::MatrixBase<JacobianMatrix> & _jacobian_matrix) const
    {
      typedef DataTpl<Scalar, OtherOptions, JointCollectionTpl> Data;
      JacobianMatrix & jacobian_matrix = _jacobian_matrix.const_cast_derived();

      const auto & colwise_joint1_sparsity = model.sparsity_pattern_vector[joint1_id];
      const auto & colwise_joint2_sparsity = model.sparsity_pattern_vector[joint2_id];

      const SE3 & oMc1 = cdata.oMc1;
      const SE3 & oMc2 = cdata.oMc2;
      const SE3 & c1Mc2 = cdata.c1Mc2;
      const Vector3 & relative_position = cdata.relative_position;
      const Vector3 & direction = cdata.direction;

      for (Eigen::Index jj = 0; jj < model.nv; ++jj)
      {
        if (colwise_joint1_sparsity[jj] || colwise_joint2_sparsity[jj])
        {
          typedef typename Data::Matrix6x::ConstColXpr ConstColXpr;
          const ConstColXpr Jcol = data.J.col(jj);
          const MotionRef<const ConstColXpr> Jcol_motion(Jcol);

          // Jacobian of relative_position along the jj-th degree of freedom.
          Vector3 relative_position_jacobian_col = Vector3::Zero();
          if (colwise_joint1_sparsity[jj])
          {
            const Motion Jcol_local(oMc1.actInv(Jcol_motion));
            relative_position_jacobian_col -= Jcol_local.linear();
            relative_position_jacobian_col -= Jcol_local.angular().cross(relative_position);
          }

          if (colwise_joint2_sparsity[jj])
          {
            const Motion Jcol_local(oMc2.actInv(Jcol_motion));
            relative_position_jacobian_col += c1Mc2.rotation() * Jcol_local.linear();
          }

          jacobian_matrix(0, jj) = direction.dot(relative_position_jacobian_col);
        }
      }
    }

    /// \copydoc RootBase::jacobianMatrixProduct
    template<
      typename InputMatrix,
      int OtherOptions,
      template<typename, int> class JointCollectionTpl>
    typename traits<Self>::template JacobianMatrixProductReturnType<InputMatrix>::type
    jacobianMatrixProductImpl(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      const DataTpl<Scalar, OtherOptions, JointCollectionTpl> & data,
      const ConstraintData & cdata,
      const Eigen::MatrixBase<InputMatrix> & mat) const
    {
      typedef typename traits<Self>::template JacobianMatrixProductReturnType<InputMatrix>::type
        ReturnType;
      ReturnType res(Size, mat.cols());
      jacobianMatrixProduct(model, data, cdata, mat.derived(), res);
      return res;
    }

    /// \copydoc RootBase::jacobianMatrixProduct
    template<
      typename InputMatrix,
      typename OutputMatrix,
      int OtherOptions,
      template<typename, int> class JointCollectionTpl,
      AssignmentOperatorType op>
    void jacobianMatrixProductImpl(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      const DataTpl<Scalar, OtherOptions, JointCollectionTpl> & data,
      const ConstraintData & cdata,
      const Eigen::MatrixBase<InputMatrix> & mat,
      const Eigen::MatrixBase<OutputMatrix> & _res,
      AssignmentOperatorTag<op> aot) const
    {
      typedef DataTpl<Scalar, OtherOptions, JointCollectionTpl> Data;
      OutputMatrix & res = _res.const_cast_derived();

      PINOCCHIO_CHECK_ARGUMENT_SIZE(mat.rows(), model.nv);
      PINOCCHIO_CHECK_ARGUMENT_SIZE(mat.cols(), res.cols());
      PINOCCHIO_CHECK_ARGUMENT_SIZE(res.rows(), residualSize());
      PINOCCHIO_UNUSED_VARIABLE(aot);

      if constexpr (std::is_same<AssignmentOperatorTag<op>, SetTo>::value)
        res.setZero();

      const MatrixSize6 & A1 = cdata.A1_world;
      const MatrixSize6 & A2 = cdata.A2_world;
      const MatrixSize6 & A = cdata.A_world;

      const auto & colwise_joint1_sparsity = model.sparsity_pattern_vector[joint1_id];
      const auto & colwise_joint2_sparsity = model.sparsity_pattern_vector[joint2_id];

      for (Eigen::Index jj = 0; jj < model.nv; ++jj)
      {
        if (!(colwise_joint1_sparsity[jj] || colwise_joint2_sparsity[jj]))
          continue;

        typedef typename Data::Matrix6x::ConstColXpr ConstColXpr;
        const ConstColXpr Jcol = data.J.col(jj);

        Scalar AxSi;
        if (colwise_joint1_sparsity[jj] && colwise_joint2_sparsity[jj])
          AxSi = A.dot(Jcol);
        else if (colwise_joint1_sparsity[jj])
          AxSi = A1.dot(Jcol);
        else
          AxSi = A2.dot(Jcol);

        if constexpr (std::is_same<AssignmentOperatorTag<op>, RmTo>::value)
          res.noalias() -= AxSi * mat.row(jj);
        else // AddTo, SetTo
          res.noalias() += AxSi * mat.row(jj);
      }
    }

    /// \copydoc RootBase::jacobianTransposeMatrixProduct
    template<
      typename InputMatrix,
      int OtherOptions,
      template<typename, int> class JointCollectionTpl>
    typename traits<Self>::template JacobianTransposeMatrixProductReturnType<InputMatrix>::type
    jacobianTransposeMatrixProductImpl(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      const DataTpl<Scalar, OtherOptions, JointCollectionTpl> & data,
      const ConstraintData & cdata,
      const Eigen::MatrixBase<InputMatrix> & mat) const
    {
      typedef
        typename traits<Self>::template JacobianTransposeMatrixProductReturnType<InputMatrix>::type
          ReturnType;
      ReturnType res(model.nv, mat.cols());
      jacobianTransposeMatrixProduct(model, data, cdata, mat.derived(), res);
      return res;
    }

    /// \copydoc RootBase::jacobianTransposeMatrixProduct
    template<
      typename InputMatrix,
      typename OutputMatrix,
      int OtherOptions,
      template<typename, int> class JointCollectionTpl,
      AssignmentOperatorType op>
    void jacobianTransposeMatrixProductImpl(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      const DataTpl<Scalar, OtherOptions, JointCollectionTpl> & data,
      const ConstraintData & cdata,
      const Eigen::MatrixBase<InputMatrix> & mat,
      const Eigen::MatrixBase<OutputMatrix> & _res,
      AssignmentOperatorTag<op> aot) const
    {
      typedef DataTpl<Scalar, OtherOptions, JointCollectionTpl> Data;
      OutputMatrix & res = _res.const_cast_derived();

      PINOCCHIO_CHECK_ARGUMENT_SIZE(mat.rows(), residualSize());
      PINOCCHIO_CHECK_ARGUMENT_SIZE(res.cols(), mat.cols());
      PINOCCHIO_CHECK_ARGUMENT_SIZE(res.rows(), model.nv);
      PINOCCHIO_UNUSED_VARIABLE(aot);

      if constexpr (std::is_same<AssignmentOperatorTag<op>, SetTo>::value)
        res.setZero();

      const MatrixSize6 & A1 = cdata.A1_world;
      const MatrixSize6 & A2 = cdata.A2_world;
      const MatrixSize6 & A = cdata.A_world;

      const auto & colwise_joint1_sparsity = model.sparsity_pattern_vector[joint1_id];
      const auto & colwise_joint2_sparsity = model.sparsity_pattern_vector[joint2_id];

      for (Eigen::Index jj = 0; jj < model.nv; ++jj)
      {
        if (!(colwise_joint1_sparsity[jj] || colwise_joint2_sparsity[jj]))
          continue;

        typedef typename Data::Matrix6x::ConstColXpr ConstColXpr;
        const ConstColXpr Jcol = data.J.col(jj);

        Scalar AxSi;
        if (colwise_joint1_sparsity[jj] && colwise_joint2_sparsity[jj])
          AxSi = A.dot(Jcol);
        else if (colwise_joint1_sparsity[jj])
          AxSi = A1.dot(Jcol);
        else
          AxSi = A2.dot(Jcol);

        if constexpr (std::is_same<AssignmentOperatorTag<op>, RmTo>::value)
          res.row(jj).noalias() -= AxSi * mat;
        else
          res.row(jj).noalias() += AxSi * mat;
      }
    }

    /// \copydoc RootBase::mapConstraintForceToJointForces
    template<
      int OtherOptions,
      int ForceOptions,
      template<typename, int> class JointCollectionTpl,
      typename ForceLike,
      typename ForceAllocator,
      ReferenceFrame rf>
    void mapConstraintForceToJointForcesImpl(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      const DataTpl<Scalar, OtherOptions, JointCollectionTpl> & data,
      const ConstraintData & cdata,
      const Eigen::MatrixBase<ForceLike> & constraint_forces,
      std::vector<ForceTpl<Scalar, ForceOptions>, ForceAllocator> & joint_forces,
      ReferenceFrameTag<rf> reference_frame) const
    {
      PINOCCHIO_CHECK_ARGUMENT_SIZE(joint_forces.size(), size_t(model.njoints));
      PINOCCHIO_CHECK_ARGUMENT_SIZE(constraint_forces.rows(), residualSize());
      PINOCCHIO_UNUSED_VARIABLE(data);
      PINOCCHIO_UNUSED_VARIABLE(reference_frame);

      const MatrixSize6 & A1 =
        std::is_same<ReferenceFrameTag<rf>, WorldFrameTag>::value ? cdata.A1_world : cdata.A1_local;
      const MatrixSize6 & A2 =
        std::is_same<ReferenceFrameTag<rf>, WorldFrameTag>::value ? cdata.A2_world : cdata.A2_local;

      if (this->joint1_id > 0)
        joint_forces[this->joint1_id].toVector().noalias() +=
          A1.transpose() * constraint_forces.template head<Size>();
      if (this->joint2_id > 0)
        joint_forces[this->joint2_id].toVector().noalias() +=
          A2.transpose() * constraint_forces.template head<Size>();
    }

    /// \copydoc RootBase::mapJointMotionsToConstraintMotion
    template<
      int OtherOptions,
      int MotionOptions,
      template<typename, int> class JointCollectionTpl,
      typename MotionAllocator,
      typename VectorLike,
      ReferenceFrame rf>
    void mapJointMotionsToConstraintMotionImpl(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      const DataTpl<Scalar, OtherOptions, JointCollectionTpl> & data,
      const ConstraintData & cdata,
      const std::vector<MotionTpl<Scalar, MotionOptions>, MotionAllocator> & joint_accelerations,
      const Eigen::MatrixBase<VectorLike> & constraint_motion,
      ReferenceFrameTag<rf> reference_frame) const
    {
      PINOCCHIO_CHECK_ARGUMENT_SIZE(joint_accelerations.size(), size_t(model.njoints));
      PINOCCHIO_CHECK_ARGUMENT_SIZE(constraint_motion.rows(), residualSize());
      PINOCCHIO_UNUSED_VARIABLE(data);
      PINOCCHIO_UNUSED_VARIABLE(reference_frame);

      const MatrixSize6 & A1 =
        std::is_same<ReferenceFrameTag<rf>, WorldFrameTag>::value ? cdata.A1_world : cdata.A1_local;
      const MatrixSize6 & A2 =
        std::is_same<ReferenceFrameTag<rf>, WorldFrameTag>::value ? cdata.A2_world : cdata.A2_local;

      if (this->joint1_id > 0 && this->joint2_id > 0)
        constraint_motion.const_cast_derived().template head<Size>().noalias() =
          A1 * joint_accelerations[this->joint1_id].toVector()
          + A2 * joint_accelerations[this->joint2_id].toVector();
      else if (this->joint1_id > 0)
        constraint_motion.const_cast_derived().template head<Size>().noalias() =
          A1 * joint_accelerations[this->joint1_id].toVector();
      else if (this->joint2_id > 0)
        constraint_motion.const_cast_derived().template head<Size>().noalias() =
          A2 * joint_accelerations[this->joint2_id].toVector();
      else
        constraint_motion.const_cast_derived().setZero();
    }

    ///
    /// \brief This function computes the spatial inertias associated with the constraint.
    /// This function is useful to express the constraint inertia associated with the constraint
    /// for AL-based approaches.
    ///
    template<
      typename Matrix6LikeOut1,
      typename Matrix6LikeOut2,
      typename Matrix6LikeOut3,
      ReferenceFrame rf>
    void computeConstraintInertias(
      const ConstraintData & cdata,
      const Scalar & constraint_inertia_value,
      const Eigen::MatrixBase<Matrix6LikeOut1> & I11,
      const Eigen::MatrixBase<Matrix6LikeOut2> & I12,
      const Eigen::MatrixBase<Matrix6LikeOut3> & I22,
      const ReferenceFrameTag<rf> reference_frame) const
    {
      EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Matrix6LikeOut1, Matrix6);
      EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Matrix6LikeOut2, Matrix6);
      EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Matrix6LikeOut3, Matrix6);
      PINOCCHIO_UNUSED_VARIABLE(reference_frame);

      const MatrixSize6 & A1 =
        std::is_same<ReferenceFrameTag<rf>, WorldFrameTag>::value ? cdata.A1_world : cdata.A1_local;
      const MatrixSize6 & A2 =
        std::is_same<ReferenceFrameTag<rf>, WorldFrameTag>::value ? cdata.A2_world : cdata.A2_local;

      const MatrixSize6 constraint_inertia_time_A1 = constraint_inertia_value * A1;
      const MatrixSize6 constraint_inertia_time_A2 = constraint_inertia_value * A2;

      if (this->joint1_id > 0)
        I11.const_cast_derived().noalias() = A1.transpose() * constraint_inertia_time_A1;
      else
        I11.const_cast_derived().setZero();

      if (this->joint2_id > 0)
        I22.const_cast_derived().noalias() = A2.transpose() * constraint_inertia_time_A2;
      else
        I22.const_cast_derived().setZero();

      // Compute the cross coupling term
      if (this->joint1_id > 0 && this->joint2_id > 0)
        I12.const_cast_derived().noalias() = A1.transpose() * constraint_inertia_time_A2;
      else
        I12.const_cast_derived().setZero();
    }

    /// \copydoc RootBase::appendCouplingConstraintInertias
    template<
      int OtherOptions,
      template<typename, int> class JointCollectionTpl,
      typename VectorLike,
      ReferenceFrame rf>
    void appendCouplingConstraintInertiasImpl(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      DataTpl<Scalar, OtherOptions, JointCollectionTpl> & data,
      const ConstraintData & cdata,
      const Eigen::MatrixBase<VectorLike> & diagonal_constraint_inertia,
      const ReferenceFrameTag<rf> reference_frame) const
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(VectorLike);
      assert(diagonal_constraint_inertia.size() == Size);
      appendConstraintInertias(
        model, data, cdata, diagonal_constraint_inertia.derived()[0], reference_frame);
    }

    /// \copydoc RootBase::appendCouplingConstraintInertias
    template<
      int OtherOptions,
      template<typename, int> class JointCollectionTpl,
      typename MatrixOrMap,
      typename MapEnable,
      ReferenceFrame rf>
    void appendCouplingConstraintInertiasImpl(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      DataTpl<Scalar, OtherOptions, JointCollectionTpl> & data,
      const ConstraintData & cdata,
      const internal::MatrixBlockElementTpl<MatrixOrMap, MapEnable> & constraint_inertia,
      const ReferenceFrameTag<rf> reference_frame) const
    {
      assert(constraint_inertia.size() == Size);
      switch (constraint_inertia.type())
      {
      case internal::MatrixBlockType::Zero: {
        break;
      }
      case internal::MatrixBlockType::Identity: {
        appendConstraintInertias(model, data, cdata, Scalar(1), reference_frame);
        break;
      }
      case internal::MatrixBlockType::ScalarIdentity: {
        appendConstraintInertias(
          model, data, cdata, constraint_inertia.container()(0, 0), reference_frame);
        break;
      }
      case internal::MatrixBlockType::Diagonal: {
        ResidualVectorType cinertia;
        constraint_inertia.diagonal(cinertia);
        appendConstraintInertias(model, data, cdata, cinertia[0], reference_frame);
        break;
      }
      case internal::MatrixBlockType::Plain: {
        MatrixSize cinertia;
        constraint_inertia.matrix(cinertia);
        appendConstraintInertias(model, data, cdata, cinertia(0, 0), reference_frame);
        break;
      }
      default:
        assert(false && "Should never happened");
        PINOCCHIO_THROW_PRETTY(
          std::invalid_argument, "Invalid MatrixBlockType for ConstantLengthConstraintModelTpl.");
      }
    }

    /// \copydoc Base::getA1
    template<ReferenceFrame rf>
    MatrixSize6 getA1Impl(const ConstraintData & cdata, ReferenceFrameTag<rf> rft) const
    {
      MatrixSize6 res;
      res.noalias() = cdata.direction.transpose() * getPointA1(cdata, rft);
      return res;
    }

    /// \copydoc Base::getA2
    template<ReferenceFrame rf>
    MatrixSize6 getA2Impl(const ConstraintData & cdata, ReferenceFrameTag<rf> rft) const
    {
      MatrixSize6 res;
      res.noalias() = cdata.direction.transpose() * getPointA2(cdata, rft);
      return res;
    }

  protected:
    ///
    /// \brief Returns the projector mapping a spatial velocity of joint 1 to the time derivative
    /// of cdata.relative_position. This is the very same quantity as the one returned by
    /// PointConstraintModelBase::getA1 for a point constraint sharing the same joints and
    /// placements.
    ///
    template<ReferenceFrame rf>
    Matrix36 getPointA1(const ConstraintData & cdata, ReferenceFrameTag<rf>) const
    {
      Matrix36 res;

      if constexpr (std::is_same<ReferenceFrameTag<rf>, WorldFrameTag>::value)
      {
        const Matrix3 c1Ro = cdata.oMc1.rotation().transpose();
        res.template leftCols<3>() = -c1Ro;
        res.template rightCols<3>().noalias() = c1Ro * skew(cdata.oMc2.translation());
      }
      else if constexpr (std::is_same<ReferenceFrameTag<rf>, LocalFrameTag>::value)
      {
        const Matrix3 c1Rj1 = this->joint1_placement.rotation().transpose();
        // Position of the second point expressed in the frame of joint 1.
        const Vector3 j1_p_c2 = this->joint1_placement.act(cdata.relative_position);
        res.template leftCols<3>() = -c1Rj1;
        res.template rightCols<3>().noalias() = c1Rj1 * skew(j1_p_c2);
      }
      else
      {
        PINOCCHIO_UNREACHABLE();
      }

      return res;
    }

    ///
    /// \brief Returns the projector mapping a spatial velocity of joint 2 to the time derivative
    /// of cdata.relative_position.
    ///
    template<ReferenceFrame rf>
    Matrix36 getPointA2(const ConstraintData & cdata, ReferenceFrameTag<rf>) const
    {
      Matrix36 res;

      if constexpr (std::is_same<ReferenceFrameTag<rf>, WorldFrameTag>::value)
      {
        const Matrix3 c1Ro = cdata.oMc1.rotation().transpose();
        res.template leftCols<3>() = c1Ro;
        res.template rightCols<3>().noalias() = -c1Ro * skew(cdata.oMc2.translation());
      }
      else if constexpr (std::is_same<ReferenceFrameTag<rf>, LocalFrameTag>::value)
      {
        const Matrix3 c1Rj2 =
          cdata.c1Mc2.rotation() * this->joint2_placement.rotation().transpose();
        res.template leftCols<3>() = c1Rj2;
        res.template rightCols<3>().noalias() = -c1Rj2 * skew(this->joint2_placement.translation());
      }
      else
      {
        PINOCCHIO_UNREACHABLE();
      }

      return res;
    }

    /// \brief Adds the apparent inertia induced by the constraint to the augmented articulated
    /// body inertias and to the joint cross coupling terms stored in data.
    template<int OtherOptions, template<typename, int> class JointCollectionTpl, ReferenceFrame rf>
    void appendConstraintInertias(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      DataTpl<Scalar, OtherOptions, JointCollectionTpl> & data,
      const ConstraintData & cdata,
      const Scalar & constraint_inertia_value,
      const ReferenceFrameTag<rf> reference_frame) const
    {
      PINOCCHIO_UNUSED_VARIABLE(model);
      assert(
        (std::is_same<ReferenceFrameTag<rf>, WorldFrameTag>::value
         || std::is_same<ReferenceFrameTag<rf>, LocalFrameTag>::value)
        && "must never happened");

      Matrix6 I11, I12, I22;
      computeConstraintInertias(cdata, constraint_inertia_value, I11, I12, I22, reference_frame);

      if (this->joint1_id > 0)
        data.oYaba_augmented[this->joint1_id] += I11;

      if (this->joint2_id > 0)
        data.oYaba_augmented[this->joint2_id] += I22;

      if (this->joint1_id > 0 && this->joint2_id > 0)
      {
        assert(
          data.joint_cross_coupling.exists({this->joint1_id, this->joint2_id})
          || data.joint_cross_coupling.exists({this->joint2_id, this->joint1_id}));
        if (data.joint_cross_coupling.exists({this->joint1_id, this->joint2_id}))
        {
          data.joint_cross_coupling.get({this->joint1_id, this->joint2_id}) += I12;
        }
        else
        {
          data.joint_cross_coupling.get({this->joint2_id, this->joint1_id}) += I12.transpose();
        }
      }
    }

    // ------------------------------
    // MEMBERS
    // ------------------------------

    /// \brief Constant distance enforced between the two points of the constraint.
    Scalar m_length;

  }; // struct ConstantLengthConstraintModelTpl

  ///
  /// \brief Data structure associated with ConstantLengthConstraintModelTpl.
  ///
  template<typename _Scalar, int _Options>
  struct ConstantLengthConstraintDataTpl
  : ConstraintDataBase<ConstantLengthConstraintDataTpl<_Scalar, _Options>>
  {
    // --------------------------------------------------------------
    // Type defs
    // --------------------------------------------------------------
    // CRTP related types -------------------------------------------
    typedef ConstantLengthConstraintDataTpl Self;
    typedef ConstraintDataBase<Self> Base;

    // Retrieving traits --------------------------------------------
    typedef typename traits<Self>::ConstraintModel ConstraintModel;
    typedef typename traits<Self>::ConstraintData ConstraintData;

    typedef typename traits<Self>::Scalar Scalar;
    static constexpr int Options = traits<Self>::Options;

    static constexpr int Size = traits<Self>::Size;

    typedef typename traits<Self>::ResidualVectorType ResidualVectorType;

    // Useful types ------------------------------------------------
    typedef SE3Tpl<Scalar, Options> SE3;
    typedef Eigen::Matrix<Scalar, 3, 1, Options> Vector3;
    typedef Eigen::Matrix<Scalar, Size, 6, Eigen::RowMajor> MatrixSize6;

    // Base usage ---------------------------------------------------
    using Base::classname;

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

    // Constructors ------------------

    /// \brief Default constructor
    ConstantLengthConstraintDataTpl()
    : constraint_force(ResidualVectorType::Zero())
    , oMc1(SE3::Identity())
    , oMc2(SE3::Identity())
    , c1Mc2(SE3::Identity())
    , relative_position(Vector3::Zero())
    , distance(Scalar(0))
    , direction(Vector3::Zero())
    , constraint_position_error(ResidualVectorType::Zero())
    , constraint_velocity_error(ResidualVectorType::Zero())
    , constraint_acceleration_error(ResidualVectorType::Zero())
    , constraint_acceleration_biais_term(ResidualVectorType::Zero())
    , A1_world(MatrixSize6::Zero())
    , A2_world(MatrixSize6::Zero())
    , A_world(MatrixSize6::Zero())
    , A1_local(MatrixSize6::Zero())
    , A2_local(MatrixSize6::Zero())
    , A_local(MatrixSize6::Zero())
    {
    }

    /// \brief Constructor from a given ConstraintModel
    ///
    /// \param[in] cmodel input constraint model
    ///
    explicit ConstantLengthConstraintDataTpl(const ConstraintModel & cmodel)
    : ConstantLengthConstraintDataTpl()
    {
      PINOCCHIO_UNUSED_VARIABLE(cmodel);
    }

    // Operators ---------------------

    /// \brief Comparison operator
    bool operator==(const ConstantLengthConstraintDataTpl & other) const
    {
      return constraint_force == other.constraint_force && oMc1 == other.oMc1 && oMc2 == other.oMc2
             && c1Mc2 == other.c1Mc2 && relative_position == other.relative_position
             && distance == other.distance && direction == other.direction
             && constraint_position_error == other.constraint_position_error
             && constraint_velocity_error == other.constraint_velocity_error
             && constraint_acceleration_error == other.constraint_acceleration_error
             && constraint_acceleration_biais_term == other.constraint_acceleration_biais_term
             && A1_world == other.A1_world && A2_world == other.A2_world && A_world == other.A_world
             && A1_local == other.A1_local && A2_local == other.A2_local
             && A_local == other.A_local;
    }

    /// \brief Comparison operator
    bool operator!=(const ConstantLengthConstraintDataTpl & other) const
    {
      return !(*this == other);
    }

    // -------------------------------
    // IMPLEMENTATIONS OF BASE METHODS
    // -------------------------------

    /// \copydoc Base::classname
    static std::string classnameImpl()
    {
      return std::string("ConstantLengthConstraintData");
    }

    /// \copydoc Base::shortname
    std::string shortnameImpl() const
    {
      return classname();
    }

    // ------------------------------
    // MEMBERS
    // ------------------------------
    // note: data is always public - use at your own risk

    /// \brief Resulting constraint force, aligned with cdata.direction.
    ResidualVectorType constraint_force;

    /// \brief Placement of the constraint frame 1 with respect to the WORLD frame
    SE3 oMc1;

    /// \brief Placement of the constraint frame 2 with respect to the WORLD frame
    SE3 oMc2;

    /// \brief Relative displacement between the two frames
    SE3 c1Mc2;

    /// \brief Position of the second point expressed in the frame of the first one
    Vector3 relative_position;

    /// \brief Norm of relative_position, i.e. the current distance between the two points
    Scalar distance;

    /// \brief Unitary direction relative_position / distance
    Vector3 direction;

    /// \brief Constraint position error
    ResidualVectorType constraint_position_error;

    /// \brief Constraint velocity error
    ResidualVectorType constraint_velocity_error;

    /// \brief Constraint acceleration error
    ResidualVectorType constraint_acceleration_error;

    /// \brief Constraint acceleration biais
    ResidualVectorType constraint_acceleration_biais_term;

    MatrixSize6 A1_world;
    MatrixSize6 A2_world;
    MatrixSize6 A_world; // A1 + A2

    MatrixSize6 A1_local;
    MatrixSize6 A2_local;
    MatrixSize6 A_local; // A1 + A2
  }; // struct ConstantLengthConstraintDataTpl

} // namespace pinocchio
