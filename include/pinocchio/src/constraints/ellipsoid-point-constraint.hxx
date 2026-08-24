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
  struct CastType<NewScalar, EllipsoidPointConstraintModelTpl<Scalar, Options>>
  {
    typedef EllipsoidPointConstraintModelTpl<NewScalar, Options> type;
  };

  // --------------------------------------------------------------
  // Traits
  // --------------------------------------------------------------
  template<typename _Scalar, int _Options>
  struct traits<EllipsoidPointConstraintModelTpl<_Scalar, _Options>>
  {
    // --------------------------------------------------------------
    // Traits referencing the constraint and associated types
    // --------------------------------------------------------------
    typedef EllipsoidPointConstraintModelTpl<_Scalar, _Options> ConstraintModel;
    typedef EllipsoidPointConstraintDataTpl<_Scalar, _Options> ConstraintData;

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
  struct traits<EllipsoidPointConstraintDataTpl<_Scalar, _Options>>
  : traits<EllipsoidPointConstraintModelTpl<_Scalar, _Options>>
  {
  };

  ///
  /// \brief Constraint model keeping a material point on the surface of an ellipsoid.
  ///
  /// The ellipsoid is carried by joint 1: joint1_placement gives the pose of its principal axes
  /// in the frame of that joint, and the radii are the half-axes along those axes. Joint 1 may be
  /// the universe. The material point is carried by joint 2, at joint2_placement.
  ///
  /// Denoting by \f$ x \f$ the position of the point expressed in the frame of the ellipsoid and
  /// by \f$ A = \mathrm{diag}(1/a_i^2) \f$, the surface is \f$ x^T A x = 1 \f$ and the constraint
  /// residual reads
  /// \f[
  ///   \varphi(q) = \frac{x^T A x - 1}{2 \| A x \|} - \delta
  /// \f]
  /// where \f$ \delta \f$ is the generic desired_constraint_offset inherited from the binary
  /// kinematics constraints.
  ///
  /// The division by \f$ 2 \| A x \| \f$ makes the residual homogeneous to a length: it is the
  /// signed distance to the surface to first order, positive outside, negative inside, and
  /// exactly zero on the surface. The raw algebraic residual \f$ x^T A x - 1 \f$ would be
  /// dimensionless and badly scaled -- on a 83 x 200 x 83 mm ellipsoid a 5 mm violation gives
  /// 0.11, some twenty times the metric residual -- which matters as soon as the residual is
  /// compared with, or summed against, other length-valued quantities.
  ///
  /// \remarks The constraint is singular at the centre of the ellipsoid only, where
  /// \f$ \| A x \| = 0 \f$.
  ///
  template<typename _Scalar, int _Options>
  struct EllipsoidPointConstraintModelTpl
  : BinaryKinematicsConstraintModelBase<EllipsoidPointConstraintModelTpl<_Scalar, _Options>>
  {
    // --------------------------------------------------------------
    // Type defs
    // --------------------------------------------------------------
    // CRTP related types -------------------------------------------
    typedef EllipsoidPointConstraintModelTpl Self;
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
    friend struct EllipsoidPointConstraintModelTpl;

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
    /// \brief Default constructor. The ellipsoid is the unit sphere.
    ///
    EllipsoidPointConstraintModelTpl()
    : Base()
    , m_radii(Vector3::Ones())
    {
    }

    ///
    /// \brief Constructor from the model only. The ellipsoid is the unit sphere.
    ///
    /// \param[in] model Kinematic tree.
    ///
    template<int OtherOptions, template<typename, int> class JointCollectionTpl>
    explicit EllipsoidPointConstraintModelTpl(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model)
    : Base(model)
    , m_radii(Vector3::Ones())
    {
    }

    ///
    /// \brief Constructor from the ellipsoid and the material point.
    ///
    /// \param[in] model Model associated to the constraint.
    /// \param[in] joint1_id Index of the joint carrying the ellipsoid (0 for the universe).
    /// \param[in] joint1_placement Pose of the principal axes of the ellipsoid w.r.t joint1.
    /// \param[in] joint2_id Index of the joint carrying the material point.
    /// \param[in] joint2_placement Placement of the material point w.r.t the frame of joint2.
    /// \param[in] radii Half-axes of the ellipsoid, along the axes of joint1_placement.
    ///
    template<
      int OtherOptions,
      template<typename, int> class JointCollectionTpl,
      typename Vector3Like>
    EllipsoidPointConstraintModelTpl(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      const JointIndex joint1_id,
      const SE3 & joint1_placement,
      const JointIndex joint2_id,
      const SE3 & joint2_placement,
      const Eigen::MatrixBase<Vector3Like> & radii)
    : Base(model, joint1_id, joint1_placement, joint2_id, joint2_placement)
    , m_radii(radii)
    {
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(Vector3Like, Vector3);
      checkRadii(m_radii);
    }

    ///
    /// \brief Constructor from joint indexes and placements. The ellipsoid is the unit sphere.
    ///
    template<int OtherOptions, template<typename, int> class JointCollectionTpl>
    EllipsoidPointConstraintModelTpl(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      const JointIndex joint1_id,
      const SE3 & joint1_placement,
      const JointIndex joint2_id,
      const SE3 & joint2_placement)
    : Base(model, joint1_id, joint1_placement, joint2_id, joint2_placement)
    , m_radii(Vector3::Ones())
    {
    }

    ///
    /// \brief Constructor from joint1_id and placement. The ellipsoid is the unit sphere.
    ///
    /// \remarks The second joint id (joint2_id) is set to be 0 (the universe).
    ///
    template<int OtherOptions, template<typename, int> class JointCollectionTpl>
    EllipsoidPointConstraintModelTpl(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      const JointIndex joint1_id,
      const SE3 & joint1_placement)
    : Base(model, joint1_id, joint1_placement)
    , m_radii(Vector3::Ones())
    {
    }

    ///
    /// \brief Constructor from joint ids. The ellipsoid is the unit sphere.
    ///
    template<int OtherOptions, template<typename, int> class JointCollectionTpl>
    EllipsoidPointConstraintModelTpl(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      const JointIndex joint1_id,
      const JointIndex joint2_id)
    : Base(model, joint1_id, joint2_id)
    , m_radii(Vector3::Ones())
    {
    }

    ///
    /// \brief Constructor from joint1_id. The ellipsoid is the unit sphere.
    ///
    /// \remarks The second joint id (joint2_id) is set to be 0 (the universe).
    ///
    template<int OtherOptions, template<typename, int> class JointCollectionTpl>
    EllipsoidPointConstraintModelTpl(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model, const JointIndex joint1_id)
    : Base(model, joint1_id)
    , m_radii(Vector3::Ones())
    {
    }

    // Operators ---------------------

    /// \brief Cast operator
    template<typename NewScalar>
    typename CastType<NewScalar, EllipsoidPointConstraintModelTpl>::type cast() const
    {
      typedef typename CastType<NewScalar, EllipsoidPointConstraintModelTpl>::type ReturnType;
      ReturnType res;
      Base::template cast<NewScalar>(res);
      res.m_radii = m_radii.template cast<NewScalar>();
      return res;
    }

    ///
    /// \brief Comparison operator
    ///
    bool operator==(const EllipsoidPointConstraintModelTpl & other) const
    {
      return base() == other.base() && m_radii == other.m_radii;
    }

    ///
    /// \brief Opposite of the comparison operator.
    ///
    bool operator!=(const EllipsoidPointConstraintModelTpl & other) const
    {
      return !(*this == other);
    }

    // Accessors ---------------------

    /// \brief Returns the half-axes of the ellipsoid.
    const Vector3 & getRadii() const
    {
      return m_radii;
    }

    /// \brief Sets the half-axes of the ellipsoid.
    template<typename Vector3Like>
    void setRadii(const Eigen::MatrixBase<Vector3Like> & radii)
    {
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(Vector3Like, Vector3);
      checkRadii(radii.derived());
      m_radii = radii;
    }

    // -------------------------------
    // IMPLEMENTATIONS OF BASE METHODS
    // -------------------------------

    // General -----------------------

    /// \copydoc RootBase::classname
    static std::string classnameImpl()
    {
      return std::string("EllipsoidPointConstraintModel");
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

      // Position of the material point expressed in the frame of the ellipsoid.
      cdata.relative_position = cdata.c1Mc2.translation();
      const Vector3 & x = cdata.relative_position;

      // A = diag(1/a_i^2), so Ax = x / a^2 and A2x = A (A x).
      const Vector3 inv_radii_squared = m_radii.cwiseProduct(m_radii).cwiseInverse();
      const Vector3 Ax = x.cwiseProduct(inv_radii_squared);
      const Vector3 A2x = Ax.cwiseProduct(inv_radii_squared);

      cdata.norm_Ax = Ax.norm();
      assert(
        check_expression_if_real<Scalar>(cdata.norm_Ax > Scalar(0))
        && "The point is at the centre of the ellipsoid: the constraint is singular.");

      cdata.algebraic_error = x.dot(Ax) - Scalar(1);

      const Scalar & s = cdata.algebraic_error;
      const Scalar & n = cdata.norm_Ax;
      const Scalar n2 = n * n, n3 = n2 * n, n5 = n3 * n2;

      // grad phi = A x / ||A x||  -  phi_alg * A^2 x / (2 ||A x||^3)
      cdata.gradient = Ax / n - (s / (Scalar(2) * n3)) * A2x;
      const Vector3 & gradient = cdata.gradient;

      cdata.constraint_position_error[0] = s / (Scalar(2) * n) - this->desired_constraint_offset[0];

      // First order time derivative of the relative position.
      const Motion vf1 = this->joint1_placement.actInv(data.v[this->joint1_id]);
      const Motion vf2 = this->joint2_placement.actInv(data.v[this->joint2_id]);

      const Vector3 relative_velocity_component1 = _1R2_ * vf2.linear() - vf1.linear();
      Vector3 relative_velocity = relative_velocity_component1;
      relative_velocity -= vf1.angular().cross(x);

      cdata.constraint_velocity_error[0] =
        gradient.dot(relative_velocity) - this->desired_constraint_velocity[0];

      // Second order time derivative of the relative position.
      const Motion af1 = this->joint1_placement.actInv(data.a[this->joint1_id]);
      const Motion af2 = this->joint2_placement.actInv(data.a[this->joint2_id]);

      Vector3 relative_acceleration = _1R2_ * (af2.linear() + vf2.angular().cross(vf2.linear()))
                                      - (af1.linear() + vf1.angular().cross(vf1.linear()));
      relative_acceleration -= af1.angular().cross(x);
      relative_acceleration += vf1.angular().cross(vf1.angular().cross(x));
      relative_acceleration -= Scalar(2) * vf1.angular().cross(relative_velocity_component1);

      // d^2/dt^2 phi = grad phi . xddot + xdot^T (hessian phi) xdot, the quadratic form being
      //   xd^T A xd / n  -  2 (Ax.xd)(A2x.xd) / n^3
      //   + 3 phi_alg (A2x.xd)^2 / (2 n^5)  -  phi_alg xd^T A^2 xd / (2 n^3)
      const Vector3 A_xdot = relative_velocity.cwiseProduct(inv_radii_squared);
      const Scalar q1 = A_xdot.dot(relative_velocity);
      const Scalar q2 = Ax.dot(relative_velocity);
      const Scalar q3 = A2x.dot(relative_velocity);
      const Scalar q4 = A_xdot.cwiseProduct(inv_radii_squared).dot(relative_velocity);

      const Scalar quadratic_form = q1 / n - Scalar(2) * q2 * q3 / n3
                                    + Scalar(3) * s * q3 * q3 / (Scalar(2) * n5)
                                    - s * q4 / (Scalar(2) * n3);

      cdata.constraint_acceleration_error[0] = gradient.dot(relative_acceleration) + quadratic_form
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
      const Vector3 & gradient = cdata.gradient;

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

          jacobian_matrix(0, jj) = gradient.dot(relative_position_jacobian_col);
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
          std::invalid_argument, "Invalid MatrixBlockType for EllipsoidPointConstraintModelTpl.");
      }
    }

    /// \copydoc Base::getA1
    template<ReferenceFrame rf>
    MatrixSize6 getA1Impl(const ConstraintData & cdata, ReferenceFrameTag<rf> rft) const
    {
      MatrixSize6 res;
      res.noalias() =
        cdata.gradient.transpose()
        * internal::relativePointProjector1(
          cdata.oMc1, cdata.oMc2, this->joint1_placement, cdata.relative_position, rft);
      return res;
    }

    /// \copydoc Base::getA2
    template<ReferenceFrame rf>
    MatrixSize6 getA2Impl(const ConstraintData & cdata, ReferenceFrameTag<rf> rft) const
    {
      MatrixSize6 res;
      res.noalias() = cdata.gradient.transpose()
                      * internal::relativePointProjector2(
                        cdata.oMc1, cdata.oMc2, cdata.c1Mc2, this->joint2_placement, rft);
      return res;
    }

  protected:
    /// \brief Throws if one of the half-axes is not strictly positive.
    template<typename Vector3Like>
    static void checkRadii(const Eigen::MatrixBase<Vector3Like> & radii)
    {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(
        check_expression_if_real<Scalar>(radii.minCoeff() > Scalar(0)),
        "The radii of the ellipsoid must be strictly positive.");
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

    /// \brief Half-axes of the ellipsoid, along the axes of joint1_placement.
    Vector3 m_radii;

  }; // struct EllipsoidPointConstraintModelTpl

  ///
  /// \brief Data structure associated with EllipsoidPointConstraintModelTpl.
  ///
  template<typename _Scalar, int _Options>
  struct EllipsoidPointConstraintDataTpl
  : ConstraintDataBase<EllipsoidPointConstraintDataTpl<_Scalar, _Options>>
  {
    // --------------------------------------------------------------
    // Type defs
    // --------------------------------------------------------------
    // CRTP related types -------------------------------------------
    typedef EllipsoidPointConstraintDataTpl Self;
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
    EllipsoidPointConstraintDataTpl()
    : constraint_force(ResidualVectorType::Zero())
    , oMc1(SE3::Identity())
    , oMc2(SE3::Identity())
    , c1Mc2(SE3::Identity())
    , relative_position(Vector3::Zero())
    , algebraic_error(Scalar(0))
    , norm_Ax(Scalar(0))
    , gradient(Vector3::Zero())
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
    explicit EllipsoidPointConstraintDataTpl(const ConstraintModel & cmodel)
    : EllipsoidPointConstraintDataTpl()
    {
      PINOCCHIO_UNUSED_VARIABLE(cmodel);
    }

    // Operators ---------------------

    /// \brief Comparison operator
    bool operator==(const EllipsoidPointConstraintDataTpl & other) const
    {
      return constraint_force == other.constraint_force && oMc1 == other.oMc1 && oMc2 == other.oMc2
             && c1Mc2 == other.c1Mc2 && relative_position == other.relative_position
             && algebraic_error == other.algebraic_error && norm_Ax == other.norm_Ax
             && gradient == other.gradient
             && constraint_position_error == other.constraint_position_error
             && constraint_velocity_error == other.constraint_velocity_error
             && constraint_acceleration_error == other.constraint_acceleration_error
             && constraint_acceleration_biais_term == other.constraint_acceleration_biais_term
             && A1_world == other.A1_world && A2_world == other.A2_world && A_world == other.A_world
             && A1_local == other.A1_local && A2_local == other.A2_local
             && A_local == other.A_local;
    }

    /// \brief Comparison operator
    bool operator!=(const EllipsoidPointConstraintDataTpl & other) const
    {
      return !(*this == other);
    }

    // -------------------------------
    // IMPLEMENTATIONS OF BASE METHODS
    // -------------------------------

    /// \copydoc Base::classname
    static std::string classnameImpl()
    {
      return std::string("EllipsoidPointConstraintData");
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

    /// \brief Resulting constraint force, aligned with cdata.gradient.
    ResidualVectorType constraint_force;

    /// \brief Placement of the ellipsoid frame with respect to the WORLD frame
    SE3 oMc1;

    /// \brief Placement of the material point frame with respect to the WORLD frame
    SE3 oMc2;

    /// \brief Relative displacement between the two frames
    SE3 c1Mc2;

    /// \brief Position of the material point expressed in the frame of the ellipsoid
    Vector3 relative_position;

    /// \brief Algebraic residual x^T A x - 1, dimensionless
    Scalar algebraic_error;

    /// \brief Norm of A x, i.e. of the outward normal of the level set at the current point
    Scalar norm_Ax;

    /// \brief Gradient of the residual with respect to relative_position
    Vector3 gradient;

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
  }; // struct EllipsoidPointConstraintDataTpl

} // namespace pinocchio
