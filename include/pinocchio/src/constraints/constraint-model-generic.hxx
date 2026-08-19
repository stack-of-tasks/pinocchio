//
// Copyright (c) 2023-2025 INRIA
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
  // Traits
  // --------------------------------------------------------------

  template<
    typename _Scalar,
    int _Options,
    template<typename S, int O> class ConstraintCollectionTpl>
  struct traits<ConstraintModelTpl<_Scalar, _Options, ConstraintCollectionTpl>>
  {
    // --------------------------------------------------------------
    // Traits referencing the constraint and associated types
    // --------------------------------------------------------------
    typedef ConstraintModelTpl<_Scalar, _Options, ConstraintCollectionTpl> ConstraintModel;
    typedef ConstraintDataTpl<_Scalar, _Options, ConstraintCollectionTpl> ConstraintData;

    typedef ConstraintModel Model;
    typedef ConstraintData Data;

    // --------------------------------------------------------------
    // Traits characterizing the constraints
    // --------------------------------------------------------------
    typedef _Scalar Scalar;
    static constexpr int Options = _Options;

    static constexpr ConstraintSizeType constraint_size_type = ConstraintSizeType::GENERAL;

    // The generic behave raise an error if the underlying class does not have baumgarte
    static constexpr bool has_baumgarte_corrector = true;
    static constexpr bool has_set = true;
    // static constexpr bool is_inequality_constraint NOT USED

    // --------------------------------------------------------------
    // Traits for associated struct and sizes
    // --------------------------------------------------------------
    typedef boost::blank ConstraintSet;
    typedef boost::blank JordanOperation;
    typedef BaumgarteCorrectorParametersTpl<Scalar> BaumgarteCorrectorParameters;

    // To leverage eventual static, the Sizes are given by product with poolCount
    static constexpr int Size = Eigen::Dynamic;
    static constexpr int SymmetricConeSize = Eigen::Dynamic;
    static constexpr int SymmetricConeScalingSize = Eigen::Dynamic;

    // --------------------------------------------------------------
    // Traits that are helper for Eigen types
    // --------------------------------------------------------------
    typedef Eigen::Matrix<Scalar, Size, 1, Options> ResidualVectorType;
    typedef Eigen::Matrix<Scalar, Size, Eigen::Dynamic, Options> JacobianMatrixType;
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
      typedef Eigen::
        Matrix<Scalar, Size, InputMatrixPlain::ColsAtCompileTime, InputMatrixPlain::Options>
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

  // --------------------------------------------------------------
  // Struct
  // --------------------------------------------------------------
  template<
    typename _Scalar,
    int _Options,
    template<typename S, int O> class ConstraintCollectionTpl>
  struct ConstraintModelTpl
  : ConstraintModelBase<ConstraintModelTpl<_Scalar, _Options, ConstraintCollectionTpl>>
  , ConstraintCollectionTpl<_Scalar, _Options>::ConstraintModelVariant
  {
    // --------------------------------------------------------------
    // Type defs
    // --------------------------------------------------------------
    // CRTP related types -------------------------------------------
    typedef ConstraintModelTpl Self;
    typedef ConstraintModelBase<Self> Base;
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

    // Variant related ----------------------------------------------
    typedef ConstraintCollectionTpl<Scalar, Options> ConstraintCollection;
    typedef typename ConstraintCollection::ConstraintModelVariant ConstraintModelVariant;
    typedef typename ConstraintCollection::ConstraintDataVariant ConstraintDataVariant;

    // Base usage ---------------------------------------------------
    using Base::residualSize;
    using typename Base::BooleanVector;
    using typename Base::EigenIndexVector;

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

    ConstraintModelVariant & toVariant()
    {
      return static_cast<ConstraintModelVariant &>(*this);
    }

    const ConstraintModelVariant & toVariant() const
    {
      return static_cast<const ConstraintModelVariant &>(*this);
    }

    // Constructors ------------------

    ConstraintModelTpl()
    : ConstraintModelVariant()
    {
    }

    ConstraintModelTpl(const ConstraintModelVariant & cmodel_variant)
    : ConstraintModelVariant(cmodel_variant)
    {
    }

    template<typename ContraintModelDerived>
    ConstraintModelTpl(const ConstraintModelBase<ContraintModelDerived> & cmodel)
    : ConstraintModelVariant((ConstraintModelVariant)cmodel.derived())
    {
      BOOST_MPL_ASSERT(
        (boost::mpl::contains<typename ConstraintModelVariant::types, ContraintModelDerived>));
    }

    // Operators ---------------------

    template<typename ConstraintModelDerived>
    bool isEqual(const ConstraintModelBase<ConstraintModelDerived> & other) const
    {
      return ::pinocchio::isEqual(*this, other.derived());
    }

    bool isEqual(const ConstraintModelTpl & other) const
    {
      return toVariant() == other.toVariant();
    }

    /// \brief Comparison operator
    bool operator==(const ConstraintModelTpl & other) const
    {
      return isEqual(other);
    }

    /// \brief Comparison operator
    bool operator!=(const ConstraintModelTpl & other) const
    {
      return !(*this == other);
    }

    // -------------------------------
    // IMPLEMENTATIONS OF BASE METHODS
    // -------------------------------

    // General -----------------------

    /// \copydoc RootBase::classname
    static std::string classnameImpl()
    {
      return "ConstraintModel";
    }

    /// \copydoc RootBase::shortname
    std::string shortnameImpl() const
    {
      return ::pinocchio::visitors::shortname(*this);
    }

    /// \copydoc RootBase::createData
    ConstraintData createDataImpl() const
    {
      return ::pinocchio::visitors::createData(*this);
    }

    // Sizes -------------------------

    /// \copydoc RootBase::residualSize
    template<ConstraintSelectionType Sel>
    int residualSizeImpl(ConstraintSelectionTag<Sel> sel) const
    {
      return ::pinocchio::visitors::residualSize(*this, sel);
    }

    /// \copydoc RootBase::symmetricConeResidualSize
    template<ConstraintSelectionType Sel>
    int symmetricConeResidualSizeImpl(ConstraintSelectionTag<Sel> sel) const
    {
      return ::pinocchio::visitors::symmetricConeResidualSize(*this, sel);
    }

    /// \copydoc RootBase::symmetricConeResidualScalingSize
    template<ConstraintSelectionType Sel>
    int symmetricConeResidualScalingSizeImpl(ConstraintSelectionTag<Sel> sel) const
    {
      return ::pinocchio::visitors::symmetricConeResidualScalingSize(*this, sel);
    }

    // Hyperparameters handling -----------

    /// \copydoc RootBase::setCompliance
    template<typename VectorLike, ConstraintSelectionType Sel>
    void
    setComplianceImpl(const Eigen::MatrixBase<VectorLike> & vector, ConstraintSelectionTag<Sel> sel)
    {
      return ::pinocchio::visitors::setCompliance(*this, vector, sel);
    }

    /// \copydoc RootBase::retrieveCompliance
    template<typename VectorLike, ConstraintSelectionType Sel>
    void retrieveComplianceImpl(
      const Eigen::MatrixBase<VectorLike> & vector, ConstraintSelectionTag<Sel> sel) const
    {
      return ::pinocchio::visitors::retrieveCompliance(*this, vector, sel);
    }

    /// \copydoc RootBase::baumgarte_corrector_parameters
    const BaumgarteCorrectorParameters & baumgarte_corrector_parameters_impl() const
    {
      return ::pinocchio::visitors::getBaumgarteCorrectorParameters(*this);
    }

    /// \copydoc RootBase::baumgarte_corrector_parameters
    BaumgarteCorrectorParameters & baumgarte_corrector_parameters_impl()
    {
      return ::pinocchio::visitors::getBaumgarteCorrectorParameters(*this);
    }

    /// \copydoc RootBase::setBaumgarteCorrectorParameters
    template<typename BaumgarteCorrectorParametersIn, ConstraintSelectionType Sel>
    void setBaumgarteCorrectorParametersImpl(
      const BaumgarteCorrectorParametersIn & baumgarte_corrector_parameters_in,
      ConstraintSelectionTag<Sel> sel)
    {
      ::pinocchio::visitors::setBaumgarteCorrectorParameters(
        *this, baumgarte_corrector_parameters_in, sel);
    }

    // Methods for algorithms -------------

    /// \copydoc RootBase::set
    boost::blank setImpl(const ConstraintData & cdata) const
    {
      PINOCCHIO_UNUSED_VARIABLE(cdata);
      PINOCCHIO_THROW_PRETTY(
        std::runtime_error, "Set method is not accessible for ConstraintModelTpl.");
      return boost::blank();
    }

    /// \copydoc RootBase::calc
    template<int OtherOptions, template<typename, int> class JointCollectionTpl>
    void calcImpl(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      const DataTpl<Scalar, OtherOptions, JointCollectionTpl> & data,
      ConstraintData & cdata) const
    {
      ::pinocchio::visitors::calc(*this, model, data, cdata);
    }

    /// \copydoc RootBase::getRowSparsityPattern
    template<int OtherOptions, template<typename, int> class JointCollectionTpl>
    void getRowSparsityPatternImpl(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      const DataTpl<Scalar, OtherOptions, JointCollectionTpl> & data,
      const ConstraintData & cdata,
      const Eigen::Index row_id,
      BooleanVector & result) const
    {
      ::pinocchio::visitors::getRowSparsityPattern(*this, model, data, cdata, row_id, result);
    }

    /// \copydoc RootBase::getRowIndexes
    template<int OtherOptions, template<typename, int> class JointCollectionTpl>
    void getRowIndexesImpl(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      const DataTpl<Scalar, OtherOptions, JointCollectionTpl> & data,
      const ConstraintData & cdata,
      const Eigen::Index row_id,
      EigenIndexVector & result) const
    {
      ::pinocchio::visitors::getRowIndexes(*this, model, data, cdata, row_id, result);
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
      const Eigen::MatrixBase<JacobianMatrix> & jacobian_matrix) const
    {
      ::pinocchio::visitors::jacobian(
        *this, model, data, cdata, jacobian_matrix.const_cast_derived());
    }

    /// \copydoc RootBase::jacobianMatrixProduct
    template<
      int OtherOptions,
      template<typename, int> class JointCollectionTpl,
      typename InputMatrix,
      typename OutputMatrix>
    typename traits<Self>::template JacobianMatrixProductReturnType<InputMatrix>::type
    jacobianMatrixProductImpl(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      const DataTpl<Scalar, OtherOptions, JointCollectionTpl> & data,
      const ConstraintData & cdata,
      const Eigen::MatrixBase<InputMatrix> & input_matrix) const
    {
      typedef typename traits<Self>::template JacobianMatrixProductReturnType<InputMatrix>::type
        ReturnType;
      ReturnType res(residualSize(), input_matrix.cols());
      jacobianMatrixProduct(model, data, cdata, input_matrix.derived(), res);
      return res;
    }

    /// \copydoc RootBase::jacobianMatrixProduct
    template<
      int OtherOptions,
      template<typename, int> class JointCollectionTpl,
      typename InputMatrix,
      typename OutputMatrix,
      AssignmentOperatorType op>
    void jacobianMatrixProductImpl(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      const DataTpl<Scalar, OtherOptions, JointCollectionTpl> & data,
      const ConstraintData & cdata,
      const Eigen::MatrixBase<InputMatrix> & input_matrix,
      const Eigen::MatrixBase<OutputMatrix> & result_matrix,
      AssignmentOperatorTag<op> aot) const
    {
      ::pinocchio::visitors::jacobianMatrixProduct(
        *this, model, data, cdata, input_matrix.derived(), result_matrix.const_cast_derived(), aot);
    }

    /// \copydoc RootBase::jacobianTransposeMatrixProduct
    template<
      int OtherOptions,
      template<typename, int> class JointCollectionTpl,
      typename InputMatrix,
      typename OutputMatrix>
    typename traits<Self>::template JacobianTransposeMatrixProductReturnType<InputMatrix>::type
    jacobianTransposeMatrixProductImpl(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      const DataTpl<Scalar, OtherOptions, JointCollectionTpl> & data,
      const ConstraintData & cdata,
      const Eigen::MatrixBase<InputMatrix> & input_matrix) const
    {
      typedef
        typename traits<Self>::template JacobianTransposeMatrixProductReturnType<InputMatrix>::type
          ReturnType;
      ReturnType res(model.nv, input_matrix.cols());
      jacobianTransposeMatrixProduct(*this, model, data, cdata, input_matrix.derived(), res);
      return res;
    }

    /// \copydoc RootBase::jacobianTransposeMatrixProduct
    template<
      int OtherOptions,
      typename InputMatrix,
      typename OutputMatrix,
      template<typename, int> class JointCollectionTpl,
      AssignmentOperatorType op>
    void jacobianTransposeMatrixProductImpl(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      const DataTpl<Scalar, OtherOptions, JointCollectionTpl> & data,
      const ConstraintData & cdata,
      const Eigen::MatrixBase<InputMatrix> & input_matrix,
      const Eigen::MatrixBase<OutputMatrix> & result_matrix,
      AssignmentOperatorTag<op> aot) const
    {
      ::pinocchio::visitors::jacobianTransposeMatrixProduct(
        *this, model, data, cdata, input_matrix.derived(), result_matrix.const_cast_derived(), aot);
    }

    /// \copydoc RootBase::mapConstraintForceToJointSpace
    template<
      int OtherOptions,
      int ForceOptions,
      template<typename, int> class JointCollectionTpl,
      typename ConstraintForceLike,
      typename ForceAllocator,
      typename JointTorquesLike,
      ReferenceFrame rf>
    void mapConstraintForceToJointSpaceImpl(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      const DataTpl<Scalar, OtherOptions, JointCollectionTpl> & data,
      const ConstraintData & cdata,
      const Eigen::MatrixBase<ConstraintForceLike> & constraint_forces,
      std::vector<ForceTpl<Scalar, ForceOptions>, ForceAllocator> & joint_forces,
      const Eigen::MatrixBase<JointTorquesLike> & joint_torques,
      ReferenceFrameTag<rf> reference_frame) const
    {
      ::pinocchio::visitors::mapConstraintForceToJointSpace(
        *this, model, data, cdata, constraint_forces, joint_forces,
        joint_torques.const_cast_derived(), reference_frame);
    }

    /// \copydoc RootBase::mapJointSpaceToConstraintMotion
    template<
      int OtherOptions,
      int MotionOptions,
      template<typename, int> class JointCollectionTpl,
      typename MotionAllocator,
      typename JointMotionsLike,
      typename VectorLike,
      ReferenceFrame rf>
    void mapJointSpaceToConstraintMotionImpl(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      const DataTpl<Scalar, OtherOptions, JointCollectionTpl> & data,
      const ConstraintData & cdata,
      const std::vector<MotionTpl<Scalar, MotionOptions>, MotionAllocator> & joint_motions,
      const Eigen::MatrixBase<JointMotionsLike> & joint_generalized_velocity,
      const Eigen::MatrixBase<VectorLike> & constraint_motions,
      ReferenceFrameTag<rf> reference_frame) const
    {
      ::pinocchio::visitors::mapJointSpaceToConstraintMotion(
        *this, model, data, cdata, joint_motions, joint_generalized_velocity,
        constraint_motions.const_cast_derived(), reference_frame);
    }

    /// \copydoc RootBase::appendCouplingConstraintInertias
    template<
      int OtherOptions,
      template<typename, int> class JointCollectionTpl,
      typename VectorNLike,
      ReferenceFrame rf>
    void appendCouplingConstraintInertiasImpl(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      DataTpl<Scalar, OtherOptions, JointCollectionTpl> & data,
      const ConstraintData & cdata,
      const Eigen::MatrixBase<VectorNLike> & diagonal_constraint_inertia,
      const ReferenceFrameTag<rf> reference_frame) const
    {
      ::pinocchio::visitors::appendCouplingConstraintInertias(
        *this, model, data, cdata, diagonal_constraint_inertia.derived(), reference_frame);
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
      ::pinocchio::visitors::appendCouplingConstraintInertias(
        *this, model, data, cdata, constraint_inertia, reference_frame);
    }
  }; // struct ConstraintModelTpl

} // namespace pinocchio
