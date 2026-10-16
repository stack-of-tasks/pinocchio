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
  // Declaration
  // --------------------------------------------------------------
  template<typename ConstraintDataDerived>
  struct ConstraintDataBase;

  // --------------------------------------------------------------
  // Helper struct
  // --------------------------------------------------------------
  enum struct ConstraintSizeType
  {
    STATIC = 0,   // The size is fixed at compile time. residualSize = maxResidualSize
                  // -> no implementation to provide.
    CONSTANT = 1, // The size is fixed at build time. residualSize = maxResidualSize
                  // -> maxResidualSizeImpl to implement
    BOUNDED = 2,  // The maxResidualSize is fixed at build time. 0 <= size <= maxResidualSize
                  // -> residualSizeImpl and maxResidualSizeImpl to implement
    GENERAL = 3   // The size is not guaranteed to be bounded
                  // -> residualSizeImpl to implement
  };
  // {STATIC} \subset {CONSTANT} \subset {BOUNDED} \subset {GENERAL}
  constexpr bool operator<(ConstraintSizeType a, ConstraintSizeType b)
  {
    return static_cast<int>(a) < static_cast<int>(b);
  }
  constexpr bool operator<=(ConstraintSizeType a, ConstraintSizeType b)
  {
    return static_cast<int>(a) <= static_cast<int>(b);
  }
  constexpr bool operator>(ConstraintSizeType a, ConstraintSizeType b)
  {
    return static_cast<int>(a) > static_cast<int>(b);
  }
  constexpr bool operator>=(ConstraintSizeType a, ConstraintSizeType b)
  {
    return static_cast<int>(a) >= static_cast<int>(b);
  }
  constexpr ConstraintSizeType max(ConstraintSizeType a, ConstraintSizeType b)
  {
    return a > b ? a : b;
  }
  constexpr ConstraintSizeType min(ConstraintSizeType a, ConstraintSizeType b)
  {
    return a < b ? a : b;
  }

  // --------------------------------------------------------------
  // Struct
  // --------------------------------------------------------------
  template<class Derived>
  struct ConstraintModelBase
  : NumericalBase<Derived>
  , ModelEntity<Derived>
  , pinocchio::serialization::Serializable<Derived>
  {
    // --------------------------------------------------------------
    // Type defs
    // --------------------------------------------------------------
    // CRTP related types -------------------------------------------
    typedef ModelEntity<Derived> Base;

    // Retrieving traits --------------------------------------------
    typedef typename traits<Derived>::ConstraintModel ConstraintModel;
    typedef typename traits<Derived>::ConstraintData ConstraintData;

    typedef typename traits<Derived>::Scalar Scalar;
    static constexpr int Options = traits<Derived>::Options;

    // Specify how the constraint behave
    static constexpr ConstraintSizeType constraint_size_type =
      traits<Derived>::constraint_size_type;

    static constexpr bool has_baumgarte_corrector = traits<Derived>::has_baumgarte_corrector;
    static constexpr bool has_set = traits<Derived>::has_set;

    typedef typename traits<Derived>::ConstraintSet ConstraintSet;
    typedef typename traits<Derived>::JordanOperation JordanOperation;
    typedef typename traits<Derived>::BaumgarteCorrectorParameters BaumgarteCorrectorParameters;

    static constexpr int Size = traits<Derived>::Size;
    static constexpr int SymmetricConeSize = traits<Derived>::SymmetricConeSize;
    static constexpr int SymmetricConeScalingSize = traits<Derived>::SymmetricConeScalingSize;
    // note: any constraint set can be transformed to a constraint on a product of symmetric cones.

    typedef typename traits<Derived>::ResidualVectorType ResidualVectorType;
    typedef typename traits<Derived>::JacobianMatrixType JacobianMatrixType;
    typedef typename traits<Derived>::ConeVectorType ConeVectorType;
    typedef typename traits<Derived>::ConeScalingVectorType ConeScalingVectorType;
    // JacobianMatrixProductReturnType / JacobianTransposeMatrixProductReturnType Tpl

    // Useful types ------------------------------------------------
    typedef Eigen::Matrix<bool, Eigen::Dynamic, 1, Options> BooleanVector;
    typedef std::vector<Eigen::Index> EigenIndexVector;

    // Base usage ---------------------------------------------------
    using Base::createData;

    // -------------------------------
    // METHODS SPECIFIC TO CLASS
    // -------------------------------

    // CRTP related ------------------

    /// \brief Cast to derived class.
    Derived & derived()
    {
      return static_cast<Derived &>(*this);
    }

    /// \brief Const cast to derived class.
    const Derived & derived() const
    {
      return static_cast<const Derived &>(*this);
    }

    /// \brief Cast to base.
    ConstraintModelBase & base()
    {
      return *this;
    }

    /// \brief Const cast to base.
    const ConstraintModelBase & base() const
    {
      return *this;
    }

    // Constructors ------------------

  protected:
    /// \brief Default constructor
    /// Protected so that ConstraintModelBase cannot be constructed.
    ConstraintModelBase()
    {
    }

    /// \brief Constructor from model.
    /// Protected so that ConstraintModelBase cannot be constructed.
    template<int OtherOptions, template<typename, int> class JointCollectionTpl>
    explicit ConstraintModelBase(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & /*model*/)
    {
    }

    // Operators ---------------------

  public:
    /// \brief Cast to NewScalar.
    template<typename NewScalar>
    typename CastType<NewScalar, Derived>::type cast() const
    {
      return derived().template cast<NewScalar>();
    }

    /// \brief Cast to NewScalar.
    template<typename OtherDerived>
    void cast(ConstraintModelBase<OtherDerived> & other) const
    {
      other.name = name;
    }

    /// \brief Equality comparison operator.
    template<typename OtherDerived>
    bool operator==(const ConstraintModelBase<OtherDerived> & other) const
    {
      return name == other.name;
    }

    /// \brief Difference comparison operator.
    template<typename OtherDerived>
    bool operator!=(const ConstraintModelBase<OtherDerived> & other) const
    {
      return !(*this == other);
    }

    // -------------------------------
    // BASE METHODS
    // -------------------------------

    // General -----------------------

    /// \brief Returns the name of the underlying class if this is a variant.
    static std::string classname()
    {
      return Derived::classnameImpl();
    }

    /// \brief Returns the name of the underlying class if this is a variant.
    std::string shortname() const
    {
      return derived().shortnameImpl();
    }

    /// \brief Prints the shortname of the constraint.
    void disp(std::ostream & os) const
    {
      using namespace std;
      os << shortname() << endl;
    }

    /// \copydoc disp
    friend std::ostream &
    operator<<(std::ostream & os, const ConstraintModelBase<Derived> & constraint)
    {
      constraint.disp(os);
      return os;
    }

    /// \brief Returns a constraint data associated to this constraint model.
    ConstraintData createData() const
    {
      return derived().createDataImpl();
    }

    // Sizes -------------------------

    /// \brief Returns the maximum size of the constraint.
    template<ConstraintSelectionType Sel = ConstraintSelectionType::CURRENT>
    int residualSize(ConstraintSelectionTag<Sel> sel = CurrentSelection()) const
    {
      if constexpr (Size != Eigen::Dynamic)
      {
        // Leverage static size whatever
        return Size;
      }
      else if constexpr (constraint_size_type <= ConstraintSizeType::CONSTANT)
      {
        // Selection is trivial, do not generate multiple calls
        return derived().residualSizeImpl(CurrentSelection());
      }
      else
      {
        return derived().residualSizeImpl(sel);
      }
    }

    /// \brief Returns the size of the residual when transformed so that the constraint
    /// is expressed using only symmetric cones.
    /// Typically called after `calc` has been called.
    template<ConstraintSelectionType Sel = ConstraintSelectionType::CURRENT>
    int symmetricConeResidualSize(ConstraintSelectionTag<Sel> sel = CurrentSelection()) const
    {
      if constexpr (SymmetricConeSize != Eigen::Dynamic)
      {
        // Leverage static size whatever
        return SymmetricConeSize;
      }
      else if constexpr (constraint_size_type <= ConstraintSizeType::CONSTANT)
      {
        // Selection is trivial, do not generate multiple calls
        return derived().symmetricConeResidualSizeImpl(CurrentSelection());
      }
      else
      {
        return derived().symmetricConeResidualSizeImpl(sel);
      }
    }

    /// \brief Returns the size of the symmetric cone representation.
    /// Typically called after `calc` has been called.
    template<ConstraintSelectionType Sel = ConstraintSelectionType::CURRENT>
    int symmetricConeResidualScalingSize(ConstraintSelectionTag<Sel> sel = CurrentSelection()) const
    {
      if constexpr (SymmetricConeScalingSize != Eigen::Dynamic)
      {
        // Leverage static size whatever
        return SymmetricConeScalingSize;
      }
      else if constexpr (constraint_size_type <= ConstraintSizeType::CONSTANT)
      {
        // Selection is trivial, do not generate multiple calls
        return derived().symmetricConeResidualScalingSizeImpl(CurrentSelection());
      }
      else
      {
        return derived().symmetricConeResidualScalingSizeImpl(sel);
      }
    }

    // Hyperparameters handling -----------

    /// \brief Set the compliance
    template<typename VectorLike, ConstraintSelectionType Sel = ConstraintSelectionType::CURRENT>
    void setCompliance(
      const Eigen::MatrixBase<VectorLike> & vector,
      ConstraintSelectionTag<Sel> sel = CurrentSelection())
    {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(vector.size() == residualSize(sel));
      if constexpr (constraint_size_type <= ConstraintSizeType::CONSTANT)
      {
        // Selection is trivial, do not generate multiple calls
        return derived().setComplianceImpl(vector, CurrentSelection());
      }
      else
      {
        return derived().setComplianceImpl(vector, sel);
      }
    }

    /// \brief Fill the compliance of size residualSize relted to the courant state of the
    /// constraint
    template<typename VectorLike, ConstraintSelectionType Sel = ConstraintSelectionType::CURRENT>
    void retrieveCompliance(
      const Eigen::MatrixBase<VectorLike> & vector,
      ConstraintSelectionTag<Sel> sel = CurrentSelection()) const
    {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(vector.size() == residualSize(sel));
      if constexpr (constraint_size_type <= ConstraintSizeType::CONSTANT)
      {
        // Selection is trivial, do not generate multiple calls
        return derived().retrieveComplianceImpl(vector, CurrentSelection());
      }
      else
      {
        return derived().retrieveComplianceImpl(vector, sel);
      }
    }

    /// \brief Returns the Baumgarte parameters internally stored in the constraint model
    const BaumgarteCorrectorParameters & baumgarte_corrector_parameters() const
    {
      return derived().baumgarte_corrector_parameters_impl();
    }

    /// \brief Returns the Baumgarte parameters internally stored in the constraint model
    BaumgarteCorrectorParameters & baumgarte_corrector_parameters()
    {
      return derived().baumgarte_corrector_parameters_impl();
    }

    /// \brief Set baumgarte corrector
    template<
      typename BaumgarteCorrectorParametersIn,
      ConstraintSelectionType Sel = ConstraintSelectionType::CURRENT>
    void setBaumgarteCorrectorParameters(
      const BaumgarteCorrectorParametersIn & baumgarte_corrector_parameters_in,
      ConstraintSelectionTag<Sel> sel = CurrentSelection())
    {
      if constexpr (constraint_size_type <= ConstraintSizeType::CONSTANT)
      {
        // Selection is trivial, do not generate multiple calls
        derived().setBaumgarteCorrectorParametersImpl(
          baumgarte_corrector_parameters_in, CurrentSelection());
      }
      else
      {
        derived().setBaumgarteCorrectorParametersImpl(baumgarte_corrector_parameters_in, sel);
      }
    }

    // Methods for algorithms -------------

    /// \brief Returns an instance of the associated constraint set operator.
    ConstraintSet set(const ConstraintData & cdata) const
    {
      return derived().setImpl(cdata);
    }

    /// \brief Evaluate the constraint values at the current state given by data and store the
    /// results in cdata.
    /// \note data must be populated by results of a `forwardKinematic(model, data, q, v, a)`.
    /// The forward kinematics on q determines the constraint position error, on v the constraint
    /// velocity error, on a the constraint acceleration error.
    /// Typically, a call to `aba` will fill all the necessary fields of data.
    template<int OtherOptions, template<typename, int> class JointCollectionTpl>
    void calc(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      const DataTpl<Scalar, OtherOptions, JointCollectionTpl> & data,
      ConstraintData & cdata) const
    {
      derived().calcImpl(model, data, cdata);
    }

    /// \brief Fills the colwise sparsity associated with a given row of the active set of
    /// the constraints.
    /// \note If constraints are dynamic (e.g. joint limits), this vector is computed when
    /// calling the calc method.
    template<int OtherOptions, template<typename, int> class JointCollectionTpl>
    void getRowSparsityPattern(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      const DataTpl<Scalar, OtherOptions, JointCollectionTpl> & data,
      const ConstraintData & cdata,
      const Eigen::Index row_id,
      BooleanVector & result) const
    {
      assert(row_id < residualSize());
      derived().getRowSparsityPatternImpl(model, data, cdata, row_id, result);
    }

    /// \brief Fills the vector of the active indexes associated with a given row
    /// \note If constraints are dynamic (e.g. joint limits), this vector is computed when
    /// calling the calc method.
    template<int OtherOptions, template<typename, int> class JointCollectionTpl>
    void getRowIndexes(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      const DataTpl<Scalar, OtherOptions, JointCollectionTpl> & data,
      const ConstraintData & cdata,
      const Eigen::Index row_id,
      EigenIndexVector & result) const
    {
      assert(row_id < residualSize());
      derived().getRowIndexesImpl(model, data, cdata, row_id, result);
    }

    /// \brief Evaluate the Jacobian associated to the constraint at the given state stored in data
    /// and cdata.
    /// The results Jacobian is evaluated in the jacobian input/output matrix.
    /// This method assumes that the constrained data is up-to-date (calc has been called).
    template<
      int OtherOptions,
      template<typename, int> class JointCollectionTpl,
      typename JacobianMatrix>
    void jacobian(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      const DataTpl<Scalar, OtherOptions, JointCollectionTpl> & data,
      const ConstraintData & cdata,
      const Eigen::MatrixBase<JacobianMatrix> & jacobian_matrix) const
    {
      assert(jacobian_matrix.rows() == residualSize());
      assert(jacobian_matrix.cols() == model.nv);
      derived().jacobianImpl(model, data, cdata, jacobian_matrix.const_cast_derived());
    }

    /// \copydoc jacobian
    template<int OtherOptions, template<typename, int> class JointCollectionTpl>
    typename traits<Derived>::JacobianMatrixType jacobian(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      const DataTpl<Scalar, OtherOptions, JointCollectionTpl> & data,
      ConstraintData & cdata) const
    {
      typedef typename traits<Derived>::JacobianMatrixType ReturnType;
      ReturnType res = ReturnType::Zero(residualSize(), model.nv);

      jacobian(model, data, cdata, res);

      return res;
    }

    /// \brief Evaluates the constraint jacobian against an input matrix mat.
    template<
      typename InputMatrix,
      int OtherOptions,
      template<typename, int> class JointCollectionTpl>
    typename traits<Derived>::template JacobianMatrixProductReturnType<InputMatrix>::type
    jacobianMatrixProduct(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      const DataTpl<Scalar, OtherOptions, JointCollectionTpl> & data,
      const ConstraintData & cdata,
      const Eigen::MatrixBase<InputMatrix> & mat) const
    {
      assert(mat.rows() == model.nv);
      return derived().jacobianMatrixProductImpl(model, data, cdata, mat.derived());
    }

    /// \brief Evaluates the constraint jacobian against an input matrix mat.
    template<
      typename InputMatrix,
      typename OutputMatrix,
      int OtherOptions,
      template<typename, int> class JointCollectionTpl,
      AssignmentOperatorType op = SETTO>
    void jacobianMatrixProduct(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      const DataTpl<Scalar, OtherOptions, JointCollectionTpl> & data,
      const ConstraintData & cdata,
      const Eigen::MatrixBase<InputMatrix> & mat,
      const Eigen::MatrixBase<OutputMatrix> & res,
      AssignmentOperatorTag<op> aot = SetTo()) const
    {
      assert(mat.rows() == model.nv);
      assert(mat.cols() == res.cols());
      assert(res.rows() == residualSize());
      derived().jacobianMatrixProductImpl(
        model, data, cdata, mat.derived(), res.const_cast_derived(), aot);
    }

    /// \brief Evaluates the transpose of the constraint jacobian against an input matrix mat.
    template<
      typename InputMatrix,
      int OtherOptions,
      template<typename, int> class JointCollectionTpl>
    typename traits<Derived>::template JacobianTransposeMatrixProductReturnType<InputMatrix>::type
    jacobianTransposeMatrixProduct(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      const DataTpl<Scalar, OtherOptions, JointCollectionTpl> & data,
      const ConstraintData & cdata,
      const Eigen::MatrixBase<InputMatrix> & mat) const
    {
      assert(mat.rows() == residualSize());
      return derived().jacobianTransposeMatrixProductImpl(model, data, cdata, mat.derived());
    }

    /// \brief Evaluates the transpose of the constraint jacobian against an input matrix mat.
    template<
      typename InputMatrix,
      typename OutputMatrix,
      int OtherOptions,
      template<typename, int> class JointCollectionTpl,
      AssignmentOperatorType op = SETTO>
    void jacobianTransposeMatrixProduct(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      const DataTpl<Scalar, OtherOptions, JointCollectionTpl> & data,
      const ConstraintData & cdata,
      const Eigen::MatrixBase<InputMatrix> & mat,
      const Eigen::MatrixBase<OutputMatrix> & res,
      AssignmentOperatorTag<op> aot = SetTo()) const
    {
      assert(mat.rows() == residualSize());
      assert(mat.cols() == res.cols());
      assert(res.rows() == model.nv);
      derived().jacobianTransposeMatrixProductImpl(
        model, data, cdata, mat.derived(), res.const_cast_derived(), aot);
    }

    /// \brief Map the constraint forces (aka constraint Lagrange multipliers) to joint space (e.g.,
    /// joint forces, joint torque vector).
    ///
    /// \param[in] model The model of the rigid body system.
    /// \param[in] data The data associated with model.
    /// \param[in] cdata The constraint data associated with the constraint model.
    /// \param[in] constraint_forces Input constraint forces (Lagrange multipliers) associated with
    /// the constraint.
    /// \param[out] joint_forces Output joint forces associated with each joint of the model.
    /// \param[out] joint_torques Output joint torques associated with the model.
    /// \param[in] reference_frame Input reference frame in which the forces are expressed.
    ///
    /// \note The results will be added to the joint_forces and joint_torques ouput argument.
    template<
      int OtherOptions,
      int ForceOptions,
      template<typename, int> class JointCollectionTpl,
      typename ConstraintForceLike,
      typename ForceAllocator,
      typename JointTorquesLike,
      ReferenceFrame rf>
    void mapConstraintForceToJointSpace(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      const DataTpl<Scalar, OtherOptions, JointCollectionTpl> & data,
      const ConstraintData & cdata,
      const Eigen::MatrixBase<ConstraintForceLike> & constraint_forces,
      std::vector<ForceTpl<Scalar, ForceOptions>, ForceAllocator> & joint_forces,
      const Eigen::MatrixBase<JointTorquesLike> & joint_torques,
      ReferenceFrameTag<rf> reference_frame) const
    {
      assert(constraint_forces.rows() == residualSize());
      assert(joint_torques.rows() == model.nv);
      derived().mapConstraintForceToJointSpaceImpl(
        model, data, cdata, constraint_forces, joint_forces, joint_torques.const_cast_derived(),
        reference_frame);
    }

    /// \brief Map the joint space quantities (e.g.,
    /// joint motions, joint motion vector) to the constraint motions.
    ///
    /// \param[in] model The model of the rigid body system.
    /// \param[in] data The data associated with model.
    /// \param[in] cdata The constraint data associated with the constraint model.
    /// \param[in] joint_motions Input joint motions associated with the model.
    /// \param[in] joint_generalized_velocity Input joint motions associated with the model.
    /// \param[out] constraint_motions Output constraint motions.
    /// \param[in] reference_frame Input reference frame in which the joint motions are expressed.
    template<
      int OtherOptions,
      int MotionOptions,
      template<typename, int> class JointCollectionTpl,
      typename MotionAllocator,
      typename JointMotionsLike,
      typename VectorLike,
      ReferenceFrame rf>
    void mapJointSpaceToConstraintMotion(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      const DataTpl<Scalar, OtherOptions, JointCollectionTpl> & data,
      const ConstraintData & cdata,
      const std::vector<MotionTpl<Scalar, MotionOptions>, MotionAllocator> & joint_motions,
      const Eigen::MatrixBase<JointMotionsLike> & joint_generalized_velocity,
      const Eigen::MatrixBase<VectorLike> & constraint_motions,
      ReferenceFrameTag<rf> reference_frame) const
    {
      assert(joint_generalized_velocity.rows() == model.nv);
      assert(constraint_motions.rows() == residualSize());
      derived().mapJointSpaceToConstraintMotionImpl(
        model, data, cdata, joint_motions, joint_generalized_velocity, constraint_motions,
        reference_frame);
    }

    /// \brief Append to data the apparent inertia due to the constraint.
    /// Accepts a vector to represent a diagonal apparent inertia.
    template<
      int OtherOptions,
      template<typename, int> class JointCollectionTpl,
      typename VectorNLike,
      ReferenceFrame rf>
    void appendCouplingConstraintInertias(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      DataTpl<Scalar, OtherOptions, JointCollectionTpl> & data,
      const ConstraintData & cdata,
      const Eigen::MatrixBase<VectorNLike> & diagonal_constraint_inertia,
      const ReferenceFrameTag<rf> reference_frame) const
    {
      assert(diagonal_constraint_inertia.size() == residualSize());
      assert(diagonal_constraint_inertia.rows() == 1 || diagonal_constraint_inertia.cols() == 1);
      derived().appendCouplingConstraintInertiasImpl(
        model, data, cdata, diagonal_constraint_inertia.derived(), reference_frame);
    }

    /// \brief Append to data the apparent inertia due to the constraint.
    /// Accepts a single MatrixBlockElement (atomic constraints receive their block directly;
    /// pool constraints receive a NestedBlockDiagonal block containing one sub-block per inner
    /// constraint).
    template<
      int OtherOptions,
      template<typename, int> class JointCollectionTpl,
      typename MatrixOrMap,
      typename MapEnable,
      ReferenceFrame rf>
    void appendCouplingConstraintInertias(
      const ModelTpl<Scalar, OtherOptions, JointCollectionTpl> & model,
      DataTpl<Scalar, OtherOptions, JointCollectionTpl> & data,
      const ConstraintData & cdata,
      const internal::MatrixBlockElementTpl<MatrixOrMap, MapEnable> & constraint_inertia,
      const ReferenceFrameTag<rf> reference_frame) const
    {
      derived().appendCouplingConstraintInertiasImpl(
        model, data, cdata, constraint_inertia, reference_frame);
    }

    // -------------------------------
    // IMPLEMENTATIONS OF BASE METHODS
    // -------------------------------

    // General -----------------------
    // classnameImpl()
    // shortnameImpl()
    // createDataImpl()

    // Sizes ------------------------------
    // residualSizeImpl(ST) --> Not needed for static
    // symmetricConeResidualSizeImpl(ST) --> Not needed for static
    // symmetricConeResidualScalingSizeImpl(ST) --> Not needed for static

    // Hyperparameters handling -----------
    // setComplianceImpl(vec, ST)
    // retrieveComplianceImpl(vec, ST)
    // baumgarte_corrector_parameters_accessor_impl
    // setBaumgarteCorrectorParametersImpl(BP, ST)

    // Methods for algorithms -------------
    // setImpl(const cdata)  // Not needed if has_set=False
    // calcImpl(const model, const data, cdata)  // The only one mutating cdata
    // getRowSparsityPatternImpl(const model, const data, const cdata, ...)
    // getRowIndexesImpl(const model, const data, const cdata, ...)
    // jacobianImpl(const model, const data, const cdata, ...)
    // jacobianMatrixProductImpl(const model, const data, const cdata, ...)
    // jacobianTransposeMatrixProductImpl(const model, const data, const cdata, ...)
    // mapConstraintForceToJointSpaceImpl(const model, const data, const cdata, ...)
    // mapJointSpaceToConstraintMotionImpl(const model, const data, const cdata, ...)
    // appendCouplingConstraintInertiasImpl(const model, const data, const cdata, ...)

    /// \copydoc setBaumgarteCorrectorParameters
    template<typename BaumgarteCorrectorParametersIn, ConstraintSelectionType Sel>
    void setBaumgarteCorrectorParametersImpl(
      const BaumgarteCorrectorParametersIn & baumgarte_corrector_parameters_in,
      ConstraintSelectionTag<Sel> sel)
    {
      PINOCCHIO_UNUSED_VARIABLE(sel);
      if constexpr (has_baumgarte_corrector)
      {
        baumgarte_corrector_parameters() = baumgarte_corrector_parameters_in;
      }
      else
      {
        PINOCCHIO_THROW_PRETTY(
          std::runtime_error, "Cannot setBaumgarte for this constraint model.");
      }
    }

    // ------------------------------
    // MEMBERS
    // ------------------------------

    /// \brief Name of the constraint
    std::string name;
  }; // struct ConstraintModelBase

} // namespace pinocchio
