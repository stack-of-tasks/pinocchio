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

  /**
   * @brief Create a vector of ConstraintData from a vector of ConstraintModel.
   *
   * This function iterates over a vector of constraint models and calls
   * `createData()` on each model to generate the associated constraint data.
   * The resulting data is collected into a vector that uses the same allocator
   * as the input vector.
   *
   * @tparam ConstraintModel The type of constraint model. It must define a
   *         nested type `ConstraintData` and have a method `ConstraintData createData() const`.
   * @tparam ConstraintModelAllocator The allocator used for the input vector of constraint models.
   *
   * @param constraint_models A vector of constraint models used to generate the constraint data.
   *
   * @return A vector of `ConstraintModel::ConstraintData` using the same allocator
   *         as the input vector type. Each element corresponds to the result of
   *         `createData()` called on the respective constraint model.
   *
   * @note The return type is computed using `internal::std_vector_with_same_allocator`,
   *       which ensures that the output vector has a compatible allocator with the input vector.
   */
  template<typename ConstraintModel, class ConstraintModelAllocator>
  typename internal::template std_vector_with_same_allocator<
    std::vector<ConstraintModel, ConstraintModelAllocator>>::
    template type<typename ConstraintModel::ConstraintData>
    createData(const std::vector<ConstraintModel, ConstraintModelAllocator> & constraint_models)
  {
    typedef typename internal::template std_vector_with_same_allocator<
      std::vector<ConstraintModel, ConstraintModelAllocator>>::
      template type<typename ConstraintModel::ConstraintData>
        ReturnType;

    ReturnType constraint_datas;
    constraint_datas.reserve(constraint_models.size());

    for (const auto & cm : constraint_models)
      constraint_datas.push_back(cm.createData());

    return constraint_datas;
  }

  /**
   * @brief Compute the total size of a set of constraint models for the selection (Current and
   * Maximal).
   *
   * This function iterates through a list of constraint models and accumulates the number
   * of selected constraint sizes. For each constraint model it calls
   * `ConstraintModel::residualSize(sel)` to determine the number of selected constraints, then sums
   * these values over all constraints in the input vectors.
   *
   * @tparam ConstraintModel Type of each constraint model contained in the vector.
   * @tparam ConstraintModelAllocator Allocator type used for the vector of constraint models.
   *
   * @param[in] constraint_models Vector of constraint model objects.
   * @param[in] sel Either CURRENT or MAXIMAL.
   *
   * @return The total active size (dimension) obtained by summing the active sizes
   *         of all individual constraint models.
   *
   * @sa ConstraintModelTpl::residualSize
   */
  template<
    typename ConstraintModel,
    class ConstraintModelAllocator,
    ConstraintSelectionType Sel = ConstraintSelectionType::CURRENT>
  Eigen::Index residualSize(
    const std::vector<ConstraintModel, ConstraintModelAllocator> & constraint_models,
    ConstraintSelectionTag<Sel> sel = CurrentSelection())
  {
    Eigen::Index active_size = 0;
    for (std::size_t i = 0; i < constraint_models.size(); ++i)
    {
      const auto & cmodel = internal::helper::get_ref(constraint_models[i]);
      active_size += cmodel.residualSize(sel);
    }

    return active_size;
  }

  /**
   * @brief Compute the total size of a set of constraint models for the selection (Current and
   * Maximal).
   *
   * This function iterates through a list of constraint models and accumulates the number
   * of selected constraint sizes. For each constraint model it calls
   * `ConstraintModel::symmetricConeResidualSize(sel)` to determine the number of selected
   * constraints, then sums these values over all constraints in the input vectors.
   *
   * @tparam ConstraintModel Type of each constraint model contained in the vector.
   * @tparam ConstraintModelAllocator Allocator type used for the vector of constraint models.
   *
   * @param[in] constraint_models Vector of constraint model objects.
   * @param[in] sel Either CURRENT or MAXIMAL.
   *
   * @return The total active size (dimension) obtained by summing the active sizes
   *         of all individual constraint models.
   *
   * @sa ConstraintModelTpl::symmetricConeResidualSize
   */
  template<
    typename ConstraintModel,
    class ConstraintModelAllocator,
    ConstraintSelectionType Sel = ConstraintSelectionType::CURRENT>
  Eigen::Index symmetricConeResidualSize(
    const std::vector<ConstraintModel, ConstraintModelAllocator> & constraint_models,
    ConstraintSelectionTag<Sel> sel = CurrentSelection())
  {
    Eigen::Index active_size = 0;
    for (std::size_t i = 0; i < constraint_models.size(); ++i)
    {
      const auto & cmodel = internal::helper::get_ref(constraint_models[i]);
      active_size += cmodel.symmetricConeResidualSize(sel);
    }

    return active_size;
  }

  /**
   * @brief Compute the total size of a set of constraint models for the selection (Current and
   * Maximal).
   *
   * This function iterates through a list of constraint models and accumulates the number
   * of selected constraint sizes. For each constraint model it calls
   * `ConstraintModel::symmetricConeResidualScalingSize(sel)` to determine the number of selected
   * constraints, then sums these values over all constraints in the input vectors.
   *
   * @tparam ConstraintModel Type of each constraint model contained in the vector.
   * @tparam ConstraintModelAllocator Allocator type used for the vector of constraint models.
   *
   * @param[in] constraint_models Vector of constraint model objects.
   * @param[in] sel Either CURRENT or MAXIMAL.
   *
   * @return The total active size (dimension) obtained by summing the active sizes
   *         of all individual constraint models.
   *
   * @sa ConstraintModelTpl::symmetricConeResidualScalingSize
   */
  template<
    typename ConstraintModel,
    class ConstraintModelAllocator,
    ConstraintSelectionType Sel = ConstraintSelectionType::CURRENT>
  Eigen::Index symmetricConeResidualScalingSize(
    const std::vector<ConstraintModel, ConstraintModelAllocator> & constraint_models,
    ConstraintSelectionTag<Sel> sel = CurrentSelection())
  {
    Eigen::Index active_size = 0;
    for (std::size_t i = 0; i < constraint_models.size(); ++i)
    {
      const auto & cmodel = internal::helper::get_ref(constraint_models[i]);
      active_size += cmodel.symmetricConeResidualScalingSize(sel);
    }

    return active_size;
  }

  template<
    typename ConstraintModel,
    class ConstraintModelAllocator,
    typename ComplianceVector,
    ConstraintSelectionType Sel = ConstraintSelectionType::CURRENT>
  void setConstraintCompliance(
    std::vector<ConstraintModel, ConstraintModelAllocator> & constraint_models,
    const Eigen::MatrixBase<ComplianceVector> & compliance,
    ConstraintSelectionTag<Sel> sel = CurrentSelection())
  {
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(ComplianceVector);

    Eigen::Index constraint_index = 0;

    assert(compliance.size() == residualSize(constraint_models, sel));

    for (std::size_t i = 0; i < constraint_models.size(); i++)
    {
      auto & cmodel = internal::helper::get_ref(constraint_models[i]);
      const auto csize = cmodel.residualSize(sel);
      cmodel.setCompliance(compliance.segment(constraint_index, csize), sel);
      constraint_index += csize;
    }
  }

  template<
    typename ConstraintModel,
    class ConstraintModelAllocator,
    typename ComplianceVector,
    ConstraintSelectionType Sel = ConstraintSelectionType::CURRENT>
  void retrieveConstraintCompliance(
    const std::vector<ConstraintModel, ConstraintModelAllocator> & constraint_models,
    const Eigen::MatrixBase<ComplianceVector> & compliance_,
    ConstraintSelectionTag<Sel> sel = CurrentSelection())
  {
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(ComplianceVector);

    Eigen::Index constraint_index = 0;
    auto & compliance = compliance_.const_cast_derived();

    assert(compliance.size() == residualSize(constraint_models, sel));

    for (std::size_t i = 0; i < constraint_models.size(); i++)
    {
      const auto & cmodel = internal::helper::get_ref(constraint_models[i]);
      const auto csize = cmodel.residualSize(sel);
      cmodel.retrieveCompliance(compliance.segment(constraint_index, csize), sel);
      constraint_index += csize;
    }
  }

  /**
   * @brief Evaluate all the constraint models given a Pinocchio model and data.
   *
   * This function iterates through the provided list of constraint models
   * and calls their respective `calc` methods using the given model and data.
   * The computation results for each constraint model are stored in the
   * corresponding constraint data objects.
   *
   * @tparam Scalar Type of the scalar values (typically double or float).
   * @tparam Options Template options passed to Pinocchio's ModelTpl and DataTpl.
   * @tparam JointCollectionTpl The joint collection type defining the set of joints supported by
   * the model.
   * @tparam ConstraintModel Type of each constraint model contained in the @p constraint_models
   * vector.
   * @tparam ConstraintModelAllocator Allocator type for the @p constraint_models vector.
   * @tparam ConstraintData Type of each constraint model contained in the @p constraint_models
   * vector.
   * @tparam ConstraintDataAllocator Allocator type for the @p constraint_datas vector.
   *
   * @param[in] model The Pinocchio model structure.
   * @param[in] data The Pinocchio data structure associated with the model.
   * @param[in] constraint_models A vector of constraint model objects describing the constraints to
   * be evaluated.
   * @param[out] constraint_datas A vector of constraint data objects where each corresponding
   * constraint model’s results are stored.
   *
   * @note The size of @p constraint_models and @p constraint_datas must match.
   * @warning This function assumes that each constraint model and its corresponding data object
   * refer to the same type of constraint.
   *
   * @sa ConstraintModelTpl::calc, ConstraintDataTpl
   */
  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConstraintModel,
    class ConstraintModelAllocator,
    typename ConstraintData,
    typename ConstraintDataAllocator>
  void calc(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const std::vector<ConstraintModel, ConstraintModelAllocator> & constraint_models,
    std::vector<ConstraintData, ConstraintDataAllocator> & constraint_datas)
  {
    for (size_t k = 0; k < constraint_models.size(); ++k)
    {
      const auto & cmodel = internal::helper::get_ref(constraint_models[k]);
      auto & cdata = internal::helper::get_ref(constraint_datas[k]);

      cmodel.calc(model, data, cdata);
    }
  }

  ///
  /// \brief Computes the kinematic Jacobian associatied to a given constraint model.
  ///
  /// \remarks This function assumes that the a computeJointJacobians has been called first or any
  /// algorithms that computes data.J and data.oMi.
  /// This function also assumes that the constrained datas are up-to-date.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] constraint_model Constraint model.
  /// \param[in] constraint_data Constraint data.
  /// \param[out] J A reference on the Jacobian matrix where the results will be stored in (dim 6 x
  /// model.nv). You must fill J with zero elements, e.g. J.fill(0.).
  ///
  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConstraintModelDerived,
    typename ConstraintDataDerived,
    typename Matrix6Like>
  void getConstraintJacobian(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const ConstraintModelBase<ConstraintModelDerived> & constraint_model,
    const ConstraintDataBase<ConstraintDataDerived> & constraint_data,
    const Eigen::MatrixBase<Matrix6Like> & J);

  ///
  /// \brief Computes the kinematic Jacobian associatied to a given set of constraint models.
  ///
  /// \remarks This function assumes that the a computeJointJacobians has been called first or any
  /// algorithms that computes data.J and data.oMi.
  /// This function also assumes that the constrained datas are up-to-date.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] constraint_models Vector of constraint models.
  /// \param[in] constraint_datas Vector of constraint data.
  /// \param[out] J A reference on the Jacobian matrix where the results will be stored in (dim nc x
  /// model.nv). You must fill J with zero elements, e.g. J.fill(0.).
  ///
  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    class ConstraintModel,
    class ConstraintModelAllocator,
    class ConstraintData,
    class ConstraintDataAllocator,
    typename DynamicMatrixLike>
  void getConstraintsJacobian(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const std::vector<ConstraintModel, ConstraintModelAllocator> & constraint_models,
    const std::vector<ConstraintData, ConstraintDataAllocator> & constraint_datas,
    const Eigen::MatrixBase<DynamicMatrixLike> & J);

  ///
  /// \brief Computes the kinematic Jacobian associatied to a given set of constraint models.
  ///
  /// \remarks This function assumes that the a computeJointJacobians has been called first or any
  /// algorithms that computes data.J and data.oMi.
  /// This function also assumes that the constrained datas are up-to-date.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] constraint_models Vector of constraint models.
  /// \param[in] constraint_datas Vector of constraint data.
  /// \return A Jacobian matrix where the results will be stored in (dim nc x model.nv).
  ///
  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    class ConstraintModel,
    class ConstraintModelAllocator,
    class ConstraintData,
    class ConstraintDataAllocator>
  typename DataTpl<Scalar, Options, JointCollectionTpl>::MatrixXs getConstraintsJacobian(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const std::vector<ConstraintModel, ConstraintModelAllocator> & constraint_models,
    const std::vector<ConstraintData, ConstraintDataAllocator> & constraint_datas);

  ///
  /// \brief Evaluate the operation res = J * rhs
  ///
  /// \remarks This function assumes that the a computeJointJacobians has been called first or any
  /// algorithms that computes data.J and data.oMi.
  /// This function also assumes that the constrained datas are up-to-date.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] constraint_models Vector of constraint models.
  /// \param[in] constraint_datas Vector of constraint data.
  /// \param[in] rhs Right-hand side term.
  /// \param[out] res Results.
  ///
  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    class ConstraintModel,
    class ConstraintModelAllocator,
    class ConstraintData,
    class ConstraintDataAllocator,
    typename RhsMatrixType,
    typename ResultMatrixType,
    AssignmentOperatorType op = SETTO>
  void evalConstraintJacobianMatrixProduct(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const std::vector<ConstraintModel, ConstraintModelAllocator> & constraint_models,
    const std::vector<ConstraintData, ConstraintDataAllocator> & constraint_datas,
    const Eigen::MatrixBase<RhsMatrixType> & rhs,
    const Eigen::MatrixBase<ResultMatrixType> & res,
    AssignmentOperatorTag<op> aot = SetTo());

  ///
  /// \brief Evaluate the operation res = J.T * rhs
  ///
  /// \remarks This function assumes that the a computeJointJacobians has been called first or any
  /// algorithms that computes data.J and data.oMi.
  /// This function also assumes that the constrained datas are up-to-date.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] constraint_models Vector of constraint models.
  /// \param[in] constraint_datas Vector of constraint data.
  /// \param[in] rhs Right-hand side term.
  /// \param[out] res Results.
  ///
  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    class ConstraintModel,
    class ConstraintModelAllocator,
    class ConstraintData,
    class ConstraintDataAllocator,
    typename RhsMatrixType,
    typename ResultMatrixType,
    AssignmentOperatorType op = SETTO>
  void evalConstraintJacobianTransposeMatrixProduct(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const std::vector<ConstraintModel, ConstraintModelAllocator> & constraint_models,
    const std::vector<ConstraintData, ConstraintDataAllocator> & constraint_datas,
    const Eigen::MatrixBase<RhsMatrixType> & rhs,
    const Eigen::MatrixBase<ResultMatrixType> & res,
    AssignmentOperatorTag<op> aot = SetTo());

  ///
  /// \brief Maps the constraint forces expressed in the constraint space to joint forces expressed
  /// in the local frame.
  ///
  /// \remarks This function assumes that the constrained datas are up-to-date.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] constraint_models Vector of constraint models.
  /// \param[in] constraint_datas Vector of constraint datas.
  /// \param[in] constraint_forces Matrix or vector containing the constraint forces.
  /// \param[out] joint_forces Vector of  joint forces (dimension model.njoints).
  ///
  template<
    typename Scalar,
    int Options,
    int ForceOptions,
    template<typename, int> class JointCollectionTpl,
    class ConstraintModel,
    class ConstraintModelAllocator,
    class ConstraintData,
    class ConstraintDataAllocator,
    typename ForceMatrix,
    class ForceAllocator,
    ReferenceFrame rf>
  void mapConstraintForcesToJointForces(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const std::vector<ConstraintModel, ConstraintModelAllocator> & constraint_models,
    const std::vector<ConstraintData, ConstraintDataAllocator> & constraint_datas,
    const Eigen::MatrixBase<ForceMatrix> & constraint_forces,
    std::vector<ForceTpl<Scalar, ForceOptions>, ForceAllocator> & joint_forces,
    ReferenceFrameTag<rf> reference_frame);

  ///
  /// \brief Maps the joint motions expressed in the joint space local frame to the constraint
  /// motions.
  ///
  /// \remarks This function assumes that the constrained datas are up-to-date.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] constraint_models Vector of constraint models.
  /// \param[in] constraint_datas Vector of constraint datas.
  /// \param[in] joint_motions Vector of  joint motions (dimension model.njoints).
  /// \param[out] constraint_motions Resulting matrix or vector containing the constraint motions.
  ///
  template<
    typename Scalar,
    int Options,
    int MotionOptions,
    template<typename, int> class JointCollectionTpl,
    class ConstraintModel,
    class ConstraintModelAllocator,
    class ConstraintData,
    class ConstraintDataAllocator,
    class MotionAllocator,
    typename MotionMatrix,
    ReferenceFrame rf>
  void mapJointMotionsToConstraintMotions(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const std::vector<ConstraintModel, ConstraintModelAllocator> & constraint_models,
    const std::vector<ConstraintData, ConstraintDataAllocator> & constraint_datas,
    const std::vector<MotionTpl<Scalar, MotionOptions>, MotionAllocator> & joint_motions,
    const Eigen::MatrixBase<MotionMatrix> & constraint_motions,
    ReferenceFrameTag<rf> reference_frame);

  /// \brief Block diagonal dispatcher list.
  enum struct BlockDiagonalDispatcherType
  {
    DIAGONAL_DISPATCH,
    IPM_DISPATCH,
  };

  ///  \brief Assignment operator tags
  template<BlockDiagonalDispatcherType val>
  struct BlockDiagonalDispatcherTag
  {
  };

  using DiagonalDispatcher =
    BlockDiagonalDispatcherTag<BlockDiagonalDispatcherType::DIAGONAL_DISPATCH>;
  using BlockDiagonalDispatcher =
    BlockDiagonalDispatcherTag<BlockDiagonalDispatcherType::IPM_DISPATCH>;

  ///
  /// \brief Constructs the block diagonal pattern for a given vector of constraint models.
  ///
  /// \param[in] constraint_models Vector of constraint models.
  /// \param[out] block_diagonal_infos Vector of block diagonal pattern.
  template<
    typename ConstraintModel,
    typename ConstraintModelAllocator,
    typename BlockDiagonalElement,
    BlockDiagonalDispatcherType op = BlockDiagonalDispatcherType::DIAGONAL_DISPATCH>
  void computeBlockDiagonalPattern(
    const std::vector<ConstraintModel, ConstraintModelAllocator> & constraint_models,
    std::vector<BlockDiagonalElement> & block_diagonal_infos,
    BlockDiagonalDispatcherTag<op> dispatcher = DiagonalDispatcher());

  ///
  /// \brief Construct a positive definite (hence invertible) block diagonal matrix
  /// given a vector of constraint models.
  ///
  /// \param[in] constraint_models Vector of constraint models.
  /// \param[in/out] block_diagonal_matrix Invertible (PD) block diagonal matrix.
  template<
    typename ConstraintModel,
    typename ConstraintModelAllocator,
    typename Scalar,
    int Options,
    std::size_t Alignment>
  void constructPositiveDefiniteBlockDiagonalMatrix(
    const std::vector<ConstraintModel, ConstraintModelAllocator> & constraint_models,
    internal::BlockDiagonalMatrixTpl<Scalar, Options, Alignment> & block_diagonal_matrix);

  template<
    typename Scalar,
    int Options,
    int ForceOptions,
    template<typename, int> class JointCollectionTpl,
    class ConstraintModel,
    class ConstraintModelAllocator,
    class ConstraintData,
    class ConstraintDataAllocator,
    typename ForceMatrix,
    class ForceAllocator,
    ReferenceFrame rf>
  void mapConstraintForcesToJointForces(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const std::vector<ConstraintModel, ConstraintModelAllocator> & constraint_models,
    const std::vector<ConstraintData, ConstraintDataAllocator> & constraint_datas,
    const Eigen::MatrixBase<ForceMatrix> & constraint_forces,
    std::vector<ForceTpl<Scalar, ForceOptions>, ForceAllocator> & joint_forces,
    ReferenceFrameTag<rf> reference_frame)
  {
    PINOCCHIO_CHECK_ARGUMENT_SIZE(constraint_models.size(), constraint_datas.size());
    PINOCCHIO_CHECK_ARGUMENT_SIZE(joint_forces.size(), size_t(model.njoints));

    const Eigen::Index constraint_size = getTotalConstraintResidualSize(constraint_models);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(constraint_forces.rows(), constraint_size);

    for (auto & force : joint_forces)
      force.setZero();

    Eigen::Index row_id = 0;
    for (size_t constraint_id = 0; constraint_id < constraint_models.size(); ++constraint_id)
    {
      const auto & cmodel = internal::helper::get_ref(constraint_models[constraint_id]);
      const auto & cdata = internal::helper::get_ref(constraint_datas[constraint_id]);
      const auto constraint_size = cmodel.residualSize();

      const auto constraint_force = constraint_forces.segment(row_id, constraint_size);
      cmodel.mapConstraintForceToJointForces(
        model, data, cdata, constraint_force, joint_forces, reference_frame);

      row_id += constraint_size;
    }
  }

  template<
    typename Scalar,
    int Options,
    int ForceOptions,
    template<typename, int> class JointCollectionTpl,
    class ConstraintModel,
    class ConstraintModelAllocator,
    class ConstraintData,
    class ConstraintDataAllocator,
    typename ForceMatrix,
    class ForceAllocator,
    typename GeneralizedTorqueVector,
    ReferenceFrame rf>
  void mapConstraintForcesToJointSpace(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const std::vector<ConstraintModel, ConstraintModelAllocator> & constraint_models,
    const std::vector<ConstraintData, ConstraintDataAllocator> & constraint_datas,
    const Eigen::MatrixBase<ForceMatrix> & constraint_forces,
    std::vector<ForceTpl<Scalar, ForceOptions>, ForceAllocator> & joint_forces,
    const Eigen::MatrixBase<GeneralizedTorqueVector> & joint_torques_,
    ReferenceFrameTag<rf> reference_frame)
  {
    PINOCCHIO_CHECK_ARGUMENT_SIZE(constraint_models.size(), constraint_datas.size());
    PINOCCHIO_CHECK_ARGUMENT_SIZE(joint_forces.size(), size_t(model.njoints));
    PINOCCHIO_CHECK_ARGUMENT_SIZE(joint_torques_.size(), model.nv);

    const Eigen::Index constraint_size = getTotalConstraintResidualSize(constraint_models);
    assert(constraint_forces.rows() == constraint_size);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(constraint_forces.rows(), constraint_size);

    auto & joint_torques = joint_torques_.const_cast_derived();

    // Reset quantities
    joint_torques.setZero();
    for (auto & force : joint_forces)
      force.setZero();

    Eigen::Index row_id = 0;
    for (size_t constraint_id = 0; constraint_id < constraint_models.size(); ++constraint_id)
    {
      const auto & cmodel = internal::helper::get_ref(constraint_models[constraint_id]);
      const auto & cdata = internal::helper::get_ref(constraint_datas[constraint_id]);
      const auto constraint_size = cmodel.residualSize();

      const auto constraint_force = constraint_forces.segment(row_id, constraint_size);
      cmodel.mapConstraintForceToJointSpace(
        model, data, cdata, constraint_force, joint_forces, joint_torques, reference_frame);

      row_id += constraint_size;
    }
  }

  template<
    typename Scalar,
    int Options,
    int MotionOptions,
    template<typename, int> class JointCollectionTpl,
    class ConstraintModel,
    class ConstraintModelAllocator,
    class ConstraintData,
    class ConstraintDataAllocator,
    class MotionAllocator,
    typename MotionConstraintMatrix,
    ReferenceFrame rf>
  void mapJointMotionsToConstraintMotions(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const std::vector<ConstraintModel, ConstraintModelAllocator> & constraint_models,
    const std::vector<ConstraintData, ConstraintDataAllocator> & constraint_datas,
    const std::vector<MotionTpl<Scalar, MotionOptions>, MotionAllocator> & joint_motions,
    const Eigen::MatrixBase<MotionConstraintMatrix> & constraint_motions_,
    ReferenceFrameTag<rf> reference_frame)
  {
    PINOCCHIO_CHECK_ARGUMENT_SIZE(constraint_models.size(), constraint_datas.size());
    PINOCCHIO_CHECK_ARGUMENT_SIZE(joint_motions.size(), size_t(model.njoints));

    auto & constraint_motions = constraint_motions_.const_cast_derived();
    const Eigen::Index constraint_size = getTotalConstraintResidualSize(constraint_models);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(constraint_motions.rows(), constraint_size);

    Eigen::Index row_id = 0;
    for (size_t constraint_id = 0; constraint_id < constraint_models.size(); ++constraint_id)
    {
      const auto & cmodel = internal::helper::get_ref(constraint_models[constraint_id]);
      const auto & cdata = internal::helper::get_ref(constraint_datas[constraint_id]);
      const auto constraint_size = cmodel.residualSize();

      auto constraint_motion = constraint_motions.segment(row_id, constraint_size);
      cmodel.mapJointMotionsToConstraintMotion(
        model, data, cdata, joint_motions, constraint_motion, reference_frame);

      row_id += constraint_size;
    }
  }

  template<
    typename Scalar,
    int Options,
    int MotionOptions,
    template<typename, int> class JointCollectionTpl,
    class ConstraintModel,
    class ConstraintModelAllocator,
    class ConstraintData,
    class ConstraintDataAllocator,
    class MotionAllocator,
    typename GeneralizedVelocityVector,
    typename MotionConstraintMatrix,
    ReferenceFrame rf>
  void mapJointSpaceToConstraintMotions(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const std::vector<ConstraintModel, ConstraintModelAllocator> & constraint_models,
    const std::vector<ConstraintData, ConstraintDataAllocator> & constraint_datas,
    const std::vector<MotionTpl<Scalar, MotionOptions>, MotionAllocator> & joint_motions,
    const Eigen::MatrixBase<GeneralizedVelocityVector> & generalized_velocity,
    const Eigen::MatrixBase<MotionConstraintMatrix> & constraint_motions_,
    ReferenceFrameTag<rf> reference_frame)
  {
    PINOCCHIO_CHECK_ARGUMENT_SIZE(constraint_models.size(), constraint_datas.size());
    PINOCCHIO_CHECK_ARGUMENT_SIZE(joint_motions.size(), size_t(model.njoints));
    PINOCCHIO_CHECK_ARGUMENT_SIZE(generalized_velocity.size(), model.nv);

    auto & constraint_motions = constraint_motions_.const_cast_derived();
    const Eigen::Index total_constraint_size = getTotalConstraintResidualSize(constraint_models);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(constraint_motions.rows(), total_constraint_size);

    Eigen::Index row_id = 0;
    for (size_t constraint_id = 0; constraint_id < constraint_models.size(); ++constraint_id)
    {
      const auto & cmodel = internal::helper::get_ref(constraint_models[constraint_id]);
      const auto & cdata = internal::helper::get_ref(constraint_datas[constraint_id]);
      const auto constraint_size = cmodel.residualSize();

      auto constraint_motion = constraint_motions.segment(row_id, constraint_size);
      cmodel.mapJointSpaceToConstraintMotion(
        model, data, cdata, joint_motions, generalized_velocity, constraint_motion,
        reference_frame);

      row_id += constraint_size;
    }
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConstraintModel,
    typename ConstraintData,
    typename JacobianMatrixLike>
  void getConstraintJacobian(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const ConstraintModelBase<ConstraintModel> & constraint_model_,
    const ConstraintDataBase<ConstraintData> & constraint_data_,
    const Eigen::MatrixBase<JacobianMatrixLike> & J_)
  {
    JacobianMatrixLike & J = J_.const_cast_derived();
    const auto & constraint_model = internal::helper::get_ref(constraint_model_.derived());
    const auto & constraint_data = internal::helper::get_ref(constraint_data_.derived());

    assert(model.check(data) && "data is not consistent with model.");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(J_.rows(), constraint_model.residualSize());
    PINOCCHIO_CHECK_ARGUMENT_SIZE(J_.cols(), model.nv);

    constraint_model.jacobian(model, data, constraint_data, J);
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    class ConstraintModel,
    class ConstraintModelAllocator,
    class ConstraintData,
    class ConstraintDataAllocator,
    typename DynamicMatrixLike>
  void getConstraintsJacobian(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const std::vector<ConstraintModel, ConstraintModelAllocator> & constraint_models,
    const std::vector<ConstraintData, ConstraintDataAllocator> & constraint_datas,
    const Eigen::MatrixBase<DynamicMatrixLike> & J_)
  {
    const Eigen::Index constraint_size = getTotalConstraintResidualSize(constraint_models);
    assert(J_.rows() == constraint_size);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(J_.rows(), constraint_size);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(J_.cols(), model.nv);

    assert(model.check(data) && "data is not consistent with model.");
    assert(model.check(MimicChecker()) && "Function does not support mimic joints");

    auto & J = J_.const_cast_derived();
    Eigen::Index row_id = 0;
    for (size_t k = 0; k < constraint_models.size(); ++k)
    {
      const auto & cmodel = internal::helper::get_ref(constraint_models[k]);
      const auto & cdata = internal::helper::get_ref(constraint_datas[k]);

      const auto csize = cmodel.residualSize();
      getConstraintJacobian(model, data, cmodel, cdata, J.middleRows(row_id, csize));

      row_id += csize;
    }
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    class ConstraintModel,
    class ConstraintModelAllocator,
    class ConstraintData,
    class ConstraintDataAllocator>
  typename DataTpl<Scalar, Options, JointCollectionTpl>::MatrixXs getConstraintsJacobian(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const std::vector<ConstraintModel, ConstraintModelAllocator> & constraint_models,
    const std::vector<ConstraintData, ConstraintDataAllocator> & constraint_datas)
  {
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;
    typedef typename Data::MatrixXs ReturnType;

    const auto constraint_size = getTotalConstraintResidualSize(constraint_models);

    ReturnType res = ReturnType::Zero(constraint_size, model.nv);
    getConstraintsJacobian(model, data, constraint_models, constraint_datas, res);

    return res;
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    class ConstraintModel,
    class ConstraintModelAllocator,
    class ConstraintData,
    class ConstraintDataAllocator,
    typename RhsMatrixType,
    typename ResultMatrixType,
    AssignmentOperatorType op>
  void evalConstraintJacobianMatrixProduct(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const std::vector<ConstraintModel, ConstraintModelAllocator> & constraint_models,
    const std::vector<ConstraintData, ConstraintDataAllocator> & constraint_datas,
    const Eigen::MatrixBase<RhsMatrixType> & rhs,
    const Eigen::MatrixBase<ResultMatrixType> & res_,
    AssignmentOperatorTag<op> aot)
  {
    PINOCCHIO_UNUSED_VARIABLE(aot);
    const Eigen::Index constraint_size = getTotalConstraintResidualSize(constraint_models);
    auto & res = res_.const_cast_derived();

    PINOCCHIO_CHECK_ARGUMENT_SIZE(rhs.rows(), model.nv);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(res_.rows(), constraint_size);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(res_.cols(), rhs.cols());

    using aot_internal = std::conditional_t<
      std::is_same<AssignmentOperatorTag<op>, SetTo>::value, AddTo, AssignmentOperatorTag<op>>;
    if constexpr (std::is_same<AssignmentOperatorTag<op>, SetTo>::value)
    {
      res.setZero();
    }

    Eigen::Index row_id = 0;
    for (size_t constraint_id = 0; constraint_id < constraint_models.size(); ++constraint_id)
    {
      const auto & cmodel = internal::helper::get_ref(constraint_models[constraint_id]);
      const auto & cdata = internal::helper::get_ref(constraint_datas[constraint_id]);
      const auto constraint_size = cmodel.residualSize();

      auto res_block = res.middleRows(row_id, constraint_size);
      cmodel.jacobianMatrixProduct(model, data, cdata, rhs, res_block, aot_internal());

      row_id += constraint_size;
    }
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    class ConstraintModel,
    class ConstraintModelAllocator,
    class ConstraintData,
    class ConstraintDataAllocator,
    typename RhsMatrixType,
    typename ResultMatrixType,
    AssignmentOperatorType op>
  void evalConstraintJacobianTransposeMatrixProduct(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const std::vector<ConstraintModel, ConstraintModelAllocator> & constraint_models,
    const std::vector<ConstraintData, ConstraintDataAllocator> & constraint_datas,
    const Eigen::MatrixBase<RhsMatrixType> & rhs,
    const Eigen::MatrixBase<ResultMatrixType> & res_,
    AssignmentOperatorTag<op> aot)
  {
    PINOCCHIO_UNUSED_VARIABLE(aot);
    const Eigen::Index constraint_size = getTotalConstraintResidualSize(constraint_models);
    ResultMatrixType & res = res_.const_cast_derived();

    PINOCCHIO_CHECK_ARGUMENT_SIZE(rhs.rows(), constraint_size);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(res_.rows(), model.nv);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(res_.cols(), rhs.cols());

    Eigen::Index row_id = 0;

    using aot_internal = std::conditional_t<
      std::is_same<AssignmentOperatorTag<op>, SetTo>::value, AddTo, AssignmentOperatorTag<op>>;
    if constexpr (std::is_same<AssignmentOperatorTag<op>, SetTo>::value)
    {
      res.setZero();
    }

    for (size_t constraint_id = 0; constraint_id < constraint_models.size(); ++constraint_id)
    {
      const auto & cmodel = internal::helper::get_ref(constraint_models[constraint_id]);
      const auto & cdata = internal::helper::get_ref(constraint_datas[constraint_id]);
      const auto constraint_size = cmodel.residualSize();

      const auto rhs_block = rhs.middleRows(row_id, constraint_size);
      cmodel.jacobianTransposeMatrixProduct(model, data, cdata, rhs_block, res, aot_internal());

      row_id += constraint_size;
    }
  }

  namespace internal
  {
    /**
     * @brief ComputeBlockDiagonalPatternImpl functor
     * Computes the block pattern for a specific constraint.
     */

    // Default implementation of computing the block diagonal matrix block_infos
    // for this constraint.
    template<typename ConstraintModel, class = void>
    struct ComputeBlockDiagonalPatternImpl
    {
      template<typename BlockInfoVector, BlockDiagonalDispatcherType op>
      static void run(
        const ConstraintModel & cmodel,
        BlockInfoVector & block_infos,
        BlockDiagonalDispatcherTag<op> dispatcher)
      {
        PINOCCHIO_UNUSED_VARIABLE(cmodel);
        PINOCCHIO_UNUSED_VARIABLE(block_infos);
        PINOCCHIO_UNUSED_VARIABLE(dispatcher);

        // For some reason this assert is always evaluated when building
        // with g++ 11 and 12.
        // TODO Remove when Ubuntu 22.04 and Debian 12 (ROS) is no more supported.
#if !defined(__GNUC__) || __GNUC__ >= 13
        static_assert(
          false, "ComputeBlockDiagonalPatternImpl not implemented for this constraint.");
#endif // !defined(__GNUC__) || __GNUC__ >= 13
      }
    };

    // Specialization of compute block_infos for FrameAnchor.
    template<typename Scalar, int Options>
    struct ComputeBlockDiagonalPatternImpl<FrameAnchorConstraintModelTpl<Scalar, Options>>
    {
      typedef FrameAnchorConstraintModelTpl<Scalar, Options> ConstraintModel;

      template<typename BlockInfoVector, BlockDiagonalDispatcherType op>
      static void run(
        const ConstraintModel & cmodel,
        BlockInfoVector & block_infos,
        BlockDiagonalDispatcherTag<op> dispatcher)
      {
        PINOCCHIO_UNUSED_VARIABLE(cmodel);
        PINOCCHIO_UNUSED_VARIABLE(dispatcher);
        assert(ConstraintModel::SymmetricConeSize != Eigen::Dynamic);

        // equality constraint -> prox term -> Scalar Identity
        block_infos.emplace_back(MatrixBlockType::ScalarIdentity, ConstraintModel::Size);
      }
    };

    // Specialization of compute block_infos for PointAnchor.
    template<typename Scalar, int Options>
    struct ComputeBlockDiagonalPatternImpl<PointAnchorConstraintModelTpl<Scalar, Options>>
    {
      typedef PointAnchorConstraintModelTpl<Scalar, Options> ConstraintModel;

      template<typename BlockInfoVector, BlockDiagonalDispatcherType op>
      static void run(
        const ConstraintModel & cmodel,
        BlockInfoVector & block_infos,
        BlockDiagonalDispatcherTag<op> dispatcher)
      {
        PINOCCHIO_UNUSED_VARIABLE(cmodel);
        PINOCCHIO_UNUSED_VARIABLE(dispatcher);
        assert(ConstraintModel::SymmetricConeSize != Eigen::Dynamic);

        // equality constraint -> prox term -> Scalar Identity
        block_infos.emplace_back(MatrixBlockType::ScalarIdentity, ConstraintModel::Size);
      }
    };

    // Specialization of compute block_infos for ConstantLength.
    template<typename Scalar, int Options>
    struct ComputeBlockDiagonalPatternImpl<ConstantLengthConstraintModelTpl<Scalar, Options>>
    {
      typedef ConstantLengthConstraintModelTpl<Scalar, Options> ConstraintModel;

      template<typename BlockInfoVector, BlockDiagonalDispatcherType op>
      static void run(
        const ConstraintModel & cmodel,
        BlockInfoVector & block_infos,
        BlockDiagonalDispatcherTag<op> dispatcher)
      {
        PINOCCHIO_UNUSED_VARIABLE(cmodel);
        PINOCCHIO_UNUSED_VARIABLE(dispatcher);
        assert(ConstraintModel::SymmetricConeSize != Eigen::Dynamic);

        // equality constraint -> prox term -> Scalar Identity
        block_infos.emplace_back(MatrixBlockType::ScalarIdentity, ConstraintModel::Size);
      }
    };

    // Specialization of compute block_infos for PointContact.
    template<typename Scalar, int Options>
    struct ComputeBlockDiagonalPatternImpl<PointContactConstraintModelTpl<Scalar, Options>>
    {
      typedef PointContactConstraintModelTpl<Scalar, Options> ConstraintModel;

      template<typename BlockInfoVector, BlockDiagonalDispatcherType op>
      static void run(
        const ConstraintModel & cmodel,
        BlockInfoVector & block_infos,
        BlockDiagonalDispatcherTag<op> dispatcher)
      {
        PINOCCHIO_UNUSED_VARIABLE(cmodel);
        PINOCCHIO_UNUSED_VARIABLE(dispatcher);
        assert(ConstraintModel::SymmetricConeSize != Eigen::Dynamic);

        if constexpr (std::is_same<BlockDiagonalDispatcherTag<op>, DiagonalDispatcher>::value)
        {
          block_infos.emplace_back(MatrixBlockType::ScalarIdentity, ConstraintModel::Size);
        }

        if constexpr (std::is_same<BlockDiagonalDispatcherTag<op>, BlockDiagonalDispatcher>::value)
        {
          // for ipm, point contact -> 3x3 block
          block_infos.emplace_back(MatrixBlockType::Plain, ConstraintModel::Size);
        }
      }
    };

    // Specialization of compute block_infos for JointLimit.
    template<typename Scalar, int Options>
    struct ComputeBlockDiagonalPatternImpl<JointLimitConstraintModelTpl<Scalar, Options>>
    {
      typedef JointLimitConstraintModelTpl<Scalar, Options> ConstraintModel;

      template<typename BlockInfoVector, BlockDiagonalDispatcherType op>
      static void run(
        const ConstraintModel & cmodel,
        BlockInfoVector & block_infos,
        BlockDiagonalDispatcherTag<op> dispatcher)
      {
        PINOCCHIO_UNUSED_VARIABLE(dispatcher);

        if constexpr (std::is_same<BlockDiagonalDispatcherTag<op>, DiagonalDispatcher>::value)
        {
          block_infos.emplace_back(MatrixBlockType::ScalarIdentity, cmodel.residualSize());
        }

        if constexpr (std::is_same<BlockDiagonalDispatcherTag<op>, BlockDiagonalDispatcher>::value)
        {
          block_infos.emplace_back(MatrixBlockType::Diagonal, cmodel.residualSize());
        }
      }
    };

    // Specialization of compute block_infos for JointFriction.
    template<typename Scalar, int Options>
    struct ComputeBlockDiagonalPatternImpl<JointFrictionConstraintModelTpl<Scalar, Options>>
    {
      typedef JointFrictionConstraintModelTpl<Scalar, Options> ConstraintModel;

      template<typename BlockInfoVector, BlockDiagonalDispatcherType op>
      static void run(
        const ConstraintModel & cmodel,
        BlockInfoVector & block_infos,
        BlockDiagonalDispatcherTag<op> dispatcher)
      {
        PINOCCHIO_UNUSED_VARIABLE(dispatcher);

        if constexpr (std::is_same<BlockDiagonalDispatcherTag<op>, DiagonalDispatcher>::value)
        {
          block_infos.emplace_back(MatrixBlockType::ScalarIdentity, cmodel.residualSize());
        }

        if constexpr (std::is_same<BlockDiagonalDispatcherTag<op>, BlockDiagonalDispatcher>::value)
        {
          block_infos.emplace_back(MatrixBlockType::Diagonal, cmodel.residualSize());
        }
      }
    };

    /**
     * @brief ComputeBlockDiagonalPatternVisitor visitor
     */
    template<typename BlockInfoVector, BlockDiagonalDispatcherType op>
    struct ComputeBlockDiagonalPatternVisitor
    : visitors::ConstraintUnaryVisitorBase<ComputeBlockDiagonalPatternVisitor<BlockInfoVector, op>>
    {
      typedef boost::fusion::vector<BlockInfoVector &, BlockDiagonalDispatcherTag<op>> ArgsType;

      typedef visitors::ConstraintUnaryVisitorBase<
        ComputeBlockDiagonalPatternVisitor<BlockInfoVector, op>>
        Base;
      using Base::run;

      template<typename ConstraintModel>
      static void algo(
        const ConstraintModelBase<ConstraintModel> & cmodel,
        BlockInfoVector & block_infos,
        BlockDiagonalDispatcherTag<op> dispatcher)
      {
        typedef ComputeBlockDiagonalPatternImpl<ConstraintModel> Impl;
        Impl::run(cmodel.derived(), block_infos, dispatcher);
      }

      template<typename ConstraintModel>
      static void run(
        const pinocchio::ConstraintModelBase<ConstraintModel> & cmodel,
        BlockInfoVector & block_infos,
        BlockDiagonalDispatcherTag<op> dispatcher)
      {
        algo(cmodel.derived(), block_infos, dispatcher);
      }

      template<
        typename Scalar,
        int Options,
        template<typename S, int O> class ConstraintCollectionTpl>
      static void run(
        const pinocchio::ConstraintModelTpl<Scalar, Options, ConstraintCollectionTpl> & cmodel,
        BlockInfoVector & block_infos,
        BlockDiagonalDispatcherTag<op> dispatcher)
      {
        ArgsType args(block_infos, dispatcher);
        run(cmodel.derived(), args);
      }
    }; // struct ComputeBlockDiagonalPatternVisitor
  } // namespace internal

  template<
    typename ConstraintModel,
    typename ConstraintModelAllocator,
    typename BlockDiagonalElement,
    BlockDiagonalDispatcherType op>
  void computeBlockDiagonalPattern(
    const std::vector<ConstraintModel, ConstraintModelAllocator> & constraint_models,
    std::vector<BlockDiagonalElement> & block_diagonal_infos,
    BlockDiagonalDispatcherTag<op> dispatcher)
  {
    block_diagonal_infos.clear();
    block_diagonal_infos.reserve(constraint_models.size());

    for (std::size_t i = 0; i < constraint_models.size(); ++i)
    {
      const auto & cmodel = internal::helper::get_ref(constraint_models[i]);

      typedef internal::ComputeBlockDiagonalPatternVisitor<decltype(block_diagonal_infos), op> Algo;
      Algo::run(cmodel, block_diagonal_infos, dispatcher);
    }
  }

  template<
    typename ConstraintModel,
    typename ConstraintModelAllocator,
    typename Scalar,
    int Options,
    std::size_t Alignment>
  void constructPositiveDefiniteBlockDiagonalMatrix(
    const std::vector<ConstraintModel, ConstraintModelAllocator> & constraint_models,
    internal::BlockDiagonalMatrixTpl<Scalar, Options, Alignment> & block_diagonal_matrix)
  {
    typedef internal::BlockDiagonalMatrixTpl<Scalar, Options, Alignment> BlockDiagonalMatrix;
    typedef typename BlockDiagonalMatrix::MatrixBlockElement MatrixBlockElement;
    std::vector<MatrixBlockElement> block_diagonal_infos;
    computeBlockDiagonalPattern(constraint_models, block_diagonal_infos, BlockDiagonalDispatcher());
    block_diagonal_matrix.rebuild(block_diagonal_infos);
    for (auto & block : block_diagonal_matrix.blocks())
    {
      block.setRandomPD();
    }
  }

} // namespace pinocchio

#ifdef PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
  #ifndef PINOCCHIO_SKIP_ALGORITHM_CONTACT_JACOBIAN

namespace pinocchio
{
  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void getConstraintJacobian<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    RigidConstraintModel,
    RigidConstraintData,
    context::MatrixXs>(
    const Model &,
    const Data &,
    const ConstraintModelBase<RigidConstraintModel> &,
    const ConstraintDataBase<RigidConstraintData> &,
    const Eigen::MatrixBase<context::MatrixXs> &);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void getConstraintsJacobian<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    RigidConstraintModel,
    typename RigidConstraintModelVector::allocator_type,
    RigidConstraintData,
    typename RigidConstraintDataVector::allocator_type,
    context::MatrixXs>(
    const Model &,
    const Data &,
    const RigidConstraintModelVector &,
    const RigidConstraintDataVector &,
    const Eigen::MatrixBase<context::MatrixXs> &);
} // namespace pinocchio

  #endif // ifndef PINOCCHIO_SKIP_ALGORITHM_CONTACT_JACOBIAN
#endif   // ifdef PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
