//
// Copyright (c) 2020-2024 INRIA
// Copyright (c) 2023 KU Leuven
//

#ifndef __pinocchio_algorithm_contact_delassus_hpp__
#define __pinocchio_algorithm_contact_delassus_hpp__

#include "pinocchio/algorithm/contact-info.hpp"

namespace pinocchio
{

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    class Allocator>
  PINOCCHIO_UNSUPPORTED_MESSAGE("The API will change towards more flexibility")
  inline void initPvDelassus(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const std::vector<RigidConstraintModelTpl<Scalar, Options>, Allocator> & contact_models);

  ///
  /// \brief Computes the Delassus matrix associated to a set of given constraints.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  /// \tparam ModelAllocator Allocator class for the std::vector.
  /// \tparam DataAllocator Allocator class for the std::vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration (size model.nq).
  /// \param[in] contact_models Vector of contact models.
  /// \param[in] contact_datas Vector of contact data.
  /// \param[out] delassus The resulting Delassus matrix.
  /// \param[in] mu Optional damping factor used when computing the inverse of the Delassus matrix.
  ///
  /// \return The (damped) Delassus matrix.
  ///
  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    class ModelAllocator,
    class DataAllocator,
    typename MatrixType>
  PINOCCHIO_UNSUPPORTED_MESSAGE("The API will change towards more flexibility")
  void computeDelassusMatrix(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const std::vector<RigidConstraintModelTpl<Scalar, Options>, ModelAllocator> & contact_models,
    std::vector<RigidConstraintDataTpl<Scalar, Options>, DataAllocator> & contact_data,
    const Eigen::MatrixBase<MatrixType> & delassus,
    const Scalar mu = 0);

  ///
  /// \brief Computes the inverse of the Delassus matrix associated to a set of given constraints.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  /// \tparam ModelAllocator Allocator class for the std::vector.
  /// \tparam DataAllocator Allocator class for the std::vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration (size model.nq).
  /// \param[in] contact_models Vector of contact models.
  /// \param[in] contact_datas Vector of contact data.
  /// \param[out] damped_delassus_inverse The resulting damped Delassus matrix.
  /// \param[in] mu Damping factor well-posdnessed of the problem.
  /// \param[in] scaled If set to true, the solution is scaled my a factor \f$ \mu \f$ to avoid
  /// numerical rounding issues. \param[in] Pv If set to true, uses PV-OSIMr, otherwise uses EFPA.
  ///
  /// \note A hint: a typical value for mu is 1e-4 when two contact constraints or more are
  /// redundant.
  ///
  /// \return The damped inverse Delassus matrix.
  ///
  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    class ModelAllocator,
    class DataAllocator,
    typename MatrixType>
  PINOCCHIO_UNSUPPORTED_MESSAGE("The API will change towards more flexibility")
  void computeDampedDelassusMatrixInverse(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const std::vector<RigidConstraintModelTpl<Scalar, Options>, ModelAllocator> & contact_models,
    std::vector<RigidConstraintDataTpl<Scalar, Options>, DataAllocator> & contact_data,
    const Eigen::MatrixBase<MatrixType> & damped_delassus_inverse,
    const Scalar mu,
    const bool scaled = false,
    const bool Pv = true);

} // namespace pinocchio

#include "pinocchio/algorithm/delassus.hxx"

#endif // ifndef __pinocchio_algorithm_contact_delassus_hpp__
