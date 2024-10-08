//
// Copyright (c) 2015-2021 CNRS INRIA
//

#ifndef __pinocchio_crba_hpp__
#define __pinocchio_crba_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/check.hpp"

namespace pinocchio
{
  ///
  /// \brief Computes the upper triangular part of the joint space inertia matrix M by
  ///        using the Composite Rigid Body Algorithm (Chapter 6, Rigid-Body Dynamics Algorithms, R.
  ///        Featherstone, 2008). The result is accessible through data.M.
  ///
  /// \note You can easily get data.M symmetric by copying the strictly upper triangular part
  ///       in the strictly lower triangular part with
  ///       data.M.triangularView<Eigen::StrictlyLower>() =
  ///       data.M.transpose().triangularView<Eigen::StrictlyLower>();
  ///
  /// \note This algorithm also takes into account the rotor inertia effects, by adding on the
  /// diagonal of the Joint Space Inertia Matrix their contributions.
  ///       This is done only for single DOF joint (e.g. Revolute, Prismatic, etc.).
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  ///
  /// \note In WORLD convention, a direct outcome of this algorithm is the computation of the
  /// centroidal momentum matrix (data.Ag), a forward geometry
  ///       and the joint jacobian matrix (data.J).
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] convention Convention to use.
  ///
  /// \return The joint space inertia matrix with only the upper triangular part computed.
  ///
  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType>
  const typename DataTpl<Scalar, Options, JointCollectionTpl>::MatrixXs & crba(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const Convention convention = Convention::LOCAL);

  PINOCCHIO_DEFINE_ALGO_CHECKER(CRBA);

} // namespace pinocchio

/* --- Details -------------------------------------------------------------------- */
#include "pinocchio/algorithm/crba.hxx"

#if PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
  #include "pinocchio/algorithm/crba.txx"
#endif // PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION

#endif // ifndef __pinocchio_crba_hpp__
