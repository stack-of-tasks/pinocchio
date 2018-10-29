//
// Copyright (c) 2015-2018 CNRS
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.

#ifndef __se3_crba_hpp__
#define __se3_crba_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/check.hpp"
  
namespace se3
{
  ///
  /// \brief Computes the upper triangular part of the joint space inertia matrix M by
  ///        using the Composite Rigid Body Algorithm (Chapter 6, Rigid-Body Dynamics Algorithms, R. Featherstone, 2008).
  ///        The result is accessible through data.M.
  ///
  /// \note You can easly get data.M symetric by copying the stricly upper trinangular part
  ///       in the stricly lower tringular part with
  ///       data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  ///
  /// \return The joint space inertia matrix with only the upper triangular part computed.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::MatrixXs &
  crba(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
       DataTpl<Scalar,Options,JointCollectionTpl> & data,
       const Eigen::MatrixBase<ConfigVectorType> & q);
  
  ///
  /// \brief Computes the upper triangular part of the joint space inertia matrix M by
  ///        using the Composite Rigid Body Algorithm (Chapter 6, Rigid-Body Dynamics Algorithms, R. Featherstone, 2008).
  ///        The result is accessible through data.M.
  ///
  /// \note You can easly get data.M symetric by copying the stricly upper trinangular part
  ///       in the stricly lower tringular part with
  ///       data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  ///
  /// \note A direct outcome of this algorithm is the computation of the centroidal momemntum matrix (data.Ag)
  ///       and the joint jacobian matrix (data.J).
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  ///
  /// \return The joint space inertia matrix with only the upper triangular part computed.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::MatrixXs &
  crbaMinimal(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
              DataTpl<Scalar,Options,JointCollectionTpl> & data,
              const Eigen::MatrixBase<ConfigVectorType> & q);

  DEFINE_ALGO_CHECKER(CRBA);

} // namespace se3 

/* --- Details -------------------------------------------------------------------- */
#include "pinocchio/algorithm/crba.hxx"

#endif // ifndef __se3_crba_hpp__
