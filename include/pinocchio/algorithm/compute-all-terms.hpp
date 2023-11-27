//
// Copyright (c) 2015-2021 CNRS INRIA
//

#ifndef __pinocchio_compute_all_terms_hpp__
#define __pinocchio_compute_all_terms_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

namespace pinocchio
{
  ///
  /// \brief Computes efficiently all the terms needed for dynamic simulation. It is equivalent to the call at the same time to:
  ///         - pinocchio::forwardKinematics
  ///         - pinocchio::crba
  ///         - pinocchio::nonLinearEffects
  ///         - pinocchio::computeJointJacobians
  ///         - pinocchio::centerOfMass
  ///         - pinocchio::jacobianCenterOfMass
  ///         - pinocchio::ccrba
  ///         - pinocchio::computeKineticEnergy
  ///         - pinocchio::computePotentialEnergy
  ///         - pinocchio::computeGeneralizedGravity
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  /// \tparam TangentVectorType Type of the joint velocity vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] v The joint velocity vector (dim model.nv).
  ///
  /// \return All the results are stored in data. Please refer to the specific algorithm for further details.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType>
  inline void computeAllTerms(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                              DataTpl<Scalar,Options,JointCollectionTpl> & data,
                              const Eigen::MatrixBase<ConfigVectorType> & q,
                              const Eigen::MatrixBase<TangentVectorType> & v);

} // namespace pinocchio

#include "pinocchio/algorithm/compute-all-terms.hxx"

#endif // ifndef __pinocchio_compute_all_terms_hpp__
