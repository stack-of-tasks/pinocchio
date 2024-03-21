//
// Copyright (c) 2016-2020 CNRS
//

#ifndef __pinocchio_algorithm_pv_hpp__
#define __pinocchio_algorithm_pv_hpp__


#include "pinocchio/algorithm/contact-info.hpp"

namespace pinocchio
{

  

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, class Allocator>
  inline void
  initPvSolver(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                      DataTpl<Scalar,Options,JointCollectionTpl> & data,
                      const std::vector<RigidConstraintModelTpl<Scalar,Options>,Allocator> & contact_models);
  

  ///
  /// \brief The Articulated-Body algorithm. It computes the forward dynamics, aka the joint accelerations given the current state and actuation of the model.
  /// This is the original implementation, considering all quantities to be expressed in the LOCAL coordinate systems of the joint frames.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  /// \tparam TangentVectorType1 Type of the joint velocity vector.
  /// \tparam TangentVectorType2 Type of the joint torque vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] v The joint velocity vector (dim model.nv).
  /// \param[in] tau The joint torque vector (dim model.nv).
  ///
  /// \note This also overwrites data.f, possibly leaving it in an inconsistent state
  ///
  /// \return The current joint acceleration stored in data.ddq.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2, class ContactModelAllocator, class ContactDataAllocator>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::TangentVectorType &
  pv(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
      DataTpl<Scalar,Options,JointCollectionTpl> & data,
      const Eigen::MatrixBase<ConfigVectorType> & q,
      const Eigen::MatrixBase<TangentVectorType1> & v,
      const Eigen::MatrixBase<TangentVectorType2> & tau,
      const std::vector<RigidConstraintModelTpl<Scalar,Options>,ContactModelAllocator> & contact_models,
      std::vector<RigidConstraintDataTpl<Scalar,Options>,ContactDataAllocator> & contact_datas,
      ProximalSettingsTpl<Scalar> & settings);

} // namespace pinocchio

/* --- Details -------------------------------------------------------------------- */
#include "pinocchio/algorithm/pv.hxx"

#endif // ifndef __pinocchio_algorithm_pv_hpp__
