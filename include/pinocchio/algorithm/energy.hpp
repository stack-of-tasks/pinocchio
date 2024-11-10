//
// Copyright (c) 2016-2020 CNRS INRIA
//

#ifndef __pinocchio_algorithm_energy_hpp__
#define __pinocchio_algorithm_energy_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/check.hpp"

namespace pinocchio
{

  ///
  /// \brief Computes the kinetic energy of the system.
  ///        The result is accessible through data.kinetic_energy.
  ///
  /// \tparam JointCollection Collection of Joint types.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  ///
  /// \return The kinetic energy of the system in [J].
  ///
  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  Scalar computeKineticEnergy(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data);

  ///
  /// \brief Computes the kinetic energy of the system.
  ///        The result is accessible through data.kinetic_energy.
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
  /// \return The kinetic energy of the system in [J].
  ///
  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    typename TangentVectorType>
  Scalar computeKineticEnergy(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const Eigen::MatrixBase<TangentVectorType> & v);

  ///
  /// \brief Computes the potential energy of the system, i.e. the potential energy linked to the
  /// gravity field.
  ///        The result is accessible through data.potential_energy.
  ///
  /// \tparam JointCollection Collection of Joint types.
  ///
  /// \note This potential energy are of the for \f$ \sum_{i} - m_{i}gh_{i} \f$ where:
  ///       -  \f$ m_{i} \f$ is the mass of the body \f$ i \f$,
  ///       -  \f$ h_{i} \f$ is the height of the body \f$ i \f$,
  ///       -  \f$ g \f$ is the gravity value.
  ///
  /// \tparam JointCollection Collection of Joint types.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  ///
  /// \return The potential energy of the system expressed in [J].
  ///
  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  Scalar computePotentialEnergy(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data);

  ///
  /// \brief Computes the potential energy of the system, i.e. the potential energy linked to the
  /// gravity field.
  ///        The result is accessible through data.potential_energy.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  ///
  /// \note This potential energy are of the for \f$ \sum_{i} - m_{i}gh_{i} \f$ where:
  ///       -  \f$ m_{i} \f$ is the mass of the body \f$ i \f$,
  ///       -  \f$ h_{i} \f$ is the height of the body \f$ i \f$,
  ///       -  \f$ g \f$ is the gravity value.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  ///
  /// \return The potential energy of the system expressed in [J].
  ///
  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType>
  Scalar computePotentialEnergy(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q);

  ///
  /// \brief Computes the mechanical energy of the system stored in data.mechanical_energy.
  ///        The result is accessible through data.kinetic_energy.
  ///
  /// \tparam JointCollection Collection of Joint types.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  ///
  /// \return The total mechanal energy of the system in [J].
  ///
  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  Scalar computeMechanicalEnergy(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data);

  ///
  /// \brief Computes the mechanical energy of the system stored in data.mechanical_energy.
  ///        The result is accessible through data.kinetic_energy.
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
  /// \return The total mechanal energy of the system in [J].
  ///         The fonctions also computes the data.kinetic_energy and data.potential_energy.
  ///
  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    typename TangentVectorType>
  Scalar computeMechanicalEnergy(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const Eigen::MatrixBase<TangentVectorType> & v);

} // namespace pinocchio

#include "pinocchio/algorithm/energy.hxx"

#if PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
  #include "pinocchio/algorithm/energy.txx"
#endif // PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION

#endif // __pinocchio_algorithm_energy_hpp__
