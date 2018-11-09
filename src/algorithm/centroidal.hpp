//
// Copyright (c) 2015-2018 CNRS INRIA
//

#ifndef __pinocchio_centroidal_hpp__
#define __pinocchio_centroidal_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/check.hpp"

namespace pinocchio
{
  
  ///
  /// \brief Computes the Centroidal dynamics, a.k.a. the total momenta of the system
  ///        expressed around the center of mass.
  ///
  /// \tparam Scalar The scalar type.
  /// \tparam Options Eigen Alignment options.
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  /// \tparam TangentVectorType Type of the joint velocity vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] v The joint velocity vector (dim model.nv).
  ///
  /// \returns The centroidal momenta (stored in data.hg).
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl,
          typename ConfigVectorType, typename TangentVectorType>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::Force &
  computeCentroidalDynamics(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                            DataTpl<Scalar,Options,JointCollectionTpl> & data,
                            const Eigen::MatrixBase<ConfigVectorType> & q,
                            const Eigen::MatrixBase<TangentVectorType> & v);
  ///
  /// \brief Computes the Centroidal dynamics and its time derivatives, a.k.a. the total momenta of the system and its time derivative
  ///        expressed around the center of mass.
  ///
  /// \tparam Scalar The scalar type.
  /// \tparam Options Eigen Alignment options.
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  /// \tparam TangentVectorType1 Type of the joint velocity vector.
  /// \tparam TangentVectorType2 Type of the joint acceleration vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] v The joint velocity vector (dim model.nv).
  /// \param[in] a The joint acceleration vector (dim model.nv).
  ///
  /// \returns The centroidal momenta time derivative (stored in data.dhg).
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl,
          typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::Force &
  computeCentroidalDynamics(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                            DataTpl<Scalar,Options,JointCollectionTpl> & data,
                            const Eigen::MatrixBase<ConfigVectorType> & q,
                            const Eigen::MatrixBase<TangentVectorType1> & v,
                            const Eigen::MatrixBase<TangentVectorType2> & a);
  
  ///
  /// \brief Computes the Centroidal Momentum Matrix, the Composite Ridig Body Inertia as well as the centroidal momenta
  ///        according to the current joint configuration and velocity.
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
  /// \return The Centroidal Momentum Matrix Ag.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::Matrix6x &
  ccrba(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
        DataTpl<Scalar,Options,JointCollectionTpl> & data,
        const Eigen::MatrixBase<ConfigVectorType> & q,
        const Eigen::MatrixBase<TangentVectorType> & v);
  
  ///
  /// \brief Computes the time derivative of the Centroidal Momentum Matrix according to the current configuration and velocity vectors.
  ///
  /// \note The computed terms allow to decomposed the spatial momentum variation as following: \f$ \dot{h} = A_g \ddot{q} + \dot{A_g}(q,\dot{q})\dot{q}\f$.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  /// \tparam TangentVectorType Type of the joint velocity vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] v The joint configuration vector (dim model.nv).
  ///
  /// \return The Centroidal Momentum Matrix time derivative dAg
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::Matrix6x &
  dccrba(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
         DataTpl<Scalar,Options,JointCollectionTpl> & data,
         const Eigen::MatrixBase<ConfigVectorType> & q,
         const Eigen::MatrixBase<TangentVectorType> & v);
  
} // namespace pinocchio

/* --- Details -------------------------------------------------------------------- */
#include "pinocchio/algorithm/centroidal.hxx"

#endif // ifndef __pinocchio_centroidal_hpp__
