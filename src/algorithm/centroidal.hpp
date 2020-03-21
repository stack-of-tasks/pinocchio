//
// Copyright (c) 2015-2019 CNRS INRIA
//

#ifndef __pinocchio_algorithm_centroidal_hpp__
#define __pinocchio_algorithm_centroidal_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

namespace pinocchio
{

  ///
  /// \brief Computes the Centroidal momentum, a.k.a. the total momenta of the system
  ///        expressed around the center of mass.
  ///
  /// \tparam Scalar The scalar type.
  /// \tparam Options Eigen Alignment options.
  /// \tparam JointCollection Collection of Joint types.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  ///
  /// \returns The centroidal momenta (stored in data.hg).
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::Force &
  computeCentroidalMomentum(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                            DataTpl<Scalar,Options,JointCollectionTpl> & data);
  
  ///
  /// \brief Computes the Centroidal momentum, a.k.a. the total momenta of the system
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
  computeCentroidalMomentum(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                            DataTpl<Scalar,Options,JointCollectionTpl> & data,
                            const Eigen::MatrixBase<ConfigVectorType> & q,
                            const Eigen::MatrixBase<TangentVectorType> & v)
  {
    forwardKinematics(model,data,q.derived(),v.derived());
    return computeCentroidalMomentum(model,data);
  }
  
  /// \copydoc pinocchio::computeCentroidalMomentum
  ///
  /// \deprecated This function has been renamed into \ref computeCentroidalMomentum. This signature will be removed in a future release of Pinocchio.
  ///        Please consider using this new naming.
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl,
  typename ConfigVectorType, typename TangentVectorType>
  PINOCCHIO_DEPRECATED
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::Force &
  computeCentroidalDynamics(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                            DataTpl<Scalar,Options,JointCollectionTpl> & data,
                            const Eigen::MatrixBase<ConfigVectorType> & q,
                            const Eigen::MatrixBase<TangentVectorType> & v)
  {
    return computeCentroidalMomentum(model,data,q,v);
  }

  ///
  /// \brief Computes the Centroidal momemtum and its time derivatives, a.k.a. the total momenta of the system and its time derivative
  ///        expressed around the center of mass.
  ///
  /// \tparam Scalar The scalar type.
  /// \tparam Options Eigen Alignment options.
  /// \tparam JointCollection Collection of Joint types.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  ///
  /// \returns The centroidal momenta time derivative (stored in data.dhg). The centroidal momemta is stored in data.hg.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::Force &
  computeCentroidalMomentumTimeVariation(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                         DataTpl<Scalar,Options,JointCollectionTpl> & data);

  ///
  /// \brief Computes the Centroidal momemtum and its time derivatives, a.k.a. the total momenta of the system and its time derivative
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
  /// \returns The centroidal momenta time derivative (stored in data.dhg). The centroidal momemta is stored in data.hg.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl,
          typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::Force &
  computeCentroidalMomentumTimeVariation(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                         DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                         const Eigen::MatrixBase<ConfigVectorType> & q,
                                         const Eigen::MatrixBase<TangentVectorType1> & v,
                                         const Eigen::MatrixBase<TangentVectorType2> & a)
  {
    forwardKinematics(model,data,q,v,a);
    return computeCentroidalMomentumTimeVariation(model,data);
  }

  /// \copydoc pinocchio::computeCentroidalMomentumTimeVariation
  ///
  /// \deprecated This function has been renamed into \ref computeCentroidalMomentumTimeVariation. This signature will be removed in a future release of Pinocchio.
  ///        Please consider using this new naming.
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl,
          typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2>
  PINOCCHIO_DEPRECATED
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::Force &
  computeCentroidalDynamics(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                            DataTpl<Scalar,Options,JointCollectionTpl> & data,
                            const Eigen::MatrixBase<ConfigVectorType> & q,
                            const Eigen::MatrixBase<TangentVectorType1> & v,
                            const Eigen::MatrixBase<TangentVectorType2> & a)
  {
    return computeCentroidalMomentumTimeVariation(model,data,q,v,a);
  }

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
  /// \remarks As another output, this algorithm also computes the Joint Jacobian matrix (accessible via data.J).
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::Matrix6x &
  ccrba(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
        DataTpl<Scalar,Options,JointCollectionTpl> & data,
        const Eigen::MatrixBase<ConfigVectorType> & q,
        const Eigen::MatrixBase<TangentVectorType> & v);

  ///
  /// \brief Computes the Centroidal Momentum Matrix,.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  ///
  /// \return The Centroidal Momentum Matrix Ag.
  ///
  /// \remarks As another output, this algorithm also computes the Joint Jacobian matrix (accessible via data.J).
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::Matrix6x &
  computeCentroidalMap(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                       DataTpl<Scalar,Options,JointCollectionTpl> & data,
                       const Eigen::MatrixBase<ConfigVectorType> & q);
  
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
  /// \param[in] v The joint velocity vector (dim model.nv).
  ///
  /// \return The Centroidal Momentum Matrix time derivative dAg (accessible via data.dAg).
  ///
  /// \remarks As another output, this algorithm also computes the Centroidal Momentum Matrix Ag (accessible via data.Ag), the Joint Jacobian matrix (accessible via data.J) and the time derivatibe of the Joint Jacobian matrix (accessible via data.dJ).
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::Matrix6x &
  dccrba(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
         DataTpl<Scalar,Options,JointCollectionTpl> & data,
         const Eigen::MatrixBase<ConfigVectorType> & q,
         const Eigen::MatrixBase<TangentVectorType> & v);

  ///
  /// \brief Computes the Centroidal Momentum Matrix time derivative.
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
  /// \return The Centroidal Momentum Matrix time derivative dAg (accessible via data.dAg).
  ///
  /// \remarks As another output, this algorithm also computes the Centroidal Momentum Matrix Ag (accessible via data.Ag), the Joint Jacobian matrix (accessible via data.J) and the time derivatibe of the Joint Jacobian matrix (accessible via data.dJ).
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::Matrix6x &
  computeCentroidalMapTimeVariation(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                    DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                    const Eigen::MatrixBase<ConfigVectorType> & q,
                                    const Eigen::MatrixBase<TangentVectorType> & v);
  
} // namespace pinocchio

/* --- Details -------------------------------------------------------------------- */
#include "pinocchio/algorithm/centroidal.hxx"

#endif // ifndef __pinocchio_algorithm_centroidal_hpp__
