//
// Copyright (c) 2015-2019 CNRS INRIA
//

#ifndef __pinocchio_kinematics_hpp__
#define __pinocchio_kinematics_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

namespace pinocchio
{
  
  ///
  /// \brief Update the global placement of the joints oMi according to the relative
  ///        placements of the joints.
  ///
  /// \tparam JointCollection Collection of Joint types.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  ///
  /// \remark This algorithm may be useful to call to update global joint placement
  ///         after calling pinocchio::rnea, pinocchio::aba, etc for example.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline void updateGlobalPlacements(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                     DataTpl<Scalar,Options,JointCollectionTpl> & data);
  
  ///
  /// \brief Update the joint placements according to the current joint configuration.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration (vector dim model.nq).
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
  inline void forwardKinematics(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                const Eigen::MatrixBase<ConfigVectorType> & q);

  ///
  /// \brief Update the joint placements and spatial velocities according to the current joint configuration and velocity.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  /// \tparam TangentVectorType Type of the joint velocity vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration (vector dim model.nq).
  /// \param[in] v The joint velocity (vector dim model.nv).
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType>
  inline void forwardKinematics(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                const Eigen::MatrixBase<ConfigVectorType> & q,
                                const Eigen::MatrixBase<TangentVectorType> & v);
  ///
  /// \brief Update the joint placements, spatial velocities and spatial accelerations according to the current joint configuration, velocity and acceleration.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  /// \tparam TangentVectorType1 Type of the joint velocity vector.
  /// \tparam TangentVectorType2 Type of the joint acceleration vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration (vector dim model.nq).
  /// \param[in] v The joint velocity (vector dim model.nv).
  /// \param[in] a The joint acceleration (vector dim model.nv).
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2>
  inline void forwardKinematics(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                const Eigen::MatrixBase<ConfigVectorType> & q,
                                const Eigen::MatrixBase<TangentVectorType1> & v,
                                const Eigen::MatrixBase<TangentVectorType2> & a);

  /**
   * @brief      Returns the spatial velocity of the joint expressed in the desired reference frame.
   *             You must first call pinocchio::forwardKinematics to update placement and velocity values in data structure.
   *
   * @param[in] model     The kinematic model
   * @param[in] data      Data associated to model
   * @param[in] jointId   Id of the joint
   * @param[in] rf        Reference frame in which the velocity is expressed.
   *
   * @return     The spatial velocity of the joint expressed in the desired reference frame.
   *
   * @warning    Fist or second order forwardKinematics should have been called first
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline MotionTpl<Scalar, Options>
  getVelocity(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
              const DataTpl<Scalar,Options,JointCollectionTpl> & data,
              const JointIndex jointId,
              const ReferenceFrame rf = LOCAL);

  /**
   * @brief      Returns the spatial acceleration of the joint expressed in the desired reference frame.
   *             You must first call pinocchio::forwardKinematics to update placement, velocity and acceleration values in data structure.
   *
   * @param[in] model     The kinematic model
   * @param[in] data      Data associated to model
   * @param[in] jointId   Id of the joint
   * @param[in] rf        Reference frame in which the acceleration is expressed.
   *
   * @return     The spatial acceleration of the joint expressed in the desired reference frame.
   *
   * @warning    Second order forwardKinematics should have been called first
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline MotionTpl<Scalar, Options>
  getAcceleration(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                  const JointIndex jointId,
                  const ReferenceFrame rf = LOCAL);

  /**
   * @brief      Returns the "classical" acceleration of the joint expressed in the desired reference frame.
   *             This is different from the "spatial" acceleration in that centrifugal effects are accounted for.
   *             You must first call pinocchio::forwardKinematics to update placement, velocity and acceleration values in data structure.
   *
   * @param[in] model     The kinematic model
   * @param[in] data      Data associated to model
   * @param[in] jointId   Id of the joint
   * @param[in] rf        Reference frame in which the acceleration is expressed.
   *
   * @return     The classic acceleration of the joint expressed in the desired reference frame.
   *
   * @warning    Second order forwardKinematics should have been called first
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline MotionTpl<Scalar, Options>
  getClassicalAcceleration(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                           const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                           const JointIndex jointId,
                           const ReferenceFrame rf = LOCAL);

} // namespace pinocchio 

/* --- Details -------------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------------- */
#include "pinocchio/algorithm/kinematics.hxx"


#endif // ifndef __pinocchio_kinematics_hpp__
