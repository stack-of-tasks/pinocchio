//
// Copyright (c) 2015-2020 CNRS INRIA
//

#ifndef __pinocchio_algorithm_frames_hpp__
#define __pinocchio_algorithm_frames_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

namespace pinocchio
{

  /**
   * @brief      Updates the position of each frame contained in the model.
   *
   * @tparam JointCollection Collection of Joint types.
   *
   * @param[in]  model  The kinematic model.
   * @param      data   Data associated to model.
   *
   * @warning    One of the algorithms forwardKinematics should have been called first.
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline void updateFramePlacements(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                    DataTpl<Scalar,Options,JointCollectionTpl> & data);

  /**
   * @brief      Updates the placement of the given frame.
   *
   * @param[in]  model        The kinematic model.
   * @param      data         Data associated to model.
   * @param[in]  frame_id     Id of the operational Frame.
   *
   * @return     A reference to the frame placement stored in data.oMf[frame_id]
   *
   * @warning    One of the algorithms forwardKinematics should have been called first
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::SE3 &
  updateFramePlacement(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                       DataTpl<Scalar,Options,JointCollectionTpl> & data,
                       const FrameIndex frame_id);


  /**
   * @brief      First calls the forwardKinematics on the model, then computes the placement of each frame.
   *             /sa pinocchio::forwardKinematics.
   *
   * @tparam JointCollection Collection of Joint types.
   * @tparam ConfigVectorType Type of the joint configuration vector.
   *
   * @param[in]  model                    The kinematic model.
   * @param      data                     Data associated to model.
   * @param[in]  q                        Configuration vector.
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
  inline void framesForwardKinematics(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                      DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                      const Eigen::MatrixBase<ConfigVectorType> & q);

  
  /**
   * @brief      Updates the position of each frame contained in the model.
   *             This function is now deprecated and has been renamed updateFramePlacements.
   *
   * @tparam JointCollection Collection of Joint types.
   *
   * @param[in]  model  The kinematic model.
   * @param      data   Data associated to model.
   *
   * @warning    One of the algorithms forwardKinematics should have been called first.
  */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  PINOCCHIO_DEPRECATED
  inline void framesForwardKinematics(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                      DataTpl<Scalar,Options,JointCollectionTpl> & data)
  {
    updateFramePlacements(model,data);
  }


  /**
   * @brief      Returns the spatial velocity of the Frame expressed in the desired reference frame.
   *             You must first call pinocchio::forwardKinematics to update placement and velocity values in data structure.
   *
   * @param[in]  model       The kinematic model
   * @param[in]  data        Data associated to model
   * @param[in]  frame_id    Id of the operational Frame
   * @param[in]  rf          Reference frame in which the velocity is expressed.
   *
   * @return     The spatial velocity of the Frame expressed in the desired reference frame.
   *
   * @warning    Fist or second order forwardKinematics should have been called first
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline MotionTpl<Scalar, Options>
  getFrameVelocity(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                   const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                   const FrameIndex frame_id,
                   const ReferenceFrame rf = LOCAL);

  /**
   * @brief      Returns the spatial acceleration of the Frame expressed in the desired reference frame.
   *             You must first call pinocchio::forwardKinematics to update placement, velocity and acceleration values in data structure.
   *
   * @param[in]  model       The kinematic model
   * @param[in]  data        Data associated to model
   * @param[in]  frame_id    Id of the operational Frame
   * @param[in]  rf          Reference frame in which the acceleration is expressed.
   *
   * @return The spatial acceleration of the Frame expressed in the desired reference frame.
   *
   * @warning    Second order forwardKinematics should have been called first
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline MotionTpl<Scalar, Options>
  getFrameAcceleration(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                       const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                       const FrameIndex frame_id,
                       const ReferenceFrame rf = LOCAL);

  /**
   * @brief      Returns the "classical" acceleration of the Frame expressed in the desired reference frame.
   *             This is different from the "spatial" acceleration in that centrifugal effects are accounted for.
   *             You must first call pinocchio::forwardKinematics to update placement, velocity and acceleration values in data structure.
   *
   * @param[in]  model       The kinematic model
   * @param[in]  data        Data associated to model
   * @param[in]  frame_id    Id of the operational Frame
   * @param[in]  rf          Reference frame in which the acceleration is expressed.
   *
   * @return The classical acceleration of the Frame expressed in the desired reference frame.
   *
   * @warning    Second order forwardKinematics should have been called first
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline MotionTpl<Scalar, Options>
  getFrameClassicalAcceleration(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                const FrameIndex frame_id,
                                const ReferenceFrame rf = LOCAL);

  /**
   * @brief      Returns the jacobian of the frame expressed either expressed in the LOCAL frame coordinate system or in the WORLD coordinate system,
   *             depending on the value of rf.
   *             You must first call pinocchio::computeJointJacobians followed by pinocchio::framesForwardKinematics to update placement values in data structure.
   *
   * @remark     Similarly to pinocchio::getJointJacobian with LOCAL or WORLD parameters, if rf == LOCAL, this function returns the Jacobian of the frame expressed
   *             in the local coordinates of the frame, or if rl == WORLD, it returns the Jacobian expressed of the point coincident with the origin
   *             and expressed in a coordinate system aligned with the WORLD.
   *
   * @tparam JointCollection Collection of Joint types.
   * @tparam Matrix6xLike Type of the matrix containing the joint Jacobian.
   *
   * @param[in]  model       The kinematic model
   * @param[in]  data        Data associated to model
   * @param[in]  frame_id    Id of the operational Frame
   * @param[in]  rf          Reference frame in which the Jacobian is expressed.
   * @param[out] J           The Jacobian of the Frame expressed in the coordinates Frame.
   *
   * @warning    The function pinocchio::computeJointJacobians should have been called first.
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename Matrix6xLike>
  inline void getFrameJacobian(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                               DataTpl<Scalar,Options,JointCollectionTpl> & data,
                               const FrameIndex frame_id,
                               const ReferenceFrame rf,
                               const Eigen::MatrixBase<Matrix6xLike> & J);
  
  ///
  /// \brief Computes the Jacobian of a specific Frame expressed in the desired reference_frame given as argument.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  /// \tparam Matrix6xLike Type of the matrix containing the joint Jacobian.
  ///
  /// \param[in] model                                The model structure of the rigid body system.
  /// \param[in] data                                  The data structure of the rigid body system.
  /// \param[in] q                                         The joint configuration vector (dim model.nq).
  /// \param[in] frameId                            The id of the Frame refering to model.frames[frameId].
  /// \param[in] reference_frame          Reference frame in which the Jacobian is expressed.
  /// \param[out] J                                       A reference on the Jacobian matrix where the results will be stored in (dim 6 x model.nv). You must fill J with zero elements, e.g. J.setZero().
  ///
  /// \return The Jacobian of the specific Frame expressed in the desired reference frame (matrix 6 x model.nv).
  ///
  /// \remarks The result of this function is equivalent to call first computeJointJacobians(model,data,q), then updateFramePlacements(model,data) and then call getJointJacobian(model,data,jointId,rf,J),
  ///         but forwardKinematics and updateFramePlacements are not fully computed.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename Matrix6xLike>
  inline void computeFrameJacobian(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                   DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                   const Eigen::MatrixBase<ConfigVectorType> & q,
                                   const FrameIndex frameId,
                                   const ReferenceFrame reference_frame,
                                   const Eigen::MatrixBase<Matrix6xLike> & J);

  ///
  /// \brief Computes the Jacobian of a specific Frame expressed in the LOCAL frame coordinate system.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  /// \tparam Matrix6xLike Type of the matrix containing the joint Jacobian.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] frameId The id of the Frame refering to model.frames[frameId].
  ///
  /// \param[out] J A reference on the Jacobian matrix where the results will be stored in (dim 6 x model.nv). You must fill J with zero elements, e.g. J.setZero().
  ///
  /// \return The Jacobian of the specific Frame expressed in the LOCAL frame coordinate system (matrix 6 x model.nv).
  ///
  /// \remarks The result of this function is equivalent to call first computeJointJacobians(model,data,q), then updateFramePlacements(model,data) and then call getJointJacobian(model,data,jointId,LOCAL,J),
  ///         but forwardKinematics and updateFramePlacements are not fully computed.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename Matrix6xLike>
  inline void computeFrameJacobian(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                   DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                   const Eigen::MatrixBase<ConfigVectorType> & q,
                                   const FrameIndex frameId,
                                   const Eigen::MatrixBase<Matrix6xLike> & J)
  {
    computeFrameJacobian(model,data,q.derived(),frameId,LOCAL,
                         PINOCCHIO_EIGEN_CONST_CAST(Matrix6xLike,J));
  }
                        
  ///
  /// \brief This function is now deprecated and has been renamed computeFrameJacobian. This signature will be removed in future release of Pinocchio.
  ///
  /// \copydoc pinocchio::computeFrameJacobian
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename Matrix6xLike>
  PINOCCHIO_DEPRECATED
  inline void frameJacobian(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                            DataTpl<Scalar,Options,JointCollectionTpl> & data,
                            const Eigen::MatrixBase<ConfigVectorType> & q,
                            const FrameIndex frameId,
                            const Eigen::MatrixBase<Matrix6xLike> & J)
  {
    computeFrameJacobian(model,data,q,frameId,PINOCCHIO_EIGEN_CONST_CAST(Matrix6xLike,J));
  }
  
  ///
  /// \brief Computes the Jacobian time variation of a specific frame (given by frame_id) expressed either in the WORLD frame (rf = WORLD) or in the LOCAL frame (rf = LOCAL) or in the LOCAL_WORLD_ALIGNED frame (rf = LOCAL_WORLD_ALIGNED).
  ///
  /// \note This jacobian is extracted from data.dJ. You have to run pinocchio::computeJointJacobiansTimeVariation before calling it.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam Matrix6xLike Type of the matrix containing the joint Jacobian.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] frameId The index of the frame.
  /// \param[in] rf Reference frame in which the Jacobian is expressed.
  ///
  /// \param[out] dJ A reference on the Jacobian matrix where the results will be stored in (dim 6 x model.nv). You must fill dJ with zero elements, e.g. dJ.fill(0.).
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename Matrix6xLike>
  void getFrameJacobianTimeVariation(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                     DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                     const FrameIndex frame_id,
                                     const ReferenceFrame rf,
                                     const Eigen::MatrixBase<Matrix6xLike> & dJ);

} // namespace pinocchio

/* --- Details -------------------------------------------------------------------- */
#include "pinocchio/algorithm/frames.hxx"

#endif // ifndef __pinocchio_algorithm_frames_hpp__
