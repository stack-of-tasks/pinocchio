//
// Copyright (c) 2015-2018 CNRS
//

#ifndef __pinocchio_frames_hpp__
#define __pinocchio_frames_hpp__

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
   * @warning    One of the algorithms forwardKinematics should have been called first
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
                       const typename ModelTpl<Scalar,Options,JointCollectionTpl>::FrameIndex frame_id);


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
   * @brief      Returns the spatial velocity of the frame expressed in the LOCAL frame coordinate system.
   *             You must first call pinocchio::forwardKinematics to update placement and velocity values in data structure.
   *
   * @param[in]  model       The kinematic model
   * @param[in]  data        Data associated to model
   * @param[in]  frame_id    Id of the operational Frame
   *
   * @return     The spatial velocity of the Frame expressed in the coordinates Frame.
   *
   * @warning    Fist or second order forwardKinematics should have been called first
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline MotionTpl<Scalar, Options>
  getFrameVelocity(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                   const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                   const typename ModelTpl<Scalar,Options,JointCollectionTpl>::FrameIndex frame_id);

  /**
   * @brief      Returns the spatial velocity of the frame expressed in the LOCAL frame coordinate system.
   *             You must first call pinocchio::forwardKinematics to update placement and velocity values in data structure.
   *
   * @param[in]  model       The kinematic model
   * @param[in]  data        Data associated to model
   * @param[in]  frame_id    Id of the operational Frame
   * @param[out] frame_v     The spatial velocity of the Frame expressed in the coordinates Frame.
   *
   * @deprecated This function is now deprecated. Use the return-value version instead (since: 19 feb 2019)
   *
   * @warning    Fist or second order forwardKinematics should have been called first
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename MotionLike>
  PINOCCHIO_DEPRECATED
  inline void getFrameVelocity(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                               const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                               const typename ModelTpl<Scalar,Options,JointCollectionTpl>::FrameIndex frame_id,
                               const MotionDense<MotionLike> & frame_v)
  {
    frame_v.derived() = getFrameVelocity(model, data, frame_id);
  }

  /**
   * @brief      Returns the spatial acceleration of the frame expressed in the LOCAL frame coordinate system.
   *             You must first call pinocchio::forwardKinematics to update placement values in data structure.
   *
   * @param[in]  model       The kinematic model
   * @param[in]  data        Data associated to model
   * @param[in]  frame_id    Id of the operational Frame
   *
   * @return The spatial acceleration of the Frame expressed in the coordinates Frame.
   *
   * @warning    Second order forwardKinematics should have been called first
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline MotionTpl<Scalar, Options>
  getFrameAcceleration(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                       const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                       const typename ModelTpl<Scalar,Options,JointCollectionTpl>::FrameIndex frame_id);

  /**
   * @brief      Returns the spatial acceleration of the frame expressed in the LOCAL frame coordinate system.
   *             You must first call pinocchio::forwardKinematics to update placement values in data structure.
   *
   * @param[in]  model       The kinematic model
   * @param[in]  data        Data associated to model
   * @param[in]  frame_id    Id of the operational Frame
   * @param[out] frame_a     The spatial acceleration of the Frame expressed in the coordinates Frame.
   *
   * @deprecated This function is now deprecated. Use the return-value version instead (since: 19 feb 2019)
   *
   * @warning    Second order forwardKinematics should have been called first
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename MotionLike>
  PINOCCHIO_DEPRECATED
  inline void getFrameAcceleration(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                   const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                   const typename ModelTpl<Scalar,Options,JointCollectionTpl>::FrameIndex frame_id,
                                   const MotionDense<MotionLike> & frame_a)
  { frame_a.derived() = getFrameAcceleration(model, data, frame_id); }

  /**
   * @brief      Returns the jacobian of the frame expressed either expressed in the LOCAL frame coordinate system or in the WORLD coordinate system,
   *             depending on the value of rf.
   *             You must first call pinocchio::computeJointJacobians followed by pinocchio::framesForwardKinematics to update placement values in data structure.
   *
   * @remark     Similarly to pinocchio::getJointJacobian with LOCAL or WORLD parameters, if rf == LOCAL, this function returns the Jacobian of the frame expressed
   *             in the local coordinates of the frame, or if rl == WORDL, it returns the Jacobian expressed of the point coincident with the origin
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
   * @warning    The function pinocchio::computeJointJacobians and pinocchio::framesForwardKinematics should have been called first.
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename Matrix6xLike>
  inline void getFrameJacobian(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                               const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                               const typename ModelTpl<Scalar,Options,JointCollectionTpl>::FrameIndex frame_id,
                               const ReferenceFrame rf,
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
  /// \param[out] J A reference on the Jacobian matrix where the results will be stored in (dim 6 x model.nv). You must fill J with zero elements, e.g. J.setZero().
  ///
  /// \return The Jacobian of the specific Frame expressed in the LOCAL frame coordinate system (matrix 6 x model.nv).
  ///
  /// \remark The result of this function is equivalent to call first computeJointJacobians(model,data,q), then updateFramePlacements(model,data) and then call getJointJacobian(model,data,jointId,LOCAL,J),
  ///         but forwardKinematics and updateFramePlacements are not fully computed.
  ///         It is worth to call jacobian if you only need a single Jacobian for a specific joint. Otherwise, for several Jacobians, it is better
  ///         to call computeJacobians(model,data,q) followed by getJointJacobian(model,data,jointId,LOCAL,J) for each Jacobian.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename Matrix6Like>
  inline void frameJacobian(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                            DataTpl<Scalar,Options,JointCollectionTpl> & data,
                            const Eigen::MatrixBase<ConfigVectorType> & q,
                            const FrameIndex frameId,
                            const Eigen::MatrixBase<Matrix6Like> & J);

  /**
   * @brief      Returns the jacobian of the frame expresssed either expressed in the LOCAL frame coordinate system or in the WORLD coordinate system,
   *             depending on the value of rf.
   *             You must first call pinocchio::computeJointJacobians followed by pinocchio::updateFramePlacements to update placement values in data structure.
   *
   * @tparam     rf Reference frame in which the columns of the Jacobian are expressed.
   * @deprecated This function is now deprecated. Please call pinocchio::getFrameJacobian for same functionality.
   *
   * @remark     Similarly to pinocchio::getJointJacobian with LOCAL or WORLD parameters, if rf == LOCAL, this function returns the Jacobian of the frame expressed
   *             in the local coordinates of the frame, or if rl == WORDL, it returns the Jacobian expressed of the point coincident with the origin
   *             and expressed in a coordinate system aligned with the WORLD.
   *
   * @param[in]  model       The kinematic model
   * @param[in]  data        Data associated to model
   * @param[in]  frame_id    Id of the operational Frame
   * @param[in] rf Reference frame in which the Jacobian is expressed.
   * @param[out] J           The Jacobian of the Frame expressed in the coordinates Frame.
   *
   * @warning    The functions pinocchio::computeJointJacobians and pinocchio::updateFramePlacements should have been called first.
   */
  template<ReferenceFrame rf>
  PINOCCHIO_DEPRECATED
  void getFrameJacobian(const Model & model,
                        const Data & data,
                        const Model::FrameIndex frame_id,
                        Data::Matrix6x & J)
  { getFrameJacobian(model,data,frame_id,rf,J); }
  
  /**
   * @brief      Returns the jacobian of the frame expresssed in the LOCAL coordinate system of the frame.
   *             You must first call pinocchio::computeJointJacobians followed by pinocchio::updateFramePlacements to update placement values in data structure.
   * @deprecated This function is now deprecated. Please call pinocchio::getFrameJacobian for same functionality
   *
   * @tparam JointCollection Collection of Joint types.
   * @tparam Matrix6xLike Type of the matrix containing the joint Jacobian.
   *
   * @param[in]  model       The kinematic model
   * @param[in]  data        Data associated to model
   * @param[in]  frame_id    Id of the operational Frame
   * @param[out] J           The Jacobian of the Frame expressed in the coordinates Frame.
   *
   * @warning    The function pinocchio::computeJointJacobians and pinocchio::updateFramePlacements should have been called first.
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename Matrix6xLike>
  PINOCCHIO_DEPRECATED
  inline void getFrameJacobian(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                               const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                               const typename ModelTpl<Scalar,Options,JointCollectionTpl>::FrameIndex frame_id,
                               const Eigen::MatrixBase<Matrix6xLike> & J);
  
  ///
  /// \brief Computes the Jacobian time variation of a specific frame (given by frame_id) expressed either in the world frame (rf = WORLD) or in the local frame (rf = LOCAL).
  /// \note This jacobian is extracted from data.dJ. You have to run pinocchio::computeJacobiansTimeVariation before calling it.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam Matrix6xLike Type of the matrix containing the joint Jacobian.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] frameId The index of the frame.
  /// \param[in] rf Reference frame in which the Jacobian is expressed.
  /// \param[out] dJ A reference on the Jacobian matrix where the results will be stored in (dim 6 x model.nv). You must fill dJ with zero elements, e.g. dJ.fill(0.).
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename Matrix6xLike>
  void getFrameJacobianTimeVariation(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                     const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                     const typename ModelTpl<Scalar,Options,JointCollectionTpl>::FrameIndex frame_id,
                                     const ReferenceFrame rf,
                                     const Eigen::MatrixBase<Matrix6xLike> & dJ);

  ///
  /// \brief Computes the Jacobian time variation of a specific frame (given by frame_id) expressed either in the world frame (rf = WORLD) or in the local frame (rf = LOCAL).
  /// \note This jacobian is extracted from data.dJ. You have to run pinocchio::computeJacobiansTimeVariation before calling it.
  /// \deprecated This function is now deprecated. Please call pinocchio::getFrameJacobianTimeVariation for same functionality
  ///
  /// \tparam rf Reference frame in which the Jacobian is expressed.
  ///
  /// \param[in] localFrame Expressed the Jacobian in the local frame or world frame coordinates system.
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] frameId The index of the frame.
  /// \param[out] dJ A reference on the Jacobian matrix where the results will be stored in (dim 6 x model.nv). You must fill dJ with zero elements, e.g. dJ.fill(0.).
  ///
  template<ReferenceFrame rf>
  PINOCCHIO_DEPRECATED
  void getFrameJacobianTimeVariation(const Model & model,
                                     const Data & data,
                                     const Model::FrameIndex frameId,
                                     Data::Matrix6x & dJ)
  {
    getFrameJacobianTimeVariation(model,data,frameId,rf,dJ);
  }

} // namespace pinocchio

/* --- Details -------------------------------------------------------------------- */
#include "pinocchio/algorithm/frames.hxx"

#endif // ifndef __pinocchio_frames_hpp__
