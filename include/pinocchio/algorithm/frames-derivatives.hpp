//
// Copyright (c) 2020 INRIA
//

#ifndef __pinocchio_algorithm_frames_derivatives_hpp__
#define __pinocchio_algorithm_frames_derivatives_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

namespace pinocchio
{

  /**
   * @brief      Computes the partial derivatives of the spatial velocity of a frame given by its
   * relative placement, with respect to q and v. You must first call
   * pinocchio::computeForwardKinematicsDerivatives to compute all the required quantities.
   *
   * @tparam JointCollection Collection of Joint types.
   * @tparam Matrix6xOut1 Matrix6x containing the partial derivatives of the frame spatial velocity
   * with respect to the joint configuration vector.
   * @tparam Matrix6xOut2 Matrix6x containing the partial derivatives of the frame spatial velocity
   * with respect to the joint velocity vector.
   *
   * @param[in]  model                   The kinematic model
   * @param[in]  data                     Data associated to model
   * @param[in]  joint_id            Index of the supporting joint
   * @param[in]  placement          Placement of the Frame w.r.t. the joint frame.
   * @param[in]  rf                         Reference frame in which the velocity is expressed.
   * @param[out] v_partial_dq   Partial derivative of the frame spatial velocity w.r.t. \f$ q \f$.
   * @param[out] v_partial_dv   Partial derivative of the frame spatial velociy w.r.t. \f$ v \f$.
   *
   */
  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename Matrix6xOut1,
    typename Matrix6xOut2>
  void getFrameVelocityDerivatives(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const JointIndex joint_id,
    const SE3Tpl<Scalar, Options> & placement,
    const ReferenceFrame rf,
    const Eigen::MatrixBase<Matrix6xOut1> & v_partial_dq,
    const Eigen::MatrixBase<Matrix6xOut2> & v_partial_dv);

  /**
   * @brief      Computes the partial derivatives of the frame spatial velocity with respect to q
   * and v. You must first call pinocchio::computeForwardKinematicsDerivatives to compute all the
   * required quantities.
   *
   * @tparam JointCollection Collection of Joint types.
   * @tparam Matrix6xOut1 Matrix6x containing the partial derivatives of the frame spatial velocity
   * with respect to the joint configuration vector.
   * @tparam Matrix6xOut2 Matrix6x containing the partial derivatives of the frame spatial velocity
   * with respect to the joint velocity vector.
   *
   * @param[in]  model       The kinematic model
   * @param[in]  data        Data associated to model
   * @param[in]  frame_id    Id of the operational Frame
   * @param[in]  rf          Reference frame in which the velocity is expressed.
   * @param[out] v_partial_dq Partial derivative of the frame spatial velocity w.r.t. \f$ q \f$.
   * @param[out] v_partial_dv Partial derivative of the frame spatial velociy w.r.t. \f$ v \f$.
   *
   */
  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename Matrix6xOut1,
    typename Matrix6xOut2>
  void getFrameVelocityDerivatives(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const FrameIndex frame_id,
    const ReferenceFrame rf,
    const Eigen::MatrixBase<Matrix6xOut1> & v_partial_dq,
    const Eigen::MatrixBase<Matrix6xOut2> & v_partial_dv)
  {
    PINOCCHIO_CHECK_INPUT_ARGUMENT((int)frame_id < model.nframes, "The frame_id is not valid.");
    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;
    typedef typename Model::Frame Frame;

    const Frame & frame = model.frames[frame_id];
    typename Data::SE3 & oMframe = data.oMf[frame_id];
    oMframe = data.oMi[frame.parentJoint] * frame.placement; // for backward compatibility
    getFrameVelocityDerivatives(
      model, data, frame.parentJoint, frame.placement, rf,
      PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut1, v_partial_dq),
      PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut1, v_partial_dv));
  }

  /**
   * @brief      Computes the partial derivatives of the spatial acceleration of a frame given by
   * its relative placement, with respect to q, v and a. You must first call
   * pinocchio::computeForwardKinematicsDerivatives to compute all the required quantities. It is
   * important to notice that a direct outcome (for free) of this algo is v_partial_dq and
   * v_partial_dv which is equal to a_partial_da.
   *
   * @tparam JointCollection Collection of Joint types.
   * @tparam Matrix6xOut1 Matrix6x containing the partial derivatives of the frame spatial velocity
   * with respect to the joint configuration vector.
   * @tparam Matrix6xOut2 Matrix6x containing the partial derivatives of the frame spatial
   * acceleration with respect to the joint configuration vector.
   * @tparam Matrix6xOut3 Matrix6x containing the partial derivatives of the frame spatial
   * acceleration with respect to the joint velocity vector.
   * @tparam Matrix6xOut4 Matrix6x containing the partial derivatives of the frame spatial
   * acceleration with respect to the joint acceleration vector.
   *
   * @param[in]  model                   The kinematic model
   * @param[in]  data                     Data associated to model
   * @param[in]  joint_id            Index of the supporting joint
   * @param[in]  placement          Placement of the Frame w.r.t. the joint frame.
   * @param[in]  rf                          Reference frame in which the velocity is expressed.
   * @param[out] v_partial_dq    Partial derivative of the frame spatial velocity w.r.t. \f$ q \f$.
   * @param[out] a_partial_dq    Partial derivative of the frame spatial acceleration w.r.t. \f$ q
   * \f$.
   * @param[out] a_partial_dq    Partial derivative of the frame spatial acceleration w.r.t. \f$ v
   * \f$.
   * @param[out] a_partial_dq    Partial derivative of the frame spatial acceleration w.r.t. \f$
   * \dot{v} \f$.
   *
   */
  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename Matrix6xOut1,
    typename Matrix6xOut2,
    typename Matrix6xOut3,
    typename Matrix6xOut4>
  void getFrameAccelerationDerivatives(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const JointIndex joint_id,
    const SE3Tpl<Scalar, Options> & placement,
    const ReferenceFrame rf,
    const Eigen::MatrixBase<Matrix6xOut1> & v_partial_dq,
    const Eigen::MatrixBase<Matrix6xOut2> & a_partial_dq,
    const Eigen::MatrixBase<Matrix6xOut3> & a_partial_dv,
    const Eigen::MatrixBase<Matrix6xOut4> & a_partial_da);

  /**
   * @brief      Computes the partial derivatives of the frame acceleration quantity with respect to
   * q, v and a. You must first call pinocchio::computeForwardKinematicsDerivatives to compute all
   * the required quantities. It is important to notice that a direct outcome (for free) of this
   * algo is v_partial_dq and v_partial_dv which is equal to a_partial_da.
   *
   * @tparam JointCollection Collection of Joint types.
   * @tparam Matrix6xOut1 Matrix6x containing the partial derivatives of the frame spatial velocity
   * with respect to the joint configuration vector.
   * @tparam Matrix6xOut2 Matrix6x containing the partial derivatives of the frame spatial
   * acceleration with respect to the joint configuration vector.
   * @tparam Matrix6xOut3 Matrix6x containing the partial derivatives of the frame spatial
   * acceleration with respect to the joint velocity vector.
   * @tparam Matrix6xOut4 Matrix6x containing the partial derivatives of the frame spatial
   * acceleration with respect to the joint acceleration vector.
   *
   * @param[in]  model       The kinematic model
   * @param[in]  data        Data associated to model
   * @param[in]  frame_id    Id of the operational Frame
   * @param[in]  rf          Reference frame in which the velocity is expressed.
   * @param[out] v_partial_dq Partial derivative of the frame spatial velocity w.r.t. \f$ q \f$.
   * @param[out] a_partial_dq Partial derivative of the frame spatial acceleration w.r.t. \f$ q \f$.
   * @param[out] a_partial_dq Partial derivative of the frame spatial acceleration w.r.t. \f$ v \f$.
   * @param[out] a_partial_dq Partial derivative of the frame spatial acceleration w.r.t. \f$
   * \dot{v} \f$.
   *
   */
  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename Matrix6xOut1,
    typename Matrix6xOut2,
    typename Matrix6xOut3,
    typename Matrix6xOut4>
  void getFrameAccelerationDerivatives(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const FrameIndex frame_id,
    const ReferenceFrame rf,
    const Eigen::MatrixBase<Matrix6xOut1> & v_partial_dq,
    const Eigen::MatrixBase<Matrix6xOut2> & a_partial_dq,
    const Eigen::MatrixBase<Matrix6xOut3> & a_partial_dv,
    const Eigen::MatrixBase<Matrix6xOut4> & a_partial_da)
  {
    PINOCCHIO_CHECK_INPUT_ARGUMENT((int)frame_id < model.nframes, "The frame_id is not valid.");
    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;
    typedef typename Model::Frame Frame;

    const Frame & frame = model.frames[frame_id];
    typename Data::SE3 & oMframe = data.oMf[frame_id];
    oMframe = data.oMi[frame.parentJoint] * frame.placement; // for backward compatibility
    getFrameAccelerationDerivatives(
      model, data, frame.parentJoint, frame.placement, rf,
      PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut1, v_partial_dq),
      PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut2, a_partial_dq),
      PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut3, a_partial_dv),
      PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut4, a_partial_da));
  }

  /**
   * @brief      Computes the partial derivatives of the frame acceleration quantity with respect to
   * q, v and a. You must first call pinocchio::computeForwardKinematicsDerivatives to compute all
   * the required quantities. It is important to notice that a direct outcome (for free) of this
   * algo is v_partial_dq and v_partial_dv which is equal to a_partial_da.
   *
   * @tparam JointCollection Collection of Joint types.
   * @tparam Matrix6xOut1 Matrix6x containing the partial derivatives of the frame spatial velocity
   * with respect to the joint configuration vector.
   * @tparam Matrix6xOut2 Matrix6x containing the partial derivatives of the frame spatial velocity
   * with respect to the joint velocity vector.
   * @tparam Matrix6xOut3 Matrix6x containing the partial derivatives of the frame spatial
   * acceleration with respect to the joint configuration vector.
   * @tparam Matrix6xOut4 Matrix6x containing the partial derivatives of the frame spatial
   * acceleration with respect to the joint velocity vector.
   * @tparam Matrix6xOut5 Matrix6x containing the partial derivatives of the frame spatial
   * acceleration with respect to the joint acceleration vector.
   *
   * @param[in]  model                  The kinematic model
   * @param[in]  data                     Data associated to model
   * @param[in]  joint_id            Index of the supporting joint
   * @param[in]  placement          Placement of the Frame w.r.t. the joint frame.
   * @param[in]  rf                          Reference frame in which the velocity is expressed.
   * @param[out] v_partial_dq   Partial derivative of the frame spatial velocity w.r.t. \f$ q \f$.
   * @param[out] v_partial_dv   Partial derivative of the frame spatial velociy w.r.t. \f$ v \f$.
   * @param[out] a_partial_dq   Partial derivative of the frame spatial acceleration w.r.t. \f$ q
   * \f$.
   * @param[out] a_partial_dq   Partial derivative of the frame spatial acceleration w.r.t. \f$ v
   * \f$.
   * @param[out] a_partial_dq   Partial derivative of the frame spatial acceleration w.r.t. \f$
   * \dot{v} \f$.
   *
   */
  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename Matrix6xOut1,
    typename Matrix6xOut2,
    typename Matrix6xOut3,
    typename Matrix6xOut4,
    typename Matrix6xOut5>
  void getFrameAccelerationDerivatives(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const JointIndex joint_id,
    const SE3Tpl<Scalar, Options> & placement,
    const ReferenceFrame rf,
    const Eigen::MatrixBase<Matrix6xOut1> & v_partial_dq,
    const Eigen::MatrixBase<Matrix6xOut2> & v_partial_dv,
    const Eigen::MatrixBase<Matrix6xOut3> & a_partial_dq,
    const Eigen::MatrixBase<Matrix6xOut4> & a_partial_dv,
    const Eigen::MatrixBase<Matrix6xOut5> & a_partial_da)
  {
    getFrameAccelerationDerivatives(
      model, data, joint_id, placement, rf, PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut1, v_partial_dq),
      PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut3, a_partial_dq),
      PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut4, a_partial_dv),
      PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut5, a_partial_da));

    PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut2, v_partial_dv) = a_partial_da;
  }

  /**
   * @brief      Computes the partial derivatives of the frame acceleration quantity with respect to
   * q, v and a. You must first call pinocchio::computeForwardKinematicsDerivatives to compute all
   * the required quantities. It is important to notice that a direct outcome (for free) of this
   * algo is v_partial_dq and v_partial_dv which is equal to a_partial_da.
   *
   * @tparam JointCollection Collection of Joint types.
   * @tparam Matrix6xOut1 Matrix6x containing the partial derivatives of the frame spatial velocity
   * with respect to the joint configuration vector.
   * @tparam Matrix6xOut2 Matrix6x containing the partial derivatives of the frame spatial velocity
   * with respect to the joint velocity vector.
   * @tparam Matrix6xOut3 Matrix6x containing the partial derivatives of the frame spatial
   * acceleration with respect to the joint configuration vector.
   * @tparam Matrix6xOut4 Matrix6x containing the partial derivatives of the frame spatial
   * acceleration with respect to the joint velocity vector.
   * @tparam Matrix6xOut5 Matrix6x containing the partial derivatives of the frame spatial
   * acceleration with respect to the joint acceleration vector.
   *
   * @param[in]  model       The kinematic model
   * @param[in]  data        Data associated to model
   * @param[in]  frame_id    Id of the operational Frame
   * @param[in]  rf          Reference frame in which the velocity is expressed.
   * @param[out] v_partial_dq Partial derivative of the frame spatial velocity w.r.t. \f$ q \f$.
   * @param[out] v_partial_dv Partial derivative of the frame spatial velociy w.r.t. \f$ v \f$.
   * @param[out] a_partial_dq Partial derivative of the frame spatial acceleration w.r.t. \f$ q \f$.
   * @param[out] a_partial_dq Partial derivative of the frame spatial acceleration w.r.t. \f$ v \f$.
   * @param[out] a_partial_dq Partial derivative of the frame spatial acceleration w.r.t. \f$
   * \dot{v} \f$.
   *
   */
  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename Matrix6xOut1,
    typename Matrix6xOut2,
    typename Matrix6xOut3,
    typename Matrix6xOut4,
    typename Matrix6xOut5>
  void getFrameAccelerationDerivatives(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const FrameIndex frame_id,
    const ReferenceFrame rf,
    const Eigen::MatrixBase<Matrix6xOut1> & v_partial_dq,
    const Eigen::MatrixBase<Matrix6xOut2> & v_partial_dv,
    const Eigen::MatrixBase<Matrix6xOut3> & a_partial_dq,
    const Eigen::MatrixBase<Matrix6xOut4> & a_partial_dv,
    const Eigen::MatrixBase<Matrix6xOut5> & a_partial_da)
  {
    PINOCCHIO_CHECK_INPUT_ARGUMENT((int)frame_id < model.nframes, "The frame_id is not valid.");
    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;
    typedef typename Model::Frame Frame;

    const Frame & frame = model.frames[frame_id];
    typename Data::SE3 & oMframe = data.oMf[frame_id];
    oMframe = data.oMi[frame.parentJoint] * frame.placement; // for backward compatibility
    getFrameAccelerationDerivatives(
      model, data, frame.parentJoint, frame.placement, rf,
      PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut1, v_partial_dq),
      PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut2, v_partial_dv),
      PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut3, a_partial_dq),
      PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut4, a_partial_dv),
      PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut5, a_partial_da));
  }
} // namespace pinocchio

#include "pinocchio/algorithm/frames-derivatives.hxx"

#if PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
  #include "pinocchio/algorithm/frames-derivatives.txx"
#endif // PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION

#endif // ifndef __pinocchio_algorithm_frames_derivatives_hpp__
