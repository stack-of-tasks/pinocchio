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
   * @brief      Computes the partial derivatives of the frame velocity quantity with respect to q and v.
   *             You must first call pinocchio::computeForwardKinematicsDerivatives to compute all the required quantities.
   *
   * @tparam JointCollection Collection of Joint types.
   * @tparam Matrix6xOut1 Matrix6x containing the partial derivatives of the frame spatial velocity with respect to the joint configuration vector.
   * @tparam Matrix6xOut2 Matrix6x containing the partial derivatives of the frame spatial velocity with respect to the joint velocity vector.
   *
   * @param[in]  model       The kinematic model
   * @param[in]  data        Data associated to model
   * @param[in]  frame_id    Id of the operational Frame
   * @param[in]  rf          Reference frame in which the velocity is expressed.
   * @param[out] v_partial_dq Partial derivative of the joint spatial velocity w.r.t. \f$ q \f$.
   * @param[out] v_partial_dv Partial derivative of the joint spatial velociy w.r.t. \f$ \dot{q} \f$.
   *
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename Matrix6xOut1, typename Matrix6xOut2>
  void
  getFrameVelocityDerivatives(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                              DataTpl<Scalar,Options,JointCollectionTpl> & data,
                              const typename ModelTpl<Scalar,Options,JointCollectionTpl>::FrameIndex frame_id,
                              const ReferenceFrame rf,
                              const Eigen::MatrixBase<Matrix6xOut1> & v_partial_dq,
                              const Eigen::MatrixBase<Matrix6xOut2> & v_partial_dv);
}

#include "pinocchio/algorithm/frames-derivatives.hxx"

#endif // ifndef __pinocchio_algorithm_frames_derivatives_hpp__
