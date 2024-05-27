//
// Copyright (c) 2015-2021 CNRS INRIA

#ifndef __pinocchio_spatial_log_hpp__
#define __pinocchio_spatial_log_hpp__

namespace pinocchio
{

  template<typename Scalar>
  struct log3_impl;
  template<typename Scalar>
  struct Jlog3_impl;

  template<typename Scalar>
  struct log6_impl;
  template<typename Scalar>
  struct Jlog6_impl;

  template<typename Matrix3>
  inline typename PINOCCHIO_EIGEN_PLAIN_TYPE(Matrix3)
    renormalize_rotation_matrix(const Eigen::MatrixBase<Matrix3> & R);

} // namespace pinocchio

#endif // ifndef __pinocchio_spatial_log_hpp__
