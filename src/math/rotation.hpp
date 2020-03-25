//
// Copyright (c) 2019-2020 CNRS INRIA
//

#ifndef __pinocchio_math_rotation_hpp__
#define __pinocchio_math_rotation_hpp__

#include "pinocchio/fwd.hpp"
#include "pinocchio/math/matrix.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace pinocchio
{
  ///
  /// \brief Computes a rotation matrix from a vector and values of sin and cos
  ///        orientations values.
  ///
  /// \remarks This code is issue from Eigen::AxisAngle::toRotationMatrix
  ///
  template<typename Vector3, typename Scalar, typename Matrix3>
  void toRotationMatrix(const Eigen::MatrixBase<Vector3> & axis,
                        const Scalar & cos_value, const Scalar & sin_value,
                        const Eigen::MatrixBase<Matrix3> & res)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector3,3);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Matrix3,3,3);
    
    assert(isUnitary(axis) && "The axis is not unitary.");
    
    Matrix3 & res_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix3,res);
    Vector3 sin_axis  = sin_value * axis;
    Vector3 cos1_axis = (Scalar(1)-cos_value) * axis;
    
    Scalar tmp;
    tmp = cos1_axis.x() * axis.y();
    res_.coeffRef(0,1) = tmp - sin_axis.z();
    res_.coeffRef(1,0) = tmp + sin_axis.z();
    
    tmp = cos1_axis.x() * axis.z();
    res_.coeffRef(0,2) = tmp + sin_axis.y();
    res_.coeffRef(2,0) = tmp - sin_axis.y();
    
    tmp = cos1_axis.y() * axis.z();
    res_.coeffRef(1,2) = tmp - sin_axis.x();
    res_.coeffRef(2,1) = tmp + sin_axis.x();
    
    res_.diagonal() = (cos1_axis.cwiseProduct(axis)).array() + cos_value;
  }

  ///
  /// \brief Orthogonormalization procedure for a rotation matrix (closed enough to SO(3)).
  ///
  /// \param[in,out] rot A 3x3 matrix to orthonormalize
  ///
  template<typename Matrix3>
  void normalizeRotation(const Eigen::MatrixBase<Matrix3> & rot)
  {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Matrix3,3,3);
    Matrix3 & rot_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix3,rot);
    
    typedef typename Matrix3::Scalar Scalar;
    enum { Options = PINOCCHIO_EIGEN_PLAIN_TYPE(Matrix3)::Options };
    typedef Eigen::Quaternion<Scalar,Options> Quaternion;
    Quaternion quat(rot); quat.normalize();
    rot_ = quat.toRotationMatrix();
  }
}

#endif //#ifndef __pinocchio_math_rotation_hpp__

