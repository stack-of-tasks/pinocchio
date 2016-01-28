//
// Copyright (c) 2015 CNRS
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.

#ifndef __spatial_explog_hpp__
# define __spatial_explog_hpp__

# include <Eigen/Geometry>

# include "pinocchio/math/sincos.hpp"
# include "pinocchio/spatial/motion.hpp"
# include "pinocchio/spatial/skew.hpp"
# include "pinocchio/spatial/se3.hpp"

namespace se3
{
  /// \brief Exp: so3 -> SO3.
  ///
  /// Return the integral of the input angular velocity during time 1.
  ///
  /// \param[in] v The angular velocity vector.
  ///
  /// \return The rotational matrix associated to the integration of the angular velocity during time 1.
  ///
  template <typename D> Eigen::Matrix<typename D::Scalar,3,3,D::Options>
  exp3(const Eigen::MatrixBase<D> & v)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(D,3);
    typename D::Scalar nv = v.norm();
    if (nv > 1e-14)
      return Eigen::AngleAxis<typename D::Scalar>(nv, v / nv).matrix();
    else
      return Eigen::Matrix<typename D::Scalar,3,3,D::Options>::Identity();
  }

  /// \brief Log: SO3 -> so3.
  ///
  /// Pseudo-inverse of log from SO3 -> { v \in so3, ||v|| < 2pi }.
  ///
  /// \param[in] R The rotation matrix.
  ///
  /// \return The angular velocity vector associated to the rotation matrix.
  ///
  template <typename D> Eigen::Matrix<typename D::Scalar,3,1,D::Options>
  log3(const Eigen::MatrixBase<D> & R)
  {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(D, 3, 3);
    Eigen::AngleAxis<typename D::Scalar> angleAxis(R);
    return angleAxis.axis() * angleAxis.angle();
  }

  /// \brief Exp: se3 -> SE3.
  ///
  /// Return the integral of the input twist during time 1.
  ///
  /// \param[in] nu The input twist.
  ///
  /// \return The rigid transformation associated to the integration of the twist during time 1.
  ///
  template <typename _Scalar, int _Options> SE3Tpl<_Scalar, _Options>
  exp6(const MotionTpl<_Scalar,_Options> & nu)
  {
    typedef _Scalar Scalar;
    typedef typename MotionTpl<Scalar,_Options>::Vector3 Vector3;
    typedef typename MotionTpl<Scalar,_Options>::Matrix3 Matrix3;

    const Vector3 & w = nu.angular();
    const Vector3 & v = nu.linear();
    Scalar t = w.norm();
    if (t > 1e-15)
    {
      Matrix3 S(alphaSkew(1./t, w));
      double ct,st; SINCOS (t,&st,&ct);
      Matrix3 V((1. - ct)/t * S + (1. - st/t) * S * S);
      Vector3 p(v + V * v);
      return SE3Tpl<_Scalar, _Options>(exp3(w), p);
    }
    else
    {
      return SE3Tpl<_Scalar, _Options>(Matrix3::Identity(), v);
    }
  }

  /// \brief Exp: se3 -> SE3.
  ///
  /// Return the integral of the input spatial velocity during time 1.
  ///
  /// \param[in] v The twist represented by a vector.
  ///
  /// \return The rigid transformation associated to the integration of the twist vector during time 1..
  ///
  template <typename D> SE3Tpl<typename D::Scalar, D::Options>
  exp6(const Eigen::MatrixBase<D> & v)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(D,6);
    MotionTpl<typename D::Scalar,D::Options> nu(v);
    return exp6(nu);
  }

  /// \brief Log: SE3 -> se3.
  ///
  /// Pseudo-inverse of exp from SE3 -> { v,w \in se3, ||w|| < 2pi }.
  ///
  /// \param[in] M The rigid transformation.
  ///
  /// \return The twist associated to the rigid transformation during time 1.
  ///
  template <typename _Scalar, int _Options>
  MotionTpl<_Scalar,_Options>
  log6(const SE3Tpl<_Scalar, _Options> & M)
  {
    typedef _Scalar Scalar;
    typedef typename SE3Tpl<Scalar,_Options>::Vector3 Vector3;
    typedef typename SE3Tpl<Scalar,_Options>::Matrix3 Matrix3;

    const Matrix3 & R = M.rotation();
    const Vector3 & p = M.translation();
    Vector3 w(log3(R));
    Scalar t = w.norm();
    if (t > 1e-15)
    {
      Matrix3 S(alphaSkew(1./t, w));
      double ct,st; SINCOS (t,&st,&ct);
      Matrix3 V(Matrix3::Identity() + (1. - ct)/t * S + (1. - st/t) * S * S);
   
      return MotionTpl<_Scalar,_Options>(V.inverse() * p, w);
    }
    else
      return MotionTpl<_Scalar,_Options>(p, w);
    
  }

  /// \brief Log: SE3 -> se3.
  ///
  /// Pseudo-inverse of exp from SE3 -> { v,w \in se3, ||w|| < 2pi }.
  ///
  /// \param[in] R The rigid transformation represented as an homogenous matrix.
  ///
  /// \return The twist associated to the rigid transformation during time 1.
  ///
  template <typename D> MotionTpl<typename D::Scalar,D::Options>
  log6(const Eigen::MatrixBase<D> & M)
  {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(D, 4, 4);
    typedef typename SE3Tpl<typename D::Scalar,D::Options>::Vector3 Vector3;
    typedef typename SE3Tpl<typename D::Scalar,D::Options>::Matrix3 Matrix3;

    Matrix3 rot(M.template block<3,3>(0,0));
    Vector3 trans(M.template block<3,1>(0,3));
    SE3Tpl<typename D::Scalar,D::Options> m(rot, trans);
    return log6(m);
  }
} // namespace se3

#endif //#ifndef __math_explog_hpp__
