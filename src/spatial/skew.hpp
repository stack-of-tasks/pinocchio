//
// Copyright (c) 2015 CNRS
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

#ifndef __se3_skew_hpp__
#define __se3_skew_hpp__
namespace se3
{
  template <typename D>
  inline Eigen::Matrix<typename D::Scalar,3,3,D::Options>
  skew(const Eigen::MatrixBase<D> & v)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(D,3);
    Eigen::Matrix<typename D::Scalar,3,3,D::Options> m;
    m(0,0) =  0   ;  m(0,1) = -v[2];   m(0,2) =  v[1];
    m(1,0) =  v[2];  m(1,1) =  0   ;   m(1,2) = -v[0];
    m(2,0) = -v[1];  m(2,1) =  v[0];   m(2,2) =  0   ;
    return m;
  }

  template <typename D>
  inline Eigen::Matrix<typename D::Scalar,3,3,D::Options>
  alphaSkew (const typename D::Scalar s, const Eigen::MatrixBase<D> & v)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(D,3);
    Eigen::Matrix<typename D::Scalar,3,3,D::Options> m;
    m(0,0) =  0   ;  m(0,1) = -v[2] * s;   m(0,2) =  v[1] * s;
    m(1,0) = - m(0,1);  m(1,1) =  0   ;   m(1,2) = -v[0] * s;
    m(2,0) = - m(0,2);  m(2,1) =  - m(1,2);   m(2,2) =  0;
    return m;
  }

  template <typename V,typename M>
  inline Eigen::Matrix<typename M::Scalar,3,M::ColsAtCompileTime,M::Options>
  cross(const Eigen::MatrixBase<V> & v,
	const Eigen::MatrixBase<M> & m)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(V,3);

    Eigen::Matrix<typename M::Scalar,3,M::ColsAtCompileTime,M::Options> res (3,m.cols());
    res.row(0) = v[1]*m.row(2) - v[2]*m.row(1);
    res.row(1) = v[2]*m.row(0) - v[0]*m.row(2);
    res.row(2) = v[0]*m.row(1) - v[1]*m.row(0);
    return res;
  }
} // namespace se3
#endif // ifndef __se3_skew_hpp__
