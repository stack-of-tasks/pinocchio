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

#include <iostream>
#include <Eigen/Core>
#include <pinocchio/spatial/skew.hpp>

#include <boost/test/unit_test.hpp>

template<int N>
void udut( Eigen::Matrix<double,N,N> & M )
{
    typedef Eigen::Matrix<double,N,N> MatrixNd;
    typedef Eigen::Matrix<double,1,N> VectorNd;
    typedef typename MatrixNd::DiagonalReturnType D_t;
    typedef typename MatrixNd::template TriangularViewReturnType<Eigen::StrictlyUpper>::Type U_t;

    VectorNd tmp;
    D_t D = M.diagonal();
    U_t U = M.template triangularView<Eigen::StrictlyUpper>();

    for(int j=N-1;j>=0;--j )
      {
	typename VectorNd::SegmentReturnType DUt = tmp.tail(N-j-1);
	if( j<N-1 ) DUt = M.row(j).tail(N-j-1).transpose().cwiseProduct( D.tail(N-j-1) );

	D[j] -= M.row(j).tail(N-j-1).dot(DUt);

	for(int i=j-1;i>=0;--i)
	  { U(i,j) -= M.row(i).tail(N-j-1).dot(DUt); U(i,j) /= D[j]; }
      }
  }

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE ( udut )
{
  using namespace Eigen;

  Matrix<double,6,6> A = Matrix<double,6,6>::Random(); A = A*A.transpose();
  double m = A(1,1);
  Matrix3d I = A.bottomRightCorner<3,3>();
  Vector3d c = A.block<3,1>(0,3);

  A.topLeftCorner<3,3>() = Matrix3d::Identity()*m;
  A.topRightCorner<3,3>() = se3::skew(c).transpose();
  A.bottomLeftCorner<3,3>() = se3::skew(c);
  A.bottomRightCorner<3,3>() -= A.bottomLeftCorner<3,3>()*A.bottomLeftCorner<3,3>();

  std::cout << "A = [\n" << A << "];" << std::endl;
  udut(A);
  std::cout << "U = [\n" << A << "];" << std::endl;
}

BOOST_AUTO_TEST_SUITE_END()
