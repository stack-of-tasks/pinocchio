#include <iostream>
#include <Eigen/Core>
#include <pinocchio/spatial/skew.hpp>

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


#include <Eigen/Core>
int main(int argc, const char ** argv)
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


   return 0;
}
