//
// Copyright (c) 2015-2016 CNRS
//

#include <iostream>
#include "pinocchio/utils/timer.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

using namespace Eigen;

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Quaternion<double>)

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double,1,1,RowMajor>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double,2,2,RowMajor>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double,3,3,RowMajor>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double,4,4,RowMajor>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double,5,5,RowMajor>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double,6,6,RowMajor>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double,7,7,RowMajor>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double,8,8,RowMajor>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double,9,9,RowMajor>)

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double,1,1,ColMajor>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double,2,2,ColMajor>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double,3,3,ColMajor>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double,4,4,ColMajor>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double,5,5,ColMajor>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double,6,6,ColMajor>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double,7,7,ColMajor>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double,8,8,ColMajor>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double,9,9,ColMajor>)

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double,2,1>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double,3,1>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double,4,1>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double,5,1>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double,6,1>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double,7,1>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double,8,1>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double,9,1>)

template<int NBT>
void checkQuaternionToMatrix(std::string label)
{

  using namespace Eigen;

  PinocchioTicToc timer(PinocchioTicToc::NS);

  /* Random pre-initialization of all matrices in the stack. */
  std::vector< Quaterniond > q1s     (NBT);
  std::vector< Matrix3d > R3s     (NBT);
  for(size_t i=0;i<NBT;++i)
    {
      q1s[i]     = Quaterniond(Vector4d::Random()).normalized();
      R3s[i]     = Matrix3d::Random();
    }

  /* Timed product. */
  timer.tic();
  SMOOTH(NBT)
  {
    R3s[_smooth] = q1s[_smooth].toRotationMatrix();
  }

  std::cout << label << " = \t\t" ; timer.toc(std::cout,NBT);
}

template<int NBT>
void checkQuaternion(std::string label)
{

  using namespace Eigen;

  PinocchioTicToc timer(PinocchioTicToc::NS);

  /* Random pre-initialization of all matrices in the stack. */
  std::vector< Quaterniond > q1s     (NBT);
  std::vector< Vector3d > v2s     (NBT);
  std::vector< Vector3d > v3s     (NBT);
  for(size_t i=0;i<NBT;++i)
    {
      q1s[i]     = Quaterniond(Vector4d::Random()).normalized();
      v2s[i]     = Vector3d::Random();
      v3s[i]     = Vector3d::Random();
    }

  /* Timed product. */
  timer.tic();
  SMOOTH(NBT)
  {
    v3s[_smooth] = q1s[_smooth]*v2s[_smooth];
  }

  std::cout << label << " = \t\t" ; timer.toc(std::cout,NBT);
}

template<int NBT>
void checkQuaternionQuaternion(std::string label)
{

  using namespace Eigen;

  PinocchioTicToc timer(PinocchioTicToc::NS);

  /* Random pre-initialization of all matrices in the stack. */
  std::vector< Quaterniond > q1s     (NBT);
  std::vector< Quaterniond > q2s     (NBT);
  std::vector< Quaterniond > q3s     (NBT);
  for(size_t i=0;i<NBT;++i)
    {
      q1s[i]     = Quaterniond(Vector4d::Random()).normalized();
      q2s[i]     = Quaterniond(Vector4d::Random()).normalized();
      q3s[i]     = Quaterniond(Vector4d::Random()).normalized();
    }

  /* Timed product. */
  timer.tic();
  SMOOTH(NBT)
  {
    q3s[_smooth] = q1s[_smooth]*q2s[_smooth];
  }

  std::cout << label << " = \t\t" ; timer.toc(std::cout,NBT);
}

template<int NBT>
void checkQuaternionD(std::string label)
{

  using namespace Eigen;

  PinocchioTicToc timer(PinocchioTicToc::NS);

  /* Random pre-initialization of all matrices in the stack. */
  std::vector< Quaterniond > q1s     (NBT);
  std::vector< VectorXd > v2s     (NBT);
  std::vector< VectorXd > v3s     (NBT);
  for(size_t i=0;i<NBT;++i)
    {
      q1s[i]     = Quaterniond(Vector4d::Random()).normalized();
      v2s[i]     = VectorXd::Random(3);
      v3s[i]     = VectorXd::Random(3);
    }

  /* Timed product. */
  timer.tic();
  SMOOTH(NBT)
  {
    v3s[_smooth] = q1s[_smooth]*v2s[_smooth];
  }

  std::cout << label << " = \t\t" ; timer.toc(std::cout,NBT);
}

template<int MSIZE,int NBT>
void checkMatrix(std::string label)
{
  /* Row/Col organization of the three matrices M3=M1*M2 */
  #define RC1 RowMajor
  #define RC2 RowMajor
  #define RC3 RowMajor

  using namespace Eigen;

  PinocchioTicToc timer(PinocchioTicToc::NS);

  /* Random pre-initialization of all matrices in the stack. */
  std::vector< Matrix<double,MSIZE,MSIZE,RC1> > R1s     (NBT);
  std::vector< Matrix<double,MSIZE,MSIZE,RC2> > R2s     (NBT);
  std::vector< Matrix<double,MSIZE,MSIZE,RC3> > R3s     (NBT);
  for(size_t i=0;i<NBT;++i)
    {
      R1s[i]     = Matrix<double,MSIZE,MSIZE,RC1>::Random();
      R2s[i]     = Matrix<double,MSIZE,MSIZE,RC2>::Random();
      R3s[i]     = Matrix<double,MSIZE,MSIZE,RC3>::Random();
    }

  /* Timed product. */
  timer.tic();
  SMOOTH(NBT)
  {
    R3s[_smooth] = R1s[_smooth]*R2s[_smooth];
  }

  std::cout << label << " = \t\t" ; timer.toc(std::cout,NBT);
}

template<int MSIZE,int NBT>
void checkVector(std::string label)
{
  using namespace Eigen;

  PinocchioTicToc timer(PinocchioTicToc::NS);
  std::vector< Matrix<double,MSIZE,MSIZE> > R1s     (NBT);
  std::vector< Matrix<double,MSIZE,1> > v2s     (NBT);
  std::vector< Matrix<double,MSIZE,1> > v3s     (NBT);
  for(size_t i=0;i<NBT;++i)
    {
      R1s[i]     = Matrix<double,MSIZE,MSIZE>::Random();
      v2s[i]     = Matrix<double,MSIZE,1>::Random();
      v3s[i]     = Matrix<double,MSIZE,1>::Random();
    }

   timer.tic();
   SMOOTH(NBT)
    {
      v3s[_smooth] = R1s[_smooth]*v2s[_smooth];
    }

   std::cout << label << " = \t\t"; timer.toc(std::cout,NBT);
}

template<int NBT>
void checkDMatrix(int MSIZE,std::string label)
{
  using namespace Eigen;

  PinocchioTicToc timer(PinocchioTicToc::NS);
  std::vector< MatrixXd > R1s     (NBT);
  std::vector< MatrixXd > R2s     (NBT);
  std::vector< MatrixXd > R3s     (NBT);
  for(size_t i=0;i<NBT;++i)
    {
      R1s[i]     = MatrixXd::Random(MSIZE,MSIZE);
      R2s[i]     = MatrixXd::Random(MSIZE,MSIZE);
      R3s[i]     = MatrixXd::Random(MSIZE,MSIZE);
    }

   timer.tic();
   SMOOTH(NBT)
    {
      R3s[_smooth] = R1s[_smooth]*R2s[_smooth];
    }

   std::cout << label << " = \t\t"; timer.toc(std::cout,NBT);
}

template<int NBT>
void checkDVector(int MSIZE,std::string label)
{
  using namespace Eigen;

  PinocchioTicToc timer(PinocchioTicToc::NS);
  std::vector< MatrixXd > R1s     (NBT);
  std::vector< MatrixXd > v2s     (NBT);
  std::vector< MatrixXd > v3s     (NBT);
  for(size_t i=0;i<NBT;++i)
    {
      R1s[i]     = MatrixXd::Random(MSIZE,MSIZE);
      v2s[i]     = MatrixXd::Random(MSIZE,1);
      v3s[i]     = MatrixXd::Random(MSIZE,1);
    }

   timer.tic();
   SMOOTH(NBT)
    {
      v3s[_smooth] = R1s[_smooth]*v2s[_smooth];
    }

   std::cout << label << " = \t\t"; timer.toc(std::cout,NBT);
}


int main()
{

  #ifdef NDEBUG
  const int NBT = 1000*1000;
  #else
    const int NBT = 1;
    std::cout << "(the time score in debug mode is not relevant) " << std::endl;
  #endif
    checkQuaternion<NBT>(  "quaternion-vector static ");
    checkQuaternionD<NBT>(  "quaternion-vector dynamic");
    checkQuaternionQuaternion<NBT>(  "quaternion-quaternion    ");
    checkQuaternionToMatrix<NBT>(  "quaternion->matrix static");
    std::cout << std::endl;

    checkVector<3,NBT> (  "matrix-vector static  3x3");
    checkDVector<NBT>  (3,"matrix-vector dynamic 3x3");
    checkMatrix<3,NBT> (  "matrix-matrix static  3x3");
    checkDMatrix<NBT>  (3,"matrix-matrix dynamic 3x3");
    std::cout << std::endl;

    checkVector<4,NBT> (  "matrix-vector static  4x4");
    checkDVector<NBT>  (4,"matrix-vector dynamic 4x4");
    checkMatrix<4,NBT> (  "matrix-matrix static  4x4");
    checkDMatrix<NBT>  (4,"matrix-matrix dynamic 4x4");
    std::cout << std::endl;

    std::cout << "--" << std::endl;
  return 0;
}
