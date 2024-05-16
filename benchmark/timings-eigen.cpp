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

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double, 1, 1, RowMajor>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double, 2, 2, RowMajor>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double, 3, 3, RowMajor>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double, 4, 4, RowMajor>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double, 5, 5, RowMajor>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double, 6, 6, RowMajor>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double, 7, 7, RowMajor>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double, 8, 8, RowMajor>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double, 9, 9, RowMajor>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double, 50, 50, RowMajor>)

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double, 1, 1, ColMajor>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double, 2, 2, ColMajor>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double, 3, 3, ColMajor>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double, 4, 4, ColMajor>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double, 5, 5, ColMajor>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double, 6, 6, ColMajor>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double, 7, 7, ColMajor>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double, 8, 8, ColMajor>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double, 9, 9, ColMajor>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double, 50, 50, ColMajor>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double, 2, 1>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double, 3, 1>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double, 4, 1>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double, 5, 1>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double, 6, 1>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double, 7, 1>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double, 8, 1>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double, 9, 1>)

template<int NBT>
void checkQuaternionToMatrix(std::string label)
{

  using namespace Eigen;

  PinocchioTicToc timer(PinocchioTicToc::NS);

  /* Random pre-initialization of all matrices in the stack. */
  std::vector<Quaterniond> q1s(NBT);
  std::vector<Matrix3d> R3s(NBT);
  for (size_t i = 0; i < NBT; ++i)
  {
    q1s[i] = Quaterniond(Vector4d::Random()).normalized();
    R3s[i] = Matrix3d::Random();
  }

  /* Timed product. */
  timer.tic();
  SMOOTH(NBT)
  {
    R3s[_smooth] = q1s[_smooth].toRotationMatrix();
  }

  std::cout << label << " = \t\t";
  timer.toc(std::cout, NBT);
}

template<int NBT>
void checkQuaternion(std::string label)
{

  using namespace Eigen;

  PinocchioTicToc timer(PinocchioTicToc::NS);

  /* Random pre-initialization of all matrices in the stack. */
  std::vector<Quaterniond> q1s(NBT);
  std::vector<Vector3d> v2s(NBT);
  std::vector<Vector3d> v3s(NBT);
  for (size_t i = 0; i < NBT; ++i)
  {
    q1s[i] = Quaterniond(Vector4d::Random()).normalized();
    v2s[i] = Vector3d::Random();
    v3s[i] = Vector3d::Random();
  }

  /* Timed product. */
  timer.tic();
  SMOOTH(NBT)
  {
    v3s[_smooth] = q1s[_smooth] * v2s[_smooth];
  }

  std::cout << label << " = \t\t";
  timer.toc(std::cout, NBT);
}

template<int NBT>
void checkQuaternionQuaternion(std::string label)
{

  using namespace Eigen;

  PinocchioTicToc timer(PinocchioTicToc::NS);

  /* Random pre-initialization of all matrices in the stack. */
  std::vector<Quaterniond> q1s(NBT);
  std::vector<Quaterniond> q2s(NBT);
  std::vector<Quaterniond> q3s(NBT);
  for (size_t i = 0; i < NBT; ++i)
  {
    q1s[i] = Quaterniond(Vector4d::Random()).normalized();
    q2s[i] = Quaterniond(Vector4d::Random()).normalized();
    q3s[i] = Quaterniond(Vector4d::Random()).normalized();
  }

  /* Timed product. */
  timer.tic();
  SMOOTH(NBT)
  {
    q3s[_smooth] = q1s[_smooth] * q2s[_smooth];
  }

  std::cout << label << " = \t\t";
  timer.toc(std::cout, NBT);
}

template<int NBT>
void checkQuaternionD(std::string label)
{

  using namespace Eigen;

  PinocchioTicToc timer(PinocchioTicToc::NS);

  /* Random pre-initialization of all matrices in the stack. */
  std::vector<Quaterniond> q1s(NBT);
  std::vector<VectorXd> v2s(NBT);
  std::vector<VectorXd> v3s(NBT);
  for (size_t i = 0; i < NBT; ++i)
  {
    q1s[i] = Quaterniond(Vector4d::Random()).normalized();
    v2s[i] = VectorXd::Random(3);
    v3s[i] = VectorXd::Random(3);
  }

  /* Timed product. */
  timer.tic();
  SMOOTH(NBT)
  {
    v3s[_smooth] = q1s[_smooth] * v2s[_smooth];
  }

  std::cout << label << " = \t\t";
  timer.toc(std::cout, NBT);
}

// M1*M2 = M3
template<int MSIZE, int NBT, int OptionM1, int OptionM2, int OptionM3>
void checkMatrix(std::string label)
{
  using namespace Eigen;

  PinocchioTicToc timer(PinocchioTicToc::NS);

  /* Random pre-initialization of all matrices in the stack. */
  std::vector<Matrix<double, MSIZE, MSIZE, OptionM1>> R1s(NBT);
  std::vector<Matrix<double, MSIZE, MSIZE, OptionM2>> R2s(NBT);
  std::vector<Matrix<double, MSIZE, MSIZE, OptionM3>> R3s(NBT);
  for (size_t i = 0; i < NBT; ++i)
  {
    R1s[i] = Matrix<double, MSIZE, MSIZE, OptionM1>::Random();
    R2s[i] = Matrix<double, MSIZE, MSIZE, OptionM2>::Random();
    R3s[i] = Matrix<double, MSIZE, MSIZE, OptionM3>::Random();
  }

  /* Timed product. */
  timer.tic();
  SMOOTH(NBT)
  {
    R3s[_smooth].noalias() = R1s[_smooth] * R2s[_smooth];
  }

  std::cout << label << " = \t\t";
  timer.toc(std::cout, NBT);
}

// M1.T*M2 = M3
template<int MSIZE, int NBT, int OptionM1, int OptionM2, int OptionM3>
void checkMatrixTMatrix(std::string label)
{
  using namespace Eigen;

  PinocchioTicToc timer(PinocchioTicToc::NS);

  /* Random pre-initialization of all matrices in the stack. */
  std::vector<Matrix<double, MSIZE, MSIZE, OptionM1>> R1s(NBT);
  std::vector<Matrix<double, MSIZE, MSIZE, OptionM2>> R2s(NBT);
  std::vector<Matrix<double, MSIZE, MSIZE, OptionM3>> R3s(NBT);
  for (size_t i = 0; i < NBT; ++i)
  {
    R1s[i] = Matrix<double, MSIZE, MSIZE, OptionM1>::Random();
    R2s[i] = Matrix<double, MSIZE, MSIZE, OptionM2>::Random();
    R3s[i] = Matrix<double, MSIZE, MSIZE, OptionM3>::Random();
  }

  /* Timed product. */
  timer.tic();
  SMOOTH(NBT)
  {
    R3s[_smooth].noalias() = R1s[_smooth].transpose() * R2s[_smooth];
  }

  std::cout << label << " = \t\t";
  timer.toc(std::cout, NBT);
}

template<int MSIZE, int NBT>
void checkVector(std::string label)
{
  using namespace Eigen;

  PinocchioTicToc timer(PinocchioTicToc::NS);
  std::vector<Matrix<double, MSIZE, MSIZE>> R1s(NBT);
  std::vector<Matrix<double, MSIZE, 1>> v2s(NBT);
  std::vector<Matrix<double, MSIZE, 1>> v3s(NBT);
  for (size_t i = 0; i < NBT; ++i)
  {
    R1s[i] = Matrix<double, MSIZE, MSIZE>::Random();
    v2s[i] = Matrix<double, MSIZE, 1>::Random();
    v3s[i] = Matrix<double, MSIZE, 1>::Random();
  }

  timer.tic();
  SMOOTH(NBT)
  {
    v3s[_smooth] = R1s[_smooth] * v2s[_smooth];
  }

  std::cout << label << " = \t\t";
  timer.toc(std::cout, NBT);
}

template<int NBT, int OptionM1, int OptionM2, int OptionM3>
void checkDMatrix(int MSIZE, std::string label)
{
  using namespace Eigen;

  PinocchioTicToc timer(PinocchioTicToc::NS);
  std::vector<Matrix<double, Dynamic, Dynamic, OptionM1>> R1s(NBT);
  std::vector<Matrix<double, Dynamic, Dynamic, OptionM2>> R2s(NBT);
  std::vector<Matrix<double, Dynamic, Dynamic, OptionM3>> R3s(NBT);
  for (size_t i = 0; i < NBT; ++i)
  {
    R1s[i] = Matrix<double, Dynamic, Dynamic, OptionM1>::Random(MSIZE, MSIZE);
    R2s[i] = Matrix<double, Dynamic, Dynamic, OptionM2>::Random(MSIZE, MSIZE);
    R3s[i] = Matrix<double, Dynamic, Dynamic, OptionM3>::Random(MSIZE, MSIZE);
  }

  timer.tic();
  SMOOTH(NBT)
  {
    R3s[_smooth].noalias() = R1s[_smooth] * R2s[_smooth];
  }

  std::cout << label << " = \t\t";
  timer.toc(std::cout, NBT);
}

// M1.T*M2 = M3
template<int NBT, int OptionM1, int OptionM2, int OptionM3>
void checkDMatrixTMatrix(int MSIZE, std::string label)
{
  using namespace Eigen;

  PinocchioTicToc timer(PinocchioTicToc::NS);
  std::vector<Matrix<double, Dynamic, Dynamic, OptionM1>> R1s(NBT);
  std::vector<Matrix<double, Dynamic, Dynamic, OptionM2>> R2s(NBT);
  std::vector<Matrix<double, Dynamic, Dynamic, OptionM3>> R3s(NBT);
  for (size_t i = 0; i < NBT; ++i)
  {
    R1s[i] = Matrix<double, Dynamic, Dynamic, OptionM1>::Random(MSIZE, MSIZE);
    R2s[i] = Matrix<double, Dynamic, Dynamic, OptionM2>::Random(MSIZE, MSIZE);
    R3s[i] = Matrix<double, Dynamic, Dynamic, OptionM3>::Random(MSIZE, MSIZE);
  }

  timer.tic();
  SMOOTH(NBT)
  {
    R3s[_smooth].noalias() = R1s[_smooth].transpose() * R2s[_smooth];
  }

  std::cout << label << " = \t\t";
  timer.toc(std::cout, NBT);
}

template<int NBT>
void checkDVector(int MSIZE, std::string label)
{
  using namespace Eigen;

  PinocchioTicToc timer(PinocchioTicToc::NS);
  std::vector<MatrixXd> R1s(NBT);
  std::vector<MatrixXd> v2s(NBT);
  std::vector<MatrixXd> v3s(NBT);
  for (size_t i = 0; i < NBT; ++i)
  {
    R1s[i] = MatrixXd::Random(MSIZE, MSIZE);
    v2s[i] = MatrixXd::Random(MSIZE, 1);
    v3s[i] = MatrixXd::Random(MSIZE, 1);
  }

  timer.tic();
  SMOOTH(NBT)
  {
    v3s[_smooth] = R1s[_smooth] * v2s[_smooth];
  }

  std::cout << label << " = \t\t";
  timer.toc(std::cout, NBT);
}

int main()
{

#ifdef NDEBUG
  const int NBT = 1000 * 1000;
#else
  const int NBT = 1;
  std::cout << "(the time score in debug mode is not relevant) " << std::endl;
#endif
  checkQuaternion<NBT>("quaternion-vector static ");
  checkQuaternionD<NBT>("quaternion-vector dynamic");
  checkQuaternionQuaternion<NBT>("quaternion-quaternion    ");
  checkQuaternionToMatrix<NBT>("quaternion->matrix static");
  std::cout << std::endl;

  using namespace Eigen;
  checkVector<3, NBT>("matrix-vector static  3x3");
  checkDVector<NBT>(3, "matrix-vector dynamic 3x3");
  checkMatrix<3, NBT, ColMajor, ColMajor, ColMajor>("colmatrix = colmatrix*colmatrix static  3x3");
  checkMatrix<3, NBT, RowMajor, ColMajor, ColMajor>("colmatrix = rowmatrix*colmatrix static  3x3");
  checkMatrix<3, NBT, ColMajor, RowMajor, ColMajor>("colmatrix = colmatrix*rowmatrix static  3x3");
  checkMatrix<3, NBT, RowMajor, RowMajor, ColMajor>("colmatrix = rowmatrix*rowmatrix static  3x3");
  checkMatrix<3, NBT, ColMajor, ColMajor, RowMajor>("rowmatrix = colmatrix*colmatrix static  3x3");
  checkMatrix<3, NBT, RowMajor, ColMajor, RowMajor>("rowmatrix = rowmatrix*colmatrix static  3x3");
  checkMatrix<3, NBT, ColMajor, RowMajor, RowMajor>("rowmatrix = colmatrix*rowmatrix static  3x3");
  checkMatrix<3, NBT, RowMajor, RowMajor, RowMajor>("rowmatrix = rowmatrix*rowmatrix static  3x3");

  checkDMatrix<NBT, ColMajor, ColMajor, ColMajor>(
    3, "colmatrix = colmatrix*colmatrix dynamic  3x3");
  checkDMatrix<NBT, RowMajor, ColMajor, ColMajor>(
    3, "colmatrix = rowmatrix*colmatrix dynamic  3x3");
  checkDMatrix<NBT, ColMajor, RowMajor, ColMajor>(
    3, "colmatrix = colmatrix*rowmatrix dynamic  3x3");
  checkDMatrix<NBT, RowMajor, RowMajor, ColMajor>(
    3, "colmatrix = rowmatrix*rowmatrix dynamic  3x3");
  checkDMatrix<NBT, ColMajor, ColMajor, RowMajor>(
    3, "rowmatrix = colmatrix*colmatrix dynamic  3x3");
  checkDMatrix<NBT, RowMajor, ColMajor, RowMajor>(
    3, "rowmatrix = rowmatrix*colmatrix dynamic  3x3");
  checkDMatrix<NBT, ColMajor, RowMajor, RowMajor>(
    3, "rowmatrix = colmatrix*rowmatrix dynamic  3x3");
  checkDMatrix<NBT, RowMajor, RowMajor, RowMajor>(
    3, "rowmatrix = rowmatrix*rowmatrix dynamic  3x3");

  std::cout << std::endl;

  checkVector<4, NBT>("matrix-vector static  4x4");
  checkDVector<NBT>(4, "matrix-vector dynamic 4x4");

  checkMatrix<4, NBT, ColMajor, ColMajor, ColMajor>("colmatrix = colmatrix*colmatrix static  4x4");
  checkMatrix<4, NBT, RowMajor, ColMajor, ColMajor>("colmatrix = rowmatrix*colmatrix static  4x4");
  checkMatrix<4, NBT, ColMajor, RowMajor, ColMajor>("colmatrix = colmatrix*rowmatrix static  4x4");
  checkMatrix<4, NBT, RowMajor, RowMajor, ColMajor>("colmatrix = rowmatrix*rowmatrix static  4x4");
  checkMatrix<4, NBT, ColMajor, ColMajor, RowMajor>("rowmatrix = colmatrix*colmatrix static  4x4");
  checkMatrix<4, NBT, RowMajor, ColMajor, RowMajor>("rowmatrix = rowmatrix*colmatrix static  4x4");
  checkMatrix<4, NBT, ColMajor, RowMajor, RowMajor>("rowmatrix = colmatrix*rowmatrix static  4x4");
  checkMatrix<4, NBT, RowMajor, RowMajor, RowMajor>("rowmatrix = rowmatrix*rowmatrix static  4x4");

  checkMatrixTMatrix<4, NBT, ColMajor, ColMajor, ColMajor>(
    "colmatrix = colmatrix.transpose()*colmatrix static  4x4");
  checkMatrixTMatrix<4, NBT, RowMajor, ColMajor, ColMajor>(
    "colmatrix = rowmatrix.transpose()*colmatrix static  4x4");
  checkMatrixTMatrix<4, NBT, ColMajor, RowMajor, ColMajor>(
    "colmatrix = colmatrix.transpose()*rowmatrix static  4x4");
  checkMatrixTMatrix<4, NBT, RowMajor, RowMajor, ColMajor>(
    "colmatrix = rowmatrix.transpose()*rowmatrix static  4x4");
  checkMatrixTMatrix<4, NBT, ColMajor, ColMajor, RowMajor>(
    "rowmatrix = colmatrix.transpose()*colmatrix static  4x4");
  checkMatrixTMatrix<4, NBT, RowMajor, ColMajor, RowMajor>(
    "rowmatrix = rowmatrix.transpose()*colmatrix static  4x4");
  checkMatrixTMatrix<4, NBT, ColMajor, RowMajor, RowMajor>(
    "rowmatrix = colmatrix.transpose()*rowmatrix static  4x4");
  checkMatrixTMatrix<4, NBT, RowMajor, RowMajor, RowMajor>(
    "rowmatrix = rowmatrix.transpose()*rowmatrix static  4x4");

  checkDMatrix<NBT, ColMajor, ColMajor, ColMajor>(
    4, "colmatrix = colmatrix*colmatrix dynamic  4x4");
  checkDMatrix<NBT, RowMajor, ColMajor, ColMajor>(
    4, "colmatrix = rowmatrix*colmatrix dynamic  4x4");
  checkDMatrix<NBT, ColMajor, RowMajor, ColMajor>(
    4, "colmatrix = colmatrix*rowmatrix dynamic  4x4");
  checkDMatrix<NBT, RowMajor, RowMajor, ColMajor>(
    4, "colmatrix = rowmatrix*rowmatrix dynamic  4x4");
  checkDMatrix<NBT, ColMajor, ColMajor, RowMajor>(
    4, "rowmatrix = colmatrix*colmatrix dynamic  4x4");
  checkDMatrix<NBT, RowMajor, ColMajor, RowMajor>(
    4, "rowmatrix = rowmatrix*colmatrix dynamic  4x4");
  checkDMatrix<NBT, ColMajor, RowMajor, RowMajor>(
    4, "rowmatrix = colmatrix*rowmatrix dynamic  4x4");
  checkDMatrix<NBT, RowMajor, RowMajor, RowMajor>(
    4, "rowmatrix = rowmatrix*rowmatrix dynamic  4x4");

  checkDMatrixTMatrix<NBT, ColMajor, ColMajor, ColMajor>(
    4, "colmatrix = colmatrix.transpose()*colmatrix dynamic  4x4");
  checkDMatrixTMatrix<NBT, RowMajor, ColMajor, ColMajor>(
    4, "colmatrix = rowmatrix.transpose()*colmatrix dynamic  4x4");
  checkDMatrixTMatrix<NBT, ColMajor, RowMajor, ColMajor>(
    4, "colmatrix = colmatrix.transpose()*rowmatrix dynamic  4x4");
  checkDMatrixTMatrix<NBT, RowMajor, RowMajor, ColMajor>(
    4, "colmatrix = rowmatrix.transpose()*rowmatrix dynamic  4x4");
  checkDMatrixTMatrix<NBT, ColMajor, ColMajor, RowMajor>(
    4, "rowmatrix = colmatrix.transpose()*colmatrix dynamic  4x4");
  checkDMatrixTMatrix<NBT, RowMajor, ColMajor, RowMajor>(
    4, "rowmatrix = rowmatrix.transpose()*colmatrix dynamic  4x4");
  checkDMatrixTMatrix<NBT, ColMajor, RowMajor, RowMajor>(
    4, "rowmatrix = colmatrix.transpose()*rowmatrix dynamic  4x4");
  checkDMatrixTMatrix<NBT, RowMajor, RowMajor, RowMajor>(
    4, "rowmatrix = rowmatrix.transpose()*rowmatrix dynamic  4x4");

  checkMatrix<50, NBT / 10, ColMajor, ColMajor, ColMajor>(
    "colmatrix = colmatrix*colmatrix static  50x50");
  checkMatrix<50, NBT / 10, RowMajor, ColMajor, ColMajor>(
    "colmatrix = rowmatrix*colmatrix static  50x50");
  checkMatrix<50, NBT / 10, ColMajor, RowMajor, ColMajor>(
    "colmatrix = colmatrix*rowmatrix static  50x50");
  checkMatrix<50, NBT / 10, RowMajor, RowMajor, ColMajor>(
    "colmatrix = rowmatrix*rowmatrix static  50x50");
  checkMatrix<50, NBT / 10, ColMajor, ColMajor, RowMajor>(
    "rowmatrix = colmatrix*colmatrix static  50x50");
  checkMatrix<50, NBT / 10, RowMajor, ColMajor, RowMajor>(
    "rowmatrix = rowmatrix*colmatrix static  50x50");
  checkMatrix<50, NBT / 10, ColMajor, RowMajor, RowMajor>(
    "rowmatrix = colmatrix*rowmatrix static  50x50");
  checkMatrix<50, NBT / 10, RowMajor, RowMajor, RowMajor>(
    "rowmatrix = rowmatrix*rowmatrix static  50x50");

  checkMatrixTMatrix<50, NBT / 10, ColMajor, ColMajor, ColMajor>(
    "colmatrix = colmatrix.transpose()*colmatrix static  50x50");
  checkMatrixTMatrix<50, NBT / 10, RowMajor, ColMajor, ColMajor>(
    "colmatrix = rowmatrix.transpose()*colmatrix static  50x50");
  checkMatrixTMatrix<50, NBT / 10, ColMajor, RowMajor, ColMajor>(
    "colmatrix = colmatrix.transpose()*rowmatrix static  50x50");
  checkMatrixTMatrix<50, NBT / 10, RowMajor, RowMajor, ColMajor>(
    "colmatrix = rowmatrix.transpose()*rowmatrix static  50x50");
  checkMatrixTMatrix<50, NBT / 10, ColMajor, ColMajor, RowMajor>(
    "rowmatrix = colmatrix.transpose()*colmatrix static  50x50");
  checkMatrixTMatrix<50, NBT / 10, RowMajor, ColMajor, RowMajor>(
    "rowmatrix = rowmatrix.transpose()*colmatrix static  50x50");
  checkMatrixTMatrix<50, NBT / 10, ColMajor, RowMajor, RowMajor>(
    "rowmatrix = colmatrix.transpose()*rowmatrix static  50x50");
  checkMatrixTMatrix<50, NBT / 10, RowMajor, RowMajor, RowMajor>(
    "rowmatrix = rowmatrix.transpose()*rowmatrix static  50x50");

  checkDMatrix<NBT / 10, ColMajor, ColMajor, ColMajor>(
    50, "colmatrix = colmatrix*colmatrix dynamic  50x50");
  checkDMatrix<NBT / 10, RowMajor, ColMajor, ColMajor>(
    50, "colmatrix = rowmatrix*colmatrix dynamic  50x50");
  checkDMatrix<NBT / 10, ColMajor, RowMajor, ColMajor>(
    50, "colmatrix = colmatrix*rowmatrix dynamic  50x50");
  checkDMatrix<NBT / 10, RowMajor, RowMajor, ColMajor>(
    50, "colmatrix = rowmatrix*rowmatrix dynamic  50x50");
  checkDMatrix<NBT / 10, ColMajor, ColMajor, RowMajor>(
    50, "rowmatrix = colmatrix*colmatrix dynamic  50x50");
  checkDMatrix<NBT / 10, RowMajor, ColMajor, RowMajor>(
    50, "rowmatrix = rowmatrix*colmatrix dynamic  50x50");
  checkDMatrix<NBT / 10, ColMajor, RowMajor, RowMajor>(
    50, "rowmatrix = colmatrix*rowmatrix dynamic  50x50");
  checkDMatrix<NBT / 10, RowMajor, RowMajor, RowMajor>(
    50, "rowmatrix = rowmatrix*rowmatrix dynamic  50x50");

  checkDMatrixTMatrix<NBT / 10, ColMajor, ColMajor, ColMajor>(
    50, "colmatrix = colmatrix.transpose()*colmatrix dynamic  50x50");
  checkDMatrixTMatrix<NBT / 10, RowMajor, ColMajor, ColMajor>(
    50, "colmatrix = rowmatrix.transpose()*colmatrix dynamic  50x50");
  checkDMatrixTMatrix<NBT / 10, ColMajor, RowMajor, ColMajor>(
    50, "colmatrix = colmatrix.transpose()*rowmatrix dynamic  50x50");
  checkDMatrixTMatrix<NBT / 10, RowMajor, RowMajor, ColMajor>(
    50, "colmatrix = rowmatrix.transpose()*rowmatrix dynamic  50x50");
  checkDMatrixTMatrix<NBT / 10, ColMajor, ColMajor, RowMajor>(
    50, "rowmatrix = colmatrix.transpose()*colmatrix dynamic  50x50");
  checkDMatrixTMatrix<NBT / 10, RowMajor, ColMajor, RowMajor>(
    50, "rowmatrix = rowmatrix.transpose()*colmatrix dynamic  50x50");
  checkDMatrixTMatrix<NBT / 10, ColMajor, RowMajor, RowMajor>(
    50, "rowmatrix = colmatrix.transpose()*rowmatrix dynamic  50x50");
  checkDMatrixTMatrix<NBT / 10, RowMajor, RowMajor, RowMajor>(
    50, "rowmatrix = rowmatrix.transpose()*rowmatrix dynamic  50x50");

  std::cout << std::endl;

  std::cout << "--" << std::endl;
  return 0;
}
