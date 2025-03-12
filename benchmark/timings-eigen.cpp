//
// Copyright (c) 2015-2025 CNRS
//

#include "pinocchio/macros.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <benchmark/benchmark.h>

using namespace Eigen;

static void CustomArguments(benchmark::internal::Benchmark * b)
{
  b->MinWarmUpTime(3.);
}

// quaternionToMatrix

PINOCCHIO_DONT_INLINE void quaternionToMatrixCall(const Quaterniond & q, Matrix3d & m)
{
  m = q.toRotationMatrix();
}
static void quaternionToMatrix(benchmark::State & st)
{
  Quaterniond q(Quaterniond(Vector4d::Random()).normalized());
  Matrix3d m(Matrix3d::Random());
  for (auto _ : st)
  {
    quaternionToMatrixCall(q, m);
    benchmark::DoNotOptimize(m);
  }
}
BENCHMARK(quaternionToMatrix)->Apply(CustomArguments);

// quaternionMultVector

PINOCCHIO_DONT_INLINE void
quaternionMultVectorCall(const Quaterniond & q, const Vector3d & rhs, Vector3d & lhs)
{
  lhs.noalias() = q * rhs;
}
static void quaternionMultVector(benchmark::State & st)
{
  Quaterniond q(Quaterniond(Vector4d::Random()).normalized());
  Vector3d rhs(Vector3d::Random());
  Vector3d lhs(Vector3d::Random());
  for (auto _ : st)
  {
    quaternionMultVectorCall(q, rhs, lhs);
    benchmark::DoNotOptimize(lhs);
  }
}
BENCHMARK(quaternionMultVector)->Apply(CustomArguments);

// quaternionMultQuaternion

PINOCCHIO_DONT_INLINE void
quaternionMultQuaternionCall(const Quaterniond & q, const Quaterniond & rhs, Quaterniond & lhs)
{
  lhs = q * rhs;
}
static void quaternionMultQuaternion(benchmark::State & st)
{
  Quaterniond q(Quaterniond(Vector4d::Random()).normalized());
  Quaterniond rhs(Quaterniond(Vector4d::Random()).normalized());
  Quaterniond lhs(Quaterniond(Vector4d::Random()).normalized());
  for (auto _ : st)
  {
    quaternionMultQuaternionCall(q, rhs, lhs);
    benchmark::DoNotOptimize(lhs);
  }
}
BENCHMARK(quaternionMultQuaternion)->Apply(CustomArguments);

// quaternionMultVectorX

PINOCCHIO_DONT_INLINE void
quaternionMultVectorXCall(const Quaterniond & q, const VectorXd & rhs, VectorXd & lhs)
{
  lhs.noalias() = q * rhs;
}
static void quaternionMultVectorX(benchmark::State & st)
{
  Quaterniond q(Quaterniond(Vector4d::Random()).normalized());
  VectorXd rhs(VectorXd::Random(3));
  VectorXd lhs(VectorXd::Random(3));
  for (auto _ : st)
  {
    quaternionMultVectorXCall(q, rhs, lhs);
  }
}
BENCHMARK(quaternionMultVectorX)->Apply(CustomArguments);

// matrixMultMatrix

template<int MSIZE, int OptionM1, int OptionM2, int OptionM3>
PINOCCHIO_DONT_INLINE void matrixMultMatrixCall(
  const Matrix<double, MSIZE, MSIZE, OptionM1> & m,
  const Matrix<double, MSIZE, MSIZE, OptionM2> & rhs,
  Matrix<double, MSIZE, MSIZE, OptionM3> & lhs)
{
  lhs.noalias() = m * rhs;
}
template<int MSIZE, int OptionM1, int OptionM2, int OptionM3>
static void matrixMultMatrix(benchmark::State & st)
{
  Matrix<double, MSIZE, MSIZE, OptionM1> m(Matrix<double, MSIZE, MSIZE, OptionM1>::Random());
  Matrix<double, MSIZE, MSIZE, OptionM2> rhs(Matrix<double, MSIZE, MSIZE, OptionM2>::Random());
  Matrix<double, MSIZE, MSIZE, OptionM3> lhs(Matrix<double, MSIZE, MSIZE, OptionM3>::Random());
  for (auto _ : st)
  {
    matrixMultMatrixCall(m, rhs, lhs);
  }
}

BENCHMARK(matrixMultMatrix<3, ColMajor, ColMajor, ColMajor>)->Apply(CustomArguments);
BENCHMARK(matrixMultMatrix<3, RowMajor, ColMajor, ColMajor>)->Apply(CustomArguments);
BENCHMARK(matrixMultMatrix<3, ColMajor, RowMajor, ColMajor>)->Apply(CustomArguments);
BENCHMARK(matrixMultMatrix<3, RowMajor, RowMajor, ColMajor>)->Apply(CustomArguments);
BENCHMARK(matrixMultMatrix<3, ColMajor, ColMajor, RowMajor>)->Apply(CustomArguments);
BENCHMARK(matrixMultMatrix<3, RowMajor, ColMajor, RowMajor>)->Apply(CustomArguments);
BENCHMARK(matrixMultMatrix<3, ColMajor, RowMajor, RowMajor>)->Apply(CustomArguments);
BENCHMARK(matrixMultMatrix<3, RowMajor, RowMajor, RowMajor>)->Apply(CustomArguments);
BENCHMARK(matrixMultMatrix<4, ColMajor, ColMajor, ColMajor>)->Apply(CustomArguments);
BENCHMARK(matrixMultMatrix<4, RowMajor, ColMajor, ColMajor>)->Apply(CustomArguments);
BENCHMARK(matrixMultMatrix<4, ColMajor, RowMajor, ColMajor>)->Apply(CustomArguments);
BENCHMARK(matrixMultMatrix<4, RowMajor, RowMajor, ColMajor>)->Apply(CustomArguments);
BENCHMARK(matrixMultMatrix<4, ColMajor, ColMajor, RowMajor>)->Apply(CustomArguments);
BENCHMARK(matrixMultMatrix<4, RowMajor, ColMajor, RowMajor>)->Apply(CustomArguments);
BENCHMARK(matrixMultMatrix<4, ColMajor, RowMajor, RowMajor>)->Apply(CustomArguments);
BENCHMARK(matrixMultMatrix<4, RowMajor, RowMajor, RowMajor>)->Apply(CustomArguments);
BENCHMARK(matrixMultMatrix<50, ColMajor, ColMajor, ColMajor>)->Apply(CustomArguments);
BENCHMARK(matrixMultMatrix<50, RowMajor, ColMajor, ColMajor>)->Apply(CustomArguments);
BENCHMARK(matrixMultMatrix<50, ColMajor, RowMajor, ColMajor>)->Apply(CustomArguments);
BENCHMARK(matrixMultMatrix<50, RowMajor, RowMajor, ColMajor>)->Apply(CustomArguments);
BENCHMARK(matrixMultMatrix<50, ColMajor, ColMajor, RowMajor>)->Apply(CustomArguments);
BENCHMARK(matrixMultMatrix<50, RowMajor, ColMajor, RowMajor>)->Apply(CustomArguments);
BENCHMARK(matrixMultMatrix<50, ColMajor, RowMajor, RowMajor>)->Apply(CustomArguments);
BENCHMARK(matrixMultMatrix<50, RowMajor, RowMajor, RowMajor>)->Apply(CustomArguments);

// matrixTransposeMultMatrix

template<int MSIZE, int OptionM1, int OptionM2, int OptionM3>
PINOCCHIO_DONT_INLINE void matrixTransposeMultMatrixCall(
  const Matrix<double, MSIZE, MSIZE, OptionM1> & m,
  const Matrix<double, MSIZE, MSIZE, OptionM2> & rhs,
  Matrix<double, MSIZE, MSIZE, OptionM3> & lhs)
{
  lhs.noalias() = m.transpose() * rhs;
}
template<int MSIZE, int OptionM1, int OptionM2, int OptionM3>
static void matrixTransposeMultMatrix(benchmark::State & st)
{
  Matrix<double, MSIZE, MSIZE, OptionM1> m(Matrix<double, MSIZE, MSIZE, OptionM1>::Random());
  Matrix<double, MSIZE, MSIZE, OptionM2> rhs(Matrix<double, MSIZE, MSIZE, OptionM2>::Random());
  Matrix<double, MSIZE, MSIZE, OptionM3> lhs(Matrix<double, MSIZE, MSIZE, OptionM3>::Random());
  for (auto _ : st)
  {
    matrixTransposeMultMatrixCall(m, rhs, lhs);
  }
}

BENCHMARK(matrixTransposeMultMatrix<4, ColMajor, ColMajor, ColMajor>)->Apply(CustomArguments);
BENCHMARK(matrixTransposeMultMatrix<4, RowMajor, ColMajor, ColMajor>)->Apply(CustomArguments);
BENCHMARK(matrixTransposeMultMatrix<4, ColMajor, RowMajor, ColMajor>)->Apply(CustomArguments);
BENCHMARK(matrixTransposeMultMatrix<4, RowMajor, RowMajor, ColMajor>)->Apply(CustomArguments);
BENCHMARK(matrixTransposeMultMatrix<4, ColMajor, ColMajor, RowMajor>)->Apply(CustomArguments);
BENCHMARK(matrixTransposeMultMatrix<4, RowMajor, ColMajor, RowMajor>)->Apply(CustomArguments);
BENCHMARK(matrixTransposeMultMatrix<4, ColMajor, RowMajor, RowMajor>)->Apply(CustomArguments);
BENCHMARK(matrixTransposeMultMatrix<4, RowMajor, RowMajor, RowMajor>)->Apply(CustomArguments);
BENCHMARK(matrixTransposeMultMatrix<50, ColMajor, ColMajor, ColMajor>)->Apply(CustomArguments);
BENCHMARK(matrixTransposeMultMatrix<50, RowMajor, ColMajor, ColMajor>)->Apply(CustomArguments);
BENCHMARK(matrixTransposeMultMatrix<50, ColMajor, RowMajor, ColMajor>)->Apply(CustomArguments);
BENCHMARK(matrixTransposeMultMatrix<50, RowMajor, RowMajor, ColMajor>)->Apply(CustomArguments);
BENCHMARK(matrixTransposeMultMatrix<50, ColMajor, ColMajor, RowMajor>)->Apply(CustomArguments);
BENCHMARK(matrixTransposeMultMatrix<50, RowMajor, ColMajor, RowMajor>)->Apply(CustomArguments);
BENCHMARK(matrixTransposeMultMatrix<50, ColMajor, RowMajor, RowMajor>)->Apply(CustomArguments);
BENCHMARK(matrixTransposeMultMatrix<50, RowMajor, RowMajor, RowMajor>)->Apply(CustomArguments);

// matrixMultVector

template<int MSIZE>
PINOCCHIO_DONT_INLINE void matrixMultVectorCall(
  const Matrix<double, MSIZE, MSIZE> & m,
  const Matrix<double, MSIZE, 1> & rhs,
  Matrix<double, MSIZE, 1> & lhs)
{
  lhs.noalias() = m * rhs;
}
template<int MSIZE>
static void matrixMultVector(benchmark::State & st)
{
  Matrix<double, MSIZE, MSIZE> m(Matrix<double, MSIZE, MSIZE>::Random());
  Matrix<double, MSIZE, 1> rhs(Matrix<double, MSIZE, 1>::Random());
  Matrix<double, MSIZE, 1> lhs(Matrix<double, MSIZE, 1>::Random());
  for (auto _ : st)
  {
    matrixMultVectorCall(m, rhs, lhs);
  }
}

BENCHMARK(matrixMultVector<3>)->Apply(CustomArguments);
BENCHMARK(matrixMultVector<4>)->Apply(CustomArguments);

// matrixDynamicMultMatrix

static void CustomArgumentsDynamicMatrix(benchmark::internal::Benchmark * b)
{
  b->MinWarmUpTime(3.)->Arg(3)->Arg(4)->Arg(50);
}

template<int OptionM1, int OptionM2, int OptionM3>
PINOCCHIO_DONT_INLINE void matrixDynamicMultMatrixCall(
  const Matrix<double, Dynamic, Dynamic, OptionM1> & m,
  const Matrix<double, Dynamic, Dynamic, OptionM2> & rhs,
  Matrix<double, Dynamic, Dynamic, OptionM3> & lhs)
{
  lhs.noalias() = m * rhs;
}
template<int OptionM1, int OptionM2, int OptionM3>
static void matrixDynamicMultMatrix(benchmark::State & st)
{
  const auto MSIZE = st.range(0);
  Matrix<double, Dynamic, Dynamic, OptionM1> m(
    Matrix<double, Dynamic, Dynamic, OptionM1>::Random(MSIZE, MSIZE));
  Matrix<double, Dynamic, Dynamic, OptionM2> rhs(
    Matrix<double, Dynamic, Dynamic, OptionM2>::Random(MSIZE, MSIZE));
  Matrix<double, Dynamic, Dynamic, OptionM3> lhs(
    Matrix<double, Dynamic, Dynamic, OptionM3>::Random(MSIZE, MSIZE));
  for (auto _ : st)
  {
    matrixDynamicMultMatrixCall(m, rhs, lhs);
  }
}

BENCHMARK(matrixDynamicMultMatrix<ColMajor, ColMajor, ColMajor>)
  ->Apply(CustomArgumentsDynamicMatrix);
BENCHMARK(matrixDynamicMultMatrix<RowMajor, ColMajor, ColMajor>)
  ->Apply(CustomArgumentsDynamicMatrix);
BENCHMARK(matrixDynamicMultMatrix<ColMajor, RowMajor, ColMajor>)
  ->Apply(CustomArgumentsDynamicMatrix);
BENCHMARK(matrixDynamicMultMatrix<RowMajor, RowMajor, ColMajor>)
  ->Apply(CustomArgumentsDynamicMatrix);
BENCHMARK(matrixDynamicMultMatrix<ColMajor, ColMajor, RowMajor>)
  ->Apply(CustomArgumentsDynamicMatrix);
BENCHMARK(matrixDynamicMultMatrix<RowMajor, ColMajor, RowMajor>)
  ->Apply(CustomArgumentsDynamicMatrix);
BENCHMARK(matrixDynamicMultMatrix<ColMajor, RowMajor, RowMajor>)
  ->Apply(CustomArgumentsDynamicMatrix);
BENCHMARK(matrixDynamicMultMatrix<RowMajor, RowMajor, RowMajor>)
  ->Apply(CustomArgumentsDynamicMatrix);

// matrixDynamicTransposeMultMatrix

static void CustomArgumentsDynamicMatrixTranspose(benchmark::internal::Benchmark * b)
{
  b->MinWarmUpTime(3.)->Arg(4)->Arg(50);
}

template<int OptionM1, int OptionM2, int OptionM3>
PINOCCHIO_DONT_INLINE void matrixDynamicTransposeMultMatrixCall(
  const Matrix<double, Dynamic, Dynamic, OptionM1> & m,
  const Matrix<double, Dynamic, Dynamic, OptionM2> & rhs,
  Matrix<double, Dynamic, Dynamic, OptionM3> & lhs)
{
  lhs.noalias() = m.transpose() * rhs;
}
template<int OptionM1, int OptionM2, int OptionM3>
static void matrixDynamicTransposeMultMatrix(benchmark::State & st)
{
  const auto MSIZE = st.range(0);
  Matrix<double, Dynamic, Dynamic, OptionM1> m(
    Matrix<double, Dynamic, Dynamic, OptionM1>::Random(MSIZE, MSIZE));
  Matrix<double, Dynamic, Dynamic, OptionM2> rhs(
    Matrix<double, Dynamic, Dynamic, OptionM2>::Random(MSIZE, MSIZE));
  Matrix<double, Dynamic, Dynamic, OptionM3> lhs(
    Matrix<double, Dynamic, Dynamic, OptionM3>::Random(MSIZE, MSIZE));
  for (auto _ : st)
  {
    matrixDynamicTransposeMultMatrixCall(m, rhs, lhs);
  }
}

BENCHMARK(matrixDynamicTransposeMultMatrix<ColMajor, ColMajor, ColMajor>)
  ->Apply(CustomArgumentsDynamicMatrixTranspose);
BENCHMARK(matrixDynamicTransposeMultMatrix<RowMajor, ColMajor, ColMajor>)
  ->Apply(CustomArgumentsDynamicMatrixTranspose);
BENCHMARK(matrixDynamicTransposeMultMatrix<ColMajor, RowMajor, ColMajor>)
  ->Apply(CustomArgumentsDynamicMatrixTranspose);
BENCHMARK(matrixDynamicTransposeMultMatrix<RowMajor, RowMajor, ColMajor>)
  ->Apply(CustomArgumentsDynamicMatrixTranspose);
BENCHMARK(matrixDynamicTransposeMultMatrix<ColMajor, ColMajor, RowMajor>)
  ->Apply(CustomArgumentsDynamicMatrixTranspose);
BENCHMARK(matrixDynamicTransposeMultMatrix<RowMajor, ColMajor, RowMajor>)
  ->Apply(CustomArgumentsDynamicMatrixTranspose);
BENCHMARK(matrixDynamicTransposeMultMatrix<ColMajor, RowMajor, RowMajor>)
  ->Apply(CustomArgumentsDynamicMatrixTranspose);
BENCHMARK(matrixDynamicTransposeMultMatrix<RowMajor, RowMajor, RowMajor>)
  ->Apply(CustomArgumentsDynamicMatrixTranspose);

// matrixDynamicMultVector

PINOCCHIO_DONT_INLINE void
matrixDynamicMultVectorCall(const MatrixXd & m, const MatrixXd & rhs, MatrixXd & lhs)
{
  lhs.noalias() = m * rhs;
}
static void matrixDynamicMultVector(benchmark::State & st)
{
  const auto MSIZE = st.range(0);
  MatrixXd m(MatrixXd::Random(MSIZE, MSIZE));
  MatrixXd rhs(MatrixXd::Random(MSIZE, 1));
  MatrixXd lhs(MatrixXd::Random(MSIZE, 1));
  for (auto _ : st)
  {
    matrixDynamicMultVectorCall(m, rhs, lhs);
  }
}

BENCHMARK(matrixDynamicMultVector)->Apply(CustomArguments)->Arg(3)->Arg(4);

BENCHMARK_MAIN();
