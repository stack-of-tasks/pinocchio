//
// Copyright (c) 2024 INRIA
//

#ifndef __pinocchio_algorithm_delassus_operator_dense_hpp__
#define __pinocchio_algorithm_delassus_operator_dense_hpp__

#include "pinocchio/algorithm/fwd.hpp"
#include "pinocchio/algorithm/delassus-operator-base.hpp"

namespace pinocchio
{

  template<typename _Scalar, int _Options>
  struct traits<DelassusOperatorDenseTpl<_Scalar, _Options>>
  {
    typedef _Scalar Scalar;
    enum
    {
      Options = _Options,
      RowsAtCompileTime = Eigen::Dynamic
    };

    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Options> Matrix;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1, Options> Vector;
  };

  template<typename _Scalar, int _Options>
  struct DelassusOperatorDenseTpl
  : DelassusOperatorBase<DelassusOperatorDenseTpl<_Scalar, _Options>>
  {
    typedef _Scalar Scalar;
    typedef DelassusOperatorDenseTpl Self;
    enum
    {
      Options = _Options,
      RowsAtCompileTime = traits<DelassusOperatorDenseTpl>::RowsAtCompileTime
    };

    typedef typename traits<Self>::Matrix Matrix;
    typedef typename traits<Self>::Vector Vector;
    typedef Eigen::LLT<Matrix> CholeskyDecomposition;
    typedef DelassusOperatorBase<Self> Base;

    template<typename MatrixDerived>
    explicit DelassusOperatorDenseTpl(const Eigen::MatrixBase<MatrixDerived> & mat)
    : Base(mat.rows())
    , delassus_matrix(mat)
    , mat_tmp(mat.rows(), mat.cols())
    , llt(mat)
    , damping(Vector::Zero(mat.rows()))
    {
      PINOCCHIO_CHECK_ARGUMENT_SIZE(mat.rows(), mat.cols());
    }

    template<typename VectorLike>
    void updateDamping(const Eigen::MatrixBase<VectorLike> & vec)
    {
      damping = vec;
      mat_tmp = delassus_matrix;
      mat_tmp += vec.asDiagonal();
      llt.compute(mat_tmp);
    }

    void updateDamping(const Scalar & mu)
    {
      updateDamping(Vector::Constant(size(), mu));
    }

    template<typename MatrixLike>
    void solveInPlace(const Eigen::MatrixBase<MatrixLike> & mat) const
    {
      llt.solveInPlace(mat.const_cast_derived());
    }

    template<typename MatrixLike>
    typename PINOCCHIO_EIGEN_PLAIN_TYPE(MatrixLike)
      solve(const Eigen::MatrixBase<MatrixLike> & mat) const
    {
      typename PINOCCHIO_EIGEN_PLAIN_TYPE(MatrixLike) res(mat);
      solveInPlace(res);
      return res;
    }

    template<typename MatrixDerivedIn, typename MatrixDerivedOut>
    void solve(
      const Eigen::MatrixBase<MatrixDerivedIn> & x,
      const Eigen::MatrixBase<MatrixDerivedOut> & res) const
    {
      res.const_cast_derived() = x;
      solveInPlace(res.const_cast_derived());
    }

    template<typename MatrixIn, typename MatrixOut>
    void applyOnTheRight(
      const Eigen::MatrixBase<MatrixIn> & x, const Eigen::MatrixBase<MatrixOut> & res_) const
    {
      MatrixOut & res = res_.const_cast_derived();
      res.noalias() = delassus_matrix * x;
      res.array() += damping.array() * x.array();
    }

    template<typename MatrixDerived>
    typename PINOCCHIO_EIGEN_PLAIN_TYPE(MatrixDerived)
    operator*(const Eigen::MatrixBase<MatrixDerived> & x) const
    {
      typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(MatrixDerived) ReturnType;

      PINOCCHIO_CHECK_ARGUMENT_SIZE(x.rows(), size());
      ReturnType res(x.rows(), x.cols());
      applyOnTheRight(x, res);
      return res;
    }

    Eigen::DenseIndex size() const
    {
      return delassus_matrix.rows();
    }
    Eigen::DenseIndex rows() const
    {
      return delassus_matrix.rows();
    }
    Eigen::DenseIndex cols() const
    {
      return delassus_matrix.cols();
    }

    Matrix matrix() const
    {
      mat_tmp = delassus_matrix;
      mat_tmp += damping.asDiagonal();
      return mat_tmp;
    }

    Matrix inverse() const
    {
      Matrix res = Matrix::Identity(size(), size());
      llt.solveInPlace(res);
      return res;
    }

  protected:
    Matrix delassus_matrix;
    mutable Matrix mat_tmp;
    CholeskyDecomposition llt;
    Vector damping;

  }; // struct DelassusOperatorDenseTpl

} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_delassus_operator_dense_hpp__
