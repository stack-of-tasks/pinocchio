//
// Copyright (c) 2024 INRIA
//

#ifndef __pinocchio_algorithm_delassus_operator_sparse_hpp__
#define __pinocchio_algorithm_delassus_operator_sparse_hpp__

#include "pinocchio/algorithm/fwd.hpp"
#include "pinocchio/algorithm/delassus-operator-base.hpp"

#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

namespace pinocchio {

namespace internal {

template<typename Derived>
struct SimplicialCholeskyWrapper : public Derived
{
  typedef Eigen::SimplicialCholeskyBase<Derived> Base;

  using Base::m_info;
  using Base::m_P;
  using Base::m_Pinv;
  using Base::m_diag;
  using Base::m_matrix;
  using Base::derived;

  template<typename Rhs,typename Dest, typename Temporary>
  void _solve_impl(const Eigen::MatrixBase<Rhs> &b,
                   Eigen::MatrixBase<Dest> &dest,
                   Eigen::MatrixBase<Temporary> &tmp) const
  {
//    eigen_assert(m_factorizationIsOk && "The decomposition is not in a valid state for solving, you must first call either compute() or symbolic()/numeric()");
//    eigen_assert(m_matrix.rows()==b.rows());

    if(m_info!=Eigen::Success)
      return;

    if(m_P.size()>0)
      tmp.noalias() = m_P * b;
    else
      tmp = b;

    if(m_matrix.nonZeros()>0) // otherwise L==I
      derived().matrixL().solveInPlace(tmp);

    if(m_diag.size()>0)
      tmp = m_diag.asDiagonal().inverse() * tmp;

    if (m_matrix.nonZeros()>0) // otherwise U==I
      derived().matrixU().solveInPlace(tmp);

    if(m_P.size()>0)
      dest.noalias() = m_Pinv * tmp;
  }

}; // SimplicialCholeskyWrapper

} // namespace internal

template<typename _Scalar, int _Options>
struct traits<DelassusOperatorSparseTpl<_Scalar,_Options> >
{
  typedef _Scalar Scalar;
  enum { Options = _Options, RowsAtCompileTime = Eigen::Dynamic  };

  typedef Eigen::SparseMatrix<Scalar,Options> Matrix;
  typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1,Options> Vector;
  typedef Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic,Options> DenseMatrix;
};

template<typename _Scalar, int _Options>
struct DelassusOperatorSparseTpl
: DelassusOperatorBase< DelassusOperatorSparseTpl<_Scalar,_Options> >
{
  typedef _Scalar Scalar;
  typedef DelassusOperatorSparseTpl Self;
  enum {
    Options = _Options,
    RowsAtCompileTime = traits<DelassusOperatorSparseTpl>::RowsAtCompileTime
  };

  typedef typename traits<Self>::Matrix Matrix;
  typedef typename traits<Self>::Vector Vector;
  typedef typename traits<Self>::DenseMatrix DenseMatrix;
  typedef Eigen::SimplicialLLT<Matrix> LLTDecomposition;
  typedef DelassusOperatorBase<Self> Base;

  template<typename MatrixDerived>
  explicit DelassusOperatorSparseTpl(const Eigen::SparseMatrixBase<MatrixDerived> & mat)
  : Base(mat.rows())
  , delassus_matrix(mat)
  , delassus_matrix_plus_damping(mat)
  , llt(mat)
  , damping(Vector::Zero(mat.rows()))
  , tmp(mat.rows())
  {
    PINOCCHIO_CHECK_ARGUMENT_SIZE(mat.rows(),mat.cols());
  }

  template<typename VectorLike>
  void updateDamping(const Eigen::MatrixBase<VectorLike> & vec)
  {
    for(Eigen::DenseIndex k = 0; k < size(); ++k)
    {
      delassus_matrix_plus_damping.coeffRef(k,k) += -damping[k] + vec[k];
    }
    damping = vec;
    PINOCCHIO_EIGEN_MALLOC_SAVE_STATUS();
    PINOCCHIO_EIGEN_MALLOC_ALLOWED();
    llt.factorize(delassus_matrix_plus_damping);
    PINOCCHIO_EIGEN_MALLOC_RESTORE_STATUS();
  }

  void updateDamping(const Scalar & mu)
  {
    updateDamping(Vector::Constant(size(),mu));
  }

  template<typename MatrixLike>
  void solveInPlace(const Eigen::MatrixBase<MatrixLike> & mat) const
  {
    typedef internal::SimplicialCholeskyWrapper<LLTDecomposition> WrapperLLT;

    const WrapperLLT & wrapper = reinterpret_cast<const WrapperLLT &>(llt);
    wrapper._solve_impl(mat,mat.const_cast_derived(),tmp);
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
  void solve(const Eigen::MatrixBase<MatrixDerivedIn> & x,
             const Eigen::MatrixBase<MatrixDerivedOut> & res) const
  {
    res.const_cast_derived() = x;
    llt._solve_impl(x,res.const_cast_derived());
  }

  template<typename MatrixIn, typename MatrixOut>
  void applyOnTheRight(const Eigen::MatrixBase<MatrixIn> & x,
                       const Eigen::MatrixBase<MatrixOut> & res_) const
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

    PINOCCHIO_CHECK_ARGUMENT_SIZE(x.rows(),size());
    ReturnType res(x.rows(),x.cols());
    applyOnTheRight(x,res);
    return res;
  }

  Eigen::DenseIndex size() const { return delassus_matrix.rows(); }
  Eigen::DenseIndex rows() const { return delassus_matrix.rows(); }
  Eigen::DenseIndex cols() const { return delassus_matrix.cols(); }

  Matrix matrix() const
  {
    delassus_matrix_plus_damping = delassus_matrix;
    delassus_matrix_plus_damping += damping.asDiagonal();
    return delassus_matrix_plus_damping;
  }
  
  DenseMatrix inverse() const
  {
    DenseMatrix res = llt.solve(DenseMatrix::Identity(size(),size()));
    return res;
  }

protected:

  Matrix delassus_matrix;
  mutable Matrix delassus_matrix_plus_damping;
  LLTDecomposition llt;
  Vector damping;
  mutable Vector tmp;


}; // struct DelassusOperatorSparseTpl

} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_delassus_operator_sparse_hpp__
