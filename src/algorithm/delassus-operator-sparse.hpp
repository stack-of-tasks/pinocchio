//
// Copyright (c) 2024 INRIA
//

#ifndef __pinocchio_algorithm_delassus_operator_sparse_hpp__
#define __pinocchio_algorithm_delassus_operator_sparse_hpp__

#include "pinocchio/algorithm/fwd.hpp"
#include "pinocchio/algorithm/delassus-operator-base.hpp"

#include "pinocchio/math/eigenvalues.hpp"

#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

namespace pinocchio {

template<typename _Scalar, int _Options>
struct traits<DelassusOperatorSparseTpl<_Scalar,_Options> >
{
  typedef _Scalar Scalar;
  enum { Options = _Options };

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
  enum { Options = _Options };

  typedef typename traits<Self>::Matrix Matrix;
  typedef typename traits<Self>::Vector Vector;
  typedef typename traits<Self>::DenseMatrix DenseMatrix;
  typedef Eigen::SimplicialLLT<Matrix> LLTDecomposition;

  template<typename MatrixDerived>
  explicit DelassusOperatorSparseTpl(const Eigen::SparseMatrixBase<MatrixDerived> & mat)
  : delassus_matrix(mat)
  , delassus_matrix_plus_damping(mat)
  , llt(mat)
  , damping(Vector::Zero(mat.rows()))
  , tmp(mat.rows())
  {
    PINOCCHIO_CHECK_ARGUMENT_SIZE(mat.rows(),mat.cols());
  }

  template<typename VectorLike>
  Scalar computeLargestEigenValue(const Eigen::PlainObjectBase<VectorLike> & eigenvector_est,
                                  const int max_it = 10,
                                  const Scalar rel_tol = Scalar(1e-8)) const
  {
    PINOCCHIO_CHECK_ARGUMENT_SIZE(eigenvector_est.size(),size());
    computeLargestEigenvector(*this,eigenvector_est.const_cast_derived(),max_it,rel_tol);
    return retrieveLargestEigenvalue(eigenvector_est);
  }

  Scalar computeLargestEigenValue(const int max_it = 10, const Scalar rel_tol = Scalar(1e-8)) const
  {
    Vector eigenvector_est(Vector::Constant(size(),Scalar(1)/Scalar(math::sqrt(size()))));
    computeLargestEigenvector(*this,eigenvector_est.const_cast_derived(),max_it,rel_tol);
    return retrieveLargestEigenvalue(eigenvector_est);
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
    llt._solve_impl(mat,mat.const_cast_derived(),tmp);
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

