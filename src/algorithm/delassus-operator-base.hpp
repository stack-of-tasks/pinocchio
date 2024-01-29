//
// Copyright (c) 2024 INRIA
//

#ifndef __pinocchio_algorithm_delassus_operator_base_hpp__
#define __pinocchio_algorithm_delassus_operator_base_hpp__

#include "pinocchio/algorithm/fwd.hpp"

namespace pinocchio {

template<typename DelassusOperatorDerived>
struct DelassusOperatorBase
{
  typedef typename traits<DelassusOperatorDerived>::Scalar Scalar;

  DelassusOperatorDerived & derived() { return static_cast<DelassusOperatorDerived&>(*this); }
  const DelassusOperatorDerived & derived() const { return static_cast<const DelassusOperatorDerived&>(*this); }

  Scalar computeLargestEigenValue(const int max_it = 10, const Scalar rel_tol = Scalar(1e-8)) const
  {
    return derived().computeLargestEigenValue(max_it, rel_tol);
  }

  template<typename VectorLike>
  void updateDamping(const Eigen::MatrixBase<VectorLike> & vec)
  {
    derived().updateDamping(vec.derived());
  }

  void updateDamping(const Scalar mu)
  {
    derived().updateDamping(mu);
  }

  template<typename MatrixLike>
  void solveInPlace(const Eigen::MatrixBase<MatrixLike> & mat) const
  {
    derived().solveInPlace(mat.const_cast_derived());
  }

  template<typename MatrixLike>
  typename PINOCCHIO_EIGEN_PLAIN_TYPE(MatrixLike)
  solve(const Eigen::MatrixBase<MatrixLike> & mat) const
  {
    return derived().solve(mat);
  }

  template<typename MatrixDerivedIn, typename MatrixDerivedOut>
  void solve(const Eigen::MatrixBase<MatrixDerivedIn> & x,
             const Eigen::MatrixBase<MatrixDerivedOut> & res) const
  {
    derived().solve(x.derived(), res.const_cast_derived());
  }

  template<typename MatrixIn, typename MatrixOut>
  void applyOnTheRight(const Eigen::MatrixBase<MatrixIn> & x,
                       const Eigen::MatrixBase<MatrixOut> & res) const
  {
    derived().applyOnTheRight(x.derived(), res.const_cast_derived());
  }

  template<typename MatrixDerived>
  typename PINOCCHIO_EIGEN_PLAIN_TYPE(MatrixDerived)
  operator*(const Eigen::MatrixBase<MatrixDerived> & x) const
  {
    return derived() * x.derived();
  }

  Eigen::DenseIndex size() const { return derived().size(); }
  Eigen::DenseIndex rows() const { return derived().rows(); }
  Eigen::DenseIndex cols() const { return derived().cols(); }
  
}; // struct DelassusOperatorBase

} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_delassus_operator_base_hpp__
