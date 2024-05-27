//
// Copyright (c) 2024 INRIA
//

#ifndef __pinocchio_algorithm_delassus_operator_base_hpp__
#define __pinocchio_algorithm_delassus_operator_base_hpp__

#include "pinocchio/algorithm/fwd.hpp"
#include "pinocchio/math/eigenvalues.hpp"

namespace pinocchio
{

  template<typename DelassusOperatorDerived>
  struct DelassusOperatorBase
  {
    typedef typename traits<DelassusOperatorDerived>::Scalar Scalar;
    typedef typename traits<DelassusOperatorDerived>::Vector Vector;
    typedef PowerIterationAlgoTpl<Vector> PowerIterationAlgo;

    DelassusOperatorDerived & derived()
    {
      return static_cast<DelassusOperatorDerived &>(*this);
    }
    const DelassusOperatorDerived & derived() const
    {
      return static_cast<const DelassusOperatorDerived &>(*this);
    }

    explicit DelassusOperatorBase(const Eigen::DenseIndex size)
    : power_iteration_algo(size)
    {
    }

    Scalar computeLargestEigenValue(
      const bool reset = true, const int max_it = 10, const Scalar rel_tol = Scalar(1e-8)) const
    {
      power_iteration_algo.max_it = max_it;
      power_iteration_algo.rel_tol = rel_tol;
      if (reset)
        power_iteration_algo.reset();

      power_iteration_algo.run(derived());

      return power_iteration_algo.largest_eigen_value;
    }

    template<typename VectorLike>
    Scalar computeLargestEigenValue(
      const Eigen::PlainObjectBase<VectorLike> & largest_eigenvector_est,
      const bool reset = true,
      const int max_it = 10,
      const Scalar rel_tol = Scalar(1e-8)) const
    {
      PINOCCHIO_CHECK_ARGUMENT_SIZE(largest_eigenvector_est.size(), size());
      power_iteration_algo.max_it = max_it;
      power_iteration_algo.rel_tol = rel_tol;
      if (reset)
        power_iteration_algo.reset();
      power_iteration_algo.principal_eigen_vector = largest_eigenvector_est;

      power_iteration_algo.run(derived());

      return power_iteration_algo.largest_eigen_value;
    }

    Scalar computeLowestEigenValue(
      const bool reset = true,
      const bool compute_largest = true,
      const int max_it = 10,
      const Scalar rel_tol = Scalar(1e-8)) const
    {
      power_iteration_algo.max_it = max_it;
      power_iteration_algo.rel_tol = rel_tol;
      if (reset)
        power_iteration_algo.reset();

      power_iteration_algo.lowest(derived(), compute_largest);

      return power_iteration_algo.lowest_eigen_value;
    }

    template<typename VectorLike1, typename VectorLike2>
    Scalar computeLowestEigenValue(
      const Eigen::PlainObjectBase<VectorLike1> & largest_eigenvector_est,
      const Eigen::PlainObjectBase<VectorLike2> & lowest_eigenvector_est,
      const bool reset = true,
      const bool compute_largest = true,
      const int max_it = 10,
      const Scalar rel_tol = Scalar(1e-8)) const
    {
      PINOCCHIO_CHECK_ARGUMENT_SIZE(largest_eigenvector_est.size(), size());
      PINOCCHIO_CHECK_ARGUMENT_SIZE(lowest_eigenvector_est.size(), size());

      power_iteration_algo.max_it = max_it;
      power_iteration_algo.rel_tol = rel_tol;
      if (reset)
        power_iteration_algo.reset();
      power_iteration_algo.principal_eigen_vector = largest_eigenvector_est;
      power_iteration_algo.lowest_eigen_vector = lowest_eigenvector_est;

      power_iteration_algo.lowest(derived(), compute_largest);

      return power_iteration_algo.lowest_eigen_value;
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
    void solve(
      const Eigen::MatrixBase<MatrixDerivedIn> & x,
      const Eigen::MatrixBase<MatrixDerivedOut> & res) const
    {
      derived().solve(x.derived(), res.const_cast_derived());
    }

    template<typename MatrixIn, typename MatrixOut>
    void applyOnTheRight(
      const Eigen::MatrixBase<MatrixIn> & x, const Eigen::MatrixBase<MatrixOut> & res) const
    {
      derived().applyOnTheRight(x.derived(), res.const_cast_derived());
    }

    template<typename MatrixDerived>
    typename PINOCCHIO_EIGEN_PLAIN_TYPE(MatrixDerived)
    operator*(const Eigen::MatrixBase<MatrixDerived> & x) const
    {
      return derived() * x.derived();
    }

    Eigen::DenseIndex size() const
    {
      return derived().size();
    }
    Eigen::DenseIndex rows() const
    {
      return derived().rows();
    }
    Eigen::DenseIndex cols() const
    {
      return derived().cols();
    }

    PowerIterationAlgo & getPowerIterationAlgo()
    {
      return power_iteration_algo;
    }

    const PowerIterationAlgo & getPowerIterationAlgo() const
    {
      return power_iteration_algo;
    }

  protected:
    mutable PowerIterationAlgo power_iteration_algo;

  }; // struct DelassusOperatorBase

} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_delassus_operator_base_hpp__
