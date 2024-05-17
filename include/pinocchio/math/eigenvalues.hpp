//
// Copyright (c) 2022-2024 INRIA
//

#ifndef __pinocchio_math_eigenvalues_hpp__
#define __pinocchio_math_eigenvalues_hpp__

#include "pinocchio/math/fwd.hpp"
#include <Eigen/Core>

namespace pinocchio
{

  /// \brief Compute the largest eigenvalues and the associated principle eigenvector via power
  /// iteration
  template<typename _Vector>
  struct PowerIterationAlgoTpl
  {
    typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(_Vector) Vector;
    typedef typename Vector::Scalar Scalar;

    explicit PowerIterationAlgoTpl(
      const Eigen::DenseIndex size, const int max_it = 10, const Scalar rel_tol = Scalar(1e-8))
    : principal_eigen_vector(size)
    , lowest_eigen_vector(size)
    , max_it(max_it)
    , it(0)
    , rel_tol(rel_tol)
    , eigen_vector_prev(size)
    {
      reset();
    }

    template<typename MatrixLike>
    void run(const MatrixLike & mat)
    {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(max_it >= 1);
      PINOCCHIO_CHECK_INPUT_ARGUMENT(rel_tol > Scalar(0));
      Scalar eigenvalue_est = principal_eigen_vector.norm();

      for (it = 0; it < max_it; ++it)
      {
        const Scalar eigenvalue_est_prev = eigenvalue_est;
        principal_eigen_vector /= eigenvalue_est;
        eigen_vector_prev = principal_eigen_vector;
        principal_eigen_vector.noalias() = mat * eigen_vector_prev;

        eigenvalue_est = principal_eigen_vector.norm();

        convergence_criteria = math::fabs(eigenvalue_est_prev - eigenvalue_est);
        if (check_expression_if_real<Scalar, false>(
              convergence_criteria
              <= rel_tol * math::max(math::fabs(eigenvalue_est_prev), math::fabs(eigenvalue_est))))
          break;
      }

      largest_eigen_value = eigenvalue_est;
    }

    template<typename MatrixLike, typename VectorLike>
    void run(const MatrixLike & mat, const Eigen::PlainObjectBase<VectorLike> & eigenvector_est)
    {
      principal_eigen_vector = eigenvector_est;
      run(mat);
    }

    template<typename MatrixLike>
    void lowest(const MatrixLike & mat, const bool compute_largest = true)
    {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(max_it >= 1);
      PINOCCHIO_CHECK_INPUT_ARGUMENT(rel_tol > Scalar(0));

      if (compute_largest)
        run(mat);

      Scalar eigenvalue_est = lowest_eigen_vector.norm();

      for (it = 0; it < max_it; ++it)
      {
        const Scalar eigenvalue_est_prev = eigenvalue_est;
        lowest_eigen_vector /= eigenvalue_est;
        eigen_vector_prev = lowest_eigen_vector;
        lowest_eigen_vector.noalias() = mat * eigen_vector_prev;
        lowest_eigen_vector -= largest_eigen_value * eigen_vector_prev;

        eigenvalue_est = lowest_eigen_vector.norm();

        convergence_criteria = math::fabs(eigenvalue_est_prev - eigenvalue_est);
        if (check_expression_if_real<Scalar, false>(
              convergence_criteria
              <= rel_tol * math::max(math::fabs(eigenvalue_est_prev), math::fabs(eigenvalue_est))))
          break;
      }

      lowest_eigen_value = largest_eigen_value - eigenvalue_est;
    }

    template<typename MatrixLike, typename VectorLike>
    void lowest(
      const MatrixLike & mat,
      const Eigen::PlainObjectBase<VectorLike> & largest_eigenvector_est,
      const Eigen::PlainObjectBase<VectorLike> & lowest_eigenvector_est,
      const bool compute_largest = true)
    {
      principal_eigen_vector = largest_eigenvector_est;
      lowest_eigen_vector = lowest_eigenvector_est;
      lowest(mat, compute_largest);
    }

    void reset()
    {
      const Scalar normalized_value = Scalar(1) / math::sqrt(Scalar(principal_eigen_vector.size()));
      principal_eigen_vector.fill(normalized_value);
      lowest_eigen_vector.fill(normalized_value);

      largest_eigen_value = std::numeric_limits<Scalar>::min();
      lowest_eigen_value = std::numeric_limits<Scalar>::max();
    }

    Vector principal_eigen_vector;
    Vector lowest_eigen_vector;
    Scalar largest_eigen_value;
    Scalar lowest_eigen_value;
    int max_it;
    int it;
    Scalar rel_tol;
    Scalar convergence_criteria;

  protected:
    Vector eigen_vector_prev;
  }; // struct PowerIterationAlgoTpl

  ///
  ///  \brief Compute the lagest eigenvector of a given matrix according to a given eigenvector
  /// estimate.
  ///
  template<typename MatrixLike, typename VectorLike>
  void computeLargestEigenvector(
    const MatrixLike & mat,
    const Eigen::PlainObjectBase<VectorLike> & _eigenvector_est,
    const int max_it = 10,
    const typename MatrixLike::Scalar rel_tol = 1e-8)
  {
    PowerIterationAlgoTpl<VectorLike> algo(mat.rows(), max_it, rel_tol);
    algo.run(mat, _eigenvector_est.derived());
    _eigenvector_est.const_cast_derived() = algo.principal_eigen_vector;
  }

  ///
  ///  \brief Compute the lagest eigenvector of a given matrix.
  ///
  template<typename MatrixLike>
  Eigen::Matrix<typename MatrixLike::Scalar, MatrixLike::RowsAtCompileTime, 1>
  computeLargestEigenvector(
    const MatrixLike & mat, const int max_it = 10, const typename MatrixLike::Scalar rel_tol = 1e-8)
  {
    typedef Eigen::Matrix<typename MatrixLike::Scalar, MatrixLike::RowsAtCompileTime, 1> Vector;
    PowerIterationAlgoTpl<Vector> algo(mat.rows(), max_it, rel_tol);
    algo.run(mat);
    return algo.principal_eigen_vector;
  }

  ///
  ///  \brief Compute the lagest eigenvector of a given matrix according to a given eigenvector
  /// estimate.
  ///
  template<typename MatrixLike, typename VectorLike1, typename VectorLike2>
  void computeLowestEigenvector(
    const MatrixLike & mat,
    const Eigen::PlainObjectBase<VectorLike1> & largest_eigenvector_est,
    const Eigen::PlainObjectBase<VectorLike2> & lowest_eigenvector_est,
    const bool compute_largest = true,
    const int max_it = 10,
    const typename MatrixLike::Scalar rel_tol = 1e-8)
  {
    PowerIterationAlgoTpl<VectorLike1> algo(mat.rows(), max_it, rel_tol);
    algo.lowest(
      mat, largest_eigenvector_est.derived(), lowest_eigenvector_est.derived(), compute_largest);
    largest_eigenvector_est.const_cast_derived() = algo.principal_eigen_vector;
    lowest_eigenvector_est.const_cast_derived() = algo.lowest_eigen_vector;
  }

  ///
  ///  \brief Compute the lagest eigenvector of a given matrix.
  ///
  template<typename MatrixLike>
  Eigen::Matrix<typename MatrixLike::Scalar, MatrixLike::RowsAtCompileTime, 1>
  computeLowestEigenvector(
    const MatrixLike & mat,
    const bool compute_largest = true,
    const int max_it = 10,
    const typename MatrixLike::Scalar rel_tol = 1e-8)
  {
    typedef Eigen::Matrix<typename MatrixLike::Scalar, MatrixLike::RowsAtCompileTime, 1> Vector;
    PowerIterationAlgoTpl<Vector> algo(mat.rows(), max_it, rel_tol);
    algo.lowest(mat, compute_largest);
    return algo.lowest_eigen_vector;
  }

  ///
  ///  \brief Compute the lagest eigenvalue of a given matrix. This is taking the eigenvector
  /// computed by the function computeLargestEigenvector.
  ///
  template<typename VectorLike>
  typename VectorLike::Scalar
  retrieveLargestEigenvalue(const Eigen::MatrixBase<VectorLike> & eigenvector)
  {
    return eigenvector.norm();
  }
} // namespace pinocchio

#endif // #ifndef __pinocchio_math_eigenvalues_hpp__
