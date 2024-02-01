//
// Copyright (c) 2022-2024 INRIA
//

#ifndef __pinocchio_math_eigenvalues_hpp__
#define __pinocchio_math_eigenvalues_hpp__

#include "pinocchio/math/fwd.hpp"
#include <Eigen/Core>

namespace pinocchio
{

  /// \brief Compute the largest eigen values and the associated principle eigen vector via power iteration
  template<typename _Vector>
  struct PowerIterationAlgoTpl
  {
    typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(_Vector) Vector;
    typedef typename Vector::Scalar Scalar;

    explicit PowerIterationAlgoTpl(const Eigen::DenseIndex size,
                                   const int max_it = 10,
                                   const Scalar rel_tol = 1e-8)
    : principal_eigen_vector(size)
    , max_it(max_it)
    , it(0)
    , rel_tol(rel_tol)
    , principal_eigen_vector_prev(size)
    {
      reset();
    }

    template<typename MatrixLike>
    void run(const MatrixLike & mat)
    {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(max_it >= 1);
      PINOCCHIO_CHECK_INPUT_ARGUMENT(rel_tol > Scalar(0));
      Scalar eigenvalue_est = principal_eigen_vector.norm();

      for(it = 0; it < max_it; ++it)
      {
        const Scalar eigenvalue_est_prev = eigenvalue_est;
        principal_eigen_vector /= eigenvalue_est;
        principal_eigen_vector_prev = principal_eigen_vector;
        principal_eigen_vector.noalias() = mat * principal_eigen_vector_prev;

        eigenvalue_est = principal_eigen_vector.norm();

        convergence_criteria = math::fabs(eigenvalue_est_prev - eigenvalue_est);
        if (check_expression_if_real<Scalar, false>(convergence_criteria <= rel_tol * math::max(math::fabs(eigenvalue_est_prev),math::fabs(eigenvalue_est))))
          break;
      }

      largest_eigen_value = eigenvalue_est;
    }

    template<typename MatrixLike, typename VectorLike>
    void run(const MatrixLike & mat,
             const Eigen::PlainObjectBase<VectorLike> & eigenvector_est)
    {
      principal_eigen_vector = eigenvector_est;
      run(mat);
    }

    void reset()
    {
      principal_eigen_vector.fill(Scalar(1)/math::sqrt(Scalar(principal_eigen_vector.size())));
      largest_eigen_value = std::numeric_limits<Scalar>::min();
    }

    Vector principal_eigen_vector;
    Scalar largest_eigen_value;
    int max_it;
    int it;
    Scalar rel_tol;
    Scalar convergence_criteria;

  protected:

    Vector principal_eigen_vector_prev;
  }; // struct PowerIterationAlgoTpl

  ///
  /// \brief Compute the lagest eigenvector of a given matrix according to a given eigenvector estimate.
  ///
  template<typename MatrixLike, typename VectorLike>
  void
  computeLargestEigenvector(const MatrixLike & mat,
                            const Eigen::PlainObjectBase<VectorLike> & _eigenvector_est,
                            const int max_it = 10,
                            const typename MatrixLike::Scalar rel_tol = 1e-8)
  {
    PowerIterationAlgoTpl<VectorLike> algo(mat.rows(),max_it,rel_tol);
    algo.run(mat,_eigenvector_est.derived());
    _eigenvector_est.const_cast_derived() = algo.principal_eigen_vector;
  }

  ///
  /// \brief Compute the lagest eigenvector of a given matrix.
  ///
  template<typename MatrixLike>
  Eigen::Matrix<typename MatrixLike::Scalar, MatrixLike::RowsAtCompileTime, 1>
  computeLargestEigenvector(const MatrixLike & mat,
                            const int max_it = 10,
                            const typename MatrixLike::Scalar rel_tol = 1e-8)
  {
    typedef Eigen::Matrix<typename MatrixLike::Scalar, MatrixLike::RowsAtCompileTime, 1> Vector;
    PowerIterationAlgoTpl<Vector> algo(mat.rows(),max_it,rel_tol);
    algo.run(mat);
    return algo.principal_eigen_vector;
  }

  ///
  /// \brief Compute the lagest eigenvalue of a given matrix. This is taking the eigenvector computed by the function computeLargestEigenvector.
  ///
  template<typename VectorLike>
  typename VectorLike::Scalar
  retrieveLargestEigenvalue(const Eigen::MatrixBase<VectorLike> & eigenvector)
  {
    return eigenvector.norm();
  }
}

#endif //#ifndef __pinocchio_math_eigenvalues_hpp__
