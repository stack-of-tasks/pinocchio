//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_math_eigenvalues_hpp__
#define __pinocchio_math_eigenvalues_hpp__

#include "pinocchio/math/fwd.hpp"
#include <Eigen/Core>

namespace pinocchio
{

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
    typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(VectorLike) VectorPlain;
    typedef typename MatrixLike::Scalar Scalar;

    VectorLike & eigenvector_est = _eigenvector_est.const_cast_derived();
    VectorPlain eigenvector_prev(eigenvector_est); // to avoid memory allocation
    Scalar eigenvalue_est = eigenvector_est.norm();

    for(int it = 0; it < max_it; ++it)
    {
      const Scalar eigenvalue_est_prev = eigenvalue_est;
      eigenvector_est /= eigenvalue_est;
      eigenvector_prev = eigenvector_est;
      eigenvector_est.noalias() = mat * eigenvector_prev;

      eigenvalue_est = eigenvector_est.norm();

      if(std::fabs(eigenvalue_est_prev - eigenvalue_est) < rel_tol)
        break;
    }
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
    typedef typename MatrixLike::Scalar Scalar;
    const Scalar constant_value = Scalar(1)/math::sqrt(mat.rows());
    Vector eigenvector_est(Vector::Constant(mat.rows(),constant_value));

    computeLargestEigenvector(mat, eigenvector_est, max_it, rel_tol);

    return eigenvector_est;
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
