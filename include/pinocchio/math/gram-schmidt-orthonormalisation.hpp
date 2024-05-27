//
// Copyright (c) 2024 INRIA
//

#ifndef __pinocchio_math_gram_schmidt_orthonormalisation_hpp__
#define __pinocchio_math_gram_schmidt_orthonormalisation_hpp__

#include "pinocchio/math/fwd.hpp"
#include <Eigen/Core>

namespace pinocchio
{
  ///  \brief Perform the Gram-Schmidt orthonormalisation on the input/output vector for a given
  /// input basis
  ///
  ///  \param[in] basis Orthonormal basis
  ///  \param[in,out] vec Vector to orthonomarlize wrt the input basis
  ///
  template<typename MatrixType, typename VectorType>
  void orthonormalisation(
    const Eigen::MatrixBase<MatrixType> & basis, const Eigen::MatrixBase<VectorType> & vec_)
  {
    typedef typename VectorType::Scalar Scalar;
    VectorType & vec = vec_.const_cast_derived();

    PINOCCHIO_CHECK_ARGUMENT_SIZE(basis.rows(), vec.size());
    assert((basis.transpose() * basis).isIdentity() && "The input basis is not orthonormal.");

    for (Eigen::DenseIndex col_id = 0; col_id < basis.cols(); ++col_id)
    {
      const auto col = basis.col(col_id);
      const Scalar alpha = col.dot(vec);
      vec -= alpha * col;
    }

    assert((basis.transpose() * vec).isZero());
  }
} // namespace pinocchio

#endif // ifndef __pinocchio_math_gram_schmidt_orthonormalisation_hpp__
