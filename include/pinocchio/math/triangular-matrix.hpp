//
// Copyright (c) 2022-2023 INRIA
//

#ifndef __pinocchio_math_triangular_matrix_hpp__
#define __pinocchio_math_triangular_matrix_hpp__

#include "pinocchio/macros.hpp"

#include <Eigen/Dense>

namespace pinocchio
{

  namespace internal
  {
    template<
      Eigen::UpLoType info,
      typename RhsMatrix,
      typename Scalar = typename RhsMatrix::Scalar,
      bool is_vector_at_compile_time = RhsMatrix::IsVectorAtCompileTime>
    struct TriangularMatrixMatrixProduct
    {
      template<typename LhsMatrix, typename ResMat>
      static void run(
        const Eigen::MatrixBase<LhsMatrix> & lhs_mat,
        const Eigen::MatrixBase<RhsMatrix> & rhs_vec,
        const Eigen::MatrixBase<ResMat> & res)
      {
        res.const_cast_derived().col(0).noalias() =
          lhs_mat.derived().template triangularView<info>() * rhs_vec.derived();
      }
    };

    template<Eigen::UpLoType info, typename RhsMatrix, typename Scalar>
    struct TriangularMatrixMatrixProduct<info, RhsMatrix, Scalar, false>
    {
      template<typename LhsMatrix, typename ResMat>
      static void run(
        const Eigen::MatrixBase<LhsMatrix> & lhs_mat,
        const Eigen::MatrixBase<RhsMatrix> & rhs_mat,
        const Eigen::MatrixBase<ResMat> & res)
      {
        res.const_cast_derived().noalias() =
          lhs_mat.derived().template triangularView<info>() * rhs_mat.derived();
      }
    };
  } // namespace internal

  ///
  /// \brief Evaluate the product of a triangular matrix times a matrix. Eigen showing a bug at this
  /// level, in the case of vector entry.
  ///
  /// \param[in] lhs_mat Input tringular matrix
  /// \param[in] rhs_mat Right hand side operand in the multplication
  /// \param[in] res Resulting matrix
  ///
  template<Eigen::UpLoType info, typename LhsMatrix, typename RhsMatrix, typename ResMat>
  inline void triangularMatrixMatrixProduct(
    const Eigen::MatrixBase<LhsMatrix> & lhs_mat,
    const Eigen::MatrixBase<RhsMatrix> & rhs_mat,
    const Eigen::MatrixBase<ResMat> & res)
  {
    internal::TriangularMatrixMatrixProduct<info, RhsMatrix>::run(
      lhs_mat.derived(), rhs_mat.derived(), res.const_cast_derived());
  }

} // namespace pinocchio

#endif // #ifndef __pinocchio_math_triangular_matrix_hpp__
