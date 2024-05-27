//
// Copyright (c) 2022-2023 INRIA
//

#ifndef __pinocchio_autodiff_casadi_math_triangular_matrix_hpp__
#define __pinocchio_autodiff_casadi_math_triangular_matrix_hpp__

#include "pinocchio/math/triangular-matrix.hpp"

namespace pinocchio
{

  namespace internal
  {
    template<Eigen::UpLoType info, typename RhsMatrix, typename Scalar>
    struct TriangularMatrixMatrixProduct<info, RhsMatrix, ::casadi::Matrix<Scalar>, true>
    {
      template<typename LhsMatrix, typename ResMat>
      static void run(
        const Eigen::MatrixBase<LhsMatrix> & lhs_mat,
        const Eigen::MatrixBase<RhsMatrix> & rhs_vec,
        const Eigen::MatrixBase<ResMat> & res)
      {
        res.const_cast_derived().col(0).noalias() = lhs_mat.derived() * rhs_vec.derived();
      }
    };

    template<Eigen::UpLoType info, typename RhsMatrix, typename Scalar>
    struct TriangularMatrixMatrixProduct<info, RhsMatrix, ::casadi::Matrix<Scalar>, false>
    {
      template<typename LhsMatrix, typename ResMat>
      static void run(
        const Eigen::MatrixBase<LhsMatrix> & lhs_mat,
        const Eigen::MatrixBase<RhsMatrix> & rhs_mat,
        const Eigen::MatrixBase<ResMat> & res)
      {
        res.const_cast_derived().noalias() = lhs_mat.derived() * rhs_mat.derived();
      }
    };
  } // namespace internal
} // namespace pinocchio

#endif // #ifndef __pinocchio_autodiff_casadi_math_triangular_matrix_hpp__
