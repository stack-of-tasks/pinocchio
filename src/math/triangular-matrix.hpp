//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_math_triangular_matrix_hpp__
#define __pinocchio_math_triangular_matrix_hpp__

#include "pinocchio/macros.hpp"

#include <Eigen/Dense>

namespace pinocchio
{
  
  namespace internal
  {
    template<typename MatrixDerived, bool is_vector_at_compile_time = MatrixDerived::IsVectorAtCompileTime>
    struct TriangularMatrixMatrixProduct
    {
      template<typename TriangularDerived, typename Result>
      static void run(const Eigen::TriangularBase<TriangularDerived> & lhs_triangular_mat,
                      const Eigen::MatrixBase<MatrixDerived> & rhs_vec,
                      const Eigen::MatrixBase<Result> & res)
      {
        res.const_cast_derived().col(0).noalias() = lhs_triangular_mat.derived() * rhs_vec.derived();
      }
    };
    
    template<typename MatrixDerived>
    struct TriangularMatrixMatrixProduct<MatrixDerived,false>
    {
      template<typename TriangularDerived, typename Result>
      static void run(const Eigen::TriangularBase<TriangularDerived> & lhs_triangular_mat,
                      const Eigen::MatrixBase<MatrixDerived> & rhs_mat,
                      const Eigen::MatrixBase<Result> & res)
      {
        res.const_cast_derived().noalias() = lhs_triangular_mat.derived() * rhs_mat.derived();
      }
    };
  }
  
  ///
  /// \brief Evaluate the product of a triangular matrix times a matrix. Eigen showing a bug at this level, in the case of vector entry.
  ///
  /// \param[in] lhs_triangular_mat Input tringular matrix
  /// \param[in] rhs_mat Right hand side operand in the multplication
  /// \param[in] res Resulting matrix
  ///
  template<typename TriangularDerived, typename MatrixDerived, typename Result>
  inline void triangularMatrixMatrixProduct(const Eigen::TriangularBase<TriangularDerived> & lhs_triangular_mat,
                                            const Eigen::MatrixBase<MatrixDerived> & rhs_mat,
                                            const Eigen::MatrixBase<Result> & res)
  {
    internal::TriangularMatrixMatrixProduct<MatrixDerived>::run(lhs_triangular_mat.derived(),
                                                                rhs_mat.derived(),
                                                                res.derived());
  }


}

#endif //#ifndef __pinocchio_math_triangular_matrix_hpp__
