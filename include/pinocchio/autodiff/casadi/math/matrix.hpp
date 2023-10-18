//
// Copyright (c) 2019-2020 INRIA
//

#ifndef __pinocchio_autodiff_casadi_math_matrix_hpp__
#define __pinocchio_autodiff_casadi_math_matrix_hpp__

#include "pinocchio/math/matrix.hpp"

namespace pinocchio
{
  namespace internal
  {
    template<typename Scalar>
    struct CallCorrectMatrixInverseAccordingToScalar< ::casadi::Matrix<Scalar> >
    {
      typedef ::casadi::Matrix<Scalar> SX;
      template<typename MatrixIn, typename MatrixOut>
      static void run(const Eigen::MatrixBase<MatrixIn> & mat,
                      const Eigen::MatrixBase<MatrixOut> & dest)
      {
        SX cs_mat(mat.rows(),mat.cols());
        casadi::copy(mat.derived(),cs_mat);
        
        SX cs_mat_inv = SX::inv(cs_mat);
        
        MatrixOut & dest_ = PINOCCHIO_EIGEN_CONST_CAST(MatrixOut,dest);
        casadi::copy(cs_mat_inv,dest_);
      }

    };
  }
} // namespace pinocchio

#endif // ifndef __pinocchio_autodiff_casadi_math_matrix_hpp__
