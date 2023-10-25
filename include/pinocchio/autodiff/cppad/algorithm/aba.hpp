//
// Copyright (c) 2019-2020 INRIA CNRS
//

#ifndef __pinocchio_autodiff_cppad_algorithm_aba_hpp__
#define __pinocchio_autodiff_cppad_algorithm_aba_hpp__

namespace pinocchio
{
  namespace internal
  {

    //Fwd def
    template<typename _Scalar>  struct SE3actOn;
    
    /// \brief Partial specialization for CppAD::AGtypes
    template<typename _Scalar>
    struct SE3actOn< CppAD::AD<_Scalar> >
    {
      typedef CppAD::AD<_Scalar> Scalar;
      
      template<int Options, typename Matrix6Type>
      static typename PINOCCHIO_EIGEN_PLAIN_TYPE(Matrix6Type)
      run(const SE3Tpl<Scalar,Options> & M,
          const Eigen::MatrixBase<Matrix6Type> & I)
      {
        typedef SE3Tpl<Scalar,Options> SE3;
        
        typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(Matrix6Type) ReturnType;
        
        typename SE3::ActionMatrixType dual_action_matrix(M.toDualActionMatrix());
        typename SE3::ActionMatrixType action_matrix(M.toActionMatrixInverse());
        ReturnType intermediate_result = dual_action_matrix*I;
        ReturnType res = intermediate_result*action_matrix;
        return res;
      }
    };
  }
}

#endif // __pinocchio_autodiff_cppad_algorithm_aba_hpp__
