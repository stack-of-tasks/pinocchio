//
// Copyright (c) 2019-2020 INRIA
//

#ifndef __pinocchio_autodiff_casadi_math_quaternion_hpp__
#define __pinocchio_autodiff_casadi_math_quaternion_hpp__

#include "pinocchio/math/quaternion.hpp"

namespace pinocchio
{
  namespace quaternion
  {
    namespace internal
    {
      
      template<typename _Scalar>
      struct quaternionbase_assign_impl< ::casadi::Matrix<_Scalar>, false >
      {
        typedef ::casadi::Matrix<_Scalar> Scalar;
        template<typename Matrix3, typename QuaternionDerived>
        static inline void run(Eigen::QuaternionBase<QuaternionDerived> & q,
                               const Matrix3 & mat)
        {
          typedef typename Eigen::internal::traits<QuaternionDerived>::Coefficients QuatCoefficients;
          
          typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(QuatCoefficients) QuatCoefficientsPlainType;
          typedef Eigen::Quaternion<Scalar,QuatCoefficientsPlainType::Options> QuaternionPlain;
          QuaternionPlain quat_t_positive;
          
          Scalar t = mat.trace();
          quaternionbase_assign_impl_if_t_positive::run(t,quat_t_positive,mat);
          
          QuaternionPlain quat_t_negative_0, quat_t_negative_1, quat_t_negative_2;
          
          quaternionbase_assign_impl_if_t_negative<0>::run(t,quat_t_negative_0,mat);
          quaternionbase_assign_impl_if_t_negative<1>::run(t,quat_t_negative_1,mat);
          quaternionbase_assign_impl_if_t_negative<2>::run(t,quat_t_negative_2,mat);
          
          // Build the expression graph
          const Scalar t_greater_than_zero = t > Scalar(0);
          const Scalar cond1 = mat.coeff(1,1) > mat.coeff(0,0);
          const Scalar cond2 = (cond1 && mat.coeff(2,2) > mat.coeff(1,1)) || (mat.coeff(2,2) > mat.coeff(0,0));
          
          for(Eigen::DenseIndex k = 0; k < 4; ++k)
          {
            Scalar t_is_negative_cond1 = Scalar::if_else(cond1,
                                                         quat_t_negative_1.coeffs().coeff(k),
                                                         quat_t_negative_0.coeffs().coeff(k));
            Scalar t_is_negative_cond2 = Scalar::if_else(cond2,
                                                         quat_t_negative_2.coeffs().coeff(k),
                                                         t_is_negative_cond1);
            
            q.coeffs().coeffRef(k) = Scalar::if_else(t_greater_than_zero,
                                                     quat_t_positive.coeffs().coeff(k),
                                                     t_is_negative_cond2);
          }
        }
      };
      
    } // namespace internal
    
  } // namespace quaternion
  
} // namespace pinocchio

#endif // ifndef __pinocchio_autodiff_casadi_math_quaternion_hpp__
