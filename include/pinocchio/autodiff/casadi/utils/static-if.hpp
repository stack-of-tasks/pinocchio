//
// Copyright (c) 2019-2020 INRIA
//

#ifndef __pinocchio_autodiff_casadi_utils_static_if_hpp__
#define __pinocchio_autodiff_casadi_utils_static_if_hpp__

#include "pinocchio/utils/static-if.hpp"

namespace pinocchio
{
  namespace internal
  {

    template<typename Scalar, typename ThenType, typename ElseType>
    struct if_then_else_impl<::casadi::Matrix<Scalar>, ::casadi::Matrix<Scalar>, ThenType, ElseType>
    {
      typedef typename internal::traits<if_then_else_impl>::ReturnType ReturnType;

      typedef ::casadi::Matrix<Scalar> CasadiType;

      static inline ReturnType run(
        const ComparisonOperators op,
        const CasadiType & lhs_value,
        const CasadiType & rhs_value,
        const ThenType & then_value,
        const ElseType & else_value)
      {
        switch (op)
        {
        case LT:
          return ::casadi::Matrix<Scalar>::if_else(lhs_value < rhs_value, then_value, else_value);
          break;
        case LE:
          return ::casadi::Matrix<Scalar>::if_else(lhs_value <= rhs_value, then_value, else_value);
          break;
        case EQ:
          return ::casadi::Matrix<Scalar>::if_else(lhs_value == rhs_value, then_value, else_value);
          break;
        case GE:
          return ::casadi::Matrix<Scalar>::if_else(lhs_value >= rhs_value, then_value, else_value);
          break;
        case GT:
          return ::casadi::Matrix<Scalar>::if_else(lhs_value > rhs_value, then_value, else_value);
          break;
        }
      }
    };

    template<typename Scalar>
    struct comparison_eq_impl<::casadi::Matrix<Scalar>, ::casadi::Matrix<Scalar>>
    {
      typedef ::casadi::Matrix<Scalar> CasadiType;

      static inline bool run(const CasadiType & lhs_value, const CasadiType & rhs_value)
      {
        return (lhs_value == rhs_value).is_zero();
      }
    };

  } // namespace internal
} // namespace pinocchio

namespace std
{

  template<typename Scalar>
  struct equal_to<::casadi::Matrix<Scalar>>
  {
    bool operator()(
      const ::casadi::Matrix<Scalar> & lhs_value, const ::casadi::Matrix<Scalar> & rhs_value) const
    {
      return (lhs_value == rhs_value).is_zero();
    }
  };
} // namespace std

#endif // ifndef __pinocchio_autodiff_casadi_utils_static_if_hpp__
