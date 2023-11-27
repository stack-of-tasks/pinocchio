//
// Copyright (c) 2019-2020 INRIA
//

#ifndef __pinocchio_utils_static_if_hpp__
#define __pinocchio_utils_static_if_hpp__

#include "pinocchio/fwd.hpp"

namespace pinocchio
{
  namespace internal
  {

    enum ComparisonOperators {LT, LE, EQ, GE, GT};

    template<typename LhsType, typename RhsType, typename ThenType, typename ElseType>
    struct if_then_else_impl;
    
    /// \brief Template specialization for  similar return types;
    template<typename LhsType, typename RhsType, typename return_type>
    struct traits< if_then_else_impl<LhsType,RhsType,return_type,return_type> >
    {
      typedef return_type ReturnType;
    };
    
    template<typename condition_type, typename ThenType, typename ElseType>
    struct if_then_else_impl<condition_type,condition_type,ThenType,ElseType>
    {
      typedef typename internal::traits<if_then_else_impl>::ReturnType ReturnType;
      
      static inline ReturnType run(const ComparisonOperators op,
                                   const condition_type & lhs_value,
                                   const condition_type & rhs_value,
                                   const ThenType & then_value,
                                   const ElseType & else_value)
      {
        switch(op)
        {
          case LT:
            if(lhs_value < rhs_value)
              return then_value;
            else
              return else_value;
            break;
          case LE:
            if(lhs_value <= rhs_value)
              return then_value;
            else
              return else_value;
            break;
          case EQ:
            if(lhs_value == rhs_value)
              return then_value;
            else
              return else_value;
            break;
          case GE:
            if(lhs_value >= rhs_value)
              return then_value;
            else
              return else_value;
            break;
          case GT:
            if(lhs_value > rhs_value)
              return then_value;
            else
              return else_value;
            break;
        }
        abort();
      }
    };

    template<typename LhsType, typename RhsType, typename ThenType, typename ElseType>
    inline typename if_then_else_impl<LhsType,RhsType,ThenType,ElseType>::ReturnType
    if_then_else(const ComparisonOperators op,
                 const LhsType & lhs_value,
                 const RhsType & rhs_value,
                 const ThenType & then_value,
                 const ElseType & else_value)
    {
      typedef if_then_else_impl<LhsType,RhsType,ThenType,ElseType> algo;
      return algo::run(op,
                       lhs_value,
                       rhs_value,
                       then_value,
                       else_value);
    }
    
    
    
  } // namespace internal
} // namespace pinocchio

#endif
