//
// Copyright (c) 2019 INRIA
//

#ifndef __pinocchio_utils_static_if_hpp__
#define __pinocchio_utils_static_if_hpp__

#include "pinocchio/fwd.hpp"
#include <boost/type_traits.hpp>

namespace pinocchio
{
  namespace internal
  {

    enum CompareOp {LT, LE, EQ, GE, GT};

    template<bool> struct is_floating_point {};
    
    //Forward Declaration
    template<typename left_type, typename right_type, typename B = is_floating_point<(boost::is_floating_point<left_type>::value && boost::is_floating_point<right_type>::value)> >
    struct if_condition_impl;

    template<typename same_left_right_type>
    struct traits<if_condition_impl<same_left_right_type,same_left_right_type,is_floating_point<true> > >
    {
      typedef bool ReturnType;
    };
    
    template<typename same_left_right_type>
    struct if_condition_impl<same_left_right_type,same_left_right_type, is_floating_point<boost::is_floating_point<same_left_right_type>::value> >
    {
      typedef typename internal::traits<if_condition_impl>::ReturnType ReturnType;
      
      static inline ReturnType run(const same_left_right_type& left_value,
                                   const same_left_right_type& right_value,
                                   CompareOp op)
      {
        switch(op)
        {
        case LT:
          return left_value < right_value;
          break;
        case LE:
          return left_value <= right_value;
          break;
        case EQ:
          return left_value == right_value;
          break;
        case GE:
          return left_value >= right_value;
          break;
        case GT:
          return left_value > right_value;
          break;          
        }
      }
    };
    
    //General implementation of Cond Exp
    template<typename left_type, typename right_type>
    inline typename if_condition_impl<left_type,right_type>::ReturnType
    if_condition(const left_type & left_value,
                 const right_type & right_value,
                 const CompareOp op)
    {
      return if_condition_impl<left_type,right_type>::run(left_value, right_value, op);
    }      
    
    template<typename if_type, typename then_type, typename else_type>
    struct if_then_else_impl;
    
    template<typename if_type, typename same_then_and_else_type>
    struct traits<if_then_else_impl<if_type,same_then_and_else_type,same_then_and_else_type> >
    {
      typedef same_then_and_else_type ReturnType;
    };
    
    template<typename then_type, typename else_type>
    struct if_then_else_impl<bool,then_type,else_type>
    {
      typedef typename internal::traits<if_then_else_impl>::ReturnType ReturnType;
      
      static inline ReturnType run(bool condition,
                                   const then_type & then_value,
                                   const else_type & else_value)
      {
        if(condition)
          return then_value;
        else
          return else_value;
      }
    };
    
    template<typename if_type, typename then_type, typename else_type>
    inline typename if_then_else_impl<if_type,then_type,else_type>::ReturnType
    if_then_else(const if_type & condition,
                 const then_type & then_value,
                 const else_type & else_value)
    {
      return if_then_else_impl<if_type,then_type,else_type>::run(condition,
                                                                 then_value,
                                                                 else_value);
    }
    
    
    
  } // namespace internal
} // namespace pinocchio

#endif
