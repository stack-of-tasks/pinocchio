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
        
    template<typename if_left_type, typename if_right_type, typename then_type, typename else_type,
             typename B = is_floating_point<(boost::is_floating_point<if_left_type>::value && boost::is_floating_point<if_right_type>::value)> >
    struct if_then_else_impl;
    
    template<typename if_left_right_type, typename same_then_and_else_type>
    struct traits<if_then_else_impl<if_left_right_type,if_left_right_type,same_then_and_else_type,same_then_and_else_type> >
    {
      typedef same_then_and_else_type ReturnType;
    };
    
    template<typename if_left_right_type, typename then_type, typename else_type>
    struct if_then_else_impl<if_left_right_type,if_left_right_type,then_type,else_type, is_floating_point<true> >
    {
      typedef typename internal::traits<if_then_else_impl>::ReturnType ReturnType;
      
      static inline ReturnType run(const CompareOp op,
                                   const if_left_right_type & if_left_value,
                                   const if_left_right_type & if_right_value,
                                   const then_type & then_value,
                                   const else_type & else_value)
      {
        switch(op)
        {
        case LT:
          if(if_left_value < if_right_value)
            return then_value;
          else
            return else_value;
          break;
        case LE:
          if(if_left_value <= if_right_value)
            return then_value;
          else
            return else_value;            
          break;
        case EQ:
          if(if_left_value == if_right_value)
            return then_value;
          else
            return else_value;
          break;
        case GE:
          if(if_left_value >= if_right_value)
            return then_value;
          else
            return else_value;
          break;
        case GT:
          if(if_left_value > if_right_value)
            return then_value;
          else
            return else_value;
          break;          
        }
      }
    };

    template<typename if_left_type, typename if_right_type, typename then_type, typename else_type>
    inline typename if_then_else_impl<if_left_type,if_right_type,then_type,else_type>::ReturnType
    if_then_else(const CompareOp op,
                 const if_left_type & if_left_value,
                 const if_right_type & if_right_value,
                 const then_type & then_value,
                 const else_type & else_value)
    {
      return if_then_else_impl<if_left_type,if_right_type,then_type,else_type>::run(op,if_left_value,
                                                                                    if_right_value,
                                                                                    then_value,
                                                                                    else_value);
    }
    
    
    
  } // namespace internal
} // namespace pinocchio

#endif
