//
// Copyright (c) 2019 INRIA
//

#ifndef __pinocchio_utils_static_if_hpp__
#define __pinocchio_utils_static_if_hpp__

#include "pinocchio/fwd.hpp"

namespace pinocchio
{
  namespace internal
  {
    
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
