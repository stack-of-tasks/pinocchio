//
// Copyright (c) 2018 CNRS
//

#ifndef __pinocchio_utils_helpers_hpp__
#define __pinocchio_utils_helpers_hpp__

namespace pinocchio
{
  namespace internal
  {
    template<typename T1, typename T2>
    struct is_same_type
    {
      static const bool value = false;
    };
  
    template<typename T>
    struct is_same_type<T,T>
    {
      static const bool value = true;
    };
  }
}

#endif // __pinocchio_utils_helpers_hpp__
