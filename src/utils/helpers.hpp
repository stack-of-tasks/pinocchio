//
// Copyright (c) 2020 INRIA
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
  
    /// \brief By casting from T1 to T2, does there exist any gain in precision.
    template<typename T1, typename T2>
    struct gain_precision
    {
      static const bool value = false;
    };
  
    template<>
    struct gain_precision<float,double>
    {
      static const bool value = true;
    };
  
    template<>
    struct gain_precision<double,long double>
    {
    static const bool value = true;
    };
    
    template<>
    struct gain_precision<float,long double>
    {
      static const bool value = true;
    };
  

  }
}

#endif // __pinocchio_utils_helpers_hpp__
