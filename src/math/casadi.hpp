//
// Copyright (c) 2019 INRIA
//

#ifndef __pinocchio_math_casadi_hpp__
#define __pinocchio_math_casadi_hpp__

#include <casadi/casadi.hpp>

namespace Eigen
{
  namespace internal
  {
    // Specialization of Eigen::internal::cast_impl for Casadi input types
    template<typename Scalar>
    struct cast_impl<casadi::SX,Scalar>
    {
#if EIGEN_VERSION_AT_LEAST(3,2,90)
      EIGEN_DEVICE_FUNC
#endif
      static inline Scalar run(const casadi::SX & x)
      {
        return static_cast<Scalar>(x);
      }
    };
  }
}

namespace Eigen
{
  template<> struct NumTraits<casadi::SX>
  {
    using Real = casadi::SX;
    using NonInteger = casadi::SX;
    using Literal = casadi::SX;
    using Nested = casadi::SX;
    
    static bool constexpr IsComplex = false;
    static bool constexpr IsInteger = false;
    static int constexpr ReadCost = 1;
    static int constexpr AddCost = 1;
    static int constexpr MulCost = 1;
    static bool constexpr IsSigned = true;
    static bool constexpr RequireInitialization = true;
    
    static double epsilon()
    {
      return std::numeric_limits<double>::epsilon();
    }
    
    static double dummy_precision()
    {
      return NumTraits<double>::dummy_precision();
    }
    
    static double hightest()
    {
      return std::numeric_limits<double>::max();
    }
    
    static double lowest()
    {
      return std::numeric_limits<double>::min();
    }
    
    static int digits10()
    {
      return std::numeric_limits<double>::digits10;
    }
  };
}

#endif // #ifndef __pinocchio_math_casadi_hpp__
