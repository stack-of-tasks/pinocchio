//
// Copyright (c) 2016-2019 CNRS INRIA
//

#ifndef __pinocchio_math_fwd_hpp__
#define __pinocchio_math_fwd_hpp__

#include "pinocchio/fwd.hpp"
#include <math.h>
#include <boost/math/constants/constants.hpp>
#include "pinocchio/math/sincos.hpp"

namespace boost
{
  namespace math
  {
    namespace constants
    {
      namespace detail
      {
        
#if defined(PINOCCHIO_WITH_CASADI_SUPPORT) && defined(PINOCCHIO_WITH_CXX11_SUPPORT)
        template <>
        struct constant_pi< casadi::SX > : constant_pi<double> {};
#endif
        
#ifdef PINOCCHIO_WITH_CPPAD_SUPPORT
        template<typename Scalar>
        struct constant_pi< CppAD::AD<Scalar> > : constant_pi<Scalar> {};
        
#if defined(PINOCCHIO_WITH_CPPADCG_SUPPORT) && defined(PINOCCHIO_WITH_CXX11_SUPPORT)
        template<typename Scalar>
        struct constant_pi< CppAD::cg::CG<Scalar> > : constant_pi<Scalar> {};
#endif
#endif
      }
    }
  }
}

namespace pinocchio
{
  ///
  /// \brief Returns the value of PI according to the template parameters Scalar
  ///
  /// \tparam Scalar The scalar type of the return pi value
  ///
  template<typename Scalar>
  const Scalar PI()
  { return boost::math::constants::pi<Scalar>(); }
  
  namespace math
  {
    
#define PINOCCHIO_OVERLOAD_MATH_UNARY_OPERATOR(name) \
    template<typename Scalar> \
    Scalar name(const Scalar & value) \
    { using std::name; return name(value); }
    
#define PINOCCHIO_OVERLOAD_MATH_BINARY_OPERATOR(name) \
    template<typename Scalar, typename OtherScalar> \
    Scalar name(const Scalar & value1, const OtherScalar & value2) \
    { using std::name; return name(value1,value2); }
    
    PINOCCHIO_OVERLOAD_MATH_UNARY_OPERATOR(fabs)
    PINOCCHIO_OVERLOAD_MATH_UNARY_OPERATOR(sqrt)
    PINOCCHIO_OVERLOAD_MATH_UNARY_OPERATOR(atan)
    PINOCCHIO_OVERLOAD_MATH_UNARY_OPERATOR(acos)
    PINOCCHIO_OVERLOAD_MATH_UNARY_OPERATOR(asin)
    PINOCCHIO_OVERLOAD_MATH_UNARY_OPERATOR(cos)
    PINOCCHIO_OVERLOAD_MATH_UNARY_OPERATOR(sin)
    
    PINOCCHIO_OVERLOAD_MATH_BINARY_OPERATOR(pow)
    PINOCCHIO_OVERLOAD_MATH_BINARY_OPERATOR(min)
    PINOCCHIO_OVERLOAD_MATH_BINARY_OPERATOR(max)
    PINOCCHIO_OVERLOAD_MATH_BINARY_OPERATOR(atan2)
  }
}

#endif //#ifndef __pinocchio_math_fwd_hpp__
