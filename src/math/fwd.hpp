//
// Copyright (c) 2016-2019 CNRS INRIA
//

#ifndef __pinocchio_math_fwd_hpp__
#define __pinocchio_math_fwd_hpp__

#include "pinocchio/fwd.hpp"
#include <math.h>
#include <boost/math/constants/constants.hpp>
#include "pinocchio/math/sincos.hpp"

#ifdef PINOCCHIO_WITH_CPPAD_SUPPORT
namespace boost
{
  namespace math
  {
    namespace constants
    {
      namespace detail
      {
        template<typename Scalar>
        struct constant_pi< CppAD::AD<Scalar> > : constant_pi<Scalar> {};
        
#if defined(PINOCCHIO_WITH_CPPADCG_SUPPORT) && defined(PINOCCHIO_WITH_CXX11_SUPPORT)
        template<typename Scalar>
        struct constant_pi< CppAD::cg::CG<Scalar> > : constant_pi<Scalar> {};
#endif
      }
    }
  }
}
#endif

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
    using std::fabs;
    using std::sqrt;
    using std::atan;
    using std::acos;
    using std::asin;
    using std::pow;
    using std::cos;
    using std::sin;
    
#ifdef PINOCCHIO_WITH_CPPAD_SUPPORT
    using CppAD::fabs;
    using CppAD::sqrt;
    using CppAD::atan;
    using CppAD::acos;
    using CppAD::asin;
    using CppAD::atan2;
    using CppAD::pow;
    using CppAD::cos;
    using CppAD::sin;
#else
    using std::atan2;
#endif
  }
}

#endif //#ifndef __pinocchio_math_fwd_hpp__
