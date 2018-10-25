//
// Copyright (c) 2015-2018 CNRS INRIA
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.

#ifndef __math_sincos_hpp__
#define __math_sincos_hpp__

#include <cmath>

namespace se3
{
  /// Forward declaration
  template<typename Scalar> struct SINCOSAlgo;
  
  
  /// 
  /// \brief Computes sin/cos values of a given input scalar.
  /// 
  /// \tparam Scalar Type of the input/output variables
  /// 
  /// \param[in] a The input scalar from which we evalute the sin and cos.
  /// \param[inout] sa Variable containing the sin of a.
  /// \param[inout] ca Variable containing the cos of a.
  /// 
  template<typename Scalar>
  void SINCOS(const Scalar & a, Scalar * sa, Scalar * ca) 
  {
    SINCOSAlgo<Scalar>::run(a,sa,ca);
  }
  
  /// Generic evaluation of sin/cos functions.
  template<typename Scalar>
  struct SINCOSAlgo
  {
    static void run(const Scalar & a, Scalar * sa, Scalar * ca) 
    {   
      (*sa) = std::sin(a); (*ca) = std::cos(a);
    }   
  };  

  /// Specific evaluation of sin/cos for double type.
  template<>
  struct SINCOSAlgo<double>
  {
    static void run(const double & a, double * sa, double * ca) 
    {   
#ifdef __linux__
      sincos(a,sa,ca);
#elif __APPLE__
      __sincos(a,sa,ca);
#else // if sincos specialization does not exist
      (*sa) = std::sin(a); (*ca) = std::cos(a);
#endif
    }   
  };
  
  /// Specific evaluation of sin/cos for float type.
  template<>
  struct SINCOSAlgo<float>
  {
    static void run(const float & a, float * sa, float * ca)
    {
#ifdef __linux__
      sincosf(a,sa,ca);
#elif __APPLE__
      __sincosf(a,sa,ca);
#else // if sincosf specialization does not exist
      (*sa) = std::sin(a); (*ca) = std::cos(a);
#endif
    }
  };
  
  /// Specific evaluation of sin/cos for long double.
  template<>
  struct SINCOSAlgo<long double>
  {
    static void run(const long double & a, long double * sa, long double * ca)
    {
#ifdef __linux__
      sincosl(a,sa,ca);
#else // if sincosl specialization does not exist
      (*sa) = std::sin(a); (*ca) = std::cos(a);
#endif
    }
  };
}

#endif //#ifndef __math_sincos_hpp__
