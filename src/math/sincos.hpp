//
// Copyright (c) 2015-2020 CNRS INRIA
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_math_sincos_hpp__
#define __pinocchio_math_sincos_hpp__

#include <cmath>

namespace pinocchio
{
  // Forward declaration
  template<typename S1, typename S2 = S1, typename S3 = S1> struct SINCOSAlgo;
  
  ///
  /// \brief Computes sin/cos values of a given input scalar.
  ///
  /// \tparam Scalar Type of the input/output variables
  ///
  /// \param[in] a The input scalar from which we evalute the sin and cos.
  /// \param[out] sa Variable containing the sin of a.
  /// \param[out] ca Variable containing the cos of a.
  ///
  template<typename S1, typename S2, typename S3>
  void SINCOS(const S1 & a, S2 * sa, S3 * ca)
  {
    SINCOSAlgo<S1,S2,S3>::run(a,sa,ca);
  }
  
  /// \brief Generic evaluation of sin/cos functions.
  template<typename S1, typename S2, typename S3>
  struct SINCOSAlgo
  {
    static void run(const S1 & a, S2 * sa, S3 * ca)
    {
      using std::sin; using std::cos;
      (*sa) = sin(a); (*ca) = cos(a);
    }   
  };  

  /// \brief Specific evaluation of sin/cos for double type.
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
  
  /// \brief Specific evaluation of sin/cos for float type.
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
  
  /// \brief Specific evaluation of sin/cos for long double.
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

#endif //#ifndef __pinocchio_math_sincos_hpp__
