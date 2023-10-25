//
// Copyright (c) 2020 INRIA
//

#ifndef __pinocchio_math_mutliprecision_hpp__
#define __pinocchio_math_mutliprecision_hpp__

#include "pinocchio/math/fwd.hpp"

#ifndef PINOCCHIO_WITH_CXX11_SUPPORT
  #error C++11 compiler required.
#endif

#include <boost/multiprecision/number.hpp>
#include <Eigen/Dense>

namespace pinocchio
{
// We check std::numeric_limits<_>::has_infinity to exclude integral, rational
// and complex types
template <typename Backend,
          boost::multiprecision::expression_template_option ET>
struct is_floating_point<boost::multiprecision::number<Backend, ET>>
    : boost::integral_constant<
          bool,
          ((std::numeric_limits<
                boost::multiprecision::number<Backend, ET>>::is_specialized and
            std::numeric_limits<
                boost::multiprecision::number<Backend, ET>>::has_infinity))>
{
};
}  // namespace pinocchio

namespace Eigen
{
  namespace internal
  {
    template <class Backend, boost::multiprecision::expression_template_option ExpressionTemplates, typename Scalar>
    struct cast_impl<boost::multiprecision::number<Backend, ExpressionTemplates>,Scalar>
    {
#if EIGEN_VERSION_AT_LEAST(3,2,90)
      EIGEN_DEVICE_FUNC
#endif
      static inline Scalar run(const boost::multiprecision::number<Backend, ExpressionTemplates> & x)
      {
        return x.template convert_to<Scalar>();
      }
    };
  }
} //namespace Eigen

#ifndef BOOST_MP_EIGEN_HPP

//  Code adapted from <boost/multiprecision/eigen.hpp>
//  Copyright 2018 John Maddock. Distributed under the Boost
//  Software License, Version 1.0. (See accompanying file
//  LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
namespace Eigen
{
  template <class Backend, boost::multiprecision::expression_template_option ExpressionTemplates>
  struct NumTraits<boost::multiprecision::number<Backend, ExpressionTemplates> >
  {
    typedef boost::multiprecision::number<Backend, ExpressionTemplates>                          self_type;
#if BOOST_VERSION / 100 % 1000 >= 68
    typedef typename boost::multiprecision::scalar_result_from_possible_complex<self_type>::type Real;
#else
    typedef self_type                                                                            Real;
#endif
    typedef self_type                                                                            NonInteger; // Not correct but we can't do much better??
    typedef double                                                                               Literal;
    typedef self_type                                                                            Nested;
    enum
    {
#if BOOST_VERSION / 100 % 1000 >= 68
      IsComplex             = boost::multiprecision::number_category<self_type>::value == boost::multiprecision::number_kind_complex,
#else
      IsComplex             = 0,
#endif
      IsInteger             = boost::multiprecision::number_category<self_type>::value == boost::multiprecision::number_kind_integer,
      ReadCost              = 1,
      AddCost               = 4,
      MulCost               = 8,
      IsSigned              = std::numeric_limits<self_type>::is_specialized ? std::numeric_limits<self_type>::is_signed : true,
      RequireInitialization = 1
    };
    static Real epsilon()
    {
      return std::numeric_limits<Real>::epsilon();
    }
    static Real dummy_precision()
    {
      return 1000 * epsilon();
    }
    static Real highest()
    {
      return (std::numeric_limits<Real>::max)();
    }
    static Real lowest()
    {
      return (std::numeric_limits<Real>::min)();
    }
    static int digits10_imp(const boost::mpl::true_&)
    {
      return std::numeric_limits<Real>::digits10;
    }
    template <bool B>
    static int digits10_imp(const boost::mpl::bool_<B>&)
    {
      return Real::default_precision();
    }
    static int digits10()
    {
      return digits10_imp(boost::mpl::bool_ < std::numeric_limits<Real>::digits10 && (std::numeric_limits<Real>::digits10 != INT_MAX) ? true : false > ());
    }
  };

  template <class tag, class Arg1, class Arg2, class Arg3, class Arg4>
  struct NumTraits<boost::multiprecision::detail::expression<tag, Arg1, Arg2, Arg3, Arg4> > : public NumTraits<typename boost::multiprecision::detail::expression<tag, Arg1, Arg2, Arg3, Arg4>::result_type>
  {
  };

#if EIGEN_VERSION_AT_LEAST(3,2,93)
#define BOOST_MP_EIGEN_SCALAR_TRAITS_DECL(A)                                                                                                                                                                           \
   template <class Backend, boost::multiprecision::expression_template_option ExpressionTemplates, typename BinaryOp>                                                                                                  \
   struct ScalarBinaryOpTraits<boost::multiprecision::number<Backend, ExpressionTemplates>, A, BinaryOp>                                                                                                               \
   {                                                                                                                                                                                                                   \
      /*static_assert(boost::multiprecision::is_compatible_arithmetic_type<A, boost::multiprecision::number<Backend, ExpressionTemplates> >::value, "Interoperability with this arithmetic type is not supported.");*/ \
      typedef boost::multiprecision::number<Backend, ExpressionTemplates> ReturnType;                                                                                                                                  \
   };                                                                                                                                                                                                                  \
   template <class Backend, boost::multiprecision::expression_template_option ExpressionTemplates, typename BinaryOp>                                                                                                  \
   struct ScalarBinaryOpTraits<A, boost::multiprecision::number<Backend, ExpressionTemplates>, BinaryOp>                                                                                                               \
   {                                                                                                                                                                                                                   \
      /*static_assert(boost::multiprecision::is_compatible_arithmetic_type<A, boost::multiprecision::number<Backend, ExpressionTemplates> >::value, "Interoperability with this arithmetic type is not supported.");*/ \
      typedef boost::multiprecision::number<Backend, ExpressionTemplates> ReturnType;                                                                                                                                  \
   };

  BOOST_MP_EIGEN_SCALAR_TRAITS_DECL(float)
  BOOST_MP_EIGEN_SCALAR_TRAITS_DECL(double)
  BOOST_MP_EIGEN_SCALAR_TRAITS_DECL(long double)
  BOOST_MP_EIGEN_SCALAR_TRAITS_DECL(char)
  BOOST_MP_EIGEN_SCALAR_TRAITS_DECL(unsigned char)
  BOOST_MP_EIGEN_SCALAR_TRAITS_DECL(signed char)
  BOOST_MP_EIGEN_SCALAR_TRAITS_DECL(short)
  BOOST_MP_EIGEN_SCALAR_TRAITS_DECL(unsigned short)
  BOOST_MP_EIGEN_SCALAR_TRAITS_DECL(int)
  BOOST_MP_EIGEN_SCALAR_TRAITS_DECL(unsigned int)
  BOOST_MP_EIGEN_SCALAR_TRAITS_DECL(long)
  BOOST_MP_EIGEN_SCALAR_TRAITS_DECL(unsigned long)

  template <class Backend, boost::multiprecision::expression_template_option ExpressionTemplates, class tag, class Arg1, class Arg2, class Arg3, class Arg4, typename BinaryOp>
  struct ScalarBinaryOpTraits<boost::multiprecision::number<Backend, ExpressionTemplates>, boost::multiprecision::detail::expression<tag, Arg1, Arg2, Arg3, Arg4>, BinaryOp>
  {
    static_assert(boost::is_convertible<typename boost::multiprecision::detail::expression<tag, Arg1, Arg2, Arg3, Arg4>::result_type, boost::multiprecision::number<Backend, ExpressionTemplates> >::value, "Interoperability with this arithmetic type is not supported.");
    typedef boost::multiprecision::number<Backend, ExpressionTemplates> ReturnType;
  };

  template <class tag, class Arg1, class Arg2, class Arg3, class Arg4, class Backend, boost::multiprecision::expression_template_option ExpressionTemplates, typename BinaryOp>
  struct ScalarBinaryOpTraits<boost::multiprecision::detail::expression<tag, Arg1, Arg2, Arg3, Arg4>, boost::multiprecision::number<Backend, ExpressionTemplates>, BinaryOp>
  {
    static_assert(boost::is_convertible<typename boost::multiprecision::detail::expression<tag, Arg1, Arg2, Arg3, Arg4>::result_type, boost::multiprecision::number<Backend, ExpressionTemplates> >::value, "Interoperability with this arithmetic type is not supported.");
    typedef boost::multiprecision::number<Backend, ExpressionTemplates> ReturnType;
  };
#endif

  namespace internal
  {

    template <class Backend, boost::multiprecision::expression_template_option ExpressionTemplates, class tag, class Arg1, class Arg2, class Arg3, class Arg4>
    struct scalar_product_traits<boost::multiprecision::number<Backend, ExpressionTemplates>, boost::multiprecision::detail::expression<tag, Arg1, Arg2, Arg3, Arg4> >
    {
      static_assert(boost::is_convertible<typename boost::multiprecision::detail::expression<tag, Arg1, Arg2, Arg3, Arg4>::result_type, boost::multiprecision::number<Backend, ExpressionTemplates> >::value, "Interoperability with this arithmetic type is not supported.");
      typedef boost::multiprecision::number<Backend, ExpressionTemplates> ReturnType;
    };

    template <class tag, class Arg1, class Arg2, class Arg3, class Arg4, class Backend, boost::multiprecision::expression_template_option ExpressionTemplates>
    struct scalar_product_traits<boost::multiprecision::detail::expression<tag, Arg1, Arg2, Arg3, Arg4>, boost::multiprecision::number<Backend, ExpressionTemplates> >
    {
      static_assert(boost::is_convertible<typename boost::multiprecision::detail::expression<tag, Arg1, Arg2, Arg3, Arg4>::result_type, boost::multiprecision::number<Backend, ExpressionTemplates> >::value, "Interoperability with this arithmetic type is not supported.");
      typedef boost::multiprecision::number<Backend, ExpressionTemplates> ReturnType;
    };

    template <typename Scalar>
    struct conj_retval;

    template <typename Scalar, bool IsComplex>
    struct conj_impl;

    template <class tag, class Arg1, class Arg2, class Arg3, class Arg4>
    struct conj_retval<boost::multiprecision::detail::expression<tag, Arg1, Arg2, Arg3, Arg4> >
    {
      typedef typename boost::multiprecision::detail::expression<tag, Arg1, Arg2, Arg3, Arg4>::result_type type;
    };

    template <class tag, class Arg1, class Arg2, class Arg3, class Arg4>
    struct conj_impl<boost::multiprecision::detail::expression<tag, Arg1, Arg2, Arg3, Arg4>, true>
    {
#if EIGEN_VERSION_AT_LEAST(3,2,90)
      EIGEN_DEVICE_FUNC
#endif
      static inline typename boost::multiprecision::detail::expression<tag, Arg1, Arg2, Arg3, Arg4>::result_type run(const typename boost::multiprecision::detail::expression<tag, Arg1, Arg2, Arg3, Arg4>& x)
      {
        return conj(x);
      }
    };

  } // namespace internal

} // namespace Eigen

#endif // ifndef BOOST_MP_EIGEN_HPP

#endif // ifndef __pinocchio_math_mutliprecision_hpp__
