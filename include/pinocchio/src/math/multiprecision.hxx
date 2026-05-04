//
// Copyright (c) 2020 INRIA
//
#pragma once

// IWYU pragma: private, include "pinocchio/math/multiprecision.hpp"

#ifdef PINOCCHIO_LSP
  #undef PINOCCHIO_LSP
  #include "pinocchio/math/multiprecision.hpp"
#endif // PINOCCHIO_LSP

namespace pinocchio
{
  // We check std::numeric_limits<_>::has_infinity to exclude integral, rational
  // and complex types
  template<typename Backend, boost::multiprecision::expression_template_option ET>
  struct is_floating_point<boost::multiprecision::number<Backend, ET>>
  : boost::integral_constant<
      bool,
      ((!std::numeric_limits<boost::multiprecision::number<Backend, ET>>::is_integer
        && std::numeric_limits<boost::multiprecision::number<Backend, ET>>::has_infinity))>
  {
  };
} // namespace pinocchio

namespace Eigen
{
  namespace internal
  {
    template<
      class Backend,
      boost::multiprecision::expression_template_option ExpressionTemplates,
      typename Scalar>
    struct cast_impl<boost::multiprecision::number<Backend, ExpressionTemplates>, Scalar>
    {
      EIGEN_DEVICE_FUNC
      static inline Scalar
      run(const boost::multiprecision::number<Backend, ExpressionTemplates> & x)
      {
        return x.template convert_to<Scalar>();
      }
    };
  } // namespace internal
} // namespace Eigen

#ifndef BOOST_MP_EIGEN_HPP

//  Code adapted from <boost/multiprecision/eigen.hpp>
//  Copyright 2018 John Maddock. Distributed under the Boost
//  Software License, Version 1.0. (See accompanying file
//  LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
namespace Eigen
{
  template<class Backend, boost::multiprecision::expression_template_option ExpressionTemplates>
  struct NumTraits<boost::multiprecision::number<Backend, ExpressionTemplates>>
  {
    typedef boost::multiprecision::number<Backend, ExpressionTemplates> self_type;
    typedef
      typename boost::multiprecision::scalar_result_from_possible_complex<self_type>::type Real;
    typedef self_type NonInteger; // Not correct but we can't do much better??
    typedef double Literal;
    typedef self_type Nested;
    enum
    {
      IsComplex = boost::multiprecision::number_category<self_type>::value
                  == boost::multiprecision::number_kind_complex,
      IsInteger = boost::multiprecision::number_category<self_type>::value
                  == boost::multiprecision::number_kind_integer,
      ReadCost = 1,
      AddCost = 4,
      MulCost = 8,
      IsSigned = std::numeric_limits<self_type>::is_specialized
                   ? std::numeric_limits<self_type>::is_signed
                   : true,
      RequireInitialization = 1
    };
    EIGEN_DEVICE_FUNC EIGEN_CONSTEXPR static inline Real epsilon()
    {
      return std::numeric_limits<Real>::epsilon();
    }
    EIGEN_DEVICE_FUNC EIGEN_CONSTEXPR static inline Real dummy_precision()
    {
      return 1000 * epsilon();
    }
    EIGEN_DEVICE_FUNC EIGEN_CONSTEXPR static inline Real highest()
    {
      return (std::numeric_limits<Real>::max)();
    }
    EIGEN_DEVICE_FUNC EIGEN_CONSTEXPR static inline Real lowest()
    {
      return (std::numeric_limits<Real>::min)();
    }
    EIGEN_DEVICE_FUNC EIGEN_CONSTEXPR static inline int digits10_imp(const std::true_type &)
    {
      return std::numeric_limits<Real>::digits10;
    }
    template<bool B>
    EIGEN_DEVICE_FUNC EIGEN_CONSTEXPR static inline int digits10_imp(const boost::mpl::bool_<B> &)
    {
      return static_cast<int>(Real::default_precision());
    }
    EIGEN_DEVICE_FUNC EIGEN_CONSTEXPR static inline int digits10()
    {
      return digits10_imp(
        boost::mpl::bool_<
          std::numeric_limits<Real>::digits10 && (std::numeric_limits<Real>::digits10 != INT_MAX)
            ? true
            : false>());
    }

    EIGEN_DEVICE_FUNC EIGEN_CONSTEXPR static inline int digits()
    {
      return internal::default_digits_impl<self_type>::run();
    }

  #if EIGEN_VERSION_AT_LEAST(3, 4, 90)
    EIGEN_DEVICE_FUNC EIGEN_CONSTEXPR static inline int max_digits10()
    {
      return internal::default_max_digits10_impl<Real>::run();
    }

    EIGEN_DEVICE_FUNC EIGEN_CONSTEXPR static inline Real infinity()
    {
      return std::numeric_limits<Real>::infinity();
    }

    EIGEN_DEVICE_FUNC EIGEN_CONSTEXPR static inline Real quiet_NaN()
    {
      return std::numeric_limits<Real>::quiet_NaN();
    }
  #endif
  };

  template<class tag, class Arg1, class Arg2, class Arg3, class Arg4>
  struct NumTraits<boost::multiprecision::detail::expression<tag, Arg1, Arg2, Arg3, Arg4>>
  : public NumTraits<
      typename boost::multiprecision::detail::expression<tag, Arg1, Arg2, Arg3, Arg4>::result_type>
  {
  };

  #if EIGEN_VERSION_AT_LEAST(3, 4, 90)
  // Fix random generator number in 3.4.90. TODO(jcarpent): Yet, we should wait for Eigen 3.5.0 to
  // the proper fix in Eigen/src/Core/MathFunctions.h.
  namespace internal
  {
    template<class Backend, boost::multiprecision::expression_template_option ExpressionTemplates>
    struct random_default_impl<
      boost::multiprecision::number<Backend, ExpressionTemplates>,
      false,
      false>
    {
      typedef boost::multiprecision::number<Backend, ExpressionTemplates> Scalar;

      static inline Scalar run(const Scalar & x, const Scalar & y)
      {
        return x + (y - x) * Scalar(std::rand()) / Scalar(RAND_MAX);
      }
      static inline Scalar run()
      {
        return run(Scalar(NumTraits<Scalar>::IsSigned ? -1 : 0), Scalar(1));
      }
    };
  } // namespace internal
  #endif

  #define BOOST_MP_EIGEN_SCALAR_TRAITS_DECL(A)                                                     \
    template<                                                                                      \
      class Backend, boost::multiprecision::expression_template_option ExpressionTemplates,        \
      typename BinaryOp>                                                                           \
    struct ScalarBinaryOpTraits<                                                                   \
      boost::multiprecision::number<Backend, ExpressionTemplates>, A, BinaryOp>                    \
    {                                                                                              \
      /*static_assert(boost::multiprecision::is_compatible_arithmetic_type<A,                      \
       * boost::multiprecision::number<Backend, ExpressionTemplates> >::value, "Interoperability   \
       * with this arithmetic type is not supported.");*/                                          \
      typedef boost::multiprecision::number<Backend, ExpressionTemplates> ReturnType;              \
    };                                                                                             \
    template<                                                                                      \
      class Backend, boost::multiprecision::expression_template_option ExpressionTemplates,        \
      typename BinaryOp>                                                                           \
    struct ScalarBinaryOpTraits<                                                                   \
      A, boost::multiprecision::number<Backend, ExpressionTemplates>, BinaryOp>                    \
    {                                                                                              \
      /*static_assert(boost::multiprecision::is_compatible_arithmetic_type<A,                      \
       * boost::multiprecision::number<Backend, ExpressionTemplates> >::value, "Interoperability   \
       * with this arithmetic type is not supported.");*/                                          \
      typedef boost::multiprecision::number<Backend, ExpressionTemplates> ReturnType;              \
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

  template<
    class Backend,
    boost::multiprecision::expression_template_option ExpressionTemplates,
    class tag,
    class Arg1,
    class Arg2,
    class Arg3,
    class Arg4,
    typename BinaryOp>
  struct ScalarBinaryOpTraits<
    boost::multiprecision::number<Backend, ExpressionTemplates>,
    boost::multiprecision::detail::expression<tag, Arg1, Arg2, Arg3, Arg4>,
    BinaryOp>
  {
    static_assert(
      boost::is_convertible<
        typename boost::multiprecision::detail::expression<tag, Arg1, Arg2, Arg3, Arg4>::
          result_type,
        boost::multiprecision::number<Backend, ExpressionTemplates>>::value,
      "Interoperability with this arithmetic type is not supported.");
    typedef boost::multiprecision::number<Backend, ExpressionTemplates> ReturnType;
  };

  template<
    class tag,
    class Arg1,
    class Arg2,
    class Arg3,
    class Arg4,
    class Backend,
    boost::multiprecision::expression_template_option ExpressionTemplates,
    typename BinaryOp>
  struct ScalarBinaryOpTraits<
    boost::multiprecision::detail::expression<tag, Arg1, Arg2, Arg3, Arg4>,
    boost::multiprecision::number<Backend, ExpressionTemplates>,
    BinaryOp>
  {
    static_assert(
      boost::is_convertible<
        typename boost::multiprecision::detail::expression<tag, Arg1, Arg2, Arg3, Arg4>::
          result_type,
        boost::multiprecision::number<Backend, ExpressionTemplates>>::value,
      "Interoperability with this arithmetic type is not supported.");
    typedef boost::multiprecision::number<Backend, ExpressionTemplates> ReturnType;
  };

  namespace internal
  {

    template<
      class Backend,
      boost::multiprecision::expression_template_option ExpressionTemplates,
      class tag,
      class Arg1,
      class Arg2,
      class Arg3,
      class Arg4>
    struct scalar_product_traits<
      boost::multiprecision::number<Backend, ExpressionTemplates>,
      boost::multiprecision::detail::expression<tag, Arg1, Arg2, Arg3, Arg4>>
    {
      static_assert(
        boost::is_convertible<
          typename boost::multiprecision::detail::expression<tag, Arg1, Arg2, Arg3, Arg4>::
            result_type,
          boost::multiprecision::number<Backend, ExpressionTemplates>>::value,
        "Interoperability with this arithmetic type is not supported.");
      typedef boost::multiprecision::number<Backend, ExpressionTemplates> ReturnType;
    };

    template<
      class tag,
      class Arg1,
      class Arg2,
      class Arg3,
      class Arg4,
      class Backend,
      boost::multiprecision::expression_template_option ExpressionTemplates>
    struct scalar_product_traits<
      boost::multiprecision::detail::expression<tag, Arg1, Arg2, Arg3, Arg4>,
      boost::multiprecision::number<Backend, ExpressionTemplates>>
    {
      static_assert(
        boost::is_convertible<
          typename boost::multiprecision::detail::expression<tag, Arg1, Arg2, Arg3, Arg4>::
            result_type,
          boost::multiprecision::number<Backend, ExpressionTemplates>>::value,
        "Interoperability with this arithmetic type is not supported.");
      typedef boost::multiprecision::number<Backend, ExpressionTemplates> ReturnType;
    };

    template<typename Scalar>
    struct conj_retval;

    template<typename Scalar, bool IsComplex>
    struct conj_default_impl;

    template<class tag, class Arg1, class Arg2, class Arg3, class Arg4>
    struct conj_retval<boost::multiprecision::detail::expression<tag, Arg1, Arg2, Arg3, Arg4>>
    {
      typedef
        typename boost::multiprecision::detail::expression<tag, Arg1, Arg2, Arg3, Arg4>::result_type
          type;
    };

    template<class tag, class Arg1, class Arg2, class Arg3, class Arg4>
    struct conj_default_impl<
      boost::multiprecision::detail::expression<tag, Arg1, Arg2, Arg3, Arg4>,
      true>
    {
      EIGEN_DEVICE_FUNC
      static inline
        typename boost::multiprecision::detail::expression<tag, Arg1, Arg2, Arg3, Arg4>::result_type
        run(
          const typename boost::multiprecision::detail::expression<tag, Arg1, Arg2, Arg3, Arg4> & x)
      {
        return conj(x);
      }
    };

  } // namespace internal

} // namespace Eigen

#endif // ifndef BOOST_MP_EIGEN_HPP
