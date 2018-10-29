//
// Copyright (c) 2017-2018 CNRS
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

#ifndef __se3_macros_hpp__
#define __se3_macros_hpp__

#include <Eigen/Core>

/// \brief Define the current version of Pinocchio
#define PINOCCHIO_MAJOR_VERSION 1
#define PINOCCHIO_MINOR_VERSION 3
#define PINOCCHIO_PATCH_VERSION 2

/// \brief Macro to check the current Pinocchio version against a version provided by x.y.z
#define PINOCCHIO_VERSION_AT_LEAST(x,y,z) (PINOCCHIO_MAJOR_VERSION>x || (PINOCCHIO_MAJOR_VERSION>=x && \
(PINOCCHIO_MINOR_VERSION>y || (PINOCCHIO_MINOR_VERSION>=y && \
PINOCCHIO_PATCH_VERSION>=z))))

/// \brief Empty macro argument
#define PINOCCHIO_MACRO_EMPTY_ARG

/// \brief Macro giving access to the equivalent plain type of D
#define EIGEN_PLAIN_TYPE(D) Eigen::internal::plain_matrix_type<D>::type

/// \brief Macro giving access to the reference type of D
#define EIGEN_REF_CONSTTYPE(D) Eigen::internal::ref_selector<D>::type
#if EIGEN_VERSION_AT_LEAST(3,2,90)
#define EIGEN_REF_TYPE(D) Eigen::internal::ref_selector<D>::non_const_type
#else
#define EIGEN_REF_TYPE(D) \
Eigen::internal::conditional< \
bool(Eigen::internal::traits<D>::Flags & Eigen::NestByRefBit), \
D &, \
D \
>::type
#endif

/// \brief Macro giving access to the return type of the dot product operation
#if EIGEN_VERSION_AT_LEAST(3,3,0)
#define EIGEN_DOT_PRODUCT_RETURN_TYPE(D1,D2) \
Eigen::ScalarBinaryOpTraits< typename Eigen::internal::traits< D1 >::Scalar, typename Eigen::internal::traits< D2 >::Scalar >::ReturnType
#else
#define EIGEN_DOT_PRODUCT_RETURN_TYPE(D1,D2) \
Eigen::internal::scalar_product_traits<typename Eigen::internal::traits< D1 >::Scalar,typename Eigen::internal::traits< D2 >::Scalar>::ReturnType
#endif

/// \brief Fix issue concerning 3.2.90 and more versions of Eigen that do not define size_of_xpr_at_compile_time structure.
#if EIGEN_VERSION_AT_LEAST(3,2,90) && !EIGEN_VERSION_AT_LEAST(3,3,0)
namespace se3
{
  namespace internal
  {
    template<typename XprType> struct size_of_xpr_at_compile_time
    {
      enum { ret = Eigen::internal::size_at_compile_time<Eigen::internal::traits<XprType>::RowsAtCompileTime,Eigen::internal::traits<XprType>::ColsAtCompileTime>::ret };
    };
  }
}
#endif

#endif // ifndef __se3_macros_hpp__

