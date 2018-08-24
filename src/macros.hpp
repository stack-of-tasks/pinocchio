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
#include "pinocchio/utils/eigen-fix.hpp"

#if __cplusplus >= 201103L
  #define PINOCCHIO_WITH_CXX11_SUPPORT
#endif

/// \brief Define the current version of Pinocchio
#define PINOCCHIO_MAJOR_VERSION 1
#define PINOCCHIO_MINOR_VERSION 3
#define PINOCCHIO_PATCH_VERSION 2

/// \brief Macro to check the current Pinocchio version against a version provided by x.y.z
#define PINOCCHIO_VERSION_AT_LEAST(x,y,z) (PINOCCHIO_MAJOR_VERSION>x || (PINOCCHIO_MAJOR_VERSION>=x && \
(PINOCCHIO_MINOR_VERSION>y || (PINOCCHIO_MINOR_VERSION>=y && \
PINOCCHIO_PATCH_VERSION>=z))))

namespace se3
{
  namespace helper
  {
    template<typename T> struct argument_type;
    template<typename T, typename U> struct argument_type<T(U)> { typedef U type; };
  }
}

/// \brief Empty macro argument
#define PINOCCHIO_MACRO_EMPTY_ARG

namespace se3
{
  namespace helper
  {
    template<typename D, template<typename> class TypeAccess>
    struct handle_return_type_without_typename
    {
      typedef typename TypeAccess< typename argument_type<void(D)>::type >::type type;
    };
  }
}

/// \brief Macro giving access to the equivalent plain type of D
#define EIGEN_PLAIN_TYPE(D) Eigen::internal::plain_matrix_type< typename se3::helper::argument_type<void(D)>::type >::type

/// \brief Similar to macro EIGEN_PLAIN_TYPE but with guaranty to provite a column major type
#define EIGEN_PLAIN_COLUMN_MAJOR_TYPE(D) se3::helper::handle_return_type_without_typename<D,Eigen::internal::plain_matrix_type_column_major>::type

/// \brief Similar to macro EIGEN_PLAIN_TYPE but with guaranty to provite a row major type
#define EIGEN_PLAIN_ROW_MAJOR_TYPE(D) se3::helper::handle_return_type_without_typename<D,Eigen::internal::fix::plain_matrix_type_row_major>::type

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

/// \brief Macro for an automatic const_cast
#define EIGEN_CONST_CAST(TYPE,OBJ) const_cast<TYPE &>(OBJ.derived())

#endif // ifndef __se3_macros_hpp__
