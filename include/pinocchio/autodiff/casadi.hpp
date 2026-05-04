//
// Copyright (c) 2019-2024 INRIA CNRS
//

#pragma once

// IWYU pragma: always_keep

#define PINOCCHIO_WITH_CASADI_SUPPORT

// IWYU pragma: begin_keep
#include <limits>
#include <string>

#include <casadi/casadi.hpp>

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <unsupported/Eigen/CXX11/Tensor>

#include <boost/math/constants/constants.hpp>

#include "pinocchio/fwd.hpp"
#include "pinocchio/macros.hpp"
#include "pinocchio/eigen-common.hpp"
#include "pinocchio/math.hpp"
#include "pinocchio/spatial.hpp"
// IWYU pragma: end_keep

namespace boost
{
  namespace math
  {
    namespace constants
    {
      namespace detail
      {
        template<>
        struct constant_pi<::casadi::SX> : constant_pi<double>
        {
        };

        template<typename Scalar>
        struct constant_pi<::casadi::Matrix<Scalar>> : constant_pi<Scalar>
        {
        };
      } // namespace detail
    } // namespace constants
  } // namespace math
} // namespace boost

// This is a workaround to make the code compiling with Eigen.
namespace casadi
{
  inline bool operator||(const bool x, const ::casadi::Matrix<SXElem> & /*y*/)
  {
    return x;
  }
} // namespace casadi

namespace Eigen
{
  namespace internal
  {
    // Specialization of Eigen::internal::cast_impl for Casadi input types
    template<typename Scalar>
    struct cast_impl<::casadi::Matrix<Scalar>, Scalar>
    {
      EIGEN_DEVICE_FUNC
      static inline Scalar run(const ::casadi::Matrix<Scalar> & x)
      {
        return static_cast<Scalar>(x);
      }
    };
  } // namespace internal

#if EIGEN_VERSION_AT_LEAST(3, 4, 90)
  // The function Eigen::internal::gemv_dense_selector::run defined in
  // Eigen/src/Core/GeneralProduct.h call at some point Eigen::numext::is_exactly_zero function.
  // To avoid this issue we specialize this function for casadi.
  namespace numext
  {
    template<>
    EIGEN_STRONG_INLINE EIGEN_DEVICE_FUNC bool
    is_exactly_zero(const ::casadi::Matrix<::casadi::SXElem> & x)
    {
      return x.is_zero();
    }
  } // namespace numext
#endif // if EIGEN_VERSION_AT_LEAST(3, 4, 90)

} // namespace Eigen

namespace Eigen
{
  /// @brief Eigen::NumTraits<> specialization for casadi::SX
  ///
  template<typename Scalar>
  struct NumTraits<::casadi::Matrix<Scalar>>
  {
    using Real = ::casadi::Matrix<Scalar>;
    using NonInteger = ::casadi::Matrix<Scalar>;
    using Literal = ::casadi::Matrix<Scalar>;
    using Nested = ::casadi::Matrix<Scalar>;

    enum
    {
      // does not support complex Base types
      IsComplex = 0,
      // does not support integer Base types
      IsInteger = 0,
      // only support signed Base types
      IsSigned = 1,
      // must initialize an AD<Base> object
      RequireInitialization = 1,
      // computational cost of the corresponding operations
      ReadCost = 1,
      AddCost = 2,
      MulCost = 2
    };

    EIGEN_DEVICE_FUNC EIGEN_CONSTEXPR static inline ::casadi::Matrix<Scalar> epsilon()
    {
      return ::casadi::Matrix<Scalar>(std::numeric_limits<double>::epsilon());
    }

    EIGEN_DEVICE_FUNC EIGEN_CONSTEXPR static inline ::casadi::Matrix<Scalar> dummy_precision()
    {
      return ::casadi::Matrix<Scalar>(NumTraits<double>::dummy_precision());
    }

    EIGEN_DEVICE_FUNC EIGEN_CONSTEXPR static inline ::casadi::Matrix<Scalar> highest()
    {
      return ::casadi::Matrix<Scalar>(std::numeric_limits<double>::max());
    }

    EIGEN_DEVICE_FUNC EIGEN_CONSTEXPR static inline ::casadi::Matrix<Scalar> lowest()
    {
      return ::casadi::Matrix<Scalar>(std::numeric_limits<double>::min());
    }

    EIGEN_DEVICE_FUNC EIGEN_CONSTEXPR static inline int digits10()
    {
      return std::numeric_limits<double>::digits10;
    }

    EIGEN_DEVICE_FUNC EIGEN_CONSTEXPR static inline int digits()
    {
      return NumTraits<double>::digits();
    }

#if EIGEN_VERSION_AT_LEAST(3, 4, 90)
    EIGEN_DEVICE_FUNC EIGEN_CONSTEXPR static inline int max_digits10()
    {
      return NumTraits<double>::max_digits10();
    }
#endif
  };
} // namespace Eigen

namespace pinocchio
{
  namespace casadi
  {
    // Copy casadi matrix to Eigen matrix
    template<typename MT, typename Scalar>
    inline void copy(::casadi::Matrix<Scalar> const & src, Eigen::MatrixBase<MT> & dst)
    {
      Eigen::Index const m = src.size1();
      Eigen::Index const n = src.size2();

      dst.resize(m, n);

      for (Eigen::Index i = 0; i < m; ++i)
        for (Eigen::Index j = 0; j < n; ++j)
          dst(i, j) = src(i, j);
    }

    // Copy casadi matrix to Eigen::Tensor
    template<typename TensorDerived, typename Scalar>
    inline void copy(::casadi::Matrix<Scalar> const & src, Eigen::TensorBase<TensorDerived> & dst_)
    {
      TensorDerived & dst = static_cast<TensorDerived &>(dst_);
      Eigen::Index const m = src.size1();
      Eigen::Index const n = src.size2();
      // Eigen::Index const d = 1;
      // dst.resize(d, m, n); // TODO(jcarpent) enable the resize depending on the tensor type.
      // Otherwise we should throw an error.

      for (Eigen::Index i = 0; i < m; ++i)
        for (Eigen::Index j = 0; j < n; ++j)
          dst(0, i, j) = src(i, j);
    }

    // Copy Eigen matrix to casadi matrix
    template<typename TensorDerived, typename Scalar>
    inline void copy(Eigen::TensorBase<TensorDerived> const & _src, ::casadi::Matrix<Scalar> & dst)
    {
      const TensorDerived & src = static_cast<const TensorDerived &>(_src);
      Eigen::Index const m = src.dimension(1);
      Eigen::Index const n = src.dimension(2);

      PINOCCHIO_CHECK_ARGUMENT_SIZE(src.dimension(0), 1);
      dst.resize(m, n);

      for (Eigen::Index i = 0; i < m; ++i)
        for (Eigen::Index j = 0; j < n; ++j)
          dst(i, j) = src(0, i, j);
    }

    // Copy Eigen matrix to casadi matrix
    template<typename MT, typename Scalar>
    inline void copy(Eigen::MatrixBase<MT> const & src, ::casadi::Matrix<Scalar> & dst)
    {
      Eigen::Index const m = src.rows();
      Eigen::Index const n = src.cols();

      dst.resize(m, n);

      for (Eigen::Index i = 0; i < m; ++i)
        for (Eigen::Index j = 0; j < n; ++j)
          dst(i, j) = src(i, j);
    }

    // Copy Eigen sparse matrix to casadi matrix
    template<typename MT, typename Scalar>
    inline void copy(Eigen::SparseMatrixBase<MT> const & src, ::casadi::Matrix<Scalar> & dst)
    {
      Eigen::Index const m = src.rows();
      Eigen::Index const n = src.cols();

      dst.resize(m, n);

      // TODO this method is not optimal
      // (we could copy only non zero values)
      for (Eigen::Index i = 0; i < m; ++i)
        for (Eigen::Index j = 0; j < n; ++j)
          dst(i, j) = src.derived().coeff(i, j);
    }

    // Make an Eigen matrix consisting of pure casadi symbolics
    template<typename MatrixDerived>
    inline void sym(const Eigen::MatrixBase<MatrixDerived> & eig_mat, std::string const & name)
    {
      typedef typename MatrixDerived::Scalar SX;

      MatrixDerived & eig_mat_ = PINOCCHIO_EIGEN_CONST_CAST(MatrixDerived, eig_mat);
      for (Eigen::Index i = 0; i < eig_mat.rows(); ++i)
        for (Eigen::Index j = 0; j < eig_mat.cols(); ++j)
          eig_mat_(i, j) = SX::sym(name + "_" + std::to_string(i) + "_" + std::to_string(j));
    }

  } // namespace casadi
} // namespace pinocchio

// IWYU pragma: begin_exports
#include "pinocchio/src/autodiff/casadi/math/alias.hxx"
// IWYU pragma: end_exports

namespace Eigen
{
  // Max operator
  template<typename Scalar>
  struct ScalarBinaryOpTraits<
    ::casadi::Matrix<Scalar>,
    ::casadi::Matrix<Scalar>,
    internal::scalar_max_op<::casadi::Matrix<Scalar>, ::casadi::Matrix<Scalar>>>
  {
    typedef ::casadi::Matrix<Scalar> ReturnType;
  };

  template<typename S1, typename S2>
  struct ScalarBinaryOpTraits<
    ::casadi::Matrix<S1>,
    S2,
    internal::scalar_max_op<::casadi::Matrix<S1>, S2>>
  {
    typedef ::casadi::Matrix<S1> ReturnType;
  };

  template<typename S1, typename S2>
  struct ScalarBinaryOpTraits<
    S1,
    ::casadi::Matrix<S2>,
    internal::scalar_max_op<S1, ::casadi::Matrix<S2>>>
  {
    typedef ::casadi::Matrix<S1> ReturnType;
  };

  namespace internal
  {

    template<typename Scalar>
    struct scalar_max_op<::casadi::Matrix<Scalar>, ::casadi::Matrix<Scalar>>
    : binary_op_base<::casadi::Matrix<Scalar>, ::casadi::Matrix<Scalar>>
    {
      typedef ::casadi::Matrix<Scalar> LhsScalar;
      typedef ::casadi::Matrix<Scalar> RhsScalar;
      typedef
        typename ScalarBinaryOpTraits<LhsScalar, RhsScalar, scalar_max_op>::ReturnType result_type;

      EIGEN_EMPTY_STRUCT_CTOR(scalar_max_op)
      EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE const result_type
      operator()(const LhsScalar & a, const RhsScalar & b) const
      {
        return ::pinocchio::math::internal::call_max<LhsScalar, RhsScalar>::run(a, b);
      }
    };

    template<typename S1, typename S2>
    struct scalar_max_op<S1, ::casadi::Matrix<S2>> : binary_op_base<S1, ::casadi::Matrix<S2>>
    {
      typedef S1 LhsScalar;
      typedef ::casadi::Matrix<S2> RhsScalar;
      typedef
        typename ScalarBinaryOpTraits<LhsScalar, RhsScalar, scalar_max_op>::ReturnType result_type;

      EIGEN_EMPTY_STRUCT_CTOR(scalar_max_op)
      EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE const result_type
      operator()(const LhsScalar & a, const RhsScalar & b) const
      {
        return ::pinocchio::math::internal::call_max<LhsScalar, RhsScalar>::run(a, b);
      }
    };

    template<typename S1, typename S2>
    struct scalar_max_op<::casadi::Matrix<S1>, S2> : binary_op_base<::casadi::Matrix<S1>, S2>
    {
      typedef ::casadi::Matrix<S1> LhsScalar;
      typedef S2 RhsScalar;
      typedef
        typename ScalarBinaryOpTraits<LhsScalar, RhsScalar, scalar_max_op>::ReturnType result_type;

      EIGEN_EMPTY_STRUCT_CTOR(scalar_max_op)
      EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE const result_type
      operator()(const LhsScalar & a, const RhsScalar & b) const
      {
        return ::pinocchio::math::internal::call_max<LhsScalar, RhsScalar>::run(a, b);
      }
    };
  } // namespace internal
} // namespace Eigen

// IWYU pragma: begin_exports
#include "pinocchio/src/autodiff/casadi/math/taylor-series-expansion.hxx"
#include "pinocchio/src/autodiff/casadi/math/alias.hxx"
#include "pinocchio/src/autodiff/casadi/math/matrix.hxx"
#include "pinocchio/src/autodiff/casadi/math/quaternion.hxx"
#include "pinocchio/src/autodiff/casadi/math/triangular-matrix.hxx"
#include "pinocchio/src/autodiff/casadi/spatial/se3-tpl.hxx"
#include "pinocchio/src/autodiff/casadi/utils/static-if.hxx"
// IWYU pragma: end_exports
