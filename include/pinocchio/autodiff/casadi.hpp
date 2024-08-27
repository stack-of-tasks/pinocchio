//
// Copyright (c) 2019-2024 INRIA CNRS
//

#ifndef __pinocchio_autodiff_casadi_hpp__
#define __pinocchio_autodiff_casadi_hpp__

#define PINOCCHIO_WITH_CASADI_SUPPORT

#include "pinocchio/math/fwd.hpp"

#include <casadi/casadi.hpp>
#include <Eigen/Core>
#include <Eigen/Sparse>

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

namespace pinocchio
{
  template<typename Scalar>
  struct TaylorSeriesExpansion<::casadi::Matrix<Scalar>> : TaylorSeriesExpansion<Scalar>
  {
    typedef TaylorSeriesExpansion<Scalar> Base;

    template<int degree>
    static ::casadi::Matrix<Scalar> precision()
    {
      return ::casadi::Matrix<Scalar>(Base::template precision<degree>());
    }
  };

} // namespace pinocchio

namespace Eigen
{
  namespace internal
  {
    // Specialization of Eigen::internal::cast_impl for Casadi input types
    template<typename Scalar>
    struct cast_impl<::casadi::Matrix<Scalar>, Scalar>
    {
#if EIGEN_VERSION_AT_LEAST(3, 2, 90)
      EIGEN_DEVICE_FUNC
#endif
      static inline Scalar run(const ::casadi::Matrix<Scalar> & x)
      {
        return static_cast<Scalar>(x);
      }
    };

#if EIGEN_VERSION_AT_LEAST(3, 2, 90) && !EIGEN_VERSION_AT_LEAST(3, 2, 93)
    template<typename Scalar, bool IsInteger>
    struct significant_decimals_default_impl<::casadi::Matrix<Scalar>, IsInteger>
    {
      static inline int run()
      {
        return std::numeric_limits<Scalar>::digits10;
      }
    };
#endif
  } // namespace internal
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

    static ::casadi::Matrix<Scalar> epsilon()
    {
      return ::casadi::Matrix<Scalar>(std::numeric_limits<double>::epsilon());
    }

    static ::casadi::Matrix<Scalar> dummy_precision()
    {
      return ::casadi::Matrix<Scalar>(NumTraits<double>::dummy_precision());
    }

    static ::casadi::Matrix<Scalar> highest()
    {
      return ::casadi::Matrix<Scalar>(std::numeric_limits<double>::max());
    }

    static ::casadi::Matrix<Scalar> lowest()
    {
      return ::casadi::Matrix<Scalar>(std::numeric_limits<double>::min());
    }

    static int digits10()
    {
      return std::numeric_limits<double>::digits10;
    }
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
      Eigen::DenseIndex const m = src.size1();
      Eigen::DenseIndex const n = src.size2();

      dst.resize(m, n);

      for (Eigen::DenseIndex i = 0; i < m; ++i)
        for (Eigen::DenseIndex j = 0; j < n; ++j)
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
      Eigen::DenseIndex const m = src.rows();
      Eigen::DenseIndex const n = src.cols();

      dst.resize(m, n);

      for (Eigen::DenseIndex i = 0; i < m; ++i)
        for (Eigen::DenseIndex j = 0; j < n; ++j)
          dst(i, j) = src(i, j);
    }

    // Copy Eigen sparse matrix to casadi matrix
    template<typename MT, typename Scalar>
    inline void copy(Eigen::SparseMatrixBase<MT> const & src, ::casadi::Matrix<Scalar> & dst)
    {
      Eigen::DenseIndex const m = src.rows();
      Eigen::DenseIndex const n = src.cols();

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
      for (Eigen::DenseIndex i = 0; i < eig_mat.rows(); ++i)
        for (Eigen::DenseIndex j = 0; j < eig_mat.cols(); ++j)
          eig_mat_(i, j) = SX::sym(name + "_" + std::to_string(i) + "_" + std::to_string(j));
    }

  } // namespace casadi
} // namespace pinocchio

// Overloading of min/max operator
namespace pinocchio
{
  namespace math
  {
    namespace internal
    {
      template<typename Scalar>
      struct return_type_min<::casadi::Matrix<Scalar>, ::casadi::Matrix<Scalar>>
      {
        typedef ::casadi::Matrix<Scalar> type;
      };

      template<typename Scalar, typename T>
      struct return_type_min<::casadi::Matrix<Scalar>, T>
      {
        typedef ::casadi::Matrix<Scalar> type;
      };

      template<typename Scalar, typename T>
      struct return_type_min<T, ::casadi::Matrix<Scalar>>
      {
        typedef ::casadi::Matrix<Scalar> type;
      };

      template<typename Scalar>
      struct call_min<::casadi::Matrix<Scalar>, ::casadi::Matrix<Scalar>>
      {
        static inline ::casadi::Matrix<Scalar>
        run(const ::casadi::Matrix<Scalar> & a, const ::casadi::Matrix<Scalar> & b)
        {
          return fmin(a, b);
        }
      };

      template<typename S1, typename S2>
      struct call_min<::casadi::Matrix<S1>, S2>
      {
        typedef ::casadi::Matrix<S1> CasadiType;
        static inline ::casadi::Matrix<S1> run(const ::casadi::Matrix<S1> & a, const S2 & b)
        {
          return fmin(a, static_cast<CasadiType>(b));
        }
      };

      template<typename S1, typename S2>
      struct call_min<S1, ::casadi::Matrix<S2>>
      {
        typedef ::casadi::Matrix<S2> CasadiType;
        static inline ::casadi::Matrix<S2> run(const S1 & a, const ::casadi::Matrix<S2> & b)
        {
          return fmin(static_cast<CasadiType>(a), b);
        }
      };

      template<typename Scalar>
      struct return_type_max<::casadi::Matrix<Scalar>, ::casadi::Matrix<Scalar>>
      {
        typedef ::casadi::Matrix<Scalar> type;
      };

      template<typename Scalar, typename T>
      struct return_type_max<::casadi::Matrix<Scalar>, T>
      {
        typedef ::casadi::Matrix<Scalar> type;
      };

      template<typename Scalar, typename T>
      struct return_type_max<T, ::casadi::Matrix<Scalar>>
      {
        typedef ::casadi::Matrix<Scalar> type;
      };

      template<typename Scalar>
      struct call_max<::casadi::Matrix<Scalar>, ::casadi::Matrix<Scalar>>
      {
        static inline ::casadi::Matrix<Scalar>
        run(const ::casadi::Matrix<Scalar> & a, const ::casadi::Matrix<Scalar> & b)
        {
          return fmax(a, b);
        }
      };

      template<typename S1, typename S2>
      struct call_max<::casadi::Matrix<S1>, S2>
      {
        typedef ::casadi::Matrix<S1> CasadiType;
        static inline ::casadi::Matrix<S1> run(const ::casadi::Matrix<S1> & a, const S2 & b)
        {
          return fmax(a, static_cast<CasadiType>(b));
        }
      };

      template<typename S1, typename S2>
      struct call_max<S1, ::casadi::Matrix<S2>>
      {
        typedef ::casadi::Matrix<S2> CasadiType;
        static inline ::casadi::Matrix<S2> run(const S1 & a, const ::casadi::Matrix<S2> & b)
        {
          return fmax(static_cast<CasadiType>(a), b);
        }
      };
    } // namespace internal

  } // namespace math

} // namespace pinocchio

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
      //      template<typename Packet>
      //      EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE const Packet packetOp(const Packet& a, const
      //      Packet& b) const { return internal::pmax(a,b); } template<typename Packet>
      //      EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE const result_type predux(const Packet& a) const
      //      { return internal::predux_max(a); }
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

#include "pinocchio/autodiff/casadi/spatial/se3-tpl.hpp"
#include "pinocchio/autodiff/casadi/utils/static-if.hpp"
#include "pinocchio/autodiff/casadi/math/matrix.hpp"
#include "pinocchio/autodiff/casadi/math/quaternion.hpp"
#include "pinocchio/autodiff/casadi/math/triangular-matrix.hpp"

#endif // #ifndef __pinocchio_autodiff_casadi_hpp__
