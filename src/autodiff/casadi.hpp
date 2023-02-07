//
// Copyright (c) 2019-2020 INRIA, CNRS
//

#ifndef __pinocchio_autodiff_casadi_hpp__
#define __pinocchio_autodiff_casadi_hpp__

#define PINOCCHIO_WITH_CASADI_SUPPORT

#include "pinocchio/math/fwd.hpp"

#include <casadi/casadi.hpp>
#include <Eigen/Core>

namespace boost { namespace math { namespace constants { namespace detail {
  template<>
  struct constant_pi<::casadi::SX> : constant_pi<double> {};

  template<typename Scalar>
  struct constant_pi< ::casadi::Matrix<Scalar> > : constant_pi<Scalar> {};
}}}}

// This is a workaround to make the code compiling with Eigen.
namespace casadi
{
  inline bool operator||(const bool x, const casadi::Matrix<SXElem> & /*y*/)
  {
    return x;
  }
}

namespace pinocchio
{
  template<typename Scalar>
  struct TaylorSeriesExpansion< ::casadi::Matrix<Scalar> >
  : TaylorSeriesExpansion<Scalar>
  {
    typedef TaylorSeriesExpansion<Scalar> Base;

    template<int degree>
    static ::casadi::Matrix<Scalar> precision()
    {
      return ::casadi::Matrix<Scalar>(Base::template precision<degree>());
    }
    
  };
}

namespace Eigen
{
  namespace internal
  {
    // Specialization of Eigen::internal::cast_impl for Casadi input types
    template<typename Scalar>
    struct cast_impl<casadi::SX,Scalar>
    {
#if EIGEN_VERSION_AT_LEAST(3,2,90)
      EIGEN_DEVICE_FUNC
#endif
      static inline Scalar run(const casadi::SX & x)
      {
        return static_cast<Scalar>(x);
      }
    };
  
#if EIGEN_VERSION_AT_LEAST(3,2,90) && !EIGEN_VERSION_AT_LEAST(3,2,93)
    template<typename Scalar, bool IsInteger>
    struct significant_decimals_default_impl< ::casadi::Matrix<Scalar>,IsInteger>
    {
      static inline int run()
      {
        return std::numeric_limits<Scalar>::digits10;
      }
    };
#endif
  }
}

namespace Eigen
{
  /// @brief Eigen::NumTraits<> specialization for casadi::SX
  ///
  template<typename Scalar>
  struct NumTraits< casadi::Matrix<Scalar> >
  {
    using Real = casadi::Matrix<Scalar>;
    using NonInteger = casadi::Matrix<Scalar>;
    using Literal = casadi::Matrix<Scalar>;
    using Nested = casadi::Matrix<Scalar>;
    
    enum {
      // does not support complex Base types
      IsComplex             = 0 ,
      // does not support integer Base types
      IsInteger             = 0 ,
      // only support signed Base types
      IsSigned              = 1 ,
      // must initialize an AD<Base> object
      RequireInitialization = 1 ,
      // computational cost of the corresponding operations
      ReadCost              = 1 ,
      AddCost               = 2 ,
      MulCost               = 2
    };
    
    static casadi::Matrix<Scalar> epsilon()
    {
      return casadi::Matrix<Scalar>(std::numeric_limits<double>::epsilon());
    }
    
    static casadi::Matrix<Scalar> dummy_precision()
    {
      return casadi::Matrix<Scalar>(NumTraits<double>::dummy_precision());
    }
    
    static casadi::Matrix<Scalar> highest()
    {
      return casadi::Matrix<Scalar>(std::numeric_limits<double>::max());
    }
    
    static casadi::Matrix<Scalar> lowest()
    {
      return casadi::Matrix<Scalar>(std::numeric_limits<double>::min());
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
    inline void copy(::casadi::Matrix<Scalar> const & src,
                     Eigen::MatrixBase<MT> & dst)
    {
      Eigen::DenseIndex const m = src.size1();
      Eigen::DenseIndex const n = src.size2();
      
      dst.resize(m, n);
      
      for (Eigen::DenseIndex i = 0; i < m; ++i)
        for (Eigen::DenseIndex j = 0; j < n; ++j)
          dst(i, j) = src(i, j);
    }
    
    
    // Copy Eigen matrix to casadi matrix
    template<typename MT, typename Scalar>
    inline void copy(Eigen::MatrixBase<MT> const & src,
                     ::casadi::Matrix<Scalar> & dst)
    {
      Eigen::DenseIndex const m = src.rows();
      Eigen::DenseIndex const n = src.cols();
      
      dst.resize(m, n);
      
      for (Eigen::DenseIndex i = 0; i < m; ++i)
        for (Eigen::DenseIndex j = 0; j < n; ++j)
          dst(i, j) = src(i, j);
    }
    
    // Make an Eigen matrix consisting of pure casadi symbolics
    template<typename MatrixDerived>
    inline void sym(const Eigen::MatrixBase<MatrixDerived> & eig_mat,
                    std::string const & name)
    {
      typedef typename MatrixDerived::Scalar SX;
      
      MatrixDerived & eig_mat_ = PINOCCHIO_EIGEN_CONST_CAST(MatrixDerived,eig_mat);
      for (Eigen::DenseIndex i = 0; i < eig_mat.rows(); ++i)
        for (Eigen::DenseIndex j = 0; j < eig_mat.cols(); ++j)
          eig_mat_(i, j) = SX::sym(name + "_" + std::to_string(i) + "_" + std::to_string(j));
    }
  
  } // namespace casadi
} // namespace pinocchio

// Overloading of max operator
namespace pinocchio
{
  namespace math
  {
    namespace internal
    {
      template<typename Scalar>
      struct return_type_max< ::casadi::Matrix<Scalar>,::casadi::Matrix<Scalar>>
      {
        typedef ::casadi::Matrix<Scalar> type;
      };
      
      template<typename Scalar, typename T>
      struct return_type_max< ::casadi::Matrix<Scalar>,T>
      {
        typedef ::casadi::Matrix<Scalar> type;
      };
      
      template<typename Scalar, typename T>
      struct return_type_max<T,::casadi::Matrix<Scalar> >
      {
        typedef ::casadi::Matrix<Scalar> type;
      };
      
      template<typename Scalar>
      struct call_max< ::casadi::Matrix<Scalar>,::casadi::Matrix<Scalar> >
      {
        static inline ::casadi::Matrix<Scalar> run(const ::casadi::Matrix<Scalar> & a,
                                                   const ::casadi::Matrix<Scalar> & b)
        { return fmax(a,b); }
      };
      
      template<typename S1, typename S2>
      struct call_max< ::casadi::Matrix<S1>,S2>
      {
        typedef ::casadi::Matrix<S1> CasadiType;
        static inline ::casadi::Matrix<S1> run(const ::casadi::Matrix<S1> & a,
                                               const S2 & b)
        { return fmax(a,static_cast<CasadiType>(b)); }
      };
      
      template<typename S1, typename S2>
      struct call_max<S1,::casadi::Matrix<S2>>
      {
        typedef ::casadi::Matrix<S2> CasadiType;
        static inline ::casadi::Matrix<S2> run(const S1 & a,
                                               const ::casadi::Matrix<S2> & b)
        { return fmax(static_cast<CasadiType>(a),b); }
      };
    } // namespace internal
    
  } // namespace math
  
} // namespace pinocchio

#include "pinocchio/autodiff/casadi/spatial/se3-tpl.hpp"
#include "pinocchio/autodiff/casadi/utils/static-if.hpp"
#include "pinocchio/autodiff/casadi/math/matrix.hpp"
#include "pinocchio/autodiff/casadi/math/quaternion.hpp"

#endif // #ifndef __pinocchio_autodiff_casadi_hpp__
