//
// Copyright (c) 2019 INRIA
//

#ifndef __pinocchio_math_casadi_hpp__
#define __pinocchio_math_casadi_hpp__

#include <casadi/casadi.hpp>
#include "pinocchio/math/taylor-expansion.hpp"
#include <Eigen/Core>

// This is a workaround to make the code compiling with Eigen.
namespace casadi
{
  inline bool operator||(const bool & x, const casadi::Matrix<SXElem> & /*y*/)
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
    using Base::precision;
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
    
    static double epsilon()
    {
      return std::numeric_limits<double>::epsilon();
    }
    
    static double dummy_precision()
    {
      return NumTraits<double>::dummy_precision();
    }
    
    static double highest()
    {
      return std::numeric_limits<double>::max();
    }
    
    static double lowest()
    {
      return std::numeric_limits<double>::min();
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
      Eigen::Index const m = src.size1();
      Eigen::Index const n = src.size2();
      
      dst.resize(m, n);
      
      for (Eigen::Index i = 0; i < m; ++i)
        for (Eigen::Index j = 0; j < n; ++j)
          dst(i, j) = src(i, j);
    }
    
    
    // Copy Eigen matrix to casadi matrix
    template<typename MT, typename Scalar>
    inline void copy(Eigen::MatrixBase<MT> const & src,
                     ::casadi::Matrix<Scalar> & dst)
    {
      Eigen::Index const m = src.rows();
      Eigen::Index const n = src.cols();
      
      dst.resize(m, n);
      
      for (Eigen::Index i = 0; i < m; ++i)
        for (Eigen::Index j = 0; j < n; ++j)
          dst(i, j) = src(i, j);
    }
    
    // Make an Eigen matrix consisting of pure casadi symbolics
    template<typename MatrixDerived>
    inline void sym(const Eigen::MatrixBase<MatrixDerived> & eig_mat,
                    std::string const & name)
    {
      typedef typename MatrixDerived::Scalar SX;
      
      MatrixDerived & eig_mat_ = const_cast<MatrixDerived &>(eig_mat.derived());
      for (Eigen::Index i = 0; i < eig_mat.rows(); ++i)
        for (Eigen::Index j = 0; j < eig_mat.cols(); ++j)
          eig_mat_(i, j) = SX::sym(name + "_" + std::to_string(i) + "_" + std::to_string(j));
    }
  
  } // namespace casadi
} // namespace pinocchio

#include "pinocchio/math/matrix.hpp"

namespace pinocchio
{
  namespace internal
  {
    template<typename Scalar>
    struct CallCorrectMatrixInverseAccordingToScalar< ::casadi::Matrix<Scalar> >
    {
      typedef ::casadi::Matrix<Scalar> SX;
      template<typename MatrixIn, typename MatrixOut>
      static void run(const Eigen::MatrixBase<MatrixIn> & mat,
                      const Eigen::MatrixBase<MatrixOut> & dest)
      {
        SX cs_mat(mat.rows(),mat.cols());
        casadi::copy(mat.derived(),cs_mat);
        
        SX cs_mat_inv = SX::inv(cs_mat);
        
        MatrixOut & dest_ = const_cast<MatrixOut &>(dest.derived());
        casadi::copy(cs_mat_inv,dest_);
      }

    };

  }
}

#endif // #ifndef __pinocchio_math_casadi_hpp__
