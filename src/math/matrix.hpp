//
// Copyright (c) 2016-2019 CNRS INRIA
//

#ifndef __pinocchio_math_matrix_hpp__
#define __pinocchio_math_matrix_hpp__

#include <Eigen/Core>
#include <boost/type_traits.hpp>

namespace pinocchio
{

  template<typename Derived>
  inline bool hasNaN(const Eigen::DenseBase<Derived> & m) 
  {
    return !((m.derived().array()==m.derived().array()).all());
  }

  template<typename M1, typename M2>
  struct MatrixProduct
  {
#if EIGEN_VERSION_AT_LEAST(3,2,90)
    typedef typename Eigen::Product<M1,M2> type;
#else
    typedef typename Eigen::ProductReturnType<M1,M2>::Type type;
#endif
  };
  
  namespace internal
  {
    template<typename MatrixLike, bool value = boost::is_floating_point<typename MatrixLike::Scalar>::value>
    struct isUnitaryAlgo
    {
      typedef typename MatrixLike::Scalar Scalar;
      typedef typename MatrixLike::RealScalar RealScalar;
      
      static bool run(const Eigen::MatrixBase<MatrixLike> & mat,
                      const RealScalar & prec =
                      Eigen::NumTraits< Scalar >::dummy_precision())
      {
        return mat.isUnitary(prec);
      }
    };
    
    template<typename MatrixLike>
    struct isUnitaryAlgo<MatrixLike,false>
    {
      typedef typename MatrixLike::Scalar Scalar;
      typedef typename MatrixLike::RealScalar RealScalar;
      
      static bool run(const Eigen::MatrixBase<MatrixLike> & /*vec*/,
                      const RealScalar & prec =
                      Eigen::NumTraits< Scalar >::dummy_precision())
      {
        PINOCCHIO_UNUSED_VARIABLE(prec);
        return true;
      }
    };
  }
  
  template<typename MatrixLike>
  inline bool isUnitary(const Eigen::MatrixBase<MatrixLike> & mat,
                        const typename MatrixLike::RealScalar & prec =
                        Eigen::NumTraits< typename MatrixLike::Scalar >::dummy_precision())
  {
    return internal::isUnitaryAlgo<MatrixLike>::run(mat,prec);
  }
  
  namespace internal
  {
    template<typename Scalar>
    struct CallCorrectMatrixInverseAccordingToScalar
    {
      template<typename MatrixIn, typename MatrixOut>
      static void run(const Eigen::MatrixBase<MatrixIn> & m_in,
                      const Eigen::MatrixBase<MatrixOut> & dest)
      {
        MatrixOut & dest_ = const_cast<MatrixOut &>(dest.derived());
        dest_.noalias() = m_in.inverse();
      }
    };
  
  }
  
  template<typename MatrixIn, typename MatrixOut>
  inline void inverse(const Eigen::MatrixBase<MatrixIn> & m_in,
                      const Eigen::MatrixBase<MatrixOut> & dest)
  {
    MatrixOut & dest_ = const_cast<MatrixOut &>(dest.derived());
    internal::CallCorrectMatrixInverseAccordingToScalar<typename MatrixIn::Scalar>::run(m_in,dest_);
  }

}

#endif //#ifndef __pinocchio_math_matrix_hpp__
