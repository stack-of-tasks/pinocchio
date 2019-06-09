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
    template<typename VectorLike, bool value = boost::is_floating_point<typename VectorLike::Scalar>::value>
    struct isUnitaryAlgo
    {
      typedef typename VectorLike::Scalar Scalar;
      typedef typename VectorLike::RealScalar RealScalar;
      
      static bool run(const Eigen::MatrixBase<VectorLike> & vec,
                      const RealScalar & prec =
                      Eigen::NumTraits< Scalar >::dummy_precision())
      {
        return vec.isUnitary(prec);
      }
    };
    
    template<typename VectorLike>
    struct isUnitaryAlgo<VectorLike,false>
    {
      typedef typename VectorLike::Scalar Scalar;
      typedef typename VectorLike::RealScalar RealScalar;
      
      static bool run(const Eigen::MatrixBase<VectorLike> & /*vec*/,
                      const RealScalar & prec =
                      Eigen::NumTraits< Scalar >::dummy_precision())
      {
        PINOCCHIO_UNUSED_VARIABLE(prec);
        return true;
      }
    };
  }
  
  template<typename VectorLike>
  inline bool isUnitary(const Eigen::MatrixBase<VectorLike> & vec,
                        const typename VectorLike::RealScalar & prec =
                        Eigen::NumTraits< typename VectorLike::Scalar >::dummy_precision())
  {
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(VectorLike);
    return internal::isUnitaryAlgo<VectorLike>::run(vec,prec);
  }
  


}
#endif //#ifndef __pinocchio_math_matrix_hpp__
