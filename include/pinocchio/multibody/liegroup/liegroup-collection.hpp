//
// Copyright (c) 2018 CNRS
//

#ifndef __pinocchio_lie_group_collection_hpp__
#define __pinocchio_lie_group_collection_hpp__

#include "pinocchio/multibody/liegroup/vector-space.hpp"
#include "pinocchio/multibody/liegroup/cartesian-product.hpp"
#include "pinocchio/multibody/liegroup/special-orthogonal.hpp"
#include "pinocchio/multibody/liegroup/special-euclidean.hpp"

#include <boost/variant.hpp>

namespace pinocchio
{
  template<typename _Scalar, int _Options = 0>
  struct LieGroupCollectionDefaultTpl
  {
    typedef _Scalar Scalar;
    enum { Options = _Options };
    
    typedef boost::variant<
     SpecialOrthogonalOperationTpl<2,Scalar,Options>
    ,SpecialOrthogonalOperationTpl<3,Scalar,Options>
    ,SpecialEuclideanOperationTpl<2,Scalar,Options>
    ,SpecialEuclideanOperationTpl<3,Scalar,Options>
    ,VectorSpaceOperationTpl<1,Scalar,Options>
    ,VectorSpaceOperationTpl<2,Scalar,Options>
    ,VectorSpaceOperationTpl<3,Scalar,Options>
    ,VectorSpaceOperationTpl<Eigen::Dynamic,Scalar,Options>
    > LieGroupVariant;
    
  };
  
  typedef LieGroupCollectionDefaultTpl<double> LieGroupCollectionDefault;
  
}

#endif // ifndef __pinocchio_lie_group_collection_hpp__

