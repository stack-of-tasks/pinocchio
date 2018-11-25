//
// Copyright (c) 2018 CNRS
//

#ifndef __pinocchio_lie_group_variant_hpp__
#define __pinocchio_lie_group_variant_hpp__

#include "pinocchio/multibody/liegroup/vector-space.hpp"
#include "pinocchio/multibody/liegroup/cartesian-product.hpp"
#include "pinocchio/multibody/liegroup/special-orthogonal.hpp"
#include "pinocchio/multibody/liegroup/special-euclidean.hpp"

#include <boost/variant.hpp>

namespace pinocchio
{
  typedef boost::variant< SpecialOrthogonalOperationTpl<2,double,0>
                         ,SpecialOrthogonalOperationTpl<3,double,0>
                         ,SpecialEuclideanOperationTpl<2,double,0>
                         ,SpecialEuclideanOperationTpl<3,double,0>
                         ,VectorSpaceOperationTpl<1,double>
                         ,VectorSpaceOperationTpl<2,double>
                         ,VectorSpaceOperationTpl<3,double>
                         ,VectorSpaceOperationTpl<Eigen::Dynamic,double>
                        > LieGroupVariant;
  
}

#endif // ifndef __pinocchio_lie_group_variant_hpp__
