//
// Copyright (c) 2018 CNRS
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

#ifndef __se3_lie_group_collection_hpp__
#define __se3_lie_group_collection_hpp__

#include "pinocchio/multibody/liegroup/vector-space.hpp"
#include "pinocchio/multibody/liegroup/cartesian-product.hpp"
#include "pinocchio/multibody/liegroup/special-orthogonal.hpp"
#include "pinocchio/multibody/liegroup/special-euclidean.hpp"

#include <boost/variant.hpp>

namespace se3
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

#endif // ifndef __se3_lie_group_collection_hpp__

