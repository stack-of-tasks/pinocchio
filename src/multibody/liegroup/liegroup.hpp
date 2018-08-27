//
// Copyright (c) 2016-2018 CNRS
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

#ifndef __se3_lie_group_hpp__
#define __se3_lie_group_hpp__

#include "pinocchio/assert.hpp"
#include "pinocchio/multibody/liegroup/vector-space.hpp"
#include "pinocchio/multibody/liegroup/cartesian-product.hpp"
#include "pinocchio/multibody/liegroup/special-orthogonal.hpp"
#include "pinocchio/multibody/liegroup/special-euclidean.hpp"

#include "pinocchio/multibody/joint/fwd.hpp"

namespace se3 {
  struct LieGroupMap {
    template<typename JointModel> struct operation {
      typedef VectorSpaceOperation<JointModel::NQ> type;
    };
  };
  template<typename JointModel>
  struct LieGroup {
    typedef typename LieGroupMap::operation<JointModel>::type type;
  };

  template<> struct LieGroupMap::operation <JointModelComposite> {};
  template<> struct LieGroupMap::operation <JointModelSpherical> {
    typedef SpecialOrthogonalOperation<3> type;
  };
  template<> struct LieGroupMap::operation <JointModelFreeFlyer> {
    typedef SpecialEuclideanOperation<3> type;
  };
  template<> struct LieGroupMap::operation <JointModelPlanar> {
    typedef SpecialEuclideanOperation<2> type;
  };
  template<typename Scalar, int Options, int axis>
  struct LieGroupMap::operation <JointModelRevoluteUnboundedTpl<Scalar,Options,axis> > {
    typedef SpecialOrthogonalOperation<2> type;
  };
}

#endif // ifndef __se3_lie_group_hpp__
