//
// Copyright (c) 2016 CNRS
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
  struct LieGroupTpl {
    template<typename JointModel> struct operation {
      typedef VectorSpaceOperation<JointModel::NQ> type;
    };
  };
  template<typename JointModel>
  struct LieGroup {
    typedef typename LieGroupTpl::operation<JointModel>::type type;
  };

  template<> struct LieGroupTpl::operation <JointModelComposite> {};
  template<> struct LieGroupTpl::operation <JointModelSpherical> {
    typedef SpecialOrthogonalOperation<3> type;
  };
  template<> struct LieGroupTpl::operation <JointModelFreeFlyer> {
    typedef SpecialEuclideanOperation<3> type;
  };
  template<> struct LieGroupTpl::operation <JointModelPlanar> {
    typedef SpecialEuclideanOperation<2> type;
  };
  template<int Axis> struct LieGroupTpl::operation <JointModelRevoluteUnbounded<Axis> > {
    typedef SpecialOrthogonalOperation<2> type;
  };

  // TODO REMOVE: For testing purposes only
  // template<>
  // struct LieGroupTpl::operation <JointModelTranslation> {
    // typedef CartesianProductOperation<
      // CartesianProductOperation<typename LieGroup<JointModelPX>::type, typename LieGroup<JointModelPY>::type>,
      // typename LieGroup<JointModelPZ>::type
      // > type;
  // };
}

#endif // ifndef __se3_lie_group_hpp__
