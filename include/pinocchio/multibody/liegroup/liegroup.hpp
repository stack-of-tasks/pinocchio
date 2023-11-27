//
// Copyright (c) 2016-2019 CNRS INRIA
//

#ifndef __pinocchio_lie_group_hpp__
#define __pinocchio_lie_group_hpp__

#include "pinocchio/multibody/liegroup/vector-space.hpp"
#include "pinocchio/multibody/liegroup/cartesian-product.hpp"
#include "pinocchio/multibody/liegroup/special-orthogonal.hpp"
#include "pinocchio/multibody/liegroup/special-euclidean.hpp"

#include "pinocchio/multibody/joint/fwd.hpp"

namespace pinocchio
{
  struct LieGroupMap
  {
    template<typename JointModel>
    struct operation
    {
      typedef VectorSpaceOperationTpl<JointModel::NQ,typename JointModel::Scalar, JointModel::Options> type;
    };
  };
  
  template<typename JointModel>
  struct LieGroup
  {
    typedef typename LieGroupMap::operation<JointModel>::type type;
  };

  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct LieGroupMap::operation< JointModelCompositeTpl<Scalar,Options,JointCollectionTpl> >
  {};
  
  template<typename Scalar, int Options>
  struct LieGroupMap::operation< JointModelSphericalTpl<Scalar,Options> >
  {
    typedef SpecialOrthogonalOperationTpl<3,Scalar,Options> type;
  };
  
  template<typename Scalar, int Options>
  struct LieGroupMap::operation< JointModelFreeFlyerTpl<Scalar,Options> >
  {
    typedef SpecialEuclideanOperationTpl<3,Scalar,Options> type;
  };
  
  template<typename Scalar, int Options>
  struct LieGroupMap::operation< JointModelPlanarTpl<Scalar,Options> >
  {
    typedef SpecialEuclideanOperationTpl<2,Scalar,Options> type;
  };
  
  template<typename Scalar, int Options, int axis>
  struct LieGroupMap::operation<JointModelRevoluteUnboundedTpl<Scalar,Options,axis> >
  {
    typedef SpecialOrthogonalOperationTpl<2,Scalar,Options> type;
  };
  
  template<typename Scalar, int Options>
  struct LieGroupMap::operation<JointModelRevoluteUnboundedUnalignedTpl<Scalar,Options> >
  {
    typedef SpecialOrthogonalOperationTpl<2,Scalar,Options> type;
  };
  
}

#endif // ifndef __pinocchio_lie_group_hpp__
