//
// Copyright (c) 2019 CNRS
//

#ifndef __pinocchio_algorithm_model_hpp__
#define __pinocchio_algorithm_model_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/geometry.hpp"

namespace pinocchio
{
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  void
  appendModel(const ModelTpl<Scalar,Options,JointCollectionTpl> & modelA,
              const ModelTpl<Scalar,Options,JointCollectionTpl> & modelB,
              const GeometryModel& geomModelA,
              const GeometryModel& geomModelB,
              FrameIndex frameInModelA,
              SE3 aMb,
              ModelTpl<Scalar,Options,JointCollectionTpl>& model,
              GeometryModel& geomModel);

} // namespace pinocchio

#include "pinocchio/algorithm/model.hxx"

#endif // ifndef __pinocchio_algorithm_model_hpp__
