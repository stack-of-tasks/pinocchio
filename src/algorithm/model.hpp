//
// Copyright (c) 2019 CNRS
//

#ifndef __pinocchio_algorithm_model_hpp__
#define __pinocchio_algorithm_model_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/geometry.hpp"

namespace pinocchio
{
  /**
   *  \brief Append a child model into a parent model, after a specific frame.
   *  the model given as reference argument.
   *
   *  \param[in] modelA the parent model
   *  \param[in] modelB the child model
   *  \param[in] geomModelA the parent geometry model
   *  \param[in] geomModelB the child geometry model
   *  \param[in] frameInModelA index of the frame of modelA where to append modelB.
   *  \param[in] aMb pose of modelB universe joint (index 0) in frameInModelA.
   *  \param[out] model the resulting model
   *
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  void
  appendModel(const ModelTpl<Scalar,Options,JointCollectionTpl> & modelA,
              const ModelTpl<Scalar,Options,JointCollectionTpl> & modelB,
              FrameIndex frameInModelA,
              const SE3Tpl<Scalar, Options>& aMb,
              ModelTpl<Scalar,Options,JointCollectionTpl>& model);

  /**
   *  \copydoc pinocchio::appendModel(const ModelTpl<Scalar,Options,JointCollectionTpl>&, const ModelTpl<Scalar,Options,JointCollectionTpl> & modelB, FrameIndex frameInModelA, const SE3Tpl<Scalar, Options>& aMb, ModelTpl<Scalar,Options,JointCollectionTpl>& model);
   *
   *  \param[in] geomModelA the parent geometry model
   *  \param[in] geomModelB the child geometry model
   *  \param[out] geomModel
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  void
  appendModel(const ModelTpl<Scalar,Options,JointCollectionTpl> & modelA,
              const ModelTpl<Scalar,Options,JointCollectionTpl> & modelB,
              const GeometryModel& geomModelA,
              const GeometryModel& geomModelB,
              FrameIndex frameInModelA,
              const SE3Tpl<Scalar, Options>& aMb,
              ModelTpl<Scalar,Options,JointCollectionTpl>& model,
              GeometryModel& geomModel);

} // namespace pinocchio

#include "pinocchio/algorithm/model.hxx"

#endif // ifndef __pinocchio_algorithm_model_hpp__
