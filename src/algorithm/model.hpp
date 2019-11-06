//
// Copyright (c) 2019 CNRS INRIA
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
   *  \param[in] modelA the parent model.
   *  \param[in] modelB the child model.
   *  \param[in] geomModelA the parent geometry model.
   *  \param[in] geomModelB the child geometry model.
   *  \param[in] frameInModelA index of the frame of modelA where to append modelB.
   *  \param[in] aMb pose of modelB universe joint (index 0) in frameInModelA.
   *  \param[out] model the resulting model.
   *
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  void
  appendModel(const ModelTpl<Scalar,Options,JointCollectionTpl> & modelA,
              const ModelTpl<Scalar,Options,JointCollectionTpl> & modelB,
              FrameIndex frameInModelA,
              const SE3Tpl<Scalar,Options> & aMb,
              ModelTpl<Scalar,Options,JointCollectionTpl> & model);

  /**
   *  \copydoc pinocchio::appendModel(const ModelTpl<Scalar,Options,JointCollectionTpl>&, const ModelTpl<Scalar,Options,JointCollectionTpl> & modelB, FrameIndex frameInModelA, const SE3Tpl<Scalar, Options>& aMb, ModelTpl<Scalar,Options,JointCollectionTpl>& model);
   *
   *  \param[in] geomModelA the parent geometry model.
   *  \param[in] geomModelB the child geometry model.
   *  \param[out] geomModel the resulting geometry model.
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  void
  appendModel(const ModelTpl<Scalar,Options,JointCollectionTpl> & modelA,
              const ModelTpl<Scalar,Options,JointCollectionTpl> & modelB,
              const GeometryModel & geomModelA,
              const GeometryModel & geomModelB,
              FrameIndex frameInModelA,
              const SE3Tpl<Scalar,Options> & aMb,
              ModelTpl<Scalar,Options,JointCollectionTpl> & model,
              GeometryModel & geomModel);

  /**
   *
   *  \brief Build a reduce model from a given input model and a list of joint to lock.
   *
   *  \param[in] model the input model to reduce.
   *  \param[in] list_of_joints list of joints to lock in the input model.
   *  \param[in] reference_configuration reference configuration
   *
   *  \returns A reduce model of the input model.
   *
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
  ModelTpl<Scalar,Options,JointCollectionTpl>
  buildReducedModel(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                    std::vector<JointIndex> list_of_joints_to_lock,
                    const Eigen::MatrixBase<ConfigVectorType> & reference_configuration);

} // namespace pinocchio

#include "pinocchio/algorithm/model.hxx"

#endif // ifndef __pinocchio_algorithm_model_hpp__
