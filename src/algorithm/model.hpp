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
   *  \brief Append a child model into a parent model, after a specific frame given by its index.
   *
   *  \param[in] modelA the parent model.
   *  \param[in] modelB the child model.
   *  \param[in] frameInModelA index of the frame of modelA where to append modelB.
   *  \param[in] aMb pose of modelB universe joint (index 0) in frameInModelA.
   *  \param[out] model the resulting model.
   *
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  void
  appendModel(const ModelTpl<Scalar,Options,JointCollectionTpl> & modelA,
              const ModelTpl<Scalar,Options,JointCollectionTpl> & modelB,
              const FrameIndex frameInModelA,
              const SE3Tpl<Scalar,Options> & aMb,
              ModelTpl<Scalar,Options,JointCollectionTpl> & model);

  /**
   *  \brief Append a child model into a parent model, after a specific frame given by its index.
   *
   *  \param[in] modelA the parent model.
   *  \param[in] modelB the child model.
   *  \param[in] frameInModelA index of the frame of modelA where to append modelB.
   *  \param[in] aMb pose of modelB universe joint (index 0) in frameInModelA.
   *
   *  \return A new model containing the fusion of modelA and modelB.
   *
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  ModelTpl<Scalar,Options,JointCollectionTpl>
  appendModel(const ModelTpl<Scalar,Options,JointCollectionTpl> & modelA,
              const ModelTpl<Scalar,Options,JointCollectionTpl> & modelB,
              const FrameIndex frameInModelA,
              const SE3Tpl<Scalar,Options> & aMb)
  {
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    Model model;
    
    appendModel(modelA,modelB,frameInModelA,aMb,model);
    
    return model;
  }

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
              const FrameIndex frameInModelA,
              const SE3Tpl<Scalar,Options> & aMb,
              ModelTpl<Scalar,Options,JointCollectionTpl> & model,
              GeometryModel & geomModel);

  /**
   *
   *  \brief Build a reduced model from a given input model and a list of joint to lock.
   *
   *  \param[in] model the input model to reduce.
   *  \param[in] list_of_joints_to_lock list of joints to lock in the input model.
   *  \param[in] reference_configuration reference configuration.
   *  \param[out] reduced_model the reduced model.
   *
   *  \remarks All the joints that have been set to be fixed in the new reduced_model now appear in the kinematic tree as a Frame as FIXED_JOINT.
   *
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
  void
  buildReducedModel(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                    std::vector<JointIndex> list_of_joints_to_lock,
                    const Eigen::MatrixBase<ConfigVectorType> & reference_configuration,
                    ModelTpl<Scalar,Options,JointCollectionTpl> & reduced_model);

  /**
   *
   *  \brief Build a reduced model from a given input model and a list of joint to lock.
   *
   *  \param[in] model the input model to reduce.
   *  \param[in] list_of_joints_to_lock list of joints to lock in the input model.
   *  \param[in] reference_configuration reference configuration.
   *
   *  \returns A reduce model of the input model.
   *
   *  \remarks All the joints that have been set to be fixed in the new reduced_model now appear in the kinematic tree as a Frame as FIXED_JOINT.
   *
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
  ModelTpl<Scalar,Options,JointCollectionTpl>
  buildReducedModel(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                    const std::vector<JointIndex> & list_of_joints_to_lock,
                    const Eigen::MatrixBase<ConfigVectorType> & reference_configuration)
  {
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    Model reduced_model;
    
    buildReducedModel(model,list_of_joints_to_lock,reference_configuration,reduced_model);
   
    return reduced_model;
  }

  /**
   *
   *  \brief Build a reduced model and a rededuced geometry model  from a given input model, a given input geometry model and a list of joint to lock.
   *
   *  \param[in] model the input model to reduce.
   *  \param[in] geom_model the input geometry model to reduce.
   *  \param[in] list_of_joints_to_lock list of joints to lock in the input model.
   *  \param[in] reference_configuration reference configuration.
   *  \param[out] reduced_model the reduced model.
   *  \param[out] reduced_geom_model the reduced geometry model.
   *
   *  \remarks All the joints that have been set to be fixed in the new reduced_model now appear in the kinematic tree as a Frame as FIXED_JOINT.  
   *
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
  void
  buildReducedModel(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                    const GeometryModel & geom_model,
                    const std::vector<JointIndex> & list_of_joints_to_lock,
                    const Eigen::MatrixBase<ConfigVectorType> & reference_configuration,
                    ModelTpl<Scalar,Options,JointCollectionTpl> & reduced_model,
                    GeometryModel & reduced_geom_model);

} // namespace pinocchio

#include "pinocchio/algorithm/model.hxx"

#endif // ifndef __pinocchio_algorithm_model_hpp__
