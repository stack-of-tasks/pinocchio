//
// Copyright (c) 2018-2018 CNRS
// Copyright (c) 2018-2025 INRIA
//

#pragma once

// IWYU pragma: private, include "pinocchio/algorithm/model.hpp"

#ifdef PINOCCHIO_LSP
  #undef PINOCCHIO_LSP
  #include "pinocchio/algorithm/model.hpp"
#endif // PINOCCHIO_LSP

namespace pinocchio
{
  namespace details
  {

    // Retrieve the joint id in model_out, given the info of model_in.
    // If the user change all the joint names, the universe name won't correspond to the first joint
    // in the tree when searching by name. We thus need to retrieve it with other means, e.g.
    // checking the index of the joints.
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    JointIndex getJointId(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model_in,
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model_out,
      const std::string & joint_name_in_model_in)
    {
      const JointIndex joint_id = model_in.getJointId(joint_name_in_model_in);
      assert(joint_id < model_in.joints.size());
      if (joint_id == 0 && model_in.parents[0] == 0) // This is the universe, maybe renamed.
        return model_out.getJointId(model_out.names[0]);
      else
        return model_out.getJointId(joint_name_in_model_in);
    }

    // Retrieve the frame id in model_out, given the info of model_in.
    // If the user change all the frame names, the universe name won't correspond to the first frame
    // in the tree when searching by name. We thus need to retrieve it with other means, e.g.
    // checking the fields of the frames.
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    FrameIndex getFrameId(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model_in,
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model_out,
      const std::string & frame_name_in_model_in,
      const FrameType & type)
    {
      const FrameIndex frame_id = model_in.getFrameId(frame_name_in_model_in);
      assert(frame_id < model_in.frames.size());
      if (
        frame_id == 0 && model_in.frames[0].parentFrame == 0
        && model_in.frames[0].parentJoint == 0) // This is the universe, maybe renamed.
        return model_out.getFrameId(model_out.frames[0].name, type);
      else
        return model_out.getFrameId(frame_name_in_model_in, type);
    }

    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    void appendUniverseToModel(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & modelAB,
      const GeometryModel & geomModelAB,
      FrameIndex parentFrame,
      const SE3Tpl<Scalar, Options> & pfMAB,
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      GeometryModel & geomModel)
    {
      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      typedef typename Model::Frame Frame;

      PINOCCHIO_THROW_IF(
        parentFrame >= model.frames.size(), std::invalid_argument,
        "parentFrame is greater than the size of the frames vector.");

      const auto pframe_placement = model.frames[parentFrame].placement;
      JointIndex jid = model.frames[parentFrame].parentJoint;
      assert(jid < model.joints.size());

      // If inertia is not NaN, add it.
      if (modelAB.inertias[0] == modelAB.inertias[0])
        model.appendBodyToJoint(jid, modelAB.inertias[0], pframe_placement * pfMAB);

      // Add all frames whose parent is this joint.
      for (FrameIndex fid = 1; fid < modelAB.frames.size(); ++fid)
      {
        Frame frame = modelAB.frames[fid];
        if (frame.parentJoint == 0)
        {
          PINOCCHIO_CHECK_INPUT_ARGUMENT(
            !model.existFrame(frame.name, frame.type),
            "The two models have conflicting frame names.");

          frame.parentJoint = jid;
          if (frame.parentFrame != 0)
          {
            frame.parentFrame = getFrameId(
              modelAB, model, modelAB.frames[frame.parentFrame].name,
              modelAB.frames[frame.parentFrame].type);
          }
          else
          {
            frame.parentFrame = parentFrame;
          }

          // Modify frame placement
          frame.placement = pframe_placement * pfMAB * frame.placement;
          // Inertias are already computed in model.appendBodyToJoint call.
          // No need to append them again.
          model.addFrame(frame, false);
        }
      }

      // Add all geometries whose parent is this joint.
      for (GeomIndex gid = 0; gid < geomModelAB.geometryObjects.size(); ++gid)
      {
        GeometryObject go = geomModelAB.geometryObjects[gid];
        if (go.parentJoint == 0)
        {
          go.parentJoint = jid;
          if (go.parentFrame != 0)
          {
            go.parentFrame = getFrameId(
              modelAB, model, modelAB.frames[go.parentFrame].name,
              modelAB.frames[go.parentFrame].type);
          }
          else
          {
            go.parentFrame = parentFrame;
          }
          typedef typename GeometryObject::SE3 GeometrySE3;
          const GeometrySE3 geometry_placement(pframe_placement * pfMAB);
          go.placement = geometry_placement * go.placement;
          geomModel.addGeometryObject(go);
        }
      }
    }

    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    struct AppendJointOfModelAlgoTpl
    : public fusion::JointUnaryVisitorBase<
        AppendJointOfModelAlgoTpl<Scalar, Options, JointCollectionTpl>>
    {

      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      typedef typename Model::Frame Frame;

      typedef boost::fusion::vector<
        const Model &,
        const GeometryModel &,
        JointIndex,
        const typename Model::SE3 &,
        Model &,
        GeometryModel &>
        ArgsType;

      template<typename JointModel>
      static std::enable_if_t<
        !std::is_same_v<JointModel, JointModelMimicTpl<Scalar, Options, JointCollectionTpl>>,
        JointModel>
      updateMimicIds(
        const JointModel & jmodel, const Model & /*old_model*/, const Model & /*new_model*/)
      {
        return jmodel;
      }

      template<typename JointModel>
      static std::enable_if_t<
        std::is_same_v<JointModel, JointModelMimicTpl<Scalar, Options, JointCollectionTpl>>,
        JointModel>
      updateMimicIds(
        const JointModelMimicTpl<Scalar, Options, JointCollectionTpl> & jmodel,
        const Model & old_model,
        const Model & new_model)
      {
        JointModel res(jmodel);
        const JointIndex mimicked_old_id = res.jmodel().id();
        const std::string mimicked_name = old_model.names[mimicked_old_id];
        const JointIndex mimicked_new_id = new_model.getJointId(mimicked_name);
        res.setMimicIndexes(
          mimicked_new_id, new_model.joints[mimicked_new_id].idx_q(),
          new_model.joints[mimicked_new_id].idx_v(),
          new_model.joints[mimicked_new_id].idx_vExtended());
        return res;
      }

      template<typename JointModel>
      static void algo(
        const JointModelBase<JointModel> & jmodel_in,
        const Model & modelAB,
        const GeometryModel & geomModelAB,
        JointIndex parent_id,
        const typename Model::SE3 & pMi,
        Model & model,
        GeometryModel & geomModel)
      {
        // If old parent is universe, use what's provided in the input.
        // otherwise, get the parent from modelAB.
        const JointIndex joint_id_in = jmodel_in.id();
        if (modelAB.parents[joint_id_in] > 0)
          parent_id = getJointId(modelAB, model, modelAB.names[modelAB.parents[joint_id_in]]);

        PINOCCHIO_CHECK_INPUT_ARGUMENT(
          !model.existJointName(modelAB.names[joint_id_in]),
          "The two models have conflicting joint names.");

        // For mimic joints, update the reference joint id
        JointModel jmodel_inter = updateMimicIds<JointModel>(jmodel_in.derived(), modelAB, model);

        JointIndex joint_id_out = model.addJoint(
          parent_id,
          jmodel_inter, // Use the intermediate joint (jmodel_inter) with updates id, idx, ...
          pMi * modelAB.jointPlacements[joint_id_in], modelAB.names[joint_id_in],
          jmodel_in.jointVelocitySelector(modelAB.lowerEffortLimit),
          jmodel_in.jointVelocitySelector(modelAB.upperEffortLimit),
          jmodel_in.jointVelocitySelector(modelAB.lowerVelocityLimit),
          jmodel_in.jointVelocitySelector(modelAB.upperVelocityLimit),
          jmodel_in.jointConfigSelector(modelAB.lowerPositionLimit),
          jmodel_in.jointConfigSelector(modelAB.upperPositionLimit),
          jmodel_in.jointConfigSelector(modelAB.positionLimitMargin),
          jmodel_in.jointVelocitySelector(modelAB.lowerDryFrictionLimit),
          jmodel_in.jointVelocitySelector(modelAB.upperDryFrictionLimit),
          jmodel_in.jointVelocitySelector(modelAB.damping),
          jmodel_in.jointVelocitySelector(modelAB.lowerAccelerationLimit),
          jmodel_in.jointVelocitySelector(modelAB.upperAccelerationLimit),
          jmodel_in.jointVelocitySelector(modelAB.lowerJerkLimit),
          jmodel_in.jointVelocitySelector(modelAB.upperJerkLimit));
        assert(joint_id_out < model.joints.size());

        model.appendBodyToJoint(joint_id_out, modelAB.inertias[joint_id_in]);

        typename Model::JointModel & jmodel_out = model.joints[joint_id_out];

        jmodel_out.jointVelocitySelector(model.rotorInertia) =
          jmodel_in.jointVelocitySelector(modelAB.rotorInertia);
        jmodel_out.jointVelocitySelector(model.rotorGearRatio) =
          jmodel_in.jointVelocitySelector(modelAB.rotorGearRatio);
        // Add all frames whose parent is this joint.
        for (FrameIndex fid = 1; fid < modelAB.frames.size(); ++fid)
        {
          Frame frame = modelAB.frames[fid];
          if (frame.parentJoint == jmodel_in.id())
          {
            PINOCCHIO_CHECK_INPUT_ARGUMENT(
              !model.existFrame(frame.name, frame.type),
              "The two models have conflicting frame names.");
            frame.parentJoint = joint_id_out;
            assert(frame.parentFrame > 0 || frame.type == JOINT);
            if (frame.parentFrame != 0)
            {
              frame.parentFrame = getFrameId(
                modelAB, model, modelAB.frames[frame.parentFrame].name,
                modelAB.frames[frame.parentFrame].type);
            }
            // Inertias are already computed in modelAB.inertias[joint_id_in].
            // No need to append them again.
            model.addFrame(frame, false);
          }
        }
        // Add all geometries whose parent is this joint.
        for (GeomIndex gid = 0; gid < geomModelAB.geometryObjects.size(); ++gid)
        {
          GeometryObject go = geomModelAB.geometryObjects[gid];
          if (go.parentJoint == joint_id_in)
          {
            go.parentJoint = joint_id_out;
            assert(go.parentFrame > 0);
            if (go.parentFrame != 0 && go.parentFrame < modelAB.frames.size())
            {
              go.parentFrame = getFrameId(
                modelAB, model, modelAB.frames[go.parentFrame].name,
                modelAB.frames[go.parentFrame].type);
            }
            geomModel.addGeometryObject(go);
          }
        }
      }
    };

    /// Insert \p value inside a sorted \p container so container stay sorted
    template<typename Container>
    void insertSort(typename Container::const_reference value, Container & container)
    {
      container.insert(std::lower_bound(container.begin(), container.end(), value), value);
    }

  } // namespace details

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  void appendModel(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & modelA,
    const ModelTpl<Scalar, Options, JointCollectionTpl> & modelB,
    const FrameIndex frameInModelA,
    const SE3Tpl<Scalar, Options> & aMb,
    ModelTpl<Scalar, Options, JointCollectionTpl> & model)
  {
    GeometryModel geomModelA, geomModelB, geomModel;

    appendModel(modelA, modelB, geomModelA, geomModelB, frameInModelA, aMb, model, geomModel);
  }

  // Compute whether Joint child is a descendent of parent in a given model
  // Joints are represented by their id in the model
  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  static bool hasAncestor(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    JointIndex child,
    JointIndex parent)
  {
    typedef typename ModelTpl<Scalar, Options, JointCollectionTpl>::IndexVector IndexVector_t;
    // Any joints has universe as an acenstor
    assert(model.supports[child][0] == 0);
    for (typename IndexVector_t::const_iterator it = model.supports[child].begin();
         it != model.supports[child].end(); ++it)
    {
      if (*it == parent)
        return true;
    }
    return false;
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  void appendModel(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & modelA,
    const ModelTpl<Scalar, Options, JointCollectionTpl> & modelB,
    const GeometryModel & geomModelA,
    const GeometryModel & geomModelB,
    const FrameIndex frameInModelA,
    const SE3Tpl<Scalar, Options> & aMb,
    ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    GeometryModel & geomModel)
  {
    typedef details::AppendJointOfModelAlgoTpl<Scalar, Options, JointCollectionTpl>
      AppendJointOfModelAlgo;
    typedef typename AppendJointOfModelAlgo::ArgsType ArgsType;

    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      (bool)(frameInModelA < (FrameIndex)modelA.nframes),
      "frameInModelA is an invalid Frame index, greater than the "
      "number of frames contained in modelA.");

    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef typename Model::SE3 SE3;
    typedef typename Model::Frame Frame;

    const Frame & frame = modelA.frames[frameInModelA];
    static const SE3 id = SE3::Identity();

    int njoints = modelA.njoints + modelB.njoints - 1;
    model.names.reserve((size_t)njoints);
    model.joints.reserve((size_t)njoints);
    model.jointPlacements.reserve((size_t)njoints);
    model.parents.reserve((size_t)njoints);
    model.inertias.reserve((size_t)njoints);
    int nframes = modelA.nframes + modelB.nframes - 1;
    model.frames.reserve((size_t)nframes);

    geomModel.geometryObjects.reserve(geomModelA.ngeoms + geomModelB.ngeoms);

    details::appendUniverseToModel(modelA, geomModelA, 0, id, model, geomModel);
    // Compute joints of A that should be added before and after joints of B
    std::vector<JointIndex> AJointsBeforeB;
    std::vector<JointIndex> AJointsAfterB;
    // All joints until the parent of frameInModelA come first
    for (JointIndex jid = 1; jid <= frame.parentJoint; ++jid)
    {
      AJointsBeforeB.push_back(jid);
    }
    // descendants of the parent of frameInModelA come also before model B
    // TODO(jcarpent): enhancement by taking into account the compactness of the joint ordering.
    for (JointIndex jid = frame.parentJoint + 1; jid < modelA.joints.size(); ++jid)
    {
      if (hasAncestor(modelA, jid, frame.parentJoint))
      {
        AJointsBeforeB.push_back(jid);
      }
      else
      {
        AJointsAfterB.push_back(jid);
      }
    }
    // Copy modelA joints that should come before model B
    for (std::vector<JointIndex>::const_iterator jid = AJointsBeforeB.begin();
         jid != AJointsBeforeB.end(); ++jid)
    {
      ArgsType args(modelA, geomModelA, 0, id, model, geomModel);
      AppendJointOfModelAlgo::run(modelA.joints[*jid], args);
    }

    // Copy modelB joints
    details::appendUniverseToModel(
      modelB, geomModelB, details::getFrameId(modelA, model, frame.name, frame.type), aMb, model,
      geomModel);
    for (JointIndex jid = 1; jid < modelB.joints.size(); ++jid)
    {
      SE3 pMi = (modelB.parents[jid] == 0 ? frame.placement * aMb : id);
      ArgsType args(modelB, geomModelB, frame.parentJoint, pMi, model, geomModel);
      AppendJointOfModelAlgo::run(modelB.joints[jid], args);
    }

    // Copy remaining joints of modelA
    // Copy modelA joints that should come before model B
    for (std::vector<JointIndex>::const_iterator jid = AJointsAfterB.begin();
         jid != AJointsAfterB.end(); ++jid)
    {
      ArgsType args(modelA, geomModelA, 0, id, model, geomModel);
      AppendJointOfModelAlgo::run(modelA.joints[*jid], args);
    }

    // Retrieve and set the reference configurations
    typedef typename Model::ConfigVectorMap ConfigVectorMap;
    typename Model::ConfigVectorType neutral_config_vector(model.nq);
    // Get neutral configuration
    neutral(model, neutral_config_vector);

    // Get all reference keys from ModelA
    for (typename ConfigVectorMap::const_iterator config_it =
           modelA.referenceConfigurations.begin();
         config_it != modelA.referenceConfigurations.end(); ++config_it)
    {
      const std::string & config_name = config_it->first;
      const typename Model::ConfigVectorType & config_vectorA = config_it->second;

      typename Model::ConfigVectorType config_vector(neutral_config_vector);
      for (JointIndex joint_idA = 1; joint_idA < modelA.joints.size(); ++joint_idA)
      {
        const JointIndex joint_id = model.getJointId(modelA.names[joint_idA]);
        const typename Model::JointModel & joint_model = model.joints[joint_id];
        const typename Model::JointModel & joint_modelA = modelA.joints[joint_idA];

        joint_model.jointConfigSelector(config_vector) =
          joint_modelA.jointConfigSelector(config_vectorA);
      }

      model.referenceConfigurations.insert(std::make_pair(config_name, config_vector));
    }

    // Get all reference keys from ModelB
    for (typename ConfigVectorMap::const_iterator config_it =
           modelB.referenceConfigurations.begin();
         config_it != modelB.referenceConfigurations.end(); ++config_it)
    {
      const std::string & config_name = config_it->first;
      const typename Model::ConfigVectorType & config_vectorB = config_it->second;

      if (model.referenceConfigurations.find(config_name) == model.referenceConfigurations.end())
      {
        // not found
        model.referenceConfigurations.insert(std::make_pair(config_name, neutral_config_vector));
      }

      typename Model::ConfigVectorType & config_vector =
        model.referenceConfigurations.find(config_name)->second;
      for (JointIndex joint_idB = 1; joint_idB < modelB.joints.size(); ++joint_idB)
      {
        const JointIndex joint_id = model.getJointId(modelB.names[joint_idB]);
        const typename Model::JointModel & joint_model = model.joints[joint_id];
        const typename Model::JointModel & joint_modelB = modelB.joints[joint_idB];

        joint_model.jointConfigSelector(config_vector) =
          joint_modelB.jointConfigSelector(config_vectorB);
      }
    }

#ifdef PINOCCHIO_WITH_COLLISION
    // Add collision pairs of geomModelA and geomModelB
    geomModel.collisionPairs.reserve(
      geomModelA.collisionPairs.size() + geomModelB.collisionPairs.size()
      + geomModelA.geometryObjects.size() * geomModelB.geometryObjects.size());

    for (std::size_t icp = 0; icp < geomModelA.collisionPairs.size(); ++icp)
    {
      const CollisionPair & cp = geomModelA.collisionPairs[icp];
      GeomIndex go1 = geomModel.getGeometryId(geomModelA.geometryObjects[cp.first].name);
      GeomIndex go2 = geomModel.getGeometryId(geomModelA.geometryObjects[cp.second].name);
      geomModel.addCollisionPair(CollisionPair(go1, go2));
    }

    for (std::size_t icp = 0; icp < geomModelB.collisionPairs.size(); ++icp)
    {
      const CollisionPair & cp = geomModelB.collisionPairs[icp];
      GeomIndex go1 = geomModel.getGeometryId(geomModelB.geometryObjects[cp.first].name);
      GeomIndex go2 = geomModel.getGeometryId(geomModelB.geometryObjects[cp.second].name);
      geomModel.addCollisionPair(CollisionPair(go1, go2));
    }

    // add the collision pairs between geomModelA and geomModelB.
    for (Index i = 0; i < geomModelA.geometryObjects.size(); ++i)
    {
      GeomIndex go1 = geomModel.getGeometryId(geomModelA.geometryObjects[i].name);
      for (Index j = 0; j < geomModelB.geometryObjects.size(); ++j)
      {
        GeomIndex go2 = geomModel.getGeometryId(geomModelB.geometryObjects[j].name);
        if (
          geomModel.geometryObjects[go1].parentJoint != geomModel.geometryObjects[go2].parentJoint)
          geomModel.addCollisionPair(CollisionPair(go1, go2));
      }
    }
#endif
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType>
  void buildReducedModel(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & input_model,
    std::vector<JointIndex> list_of_joints_to_lock,
    const Eigen::MatrixBase<ConfigVectorType> & reference_configuration,
    ModelTpl<Scalar, Options, JointCollectionTpl> & reduced_model)
  {
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      reference_configuration.size(), input_model.nq,
      "The configuration vector is not equal to model.nq.");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      list_of_joints_to_lock.size() <= (size_t)input_model.njoints,
      "The number of joints to lock is greater than the total of joints in the reduced_model");

    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef typename Model::JointModel JointModel;
    typedef typename Model::JointData JointData;
    typedef typename Model::Frame Frame;
    typedef typename Model::SE3 SE3;

    // Sort indexes
    std::sort(list_of_joints_to_lock.begin(), list_of_joints_to_lock.end());

    typename Model::FrameVector::const_iterator frame_it = input_model.frames.begin();

    // Check that they are not two identical elements
    for (size_t id = 1; id < list_of_joints_to_lock.size(); ++id)
    {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(
        list_of_joints_to_lock[id - 1] < list_of_joints_to_lock[id],
        "The input list_of_joints_to_lock contains two identical indexes.");
    }

    // Reserve memory
    const Eigen::Index njoints = input_model.njoints - (Eigen::Index)list_of_joints_to_lock.size();
    reduced_model.names.reserve((size_t)njoints);
    reduced_model.joints.reserve((size_t)njoints);
    reduced_model.jointPlacements.reserve((size_t)njoints);
    reduced_model.parents.reserve((size_t)njoints);
    reduced_model.inertias.reserve((size_t)njoints);

    reduced_model.names[0] = input_model.names[0];
    reduced_model.joints[0] = input_model.joints[0];
    reduced_model.jointPlacements[0] = input_model.jointPlacements[0];
    reduced_model.parents[0] = input_model.parents[0];
    reduced_model.inertias[0] = input_model.inertias[0];

    reduced_model.name = input_model.name;
    reduced_model.gravity = input_model.gravity;

    size_t current_index_to_lock = 0;

    for (JointIndex joint_id = 1; joint_id < (JointIndex)input_model.njoints; ++joint_id)
    {
      const JointIndex joint_id_to_lock = (current_index_to_lock < list_of_joints_to_lock.size())
                                            ? list_of_joints_to_lock[current_index_to_lock]
                                            : 0;

      const JointIndex input_parent_joint_index = input_model.parents[joint_id];
      const std::string & joint_name = input_model.names[joint_id];
      const JointModel & joint_input_model = input_model.joints[joint_id];

      // retrieve the closest joint parent in the new kinematic tree
      const std::string & parent_joint_name = input_model.names[input_parent_joint_index];
      const bool exist_parent_joint = reduced_model.existJointName(parent_joint_name);

      const JointIndex reduced_parent_joint_index =
        exist_parent_joint
          ? reduced_model.getJointId(parent_joint_name)
          : reduced_model.frames[reduced_model.getFrameId(parent_joint_name)].parentJoint;

      const SE3 parent_frame_placement =
        exist_parent_joint
          ? SE3::Identity()
          : reduced_model.frames[reduced_model.getFrameId(parent_joint_name)].placement;

      const FrameIndex reduced_previous_frame_index =
        exist_parent_joint ? 0 : reduced_model.getFrameId(parent_joint_name);

      if (joint_id == joint_id_to_lock)
      {
        // the joint should not be added to the Model but aggragated to its parent joint
        // Add frames up to the joint to lock
        while ((*frame_it).name != joint_name)
        {
          ++frame_it;
          const Frame & input_frame = *frame_it;
          if (input_frame.name == joint_name)
            break;
          const std::string & support_joint_name = input_model.names[input_frame.parentJoint];

          std::vector<JointIndex>::const_iterator support_joint_it = std::find(
            list_of_joints_to_lock.begin(), list_of_joints_to_lock.end(), input_frame.parentJoint);

          if (support_joint_it != list_of_joints_to_lock.end())
          {
            if (
              input_frame.type == JOINT && reduced_model.existFrame(input_frame.name)
              && support_joint_name == input_frame.name)
              continue; // this means that the Joint is now fixed and has been replaced by a Frame.
                        // No need to add a new one.

            // The joint has been removed and replaced by a Frame
            const FrameIndex joint_frame_id = reduced_model.getFrameId(support_joint_name);
            const Frame & joint_frame = reduced_model.frames[joint_frame_id];
            Frame reduced_frame = input_frame;
            reduced_frame.placement = joint_frame.placement * input_frame.placement;
            reduced_frame.parentJoint = joint_frame.parentJoint;
            reduced_frame.parentFrame =
              reduced_model.getFrameId(input_model.frames[input_frame.parentFrame].name);
            reduced_model.addFrame(reduced_frame, false);
          }
          else
          {
            Frame reduced_frame = input_frame;
            reduced_frame.parentJoint =
              reduced_model.getJointId(input_model.names[input_frame.parentJoint]);
            reduced_frame.parentFrame =
              reduced_model.getFrameId(input_model.frames[input_frame.parentFrame].name);
            reduced_model.addFrame(reduced_frame, false);
          }
        }

        // Compute the new placement of the joint with respect to its parent joint in the new
        // kinematic tree.
        JointData joint_data = joint_input_model.createData();
        joint_input_model.calc(joint_data, reference_configuration);
        const SE3 liMi =
          parent_frame_placement * input_model.jointPlacements[joint_id] * joint_data.M();

        Frame frame = Frame(
          joint_name, reduced_parent_joint_index, reduced_previous_frame_index, liMi, FIXED_JOINT,
          input_model.inertias[joint_id]);

        FrameIndex frame_id = reduced_model.addFrame(frame);
        reduced_model.frames[frame_id].parentFrame =
          frame_id; // a bit weird, but this is a solution for missing parent frame

        current_index_to_lock++;
      }
      else
      {
        JointIndex reduced_joint_id;
        if (boost::get<JointModelMimicTpl<Scalar, Options, JointCollectionTpl>>(&joint_input_model))
        {
          auto mimic_joint =
            boost::get<JointModelMimicTpl<Scalar, Options, JointCollectionTpl>>(joint_input_model);

          JointIndex mimicked_id =
            reduced_model.getJointId(input_model.names[mimic_joint.jmodel().id()]);
          mimic_joint.setMimicIndexes(
            mimicked_id, reduced_model.idx_qs[mimicked_id], reduced_model.idx_vs[mimicked_id],
            reduced_model.idx_vExtendeds[mimicked_id]);

          reduced_joint_id = reduced_model.addJoint(
            reduced_parent_joint_index, mimic_joint,
            parent_frame_placement * input_model.jointPlacements[joint_id], joint_name,
            mimic_joint.jointVelocitySelector(input_model.lowerEffortLimit),
            mimic_joint.jointVelocitySelector(input_model.upperEffortLimit),
            mimic_joint.jointVelocitySelector(input_model.lowerVelocityLimit),
            mimic_joint.jointVelocitySelector(input_model.upperVelocityLimit),
            mimic_joint.jointConfigSelector(input_model.lowerPositionLimit),
            mimic_joint.jointConfigSelector(input_model.upperPositionLimit),
            mimic_joint.jointVelocitySelector(input_model.positionLimitMargin),
            mimic_joint.jointVelocitySelector(input_model.lowerDryFrictionLimit),
            mimic_joint.jointVelocitySelector(input_model.upperDryFrictionLimit),
            mimic_joint.jointVelocitySelector(input_model.damping),
            mimic_joint.jointVelocitySelector(input_model.lowerAccelerationLimit),
            mimic_joint.jointVelocitySelector(input_model.upperAccelerationLimit),
            mimic_joint.jointVelocitySelector(input_model.lowerJerkLimit),
            mimic_joint.jointVelocitySelector(input_model.upperJerkLimit));
        }
        else
        {
          // the joint should be added to the Model as it is
          reduced_joint_id = reduced_model.addJoint(
            reduced_parent_joint_index, joint_input_model,
            parent_frame_placement * input_model.jointPlacements[joint_id], joint_name,
            joint_input_model.jointVelocitySelector(input_model.lowerEffortLimit),
            joint_input_model.jointVelocitySelector(input_model.upperEffortLimit),
            joint_input_model.jointVelocitySelector(input_model.lowerVelocityLimit),
            joint_input_model.jointVelocitySelector(input_model.upperVelocityLimit),
            joint_input_model.jointConfigSelector(input_model.lowerPositionLimit),
            joint_input_model.jointConfigSelector(input_model.upperPositionLimit),
            joint_input_model.jointConfigSelector(input_model.positionLimitMargin),
            joint_input_model.jointVelocitySelector(input_model.lowerDryFrictionLimit),
            joint_input_model.jointVelocitySelector(input_model.upperDryFrictionLimit),
            joint_input_model.jointVelocitySelector(input_model.damping),
            joint_input_model.jointVelocitySelector(input_model.lowerAccelerationLimit),
            joint_input_model.jointVelocitySelector(input_model.upperAccelerationLimit),
            joint_input_model.jointVelocitySelector(input_model.lowerJerkLimit),
            joint_input_model.jointVelocitySelector(input_model.upperJerkLimit));
        }
        // Append inertia
        reduced_model.appendBodyToJoint(
          reduced_joint_id, input_model.inertias[joint_id], SE3::Identity());

        // Copy other kinematics and dynamics properties
        const typename Model::JointModel & jmodel_out = reduced_model.joints[reduced_joint_id];
        jmodel_out.jointVelocitySelector(reduced_model.rotorInertia) =
          joint_input_model.jointVelocitySelector(input_model.rotorInertia);
        jmodel_out.jointVelocitySelector(reduced_model.rotorGearRatio) =
          joint_input_model.jointVelocitySelector(input_model.rotorGearRatio);

        jmodel_out.jointVelocitySelector(reduced_model.armature) =
          joint_input_model.jointVelocitySelector(input_model.armature);
      }
    }

    // Retrieve and extend the reference configurations
    typedef typename Model::ConfigVectorMap ConfigVectorMap;
    for (typename ConfigVectorMap::const_iterator config_it =
           input_model.referenceConfigurations.begin();
         config_it != input_model.referenceConfigurations.end(); ++config_it)
    {
      const std::string & config_name = config_it->first;
      const typename Model::ConfigVectorType & input_config_vector = config_it->second;

      typename Model::ConfigVectorType reduced_config_vector(reduced_model.nq);
      for (JointIndex reduced_joint_id = 1; reduced_joint_id < reduced_model.joints.size();
           ++reduced_joint_id)
      {
        const JointIndex input_joint_id =
          input_model.getJointId(reduced_model.names[reduced_joint_id]);
        const JointModel & input_joint_model = input_model.joints[input_joint_id];
        const JointModel & reduced_joint_model = reduced_model.joints[reduced_joint_id];

        reduced_joint_model.jointConfigSelector(reduced_config_vector) =
          input_joint_model.jointConfigSelector(input_config_vector);
      }

      reduced_model.referenceConfigurations.insert(
        std::make_pair(config_name, reduced_config_vector));
    }

    // Add all the missing frames
    for (; frame_it != input_model.frames.end(); ++frame_it)
    {
      const Frame & input_frame = *frame_it;
      const std::string & support_joint_name = input_model.names[input_frame.parentJoint];

      std::vector<JointIndex>::const_iterator support_joint_it = std::find(
        list_of_joints_to_lock.begin(), list_of_joints_to_lock.end(), input_frame.parentJoint);

      if (support_joint_it != list_of_joints_to_lock.end())
      {
        if (
          input_frame.type == JOINT && reduced_model.existFrame(input_frame.name)
          && support_joint_name == input_frame.name)
          continue; // this means that the Joint is now fixed and has been replaced by a Frame. No
                    // need to add a new one.

        // The joint has been removed and replaced by a Frame
        const FrameIndex joint_frame_id = reduced_model.getFrameId(support_joint_name);
        const Frame & joint_frame = reduced_model.frames[joint_frame_id];
        Frame reduced_frame = input_frame;
        reduced_frame.placement = joint_frame.placement * input_frame.placement;
        reduced_frame.parentJoint = joint_frame.parentJoint;
        reduced_frame.parentFrame =
          reduced_model.getFrameId(input_model.frames[input_frame.parentFrame].name);
        reduced_model.addFrame(reduced_frame, false);
      }
      else
      {
        Frame reduced_frame = input_frame;
        reduced_frame.parentJoint =
          reduced_model.getJointId(input_model.names[input_frame.parentJoint]);
        reduced_frame.parentFrame =
          reduced_model.getFrameId(input_model.frames[input_frame.parentFrame].name);
        reduced_model.addFrame(reduced_frame, false);
      }
    }
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType>
  void buildReducedModel(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & input_model,
    const GeometryModel & input_geom_model,
    const std::vector<JointIndex> & list_of_joints_to_lock,
    const Eigen::MatrixBase<ConfigVectorType> & reference_configuration,
    ModelTpl<Scalar, Options, JointCollectionTpl> & reduced_model,
    GeometryModel & reduced_geom_model)
  {

    const std::vector<GeometryModel> temp_input_geoms(1, input_geom_model);
    std::vector<GeometryModel> temp_reduced_geom_models;

    buildReducedModel(
      input_model, temp_input_geoms, list_of_joints_to_lock, reference_configuration, reduced_model,
      temp_reduced_geom_models);
    reduced_geom_model = temp_reduced_geom_models.front();
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType>
  void buildReducedModel(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & input_model,
    const std::vector<GeometryModel> & list_of_geom_models,
    const std::vector<JointIndex> & list_of_joints_to_lock,
    const Eigen::MatrixBase<ConfigVectorType> & reference_configuration,
    ModelTpl<Scalar, Options, JointCollectionTpl> & reduced_model,
    std::vector<GeometryModel> & list_of_reduced_geom_models)
  {
    buildReducedModel(input_model, list_of_joints_to_lock, reference_configuration, reduced_model);

    // for all GeometryModels
    for (size_t gmi = 0; gmi < list_of_geom_models.size(); ++gmi)
    {
      const GeometryModel & input_geom_model = list_of_geom_models[gmi];
      GeometryModel reduced_geom_model;

      // Add all the geometries
      typedef GeometryModel::GeometryObject GeometryObject;
      typedef GeometryModel::GeometryObjectVector GeometryObjectVector;
      for (GeometryObjectVector::const_iterator it = input_geom_model.geometryObjects.begin();
           it != input_geom_model.geometryObjects.end(); ++it)
      {
        const GeometryModel::GeometryObject & geom = *it;

        const JointIndex joint_id_in_input_model = geom.parentJoint;
        _PINOCCHIO_CHECK_INPUT_ARGUMENT_2(
          (joint_id_in_input_model < (JointIndex)input_model.njoints),
          "Invalid joint parent index for the geometry with name " + geom.name);
        const std::string & parent_joint_name = input_model.names[joint_id_in_input_model];

        JointIndex reduced_joint_id = (JointIndex)-1;
        typedef typename GeometryObject::SE3 SE3;
        SE3 relative_placement = SE3::Identity();
        if (reduced_model.existJointName(parent_joint_name))
        {
          reduced_joint_id = reduced_model.getJointId(parent_joint_name);
        }
        else // The joint is now a frame
        {
          const FrameIndex reduced_frame_id = reduced_model.getFrameId(parent_joint_name);
          reduced_joint_id = reduced_model.frames[reduced_frame_id].parentJoint;
          relative_placement = SE3(reduced_model.frames[reduced_frame_id].placement);
        }

        GeometryObject reduced_geom(geom);
        reduced_geom.parentJoint = reduced_joint_id;
        reduced_geom.parentFrame =
          reduced_model.getBodyId(input_model.frames[geom.parentFrame].name);
        reduced_geom.placement = relative_placement * geom.placement;
        reduced_geom_model.addGeometryObject(reduced_geom);
      }

#ifdef PINOCCHIO_WITH_COLLISION
      // Add all the collision pairs - the index of the geometry objects should have not changed

      typedef GeometryModel::CollisionPairVector CollisionPairVector;
      for (CollisionPairVector::const_iterator it = input_geom_model.collisionPairs.begin();
           it != input_geom_model.collisionPairs.end(); ++it)
      {
        const CollisionPair & cp = *it;
        reduced_geom_model.addCollisionPair(cp);
      }
#endif

      list_of_reduced_geom_models.push_back(reduced_geom_model);
    }
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  void transformJointIntoMimic(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & input_model,
    const JointIndex & index_mimicked,
    const JointIndex & index_mimicking,
    const Scalar & scaling,
    const Scalar & offset,
    ModelTpl<Scalar, Options, JointCollectionTpl> & output_model)
  {
    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      index_mimicked <= (size_t)input_model.njoints,
      "index_mimicked is greater than the total of joints");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      index_mimicking <= (size_t)input_model.njoints,
      "index_mimicking is greater than the total of joints");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      index_mimicked < index_mimicking, "index_mimicking is greater than index_mimicked");

    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef typename Model::JointModel JointModel;
    typedef JointModelMimicTpl<Scalar, Options, JointCollectionTpl> JointModelMimic;

    output_model = input_model;

    output_model.joints[index_mimicking] = JointModelMimic(
      input_model.joints[index_mimicking], output_model.joints[index_mimicked], scaling, offset);

    int old_nq = input_model.joints[index_mimicking].nq();
    int old_nv = input_model.joints[index_mimicking].nv();
    output_model.nq = input_model.nq - old_nq;
    output_model.nv = input_model.nv - old_nv;
    int nq = output_model.nq;
    int nv = output_model.nv;

    // Resize limits
    output_model.lowerEffortLimit.resize(nv);
    output_model.upperEffortLimit.resize(nv);
    output_model.lowerVelocityLimit.resize(nv);
    output_model.upperVelocityLimit.resize(nv);
    output_model.lowerPositionLimit.resize(nq);
    output_model.upperPositionLimit.resize(nq);
    output_model.positionLimitMargin.resize(nq);
    output_model.armature.resize(nv);
    output_model.rotorInertia.resize(nv);
    output_model.rotorGearRatio.resize(nv);
    output_model.lowerDryFrictionLimit.resize(nv);
    output_model.upperDryFrictionLimit.resize(nv);
    output_model.damping.resize(nv);
    output_model.lowerAccelerationLimit.resize(nv);
    output_model.upperAccelerationLimit.resize(nv);
    output_model.lowerJerkLimit.resize(nv);
    output_model.upperJerkLimit.resize(nv);

    // Move indexes and limits
    for (JointIndex joint_id = 1; joint_id < (JointIndex)index_mimicking; ++joint_id)
    {
      const JointModel & jmodel_input = input_model.joints[joint_id];
      const JointModel & jmodel_output = output_model.joints[joint_id];

      jmodel_output.jointVelocitySelector(output_model.lowerEffortLimit) =
        jmodel_input.jointVelocitySelector(input_model.lowerEffortLimit);
      jmodel_output.jointVelocitySelector(output_model.upperEffortLimit) =
        jmodel_input.jointVelocitySelector(input_model.upperEffortLimit);
      jmodel_output.jointVelocitySelector(output_model.lowerVelocityLimit) =
        jmodel_input.jointVelocitySelector(input_model.lowerVelocityLimit);
      jmodel_output.jointVelocitySelector(output_model.upperVelocityLimit) =
        jmodel_input.jointVelocitySelector(input_model.upperVelocityLimit);

      jmodel_output.jointConfigSelector(output_model.lowerPositionLimit) =
        jmodel_input.jointConfigSelector(input_model.lowerPositionLimit);
      jmodel_output.jointConfigSelector(output_model.upperPositionLimit) =
        jmodel_input.jointConfigSelector(input_model.upperPositionLimit);
      jmodel_output.jointConfigSelector(output_model.positionLimitMargin) =
        jmodel_input.jointConfigSelector(input_model.positionLimitMargin);

      jmodel_output.jointVelocitySelector(output_model.lowerAccelerationLimit) =
        jmodel_input.jointVelocitySelector(input_model.lowerAccelerationLimit);
      jmodel_output.jointVelocitySelector(output_model.upperAccelerationLimit) =
        jmodel_input.jointVelocitySelector(input_model.upperAccelerationLimit);
      jmodel_output.jointVelocitySelector(output_model.lowerJerkLimit) =
        jmodel_input.jointVelocitySelector(input_model.lowerJerkLimit);
      jmodel_output.jointVelocitySelector(output_model.upperJerkLimit) =
        jmodel_input.jointVelocitySelector(input_model.upperJerkLimit);

      jmodel_output.jointVelocitySelector(output_model.armature) =
        jmodel_input.jointVelocitySelector(input_model.armature);
      jmodel_output.jointVelocitySelector(output_model.rotorInertia) =
        jmodel_input.jointVelocitySelector(input_model.rotorInertia);
      jmodel_output.jointVelocitySelector(output_model.rotorGearRatio) =
        jmodel_input.jointVelocitySelector(input_model.rotorGearRatio);
      jmodel_output.jointVelocitySelector(output_model.lowerDryFrictionLimit) =
        jmodel_input.jointVelocitySelector(input_model.lowerDryFrictionLimit);
      jmodel_output.jointVelocitySelector(output_model.upperDryFrictionLimit) =
        jmodel_input.jointVelocitySelector(input_model.upperDryFrictionLimit);
      jmodel_output.jointVelocitySelector(output_model.damping) =
        jmodel_input.jointVelocitySelector(input_model.damping);
    }

    // Move indexes and limits
    int idx_q = output_model.idx_qs[index_mimicking];
    int idx_v = output_model.idx_vs[index_mimicking];
    for (JointIndex joint_id = index_mimicking; joint_id < (JointIndex)input_model.njoints;
         ++joint_id)
    {
      const JointModel & jmodel_input = input_model.joints[joint_id];
      JointModel & jmodel_output = output_model.joints[joint_id];
      jmodel_output.setIndexes(jmodel_input.id(), idx_q, idx_v, jmodel_input.idx_vExtended());
      if (
        boost::get<JointModelMimicTpl<Scalar, Options, JointCollectionTpl>>(&jmodel_output)
        && joint_id != index_mimicking)
      {
        auto & jmimic =
          boost::get<JointModelMimicTpl<Scalar, Options, JointCollectionTpl>>(jmodel_output);
        const JointIndex mimicked_id = jmimic.jmodel().id();
        jmimic.setMimicIndexes(
          mimicked_id, output_model.idx_qs[mimicked_id], output_model.idx_vs[mimicked_id],
          jmodel_input.idx_vExtended());
      }
      output_model.idx_qs[joint_id] = jmodel_output.idx_q();
      output_model.nqs[joint_id] = jmodel_output.nq();
      output_model.idx_vs[joint_id] = jmodel_output.idx_v();
      output_model.nvs[joint_id] = jmodel_output.nv();

      idx_q += jmodel_output.nq();
      idx_v += jmodel_output.nv();
      if (joint_id != index_mimicking)
      {
        jmodel_output.jointVelocitySelector(output_model.lowerEffortLimit) =
          jmodel_input.jointVelocitySelector(input_model.lowerEffortLimit);
        jmodel_output.jointVelocitySelector(output_model.upperEffortLimit) =
          jmodel_input.jointVelocitySelector(input_model.upperEffortLimit);
        jmodel_output.jointVelocitySelector(output_model.lowerVelocityLimit) =
          jmodel_input.jointVelocitySelector(input_model.lowerVelocityLimit);
        jmodel_output.jointVelocitySelector(output_model.upperVelocityLimit) =
          jmodel_input.jointVelocitySelector(input_model.upperVelocityLimit);

        jmodel_output.jointConfigSelector(output_model.lowerPositionLimit) =
          jmodel_input.jointConfigSelector(input_model.lowerPositionLimit);
        jmodel_output.jointConfigSelector(output_model.upperPositionLimit) =
          jmodel_input.jointConfigSelector(input_model.upperPositionLimit);
        jmodel_output.jointConfigSelector(output_model.positionLimitMargin) =
          jmodel_input.jointConfigSelector(input_model.positionLimitMargin);

        jmodel_output.jointVelocitySelector(output_model.lowerAccelerationLimit) =
          jmodel_input.jointVelocitySelector(input_model.lowerAccelerationLimit);
        jmodel_output.jointVelocitySelector(output_model.upperAccelerationLimit) =
          jmodel_input.jointVelocitySelector(input_model.upperAccelerationLimit);
        jmodel_output.jointVelocitySelector(output_model.lowerJerkLimit) =
          jmodel_input.jointVelocitySelector(input_model.lowerJerkLimit);
        jmodel_output.jointVelocitySelector(output_model.upperJerkLimit) =
          jmodel_input.jointVelocitySelector(input_model.upperJerkLimit);

        jmodel_output.jointVelocitySelector(output_model.armature) =
          jmodel_input.jointVelocitySelector(input_model.armature);
        jmodel_output.jointVelocitySelector(output_model.rotorInertia) =
          jmodel_input.jointVelocitySelector(input_model.rotorInertia);
        jmodel_output.jointVelocitySelector(output_model.rotorGearRatio) =
          jmodel_input.jointVelocitySelector(input_model.rotorGearRatio);
        jmodel_output.jointVelocitySelector(output_model.lowerDryFrictionLimit) =
          jmodel_input.jointVelocitySelector(input_model.lowerDryFrictionLimit);
        jmodel_output.jointVelocitySelector(output_model.upperDryFrictionLimit) =
          jmodel_input.jointVelocitySelector(input_model.upperDryFrictionLimit);
        jmodel_output.jointVelocitySelector(output_model.damping) =
          jmodel_input.jointVelocitySelector(input_model.damping);
      }
    }

    // Modify Model::mimic_joint_supports
    const auto & subtree = input_model.subtrees[index_mimicking];
    for (size_t i = 0; i < subtree.size(); ++i)
    {
      auto & i_support = output_model.mimic_joint_supports[subtree[i]];
      details::insertSort(index_mimicking, i_support);
    }

    details::insertSort(index_mimicking, output_model.mimicking_joints);
    details::insertSort(index_mimicked, output_model.mimicked_joints);

    // Rebuild sparsity patterns from the updated model structure (supports, nvs, idx_vs).
    // The patterns copied from input_model via operator= are stale because nv and joint
    // velocity indexes have changed after the mimic transformation.
    typedef typename Model::BooleanVector BooleanVector;
    typedef typename Model::EigenIndexVector EigenIndexVector;
    output_model.sparsity_pattern_vector.clear();
    output_model.span_indexes_vector.clear();
    output_model.sparsity_pattern_vector.push_back(BooleanVector::Zero(nv));
    output_model.span_indexes_vector.push_back(EigenIndexVector());
    for (JointIndex jid = 1; jid < (JointIndex)output_model.njoints; ++jid)
    {
      EigenIndexVector extended_support;
      extended_support.reserve(size_t(nv));
      const auto & jsupport = output_model.supports[jid];
      for (size_t j = 1; j < jsupport.size() - 1; ++j)
      {
        const JointIndex jsupport_id = jsupport[j];
        const int jsupport_nv = output_model.nvs[jsupport_id];
        const int jsupport_idx_v = output_model.idx_vs[jsupport_id];
        for (int k = 0; k < jsupport_nv; ++k)
          extended_support.push_back(jsupport_idx_v + k);
      }
      const int jnv = output_model.nvs[jid];
      const int jidx_v = output_model.idx_vs[jid];
      for (int k = 0; k < jnv; ++k)
        extended_support.push_back(jidx_v + k);

      BooleanVector sparsity_pattern = BooleanVector::Zero(nv);
      for (const auto col_id : extended_support)
        sparsity_pattern[col_id] = true;

      output_model.sparsity_pattern_vector.push_back(std::move(sparsity_pattern));
      output_model.span_indexes_vector.push_back(std::move(extended_support));
    }
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  void buildMimicModel(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & input_model,
    const std::vector<JointIndex> & index_mimicked,
    const std::vector<JointIndex> & index_mimicking,
    const std::vector<Scalar> & scaling,
    const std::vector<Scalar> & offset,
    ModelTpl<Scalar, Options, JointCollectionTpl> & output_model)
  {
    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;

    Model model_temp;
    model_temp = input_model;
    for (size_t i = 0; i < index_mimicked.size(); i++)
    {
      transformJointIntoMimic(
        model_temp, index_mimicked[i], index_mimicking[i], scaling[i], offset[i], output_model);
      model_temp = output_model;
    }
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  JointIndex findCommonAncestor(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    JointIndex joint1_id,
    JointIndex joint2_id,
    size_t & index_ancestor_in_support1,
    size_t & index_ancestor_in_support2)
  {
    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      joint1_id < (JointIndex)model.njoints, "joint1_id is not valid.");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      joint2_id < (JointIndex)model.njoints, "joint2_id is not valid.");

    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef typename Model::IndexVector IndexVector;

    if (joint1_id == 0 || joint2_id == 0)
    {
      index_ancestor_in_support1 = index_ancestor_in_support2 = 0;
      return 0;
    }

    const IndexVector & support1 = model.supports[joint1_id];
    const IndexVector & support2 = model.supports[joint2_id];

    index_ancestor_in_support1 = support1.size() - 1;
    index_ancestor_in_support2 = support2.size() - 1;
    while (joint1_id != joint2_id)
    {
      if (joint1_id > joint2_id)
        joint1_id = support1[--index_ancestor_in_support1];
      else
        joint2_id = support2[--index_ancestor_in_support2];
    }

    return joint1_id;
  }

} // namespace pinocchio

#ifdef PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION

  #ifndef PINOCCHIO_SKIP_ALGORITHM_MODEL

namespace pinocchio
{

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
  appendModel<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const Model &,
    const Model &,
    const FrameIndex,
    const SE3Tpl<context::Scalar, context::Options> &,
    Model &);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI Model
  appendModel<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const Model &,
    const Model &,
    const FrameIndex,
    const SE3Tpl<context::Scalar, context::Options> &);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
  appendModel<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const Model &,
    const Model &,
    const GeometryModel &,
    const GeometryModel &,
    const FrameIndex,
    const SE3Tpl<context::Scalar, context::Options> &,
    Model &,
    GeometryModel &);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void buildReducedModel<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    context::VectorXs>(
    const Model &,
    const std::vector<JointIndex>,
    const Eigen::MatrixBase<context::VectorXs> &,
    Model &);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI Model buildReducedModel<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    context::VectorXs>(
    const Model &, const std::vector<JointIndex> &, const Eigen::MatrixBase<context::VectorXs> &);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void buildReducedModel<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    context::VectorXs>(
    const Model &,
    const GeometryModel &,
    const std::vector<JointIndex> &,
    const Eigen::MatrixBase<context::VectorXs> &,
    Model &,
    GeometryModel &);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void buildReducedModel<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    context::VectorXs>(
    const Model &,
    const std::vector<GeometryModel> &,
    const std::vector<JointIndex> &,
    const Eigen::MatrixBase<context::VectorXs> &,
    Model &,
    std::vector<GeometryModel> &);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
  transformJointIntoMimic<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const Model &,
    const JointIndex &,
    const JointIndex &,
    const context::Scalar &,
    const context::Scalar &,
    Model &);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
  buildMimicModel<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const Model &,
    const std::vector<JointIndex> &,
    const std::vector<JointIndex> &,
    const std::vector<context::Scalar> &,
    const std::vector<context::Scalar> &,
    Model &);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI JointIndex
  findCommonAncestor<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const Model &, JointIndex, JointIndex, size_t &, size_t &);

} // namespace pinocchio

  #endif // PINOCCHIO_SKIP_ALGORITHM_MODEL
#endif   // ifdef PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
