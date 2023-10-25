//
// Copyright (c) 2019-2022 CNRS INRIA
//

#ifndef __pinocchio_algorithm_model_hxx__
#define __pinocchio_algorithm_model_hxx__

#include <algorithm>

namespace pinocchio
{
  namespace details
  {
    
    // Retrieve the joint id in model_out, given the info of model_in.
    // If the user change all the joint names, the universe name won't correspond to the first joint in the tree when searching by name.
    // We thus need to retrieve it with other means, e.g. checking the index of the joints.
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    JointIndex getJointId(const ModelTpl<Scalar,Options,JointCollectionTpl> & model_in,
                          const ModelTpl<Scalar,Options,JointCollectionTpl> & model_out,
                          const std::string & joint_name_in_model_in)
    {
      const JointIndex joint_id = model_in.getJointId(joint_name_in_model_in);
      assert(joint_id < model_in.joints.size());
      if(joint_id == 0 && model_in.parents[0] == 0) // This is the universe, maybe renamed.
        return model_out.getJointId(model_out.names[0]);
      else
        return model_out.getJointId(joint_name_in_model_in);
    }
  
    // Retrieve the frame id in model_out, given the info of model_in.
    // If the user change all the frame names, the universe name won't correspond to the first frame in the tree when searching by name.
    // We thus need to retrieve it with other means, e.g. checking the fields of the frames.
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    FrameIndex getFrameId(const ModelTpl<Scalar,Options,JointCollectionTpl> & model_in,
                          const ModelTpl<Scalar,Options,JointCollectionTpl> & model_out,
                          const std::string & frame_name_in_model_in,
                          const FrameType & type)
    {
      const FrameIndex frame_id = model_in.getFrameId(frame_name_in_model_in);
      assert(frame_id < model_in.frames.size());
      if(frame_id == 0 && model_in.frames[0].previousFrame == 0 && model_in.frames[0].parent == 0) // This is the universe, maybe renamed.
        return model_out.getFrameId(model_out.frames[0].name,type);
      else
        return model_out.getFrameId(frame_name_in_model_in,type);
    }
  
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    void appendUniverseToModel(const ModelTpl<Scalar,Options,JointCollectionTpl> & modelAB,
                               const GeometryModel & geomModelAB,
                               FrameIndex parentFrame,
                               const SE3Tpl<Scalar, Options> & pfMAB,
                               ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                               GeometryModel & geomModel)
    {
      typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
      typedef typename Model::Frame Frame;
      
      PINOCCHIO_THROW(parentFrame < model.frames.size(),
                      std::invalid_argument,
                      "parentFrame is greater than the size of the frames vector.");

      const Frame & pframe = model.frames[parentFrame];
      JointIndex jid = pframe.parent;
      assert(jid < model.joints.size());

      // If inertia is not NaN, add it.
      if (modelAB.inertias[0] == modelAB.inertias[0])
        model.appendBodyToJoint (jid, modelAB.inertias[0], pframe.placement * pfMAB);

      // Add all frames whose parent is this joint.
      for (FrameIndex fid = 1; fid < modelAB.frames.size(); ++fid)
      {
        Frame frame = modelAB.frames[fid];
        if (frame.parent == 0)
        {
          PINOCCHIO_CHECK_INPUT_ARGUMENT(!model.existFrame(frame.name, frame.type),
                                         "The two models have conflicting frame names.");

          frame.parent = jid;
          if (frame.previousFrame != 0)
          {
            frame.previousFrame = getFrameId(modelAB,model,modelAB.frames[frame.previousFrame].name,
                                             modelAB.frames[frame.previousFrame].type);
          }
          else
          {
            frame.previousFrame = parentFrame;
          }
          
          // Modify frame placement
          frame.placement = pframe.placement * pfMAB * frame.placement;
          model.addFrame (frame);
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
            go.parentFrame = getFrameId(modelAB,model,modelAB.frames[go.parentFrame].name,
                                        modelAB.frames[go.parentFrame].type);
          }
          else
          {
            go.parentFrame = parentFrame;
          }
          go.placement = pframe.placement * pfMAB * go.placement;
          geomModel.addGeometryObject (go);
        }
      }
    }

    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    struct AppendJointOfModelAlgoTpl
    : public fusion::JointUnaryVisitorBase< AppendJointOfModelAlgoTpl<Scalar,Options,JointCollectionTpl> >
    {
    
      typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
      typedef typename Model::Frame Frame;
      
      typedef boost::fusion::vector<
      const Model &,
      const GeometryModel&,
      JointIndex,
      const typename Model::SE3 &,
      Model &,
      GeometryModel& > ArgsType;
      
      template <typename JointModel>
      static void algo (const JointModelBase<JointModel> & jmodel_in,
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
          parent_id = getJointId(modelAB,model,modelAB.names[modelAB.parents[joint_id_in]]);
        
        PINOCCHIO_CHECK_INPUT_ARGUMENT(!model.existJointName(modelAB.names[joint_id_in]),
                                       "The two models have conflicting joint names.");
        
        JointIndex joint_id_out = model.addJoint(parent_id,
                                                 jmodel_in,
                                                 pMi * modelAB.jointPlacements[joint_id_in],
                                                 modelAB.names[joint_id_in],
                                                 jmodel_in.jointVelocitySelector(modelAB.effortLimit),
                                                 jmodel_in.jointVelocitySelector(modelAB.velocityLimit),
                                                 jmodel_in.jointConfigSelector(modelAB.lowerPositionLimit),
                                                 jmodel_in.jointConfigSelector(modelAB.upperPositionLimit),
                                                 jmodel_in.jointVelocitySelector(modelAB.friction),
                                                 jmodel_in.jointVelocitySelector(modelAB.damping));
        assert(joint_id_out < model.joints.size());
        
        model.appendBodyToJoint(joint_id_out, modelAB.inertias[joint_id_in]);
        
        const typename Model::JointModel & jmodel_out = model.joints[joint_id_out];
        jmodel_out.jointVelocitySelector(model.rotorInertia) = jmodel_in.jointVelocitySelector(modelAB.rotorInertia);
        jmodel_out.jointVelocitySelector(model.rotorGearRatio) = jmodel_in.jointVelocitySelector(modelAB.rotorGearRatio);
        
        // Add all frames whose parent is this joint.
        for (FrameIndex fid = 1; fid < modelAB.frames.size(); ++fid)
        {
          Frame frame = modelAB.frames[fid];
          if (frame.parent == jmodel_in.id())
          {
            PINOCCHIO_CHECK_INPUT_ARGUMENT(!model.existFrame(frame.name, frame.type),
                                           "The two models have conflicting frame names.");
            
            frame.parent = joint_id_out;
            assert (frame.previousFrame > 0 || frame.type == JOINT);
            if (frame.previousFrame != 0)
            {
              frame.previousFrame = getFrameId(modelAB,model,modelAB.frames[frame.previousFrame].name,modelAB.frames[frame.previousFrame].type);
            }
            
            model.addFrame(frame);
          }
        }
        // Add all geometries whose parent is this joint.
        for(GeomIndex gid = 0; gid < geomModelAB.geometryObjects.size(); ++gid)
        {
          GeometryObject go = geomModelAB.geometryObjects[gid];
          if(go.parentJoint == joint_id_in)
          {
            go.parentJoint = joint_id_out;
            assert(go.parentFrame > 0);
            if(go.parentFrame != 0 && go.parentFrame < modelAB.frames.size())
            {
              go.parentFrame = getFrameId(modelAB,model,modelAB.frames[go.parentFrame].name,modelAB.frames[go.parentFrame].type);
            }
            geomModel.addGeometryObject(go);
          }
        }
      }
    };
  
  } // namespace details

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  void
  appendModel(const ModelTpl<Scalar,Options,JointCollectionTpl> & modelA,
              const ModelTpl<Scalar,Options,JointCollectionTpl> & modelB,
              const FrameIndex frameInModelA,
              const SE3Tpl<Scalar, Options> & aMb,
              ModelTpl<Scalar,Options,JointCollectionTpl> & model)
  {
    GeometryModel geomModelA, geomModelB, geomModel;

    appendModel(modelA, modelB, geomModelA, geomModelB, frameInModelA, aMb,
         model, geomModel);
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  void
  appendModel(const ModelTpl<Scalar,Options,JointCollectionTpl> & modelA,
              const ModelTpl<Scalar,Options,JointCollectionTpl> & modelB,
              const GeometryModel& geomModelA,
              const GeometryModel& geomModelB,
              const FrameIndex frameInModelA,
              const SE3Tpl<Scalar, Options>& aMb,
              ModelTpl<Scalar,Options,JointCollectionTpl>& model,
              GeometryModel& geomModel)
  {
    typedef details::AppendJointOfModelAlgoTpl<Scalar, Options, JointCollectionTpl> AppendJointOfModelAlgo;
    typedef typename AppendJointOfModelAlgo::ArgsType ArgsType;
    
    PINOCCHIO_CHECK_INPUT_ARGUMENT((bool)(frameInModelA < (FrameIndex) modelA.nframes),
                                   "frameInModelA is an invalid Frame index, greater than the number of frames contained in modelA.");
    
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef typename Model::SE3 SE3;
    typedef typename Model::Frame Frame;

    const Frame & frame = modelA.frames[frameInModelA];
    static const SE3 id = SE3::Identity();

    int njoints = modelA.njoints + modelB.njoints - 1;
    model.names                 .reserve ((size_t)njoints);
    model.joints                .reserve ((size_t)njoints);
    model.jointPlacements       .reserve ((size_t)njoints);
    model.parents               .reserve ((size_t)njoints);
    model.inertias              .reserve ((size_t)njoints);
    int nframes = modelA.nframes + modelB.nframes - 1;
    model.frames                .reserve ((size_t)nframes);

    geomModel.geometryObjects.reserve (geomModelA.ngeoms + geomModelB.ngeoms);

    // Copy modelA joints until frame.parentJoint
    details::appendUniverseToModel (modelA, geomModelA, 0, id, model, geomModel);
    for (JointIndex jid = 1; jid <= frame.parent; ++jid)
    {
      ArgsType args (modelA, geomModelA, 0, id, model, geomModel);
      AppendJointOfModelAlgo::run (modelA.joints[jid], args);
    }

    // Copy modelB joints
    details::appendUniverseToModel (modelB, geomModelB,
                                    details::getFrameId(modelA,model,frame.name,frame.type), aMb, model, geomModel);
    for (JointIndex jid = 1; jid < modelB.joints.size(); ++jid)
    {
      SE3 pMi = (jid == 1 ? frame.placement * aMb : id);
      ArgsType args (modelB, geomModelB, frame.parent, pMi, model, geomModel);
      AppendJointOfModelAlgo::run (modelB.joints[jid], args);
    }

    // Copy remaining joints of modelA
    for (JointIndex jid = frame.parent+1; jid < modelA.joints.size(); ++jid)
    {
      ArgsType args (modelA, geomModelA, 0, id, model, geomModel);
      AppendJointOfModelAlgo::run (modelA.joints[jid], args);
    }

#ifdef PINOCCHIO_WITH_HPP_FCL
    // Add collision pairs of geomModelA and geomModelB
    geomModel.collisionPairs.reserve(geomModelA.collisionPairs.size()
       + geomModelB.collisionPairs.size()
       + geomModelA.geometryObjects.size() * geomModelB.geometryObjects.size());
    
    for (std::size_t icp = 0; icp < geomModelA.collisionPairs.size(); ++icp)
    {
      const CollisionPair& cp = geomModelA.collisionPairs[icp];
      GeomIndex go1 = geomModel.getGeometryId(geomModelA.geometryObjects[cp.first ].name);
      GeomIndex go2 = geomModel.getGeometryId(geomModelA.geometryObjects[cp.second].name);
      geomModel.addCollisionPair (CollisionPair (go1, go2));
    }
    
    for (std::size_t icp = 0; icp < geomModelB.collisionPairs.size(); ++icp)
    {
      const CollisionPair & cp = geomModelB.collisionPairs[icp];
      GeomIndex go1 = geomModel.getGeometryId(geomModelB.geometryObjects[cp.first ].name);
      GeomIndex go2 = geomModel.getGeometryId(geomModelB.geometryObjects[cp.second].name);
      geomModel.addCollisionPair (CollisionPair (go1, go2));
    }

    // add the collision pairs between geomModelA and geomModelB.
    for (Index i = 0; i < geomModelA.geometryObjects.size(); ++i)
    {
      GeomIndex go1 = geomModel.getGeometryId(geomModelA.geometryObjects[i].name);
      for (Index j = 0; j < geomModelB.geometryObjects.size(); ++j)
      {
        GeomIndex go2 = geomModel.getGeometryId(geomModelB.geometryObjects[j].name);
        if (   geomModel.geometryObjects[go1].parentJoint
            != geomModel.geometryObjects[go2].parentJoint)
          geomModel.addCollisionPair(CollisionPair(go1, go2));
      }
    }
#endif
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
  void
  buildReducedModel(const ModelTpl<Scalar,Options,JointCollectionTpl> & input_model,
                    std::vector<JointIndex> list_of_joints_to_lock,
                    const Eigen::MatrixBase<ConfigVectorType> & reference_configuration,
                    ModelTpl<Scalar,Options,JointCollectionTpl> & reduced_model)
  {
    PINOCCHIO_CHECK_ARGUMENT_SIZE(reference_configuration.size(), input_model.nq,
                                   "The configuration vector is not of right size");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(list_of_joints_to_lock.size() <= (size_t)input_model.njoints,
                                   "The number of joints to lock is greater than the total of joints in the reduced_model");
    
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef typename Model::JointModel JointModel;
    typedef typename Model::JointData JointData;
    typedef typename Model::Frame Frame;
    typedef typename Model::SE3 SE3;

    // Sort indexes
    std::sort(list_of_joints_to_lock.begin(),list_of_joints_to_lock.end());
    
    // Check that they are not two identical elements
    for(size_t id = 1; id < list_of_joints_to_lock.size(); ++id)
    {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(list_of_joints_to_lock[id-1] < list_of_joints_to_lock[id],
                                     "The input list_of_joints_to_lock contains two identical indexes.");
    }
    
    // Reserve memory
    const Eigen::DenseIndex njoints = input_model.njoints - (Eigen::DenseIndex)list_of_joints_to_lock.size();
    reduced_model.names                 .reserve((size_t)njoints);
    reduced_model.joints                .reserve((size_t)njoints);
    reduced_model.jointPlacements       .reserve((size_t)njoints);
    reduced_model.parents               .reserve((size_t)njoints);
    
    reduced_model.names[0] = input_model.names[0];
    reduced_model.joints[0] = input_model.joints[0];
    reduced_model.jointPlacements[0] = input_model.jointPlacements[0];
    reduced_model.parents[0] = input_model.parents[0];
    reduced_model.inertias[0] = input_model.inertias[0];
    
    reduced_model.name = input_model.name;
    reduced_model.gravity = input_model.gravity;
    
    size_t current_index_to_lock = 0;
    
    for(JointIndex joint_id = 1; joint_id < (JointIndex)input_model.njoints; ++joint_id)
    {
      const JointIndex joint_id_to_lock = (current_index_to_lock < list_of_joints_to_lock.size()) ? list_of_joints_to_lock[current_index_to_lock] : 0;
      
      const JointIndex input_parent_joint_index = input_model.parents[joint_id];
      const std::string & joint_name = input_model.names[joint_id];
      const JointModel & joint_input_model = input_model.joints[joint_id];
      
      // retrieve the closest joint parent in the new kinematic tree
      const std::string & parent_joint_name = input_model.names[input_parent_joint_index];
      const bool exist_parent_joint = reduced_model.existJointName(parent_joint_name);
      
      const JointIndex reduced_parent_joint_index
      = exist_parent_joint
      ? reduced_model.getJointId(parent_joint_name)
      : reduced_model.frames[reduced_model.getFrameId(parent_joint_name)].parent;
      
      const SE3 parent_frame_placement
      = exist_parent_joint
      ? SE3::Identity()
      : reduced_model.frames[reduced_model.getFrameId(parent_joint_name)].placement;
      
      const FrameIndex reduced_previous_frame_index
      = exist_parent_joint
      ? 0
      : reduced_model.getFrameId(parent_joint_name);
      
      if(joint_id == joint_id_to_lock)
      {
        // the joint should not be added to the Model but aggragated to its parent joint

        // Compute the new placement of the joint with respect to its parent joint in the new kinematic tree.
        JointData joint_data = joint_input_model.createData();
        joint_input_model.calc(joint_data,reference_configuration);
        const SE3 liMi = parent_frame_placement * input_model.jointPlacements[joint_id] * joint_data.M();
        
        Frame frame = Frame(joint_name,
                            reduced_parent_joint_index, reduced_previous_frame_index,
                            liMi,
                            FIXED_JOINT,
                            input_model.inertias[joint_id]);
        
        FrameIndex frame_id = reduced_model.addFrame(frame);
        reduced_model.frames[frame_id].previousFrame = frame_id; // a bit weird, but this is a solution for missing parent frame
        
        current_index_to_lock++;
      }
      else
      {
        // the joint should be added to the Model as it is
        JointIndex reduced_joint_id = reduced_model.addJoint(reduced_parent_joint_index,
                                                             joint_input_model,
                                                             parent_frame_placement * input_model.jointPlacements[joint_id],
                                                             joint_name,
                                                             joint_input_model.jointVelocitySelector(input_model.effortLimit),
                                                             joint_input_model.jointVelocitySelector(input_model.velocityLimit),
                                                             joint_input_model.jointConfigSelector(input_model.lowerPositionLimit),
                                                             joint_input_model.jointConfigSelector(input_model.upperPositionLimit),
                                                             joint_input_model.jointVelocitySelector(input_model.friction),
                                                             joint_input_model.jointVelocitySelector(input_model.damping));
        // Append inertia
        reduced_model.appendBodyToJoint(reduced_joint_id,
                                        input_model.inertias[joint_id],
                                        SE3::Identity());
        
        // Copy other kinematics and dynamics properties
        const typename Model::JointModel & jmodel_out = reduced_model.joints[reduced_joint_id];
        jmodel_out.jointVelocitySelector(reduced_model.rotorInertia)
        = joint_input_model.jointVelocitySelector(input_model.rotorInertia);
        jmodel_out.jointVelocitySelector(reduced_model.rotorGearRatio)
        = joint_input_model.jointVelocitySelector(input_model.rotorGearRatio);
      }
    }
    
    // Retrieve and extend the reference configurations
    typedef typename Model::ConfigVectorMap ConfigVectorMap;
    for(typename ConfigVectorMap::const_iterator config_it = input_model.referenceConfigurations.begin();
        config_it != input_model.referenceConfigurations.end(); ++config_it)
    {
      const std::string & config_name = config_it->first;
      const typename Model::ConfigVectorType & input_config_vector = config_it->second;
      
      typename Model::ConfigVectorType reduced_config_vector(reduced_model.nq);
      for(JointIndex reduced_joint_id = 1;
          reduced_joint_id < reduced_model.joints.size();
          ++reduced_joint_id)
      {
        const JointIndex input_joint_id = input_model.getJointId(reduced_model.names[reduced_joint_id]);
        const JointModel & input_joint_model = input_model.joints[input_joint_id];
        const JointModel & reduced_joint_model = reduced_model.joints[reduced_joint_id];
        
        reduced_joint_model.jointConfigSelector(reduced_config_vector) = input_joint_model.jointConfigSelector(input_config_vector);
      }
      
      reduced_model.referenceConfigurations.insert(std::make_pair(config_name, reduced_config_vector));
    }
    
    // Add all frames
    typename Model::FrameVector::const_iterator frame_it = input_model.frames.begin();
    for(++frame_it;frame_it != input_model.frames.end(); ++frame_it)
    {
      const Frame & input_frame = *frame_it;
      const std::string & support_joint_name = input_model.names[input_frame.parent];
      
      std::vector<JointIndex>::const_iterator support_joint_it = std::find(list_of_joints_to_lock.begin(),
                                                                           list_of_joints_to_lock.end(),
                                                                           input_frame.parent);
      
      if(support_joint_it != list_of_joints_to_lock.end())
      {
        if(   input_frame.type == JOINT
           && reduced_model.existFrame(input_frame.name)
           && support_joint_name == input_frame.name)
          continue; // this means that the Joint is now fixed and has been replaced by a Frame. No need to add a new one.
        
        // The joint has been removed and replaced by a Frame
        const FrameIndex joint_frame_id = reduced_model.getFrameId(support_joint_name);
        const Frame & joint_frame = reduced_model.frames[joint_frame_id];
        Frame reduced_frame = input_frame;
        reduced_frame.placement = joint_frame.placement * input_frame.placement;
        reduced_frame.parent = joint_frame.parent;
        reduced_frame.previousFrame = reduced_model.getFrameId(input_model.frames[input_frame.previousFrame].name);
        reduced_model.addFrame(reduced_frame, false);
      }
      else
      {
        Frame reduced_frame = input_frame;
        reduced_frame.parent = reduced_model.getJointId(input_model.names[input_frame.parent]);
        reduced_frame.previousFrame = reduced_model.getFrameId(input_model.frames[input_frame.previousFrame].name);
        reduced_model.addFrame(reduced_frame, false);
      }
    }
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
  void
  buildReducedModel(const ModelTpl<Scalar,Options,JointCollectionTpl> & input_model,
                    const GeometryModel & input_geom_model,
                    const std::vector<JointIndex> & list_of_joints_to_lock,
                    const Eigen::MatrixBase<ConfigVectorType> & reference_configuration,
                    ModelTpl<Scalar,Options,JointCollectionTpl> & reduced_model,
                    GeometryModel & reduced_geom_model)
  {

    const std::vector<GeometryModel> temp_input_geoms(1,input_geom_model);
    std::vector<GeometryModel> temp_reduced_geom_models;

    buildReducedModel(input_model, temp_input_geoms, list_of_joints_to_lock,
                      reference_configuration, reduced_model,
                      temp_reduced_geom_models);
    reduced_geom_model = temp_reduced_geom_models.front();
  }

  template <typename Scalar, int Options,
            template <typename, int> class JointCollectionTpl,
            typename GeometryModelAllocator,
            typename ConfigVectorType>
  void buildReducedModel(
      const ModelTpl<Scalar, Options, JointCollectionTpl> &input_model,
      const std::vector<GeometryModel,GeometryModelAllocator> &list_of_geom_models,
      const std::vector<JointIndex> &list_of_joints_to_lock,
      const Eigen::MatrixBase<ConfigVectorType> &reference_configuration,
      ModelTpl<Scalar, Options, JointCollectionTpl> &reduced_model,
      std::vector<GeometryModel,GeometryModelAllocator> &list_of_reduced_geom_models) {

    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    buildReducedModel(input_model, list_of_joints_to_lock, reference_configuration, reduced_model);

    // for all GeometryModels
    for (size_t gmi = 0; gmi < list_of_geom_models.size(); ++gmi) {
      const GeometryModel &input_geom_model = list_of_geom_models[gmi];
      GeometryModel reduced_geom_model;

      // Add all the geometries
      typedef GeometryModel::GeometryObject GeometryObject;
      typedef GeometryModel::GeometryObjectVector GeometryObjectVector;
      for(GeometryObjectVector::const_iterator it = input_geom_model.geometryObjects.begin();
          it != input_geom_model.geometryObjects.end(); ++it)
      {
        const GeometryModel::GeometryObject & geom = *it;

        const JointIndex joint_id_in_input_model = geom.parentJoint;
        _PINOCCHIO_CHECK_INPUT_ARGUMENT_2((joint_id_in_input_model < (JointIndex)input_model.njoints),
                                          "Invalid joint parent index for the geometry with name " + geom.name);
        const std::string & parent_joint_name = input_model.names[joint_id_in_input_model];

        JointIndex reduced_joint_id = (JointIndex)-1;
        typedef typename Model::SE3 SE3;
        SE3 relative_placement = SE3::Identity();
        if(reduced_model.existJointName(parent_joint_name))
        {
          reduced_joint_id = reduced_model.getJointId(parent_joint_name);
        }
        else // The joint is now a frame
        {
          const FrameIndex reduced_frame_id = reduced_model.getFrameId(parent_joint_name);
          reduced_joint_id = reduced_model.frames[reduced_frame_id].parent;
          relative_placement = reduced_model.frames[reduced_frame_id].placement;
        }

        GeometryObject reduced_geom(geom);
        reduced_geom.parentJoint = reduced_joint_id;
        reduced_geom.parentFrame = reduced_model.getBodyId(
            input_model.frames[geom.parentFrame].name);
        reduced_geom.placement = relative_placement * geom.placement;
        reduced_geom_model.addGeometryObject(reduced_geom);
      }

  #ifdef PINOCCHIO_WITH_HPP_FCL
      // Add all the collision pairs - the index of the geometry objects should have not changed

      typedef GeometryModel::CollisionPairVector CollisionPairVector;
      for(CollisionPairVector::const_iterator it = input_geom_model.collisionPairs.begin();
          it != input_geom_model.collisionPairs.end(); ++it)
      {
        const CollisionPair & cp = *it;
        reduced_geom_model.addCollisionPair(cp);
      }
  #endif

    list_of_reduced_geom_models.push_back(reduced_geom_model);
    }
  }

} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_model_hxx__
