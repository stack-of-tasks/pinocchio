//
// Copyright (c) 2019 CNRS INRIA
//

#ifndef __pinocchio_algorithm_model_hxx__
#define __pinocchio_algorithm_model_hxx__

#include <algorithm>

namespace pinocchio
{
  namespace details
  {
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
            frame.previousFrame = model.getFrameId (
                modelAB.frames[frame.previousFrame].name,
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
            go.parentFrame = model.getFrameId (
                modelAB.frames[go.parentFrame].name,
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
      static void algo (const JointModelBase<JointModel> & jmodel,
                        const Model & modelAB,
                        const GeometryModel & geomModelAB,
                        JointIndex parentId,
                        const typename Model::SE3 & pMi,
                        Model & model,
                        GeometryModel & geomModel)
      {
        // If old parent is universe, use what's provided in the input.
        // otherwise, get the parent from modelAB.
        if (modelAB.parents[jmodel.id()] > 0)
          parentId = model.getJointId(modelAB.names[modelAB.parents[jmodel.id()]]);
        
        PINOCCHIO_CHECK_INPUT_ARGUMENT(!model.existJointName(modelAB.names[jmodel.id()]),
                                       "The two models have conflicting joint names.");
        
        JointIndex jid = model.addJoint (
                                         parentId,
                                         jmodel,
                                         pMi * modelAB.jointPlacements[jmodel.id()],
                                         modelAB.names[jmodel.id()],
                                         jmodel.jointVelocitySelector(modelAB.effortLimit),
                                         jmodel.jointVelocitySelector(modelAB.velocityLimit),
                                         jmodel.jointConfigSelector(modelAB.lowerPositionLimit),
                                         jmodel.jointConfigSelector(modelAB.upperPositionLimit));
        assert (jid < model.joints.size());
        
        model.appendBodyToJoint (jid, modelAB.inertias[jmodel.id()]);
        
        // Add all frames whose parent is this joint.
        for (FrameIndex fid = 1; fid < modelAB.frames.size(); ++fid) {
          Frame frame = modelAB.frames[fid];
          if (frame.parent == jmodel.id())
          {
            PINOCCHIO_CHECK_INPUT_ARGUMENT(!model.existFrame(frame.name, frame.type),
                                           "The two models have conflicting frame names.");
            
            frame.parent = jid;
            assert (frame.previousFrame > 0 || frame.type == JOINT);
            if (frame.previousFrame != 0)
            {
              frame.previousFrame = model.getFrameId(modelAB.frames[frame.previousFrame].name,
                                                     modelAB.frames[frame.previousFrame].type);
            }
            
            model.addFrame(frame);
          }
        }
        // Add all geometries whose parent is this joint.
        for (GeomIndex gid = 0; gid < geomModelAB.geometryObjects.size(); ++gid)
        {
          GeometryObject go = geomModelAB.geometryObjects[gid];
          if (go.parentJoint == jmodel.id())
          {
            go.parentJoint = jid;
            assert (go.parentFrame > 0);
            if (go.parentFrame != 0)
            {
              go.parentFrame = model.getFrameId(modelAB.frames[go.parentFrame].name,
                                                modelAB.frames[go.parentFrame].type);
            }
            geomModel.addGeometryObject (go);
          }
        }
      }
    };
  }

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
    
    PINOCCHIO_CHECK_INPUT_ARGUMENT(frameInModelA < (FrameIndex) modelA.nframes,
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
        model.getFrameId (frame.name, frame.type), aMb, model, geomModel);
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
    PINOCCHIO_CHECK_INPUT_ARGUMENT(reference_configuration.size() == input_model.nq,
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
    reduced_model.inertias              .reserve((size_t)njoints);
    
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
      const JointIndex joint_id_to_lock = (current_index_to_lock < list_of_joints_to_lock.size()) ? list_of_joints_to_lock[current_index_to_lock] : input_model.joints.size();
      
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
                            FIXED_JOINT);
        
        FrameIndex frame_id = reduced_model.addFrame(frame);
        reduced_model.frames[frame_id].previousFrame = frame_id; // a bit weird, but this is a solution for missing parent frame
        
        // Add the Inertia of the link supported by joint_id
        reduced_model.appendBodyToJoint(reduced_parent_joint_index,
                                        input_model.inertias[joint_id],
                                        frame.placement);
        
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
                                                             joint_input_model.jointConfigSelector(input_model.upperPositionLimit));
        // Append inertia
        reduced_model.appendBodyToJoint(reduced_joint_id,
                                        input_model.inertias[joint_id],
                                        SE3::Identity());
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
      const std::string & input_joint_name = input_model.names[input_frame.parent];
      
      std::vector<JointIndex>::const_iterator joint_id_it = std::find(list_of_joints_to_lock.begin(),
                                                                      list_of_joints_to_lock.end(),
                                                                      input_frame.parent);
      
      if(joint_id_it != list_of_joints_to_lock.end())
      {
        if(   input_frame.type == JOINT
           && reduced_model.existFrame(input_frame.name)
           && input_joint_name == input_frame.name)
          continue; // this means that the Joint is now fixed and has been replaced by a Frame. No need to add a new one.
        
        // The joint has been removed and replaced by a Frame
        const FrameIndex joint_frame_id = reduced_model.getFrameId(input_joint_name);
        const Frame & joint_frame = reduced_model.frames[joint_frame_id];
        Frame reduced_frame = input_frame;
        reduced_frame.placement = joint_frame.placement * input_frame.placement;
        reduced_frame.parent = joint_frame.parent;
        reduced_frame.previousFrame = reduced_model.getFrameId(input_model.frames[input_frame.previousFrame].name);
        reduced_model.addFrame(reduced_frame);
      }
      else
      {
        Frame reduced_frame = input_frame;
        reduced_frame.parent = reduced_model.getJointId(input_model.names[input_frame.parent]);
        reduced_frame.previousFrame = reduced_model.getFrameId(input_model.frames[input_frame.previousFrame].name);
        reduced_model.addFrame(reduced_frame);
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
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    buildReducedModel(input_model,list_of_joints_to_lock,reference_configuration,reduced_model);
    
    // Add all the geometries
    typedef GeometryModel::GeometryObject GeometryObject;
    typedef GeometryModel::GeometryObjectVector GeometryObjectVector;
    for(GeometryObjectVector::const_iterator it = input_geom_model.geometryObjects.begin();
        it != input_geom_model.geometryObjects.end(); ++it)
    {
      const GeometryModel::GeometryObject & geom = *it;
      
      const JointIndex joint_id_in_input_model = geom.parentJoint;
      PINOCCHIO_CHECK_INPUT_ARGUMENT(joint_id_in_input_model < (JointIndex)input_model.njoints,
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
    
  }

} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_model_hxx__
