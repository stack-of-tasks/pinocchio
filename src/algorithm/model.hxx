//
// Copyright (c) 2019 CNRS
//

#ifndef __pinocchio_algorithm_model_hxx__
#define __pinocchio_algorithm_model_hxx__

namespace pinocchio
{
  namespace details {
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    void appendUniverseToModel (const ModelTpl<Scalar,Options,JointCollectionTpl> & modelAB,
                                const GeometryModel& geomModelAB,
                                FrameIndex parentFrame,
                                const SE3Tpl<Scalar, Options>& pfMAB,
                                ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                GeometryModel& geomModel)
    {
      typedef FrameTpl<Scalar, Options> Frame;

      const Frame& pframe = model.frames[parentFrame];
      JointIndex jid = pframe.parent;
      assert (jid < model.joints.size());

      // If inertia is not NaN, add it.
      if (modelAB.inertias[0] == modelAB.inertias[0])
        model.appendBodyToJoint (jid, modelAB.inertias[0], pframe.placement * pfMAB);

      // Add all frames whose parent is this joint.
      for (FrameIndex fid = 1; fid < modelAB.frames.size(); ++fid) {
        Frame frame = modelAB.frames[fid];
        if (frame.parent == 0) {
          frame.parent = jid;
          if (frame.previousFrame != 0) {
            frame.previousFrame = model.getFrameId (
                modelAB.frames[frame.previousFrame].name,
                modelAB.frames[frame.previousFrame].type);
          } else {
            frame.previousFrame = parentFrame;
          }
          // Modify frame placement
          frame.placement = pframe.placement * pfMAB * frame.placement;
          model.addFrame (frame);
        }
      }
      // Add all geometries whose parent is this joint.
      for (GeomIndex gid = 0; gid < geomModelAB.geometryObjects.size(); ++gid) {
        GeometryObject go = geomModelAB.geometryObjects[gid];
        if (go.parentJoint == 0) {
          go.parentJoint = jid;
          if (go.parentFrame != 0) {
            go.parentFrame = model.getFrameId (
                modelAB.frames[go.parentFrame].name,
                modelAB.frames[go.parentFrame].type);
          } else {
            go.parentFrame = parentFrame;
          }
          go.placement = pframe.placement * pfMAB * go.placement;
          geomModel.addGeometryObject (go);
        }
      }
    }

    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    struct AppendJointOfModelAlgoTpl : public fusion::JointVisitorBase< AppendJointOfModelAlgoTpl<Scalar,Options,JointCollectionTpl> >
    {
      typedef boost::fusion::vector<
          const ModelTpl<Scalar,Options,JointCollectionTpl> &,
          const GeometryModel&,
          JointIndex,
          const SE3Tpl<Scalar, Options>&,
          ModelTpl<Scalar,Options,JointCollectionTpl> &,
          GeometryModel& > ArgsType;

      template <typename JointModel>
      static void algo (const JointModelBase<JointModel> & jmodel,
          const ModelTpl<Scalar,Options,JointCollectionTpl> & modelAB,
          const GeometryModel& geomModelAB,
          JointIndex parentId,
          const SE3Tpl<Scalar, Options>& pMi,
          ModelTpl<Scalar,Options,JointCollectionTpl> & model,
          GeometryModel& geomModel)
      {
        // If old parent is universe, use what's provided in the input.
        // otherwise, get the parent from modelAB.
        if (modelAB.parents[jmodel.id()] > 0)
          parentId = model.getJointId(modelAB.names[modelAB.parents[jmodel.id()]]);
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
          FrameTpl<Scalar, Options> frame = modelAB.frames[fid];
          if (frame.parent == jmodel.id()) {
            frame.parent = jid;
            assert (frame.previousFrame > 0 || frame.type == JOINT);
            if (frame.previousFrame != 0) {
              frame.previousFrame = model.getFrameId (
                  modelAB.frames[frame.previousFrame].name,
                  modelAB.frames[frame.previousFrame].type);
            }
            model.addFrame (frame);
          }
        }
        // Add all geometries whose parent is this joint.
        for (GeomIndex gid = 0; gid < geomModelAB.geometryObjects.size(); ++gid) {
          GeometryObject go = geomModelAB.geometryObjects[gid];
          if (go.parentJoint == jmodel.id()) {
            go.parentJoint = jid;
            assert (go.parentFrame > 0);
            if (go.parentFrame != 0) {
              go.parentFrame = model.getFrameId (
                  modelAB.frames[go.parentFrame].name,
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
              FrameIndex frameInModelA,
              const SE3Tpl<Scalar, Options>& aMb,
              ModelTpl<Scalar,Options,JointCollectionTpl>& model)
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
              FrameIndex frameInModelA,
              const SE3Tpl<Scalar, Options>& aMb,
              ModelTpl<Scalar,Options,JointCollectionTpl>& model,
              GeometryModel& geomModel)
  {
    typedef details::AppendJointOfModelAlgoTpl<Scalar, Options, JointCollectionTpl> AppendJointOfModelAlgo;
    typedef typename AppendJointOfModelAlgo::ArgsType ArgsType;
    typedef SE3Tpl<Scalar, Options> SE3;

    const FrameTpl<Scalar, Options>& frame = modelA.frames[frameInModelA];
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
    for (JointIndex jid = 1; jid <= frame.parent; ++jid) {
      ArgsType args (modelA, geomModelA, 0, id, model, geomModel);
      AppendJointOfModelAlgo::run (modelA.joints[jid], args);
    }

    // Copy modelB joints
    details::appendUniverseToModel (modelB, geomModelB,
        model.getFrameId (frame.name, frame.type), aMb, model, geomModel);
    for (JointIndex jid = 1; jid < modelB.joints.size(); ++jid) {
      SE3 pMi = (jid == 1 ? frame.placement * aMb : id);
      ArgsType args (modelB, geomModelB, frame.parent, pMi, model, geomModel);
      AppendJointOfModelAlgo::run (modelB.joints[jid], args);
    }

    // Copy remaining joints of modelA
    for (JointIndex jid = frame.parent+1; jid < modelA.joints.size(); ++jid) {
      ArgsType args (modelA, geomModelA, 0, id, model, geomModel);
      AppendJointOfModelAlgo::run (modelA.joints[jid], args);
    }

#ifdef PINOCCHIO_WITH_HPP_FCL
    // Add collision pairs of geomModelA and geomModelB
    geomModel.collisionPairs.reserve(geomModelA.collisionPairs.size()
       + geomModelB.collisionPairs.size()
       + geomModelA.geometryObjects.size() * geomModelB.geometryObjects.size());
    for (std::size_t icp = 0; icp < geomModelA.collisionPairs.size(); ++icp) {
      const CollisionPair& cp (geomModelA.collisionPairs[icp]);
      GeomIndex go1 = geomModel.getGeometryId(geomModelA.geometryObjects[cp.first ].name);
      GeomIndex go2 = geomModel.getGeometryId(geomModelA.geometryObjects[cp.second].name);
      geomModel.addCollisionPair (CollisionPair (go1, go2));
    }
    for (std::size_t icp = 0; icp < geomModelB.collisionPairs.size(); ++icp) {
      const CollisionPair& cp (geomModelB.collisionPairs[icp]);
      GeomIndex go1 = geomModel.getGeometryId(geomModelB.geometryObjects[cp.first ].name);
      GeomIndex go2 = geomModel.getGeometryId(geomModelB.geometryObjects[cp.second].name);
      geomModel.addCollisionPair (CollisionPair (go1, go2));
    }

    // add the collision pairs between geomModelA and geomModelB.
    for (Index i = 0; i < geomModelA.geometryObjects.size(); ++i) {
      GeomIndex go1 = geomModel.getGeometryId(geomModelA.geometryObjects[i].name);
      for (Index j = 0; j < geomModelB.geometryObjects.size(); ++j) {
        GeomIndex go2 = geomModel.getGeometryId(geomModelB.geometryObjects[j].name);
        if (   geomModel.geometryObjects[go1].parentJoint
            != geomModel.geometryObjects[go2].parentJoint)
          geomModel.addCollisionPair(CollisionPair(go1, go2));
      }
    }
#endif
  }

} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_model_hxx__
