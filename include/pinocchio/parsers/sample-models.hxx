//
// Copyright (c) 2015-2019 CNRS INRIA
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_sample_models_hxx__
#define __pinocchio_sample_models_hxx__

namespace pinocchio
{
  namespace buildModels
  {
    namespace details
    {
      template<typename Scalar, int Options,
               template<typename,int> class JointCollectionTpl,
               typename JointModel>
      static void addJointAndBody(ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                  const JointModelBase<JointModel> & joint,
                                  const std::string & parent_name,
                                  const std::string & name,
                                  const typename ModelTpl<Scalar,Options,JointCollectionTpl>::SE3 & placement = ModelTpl<Scalar,Options,JointCollectionTpl>::SE3::Random(),
                                  bool setRandomLimits = true)
      {
        typedef typename JointModel::ConfigVector_t CV;
        typedef typename JointModel::TangentVector_t TV;
        
        JointIndex idx;
        
        if(setRandomLimits)
          idx = model.addJoint(model.getJointId(parent_name),joint,
                               placement, name + "_joint",
                               TV::Random(joint.nv(),1) + TV::Constant(joint.nv(),1,1), // effort
                               TV::Random(joint.nv(),1) + TV::Constant(joint.nv(),1,1), // vel
                               CV::Random(joint.nq(),1) - CV::Constant(joint.nq(),1,1), // qmin
                               CV::Random(joint.nq(),1) + CV::Constant(joint.nq(),1,1)  // qmax
                               );
        else
          idx = model.addJoint(model.getJointId(parent_name),joint,
                               placement, name + "_joint");
          
        model.addJointFrame(idx);
        
        model.appendBodyToJoint(idx,Inertia::Random(),SE3::Identity());
        model.addBodyFrame(name + "_body", idx);
      }
      
      template<typename Scalar, int Options,
               template<typename,int> class JointCollectionTpl>
      static void addManipulator(ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                 typename ModelTpl<Scalar,Options,JointCollectionTpl>::JointIndex rootJoint = 0,
                                 const typename ModelTpl<Scalar,Options,JointCollectionTpl>::SE3 & Mroot
                                 = ModelTpl<Scalar,Options,JointCollectionTpl>::SE3::Identity(),
                                 const std::string & pre = "")
      {
        typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
        typedef typename Model::JointIndex JointIndex;
        typedef typename Model::SE3 SE3;
        typedef typename Model::Inertia Inertia;
        
        typedef JointCollectionTpl<Scalar,Options> JC;
        typedef typename JC::JointModelRX::ConfigVector_t CV;
        typedef typename JC::JointModelRX::TangentVector_t TV;
        
        JointIndex idx = rootJoint;
        
        SE3 Marm(SE3::Matrix3::Identity(),SE3::Vector3::UnitZ());
        SE3 I4 = SE3::Identity();
        Inertia Ijoint(.1,Inertia::Vector3::Zero(),Inertia::Matrix3::Identity()*.01);
        Inertia Iarm(1.,typename Inertia::Vector3(0,0,.5),Inertia::Matrix3::Identity());
        CV qmin = CV::Constant(-3.14), qmax   = CV::Constant(3.14);
        TV vmax = TV::Constant( 10),   taumax = TV::Constant(10);
        
        idx = model.addJoint(idx,typename JC::JointModelRX(),Mroot,
                             pre+"shoulder1_joint",taumax,vmax,qmin,qmax);
        model.appendBodyToJoint(idx,Ijoint);
        model.addJointFrame(idx);
        model.addBodyFrame(pre+"shoulder1_body",idx);
        
        idx = model.addJoint(idx,typename JC::JointModelRY(),I4,
                             pre+"shoulder2_joint",taumax,vmax,qmin,qmax);
        model.appendBodyToJoint(idx,Ijoint);
        model.addJointFrame(idx);
        model.addBodyFrame(pre+"shoulder2_body",idx);
        
        idx = model.addJoint(idx,typename JC::JointModelRZ(),I4,
                             pre+"shoulder3_joint",taumax,vmax,qmin,qmax);
        model.appendBodyToJoint(idx,Iarm);
        model.addJointFrame(idx);
        model.addBodyFrame(pre+"upperarm_body",idx);
        
        idx = model.addJoint(idx,typename JC::JointModelRY(),Marm,
                             pre+"elbow_joint",taumax,vmax,qmin,qmax);
        model.appendBodyToJoint(idx,Iarm);
        model.addJointFrame(idx);
        model.addBodyFrame(pre+"lowerarm_body",idx);
        model.addBodyFrame(pre+"elbow_body",idx);
        
        idx = model.addJoint(idx,typename JC::JointModelRX(),Marm,
                             pre+"wrist1_joint",taumax,vmax,qmin,qmax);
        model.appendBodyToJoint(idx,Ijoint);
        model.addJointFrame(idx);
        model.addBodyFrame(pre+"wrist1_body",idx);
        
        idx = model.addJoint(idx,typename JC::JointModelRY(),I4,
                             pre+"wrist2_joint",taumax,vmax,qmin,qmax);
        model.appendBodyToJoint(idx,Iarm);
        model.addJointFrame(idx);
        model.addBodyFrame(pre+"effector_body",idx);
        
      }
      
#ifdef PINOCCHIO_WITH_HPP_FCL
      /* Add a 6DOF manipulator shoulder-elbow-wrist geometries to an existing model.
       * <model> is the the kinematic chain, constant.
       * <geom> is the geometry model where the new geoms are added.
       * <pre> is the prefix (string) before every name in the model.
       */
      template<typename Scalar, int Options,
               template<typename,int> class JointCollectionTpl>
      static void addManipulatorGeometries(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                           GeometryModel & geom,
                                           const std::string & pre = "")
      {
        typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
        typedef typename Model::FrameIndex FrameIndex;
        typedef typename Model::SE3 SE3;

        const Eigen::Vector4d meshColor(1., 1., 0.78, 1.0);
        
        FrameIndex parentFrame;
        
        parentFrame = model.getBodyId(pre+"shoulder1_body");
        GeometryObject shoulderBall(pre+"shoulder_object",
                                    parentFrame, model.frames[parentFrame].parent,
                                    shared_ptr<fcl::Sphere>(new fcl::Sphere(0.05)),
                                    SE3::Identity(),
                                    "SPHERE",
                                    Eigen::Vector3d::Ones(),
                                    false,
                                    meshColor);
        geom.addGeometryObject(shoulderBall);
        
        parentFrame = model.getBodyId(pre+"elbow_body");
        GeometryObject elbowBall(pre+"elbow_object",
                                 parentFrame, model.frames[parentFrame].parent,
                                 shared_ptr<fcl::Sphere>(new fcl::Sphere(0.05)),
                                 SE3::Identity(),
                                 "SPHERE",
                                 Eigen::Vector3d::Ones(),
                                 false,
                                 meshColor);
        geom.addGeometryObject(elbowBall);
        
        parentFrame = model.getBodyId(pre+"wrist1_body");
        GeometryObject wristBall(pre+"wrist_object",
                                 parentFrame, model.frames[parentFrame].parent,
                                 shared_ptr<fcl::Sphere>(new fcl::Sphere(0.05)),
                                 SE3::Identity(),
                                 "SPHERE",
                                 Eigen::Vector3d::Ones(),
                                 false,
                                 meshColor);
        geom.addGeometryObject(wristBall);
        
        parentFrame = model.getBodyId(pre+"upperarm_body");
        GeometryObject upperArm(pre+"upperarm_object",
                                parentFrame, model.frames[parentFrame].parent,
                                shared_ptr<fcl::Capsule>(new fcl::Capsule(0.05, .8)),
                                SE3(SE3::Matrix3::Identity(), typename  SE3::Vector3(0,0,0.5)),
                                "CAPSULE",
                                Eigen::Vector3d::Ones(),
                                false,
                                meshColor);
        geom.addGeometryObject(upperArm);
        
        parentFrame = model.getBodyId(pre+"lowerarm_body");
        GeometryObject lowerArm(pre+"lowerarm_object",
                                parentFrame, model.frames[parentFrame].parent,
                                shared_ptr<fcl::Capsule>(new fcl::Capsule(0.05, .8)),
                                SE3(SE3::Matrix3::Identity(), typename SE3::Vector3(0,0,0.5)),
                                "CAPSULE",
                                Eigen::Vector3d::Ones(),
                                false,
                                meshColor);
        geom.addGeometryObject(lowerArm);
        
        parentFrame = model.getBodyId(pre+"effector_body");
        GeometryObject effectorArm(pre+"effector_object",
                                   parentFrame, model.frames[parentFrame].parent,
                                   shared_ptr<fcl::Capsule>(new fcl::Capsule(0.05, .2)),
                                   SE3(SE3::Matrix3::Identity(), typename SE3::Vector3(0,0,0.1)),
                                   "CAPSULE",
                                   Eigen::Vector3d::Ones(),
                                   false,
                                   meshColor);
        geom.addGeometryObject(effectorArm);
      }
#endif
      
      template<typename Vector3Like>
      static typename Eigen::AngleAxis<typename Vector3Like::Scalar>::Matrix3
      rotate(const typename Vector3Like::Scalar angle,
             const Eigen::MatrixBase<Vector3Like> & axis)
      {
        typedef typename Vector3Like::Scalar Scalar;
        typedef Eigen::AngleAxis<Scalar> AngleAxis;
        
        return AngleAxis(angle,axis).toRotationMatrix();
      }
      
    } // namespace details
    
    template<typename Scalar, int Options,
             template<typename,int> class JointCollectionTpl>
    void manipulator(ModelTpl<Scalar,Options,JointCollectionTpl> & model)
    {
      details::addManipulator(model);
    }
    
#ifdef PINOCCHIO_WITH_HPP_FCL
    template<typename Scalar, int Options,
             template<typename,int> class JointCollectionTpl>
    void manipulatorGeometries(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                               GeometryModel & geom)
    { details::addManipulatorGeometries(model,geom); }
#endif

    template<typename Scalar, int Options,
             template<typename,int> class JointCollectionTpl>
    void humanoidRandom(ModelTpl<Scalar,Options,JointCollectionTpl> & model, bool usingFF)
    {
      typedef JointCollectionTpl<Scalar, Options> JC;
      typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
      typedef typename Model::SE3 SE3;
      using details::addJointAndBody;
      static const SE3 Id = SE3::Identity();

      // root
      if(! usingFF)
      {
        typename JC::JointModelComposite jff((typename JC::JointModelTranslation()));
        jff.addJoint(typename JC::JointModelSphericalZYX());
        addJointAndBody(model, jff, "universe", "root", Id);
      }
      else
      {
        addJointAndBody(model, typename JC::JointModelFreeFlyer(),
                        "universe", "root", Id);
        model.lowerPositionLimit.template segment<4>(3).fill(-1.);
        model.upperPositionLimit.template segment<4>(3).fill( 1.);
      }

      // lleg
      addJointAndBody(model,typename JC::JointModelRX(),"root_joint","lleg1");
      addJointAndBody(model,typename JC::JointModelRY(),"lleg1_joint","lleg2");
      addJointAndBody(model,typename JC::JointModelRZ(),"lleg2_joint","lleg3");
      addJointAndBody(model,typename JC::JointModelRY(),"lleg3_joint","lleg4");
      addJointAndBody(model,typename JC::JointModelRY(),"lleg4_joint","lleg5");
      addJointAndBody(model,typename JC::JointModelRX(),"lleg5_joint","lleg6");
      
      // rleg
      addJointAndBody(model,typename JC::JointModelRX(),"root_joint","rleg1");
      addJointAndBody(model,typename JC::JointModelRY(),"rleg1_joint","rleg2");
      addJointAndBody(model,typename JC::JointModelRZ(),"rleg2_joint","rleg3");
      addJointAndBody(model,typename JC::JointModelRY(),"rleg3_joint","rleg4");
      addJointAndBody(model,typename JC::JointModelRY(),"rleg4_joint","rleg5");
      addJointAndBody(model,typename JC::JointModelRX(),"rleg5_joint","rleg6");

      // trunc
      addJointAndBody(model,typename JC::JointModelRY(),"root_joint","torso1");
      addJointAndBody(model,typename JC::JointModelRZ(),"torso1_joint","chest");
      
      // rarm
      addJointAndBody(model,typename JC::JointModelRX(),"chest_joint","rarm1");
      addJointAndBody(model,typename JC::JointModelRY(),"rarm1_joint","rarm2");
      addJointAndBody(model,typename JC::JointModelRZ(),"rarm2_joint","rarm3");
      addJointAndBody(model,typename JC::JointModelRY(),"rarm3_joint","rarm4");
      addJointAndBody(model,typename JC::JointModelRY(),"rarm4_joint","rarm5");
      addJointAndBody(model,typename JC::JointModelRX(),"rarm5_joint","rarm6");
      
      // larm
      addJointAndBody(model,typename JC::JointModelRX(),"chest_joint","larm1");
      addJointAndBody(model,typename JC::JointModelRY(),"larm1_joint","larm2");
      addJointAndBody(model,typename JC::JointModelRZ(),"larm2_joint","larm3");
      addJointAndBody(model,typename JC::JointModelRY(),"larm3_joint","larm4");
      addJointAndBody(model,typename JC::JointModelRY(),"larm4_joint","larm5");
      addJointAndBody(model,typename JC::JointModelRX(),"larm5_joint","larm6");

    }
    
    template<typename Scalar, int Options,
             template<typename,int> class JointCollectionTpl>
    void humanoid(ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  bool usingFF)
    {
      typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
      typedef JointCollectionTpl<Scalar,Options> JC;
      typedef typename Model::SE3 SE3;
      typedef typename Model::Inertia Inertia;
      
      typedef typename JC::JointModelRX::ConfigVector_t CV;
      typedef typename JC::JointModelRX::TangentVector_t TV;
      
      typename Model::JointIndex idx,chest,ffidx;
      
      static const Scalar pi = PI<Scalar>();
      
      SE3 Marm(SE3::Matrix3::Identity(),SE3::Vector3::UnitZ());
      SE3 I4 = SE3::Identity();
      Inertia Ijoint(.1,Inertia::Vector3::Zero(),Inertia::Matrix3::Identity()*.01);
      Inertia Iarm(1.,typename Inertia::Vector3(0,0,.5),Inertia::Matrix3::Identity());
      CV qmin = CV::Constant(-3.14), qmax   = CV::Constant(3.14);
      TV vmax = TV::Constant( 10),   taumax = TV::Constant(10);
      
      /* --- Free flyer --- */
      if(usingFF)
      {
        ffidx = model.addJoint(0,typename JC::JointModelFreeFlyer(),
                               SE3::Identity(),"root_joint");
        model.lowerPositionLimit.template segment<4>(3).fill(-1.);
        model.upperPositionLimit.template segment<4>(3).fill( 1.);
      }
      else
      {
        typename JC::JointModelComposite jff((typename JC::JointModelTranslation()));
        jff.addJoint(typename JC::JointModelSphericalZYX());
        ffidx = model.addJoint(0,jff,SE3::Identity(),"root_joint");
      }
      model.appendBodyToJoint(ffidx,Ijoint);
      model.addJointFrame(ffidx);
      
      /* --- Lower limbs --- */
      
      details::addManipulator(model,ffidx,
                              SE3(details::rotate(pi,SE3::Vector3::UnitX()),
                                  typename  SE3::Vector3(0,-0.2,-.1)),"rleg_");
      details::addManipulator(model,ffidx,
                              SE3(details::rotate(pi,SE3::Vector3::UnitX()),
                                  typename  SE3::Vector3(0, 0.2,-.1)),"lleg_");
      
      model.jointPlacements[7 ].rotation()
      = details::rotate(pi/2,SE3::Vector3::UnitY()); // rotate right foot
      model.jointPlacements[13].rotation()
      = details::rotate(pi/2,SE3::Vector3::UnitY()); // rotate left  foot
      
      /* --- Chest --- */
      idx = model.addJoint(ffidx,typename JC::JointModelRX(),I4 ,"chest1_joint",
                           taumax,vmax,qmin,qmax);
      model.appendBodyToJoint(idx,Ijoint);
      model.addJointFrame(idx);
      model.addBodyFrame("chest1_body",idx);
      
      idx = model.addJoint(idx,typename JC::JointModelRY(),I4 ,"chest2_joint",
                           taumax,vmax,qmin,qmax);
      model.appendBodyToJoint(idx,Iarm);
      model.addJointFrame(idx);
      model.addBodyFrame("chest2_body",idx);
      
      chest = idx;
      
      /* --- Head --- */
      idx = model.addJoint(idx,typename JC::JointModelRX(),
                           SE3(SE3::Matrix3::Identity(),SE3::Vector3::UnitZ()),
                           "head1_joint",    taumax,vmax,qmin,qmax);
      model.appendBodyToJoint(idx,Ijoint);
      model.addJointFrame(idx);
      model.addBodyFrame("head1_body",idx);
      
      idx = model.addJoint(idx,typename JC::JointModelRY(),I4 ,"head2_joint",
                           taumax,vmax,qmin,qmax);
      model.appendBodyToJoint(idx,Iarm);
      model.addJointFrame(idx);
      model.addBodyFrame("head2_body",idx);
      
      /* --- Upper Limbs --- */
      details::addManipulator(model,chest,
                              SE3(details::rotate(pi,SE3::Vector3::UnitX()),
                                  typename SE3::Vector3(0,-0.3, 1.)),"rarm_");
      details::addManipulator(model,chest,
                              SE3(details::rotate(pi,SE3::Vector3::UnitX()),
                                  typename SE3::Vector3(0, 0.3, 1.)),"larm_");
    }
    
#ifdef PINOCCHIO_WITH_HPP_FCL
    template<typename Scalar, int Options,
             template<typename,int> class JointCollectionTpl>
    void humanoidGeometries(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                            GeometryModel & geom)
    {
      typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
      typedef typename Model::FrameIndex FrameIndex;
      typedef typename Model::SE3 SE3;
      
      details::addManipulatorGeometries(model,geom,"rleg_");
      details::addManipulatorGeometries(model,geom,"lleg_");
      details::addManipulatorGeometries(model,geom,"rarm_");
      details::addManipulatorGeometries(model,geom,"larm_");
      
      FrameIndex parentFrame;
      
      const Eigen::Vector4d meshColor(1., 1., 0.78, 1.0);
      
      parentFrame = model.getBodyId("chest1_body");
      GeometryObject chestBall("chest_object",
                               parentFrame, model.frames[parentFrame].parent,
                               shared_ptr<fcl::Sphere>(new fcl::Sphere(0.05)),
                               SE3::Identity(),
                               "SPHERE",
                               Eigen::Vector3d::Ones(),
                               false,
                               meshColor);
      geom.addGeometryObject(chestBall);
      
      parentFrame = model.getBodyId("head2_body");
      GeometryObject headBall("head_object",
                              parentFrame, model.frames[parentFrame].parent,
                              shared_ptr<fcl::Sphere>(new fcl::Sphere(0.25)),
                              SE3(SE3::Matrix3::Identity(),
                                  typename SE3::Vector3(0,0,0.5)),
                              "SPHERE",
                               Eigen::Vector3d::Ones(),
                               false,
                               meshColor);
      geom.addGeometryObject(headBall);
      
      parentFrame = model.getBodyId("chest2_body");
      GeometryObject chestArm("chest2_object",
                              parentFrame, model.frames[parentFrame].parent,
                              shared_ptr<fcl::Capsule>(new fcl::Capsule(0.05, .8)),
                              SE3(SE3::Matrix3::Identity(),
                                  typename SE3::Vector3(0,0,0.5)),
                              "SPHERE",
                              Eigen::Vector3d::Ones(),
                              false,
                              meshColor);
      geom.addGeometryObject(chestArm);
    }
#endif

  } // namespace buildModels
  
} // namespace pinocchio

#endif // ifndef __pinocchio_sample_models_hxx__
