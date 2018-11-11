//
// Copyright (c) 2015-2018 CNRS
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
                                  const SE3 & placement = SE3::Random(),
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
    } // namespace details

    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    inline void humanoidSimple(ModelTpl<Scalar,Options,JointCollectionTpl> & model, bool usingFF)
    { humanoidRandom(model,usingFF); }

    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    void humanoidRandom(ModelTpl<Scalar,Options,JointCollectionTpl> & model, bool usingFF)
    {
      using details::addJointAndBody;
      static const SE3 Id = SE3::Identity();
      typedef JointCollectionTpl<Scalar, Options> JC;

      // root
      if(! usingFF )
      {
        typename JC::JointModelComposite jff((typename JC::JointModelTranslation()));
        jff.addJoint(typename JC::JointModelSphericalZYX());
        addJointAndBody(model, jff, "universe", "root", Id);
      }
      else
      {
        addJointAndBody(model, typename JC::JointModelFreeFlyer(), "universe", "root", Id);
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

  } // namespace buildModels
  
} // namespace pinocchio

#endif // ifndef __pinocchio_sample_models_hxx__
