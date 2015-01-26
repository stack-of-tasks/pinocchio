#ifndef __se3_sample_models_hpp__ 
#define __se3_sample_models_hpp__ 

#include "pinocchio/multibody/model.hpp"

namespace se3
{
  namespace buildModels
  {

    void  humanoid2d( Model& model)
    {
      model.addBody(model.getBodyId("universe"),JointModelRX(),SE3::Identity(),Inertia::Random(),"ff1");
      model.addBody(model.getBodyId("ff1"),JointModelRY(),SE3::Identity(),Inertia::Random(),"root");

      model.addBody(model.getBodyId("root"),JointModelRZ(),SE3::Random(),Inertia::Random(),"lleg1");
      model.addBody(model.getBodyId("lleg1"),JointModelRY(),SE3::Random(),Inertia::Random(),"lleg2");

      model.addBody(model.getBodyId("root"),JointModelRZ(),SE3::Random(),Inertia::Random(),"rleg1");
      model.addBody(model.getBodyId("rleg1"),JointModelRY(),SE3::Random(),Inertia::Random(),"rleg2");

      model.addBody(model.getBodyId("root"),JointModelRY(),SE3::Random(),Inertia::Random(),"torso1");
      model.addBody(model.getBodyId("torso1"),JointModelRZ(),SE3::Random(),Inertia::Random(),"chest");

      model.addBody(model.getBodyId("chest"),JointModelRX(),SE3::Random(),Inertia::Random(),"rarm1");
      model.addBody(model.getBodyId("rarm1"),JointModelRZ(),SE3::Random(),Inertia::Random(),"rarm2");

      model.addBody(model.getBodyId("chest"),JointModelRX(),SE3::Random(),Inertia::Random(),"larm1");
      model.addBody(model.getBodyId("larm1"),JointModelRZ(),SE3::Random(),Inertia::Random(),"larm2");
    }

    void humanoidSimple( Model& model, bool usingFF = true)
    { 
      if(! usingFF ) 
	{
	  model.addBody(model.getBodyId("universe"),JointModelRX(),SE3::Identity(),Inertia::Random(),"ff1");
	  model.addBody(model.getBodyId("ff1"),JointModelRY(),SE3::Identity(),Inertia::Random(),"ff2");
	  model.addBody(model.getBodyId("ff2"),JointModelRZ(),SE3::Identity(),Inertia::Random(),"ff3");
	  model.addBody(model.getBodyId("ff3"),JointModelRZ(),SE3::Random(),Inertia::Random(),"ff4");
	  model.addBody(model.getBodyId("ff4"),JointModelRY(),SE3::Identity(),Inertia::Random(),"ff5");
	  model.addBody(model.getBodyId("ff5"),JointModelRX(),SE3::Identity(),Inertia::Random(),"root");
	}
      else
	model.addBody(model.getBodyId("universe"),JointModelFreeFlyer(),SE3::Identity(),Inertia::Random(),"root");

      model.addBody(model.getBodyId("root"),JointModelRX(),SE3::Random(),Inertia::Random(),"lleg1");
      model.addBody(model.getBodyId("lleg1"),JointModelRX(),SE3::Random(),Inertia::Random(),"lleg2");
      model.addBody(model.getBodyId("lleg2"),JointModelRX(),SE3::Random(),Inertia::Random(),"lleg3");
      model.addBody(model.getBodyId("lleg3"),JointModelRX(),SE3::Random(),Inertia::Random(),"lleg4");
      model.addBody(model.getBodyId("lleg4"),JointModelRX(),SE3::Random(),Inertia::Random(),"lleg5");
      model.addBody(model.getBodyId("lleg5"),JointModelRX(),SE3::Random(),Inertia::Random(),"lleg6");

      model.addBody(model.getBodyId("root"),JointModelRX(),SE3::Random(),Inertia::Random(),"rleg1");
      model.addBody(model.getBodyId("rleg1"),JointModelRX(),SE3::Random(),Inertia::Random(),"rleg2");
      model.addBody(model.getBodyId("rleg2"),JointModelRX(),SE3::Random(),Inertia::Random(),"rleg3");
      model.addBody(model.getBodyId("rleg3"),JointModelRX(),SE3::Random(),Inertia::Random(),"rleg4");
      model.addBody(model.getBodyId("rleg4"),JointModelRX(),SE3::Random(),Inertia::Random(),"rleg5");
      model.addBody(model.getBodyId("rleg5"),JointModelRX(),SE3::Random(),Inertia::Random(),"rleg6");

      model.addBody(model.getBodyId("root"),JointModelRX(),SE3::Random(),Inertia::Random(),"torso1");
      model.addBody(model.getBodyId("torso1"),JointModelRX(),SE3::Random(),Inertia::Random(),"chest");

      model.addBody(model.getBodyId("chest"),JointModelRX(),SE3::Random(),Inertia::Random(),"rarm1");
      model.addBody(model.getBodyId("rarm1"),JointModelRX(),SE3::Random(),Inertia::Random(),"rarm2");
      model.addBody(model.getBodyId("rarm2"),JointModelRX(),SE3::Random(),Inertia::Random(),"rarm3");
      model.addBody(model.getBodyId("rarm3"),JointModelRX(),SE3::Random(),Inertia::Random(),"rarm4");
      model.addBody(model.getBodyId("rarm4"),JointModelRX(),SE3::Random(),Inertia::Random(),"rarm5");
      model.addBody(model.getBodyId("rarm5"),JointModelRX(),SE3::Random(),Inertia::Random(),"rarm6");

      model.addBody(model.getBodyId("chest"),JointModelRX(),SE3::Random(),Inertia::Random(),"larm1");
      model.addBody(model.getBodyId("larm1"),JointModelRX(),SE3::Random(),Inertia::Random(),"larm2");
      model.addBody(model.getBodyId("larm2"),JointModelRX(),SE3::Random(),Inertia::Random(),"larm3");
      model.addBody(model.getBodyId("larm3"),JointModelRX(),SE3::Random(),Inertia::Random(),"larm4");
      model.addBody(model.getBodyId("larm4"),JointModelRX(),SE3::Random(),Inertia::Random(),"larm5");
      model.addBody(model.getBodyId("larm5"),JointModelRX(),SE3::Random(),Inertia::Random(),"larm6");
    }

  } // namespace buildModels
} // namespace se3 

#endif // ifndef __se3_sample_models_hpp__ 
