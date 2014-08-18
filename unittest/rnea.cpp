#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/multibody/joint.hpp"
#include "pinocchio/multibody/model.hpp"

#include <iostream>

#include "pinocchio/tools/timer.hpp"



int main()
{
  using namespace Eigen;
  using namespace se3;


  se3::Model model;
  model.addBody(model.getBodyId("universe"),JointModelRX(),SE3::Random(),Inertia::Random(),"root");


  model.addBody(model.getBodyId("root"),JointModelRX(),SE3::Random(),Inertia::Random(),"rleg1");
  model.addBody(model.getBodyId("rleg1"),JointModelRX(),SE3::Random(),Inertia::Random(),"rleg2");
  model.addBody(model.getBodyId("rleg2"),JointModelRX(),SE3::Random(),Inertia::Random(),"rleg3");
  model.addBody(model.getBodyId("rleg3"),JointModelRX(),SE3::Random(),Inertia::Random(),"rleg4");
  model.addBody(model.getBodyId("rleg4"),JointModelRX(),SE3::Random(),Inertia::Random(),"rleg5");
  model.addBody(model.getBodyId("rleg5"),JointModelRX(),SE3::Random(),Inertia::Random(),"rleg6");

  model.addBody(model.getBodyId("root"),JointModelRX(),SE3::Random(),Inertia::Random(),"lleg1");
  model.addBody(model.getBodyId("lleg1"),JointModelRX(),SE3::Random(),Inertia::Random(),"lleg2");
  model.addBody(model.getBodyId("lleg2"),JointModelRX(),SE3::Random(),Inertia::Random(),"lleg3");
  model.addBody(model.getBodyId("lleg3"),JointModelRX(),SE3::Random(),Inertia::Random(),"lleg4");
  model.addBody(model.getBodyId("lleg4"),JointModelRX(),SE3::Random(),Inertia::Random(),"lleg5");
  model.addBody(model.getBodyId("lleg5"),JointModelRX(),SE3::Random(),Inertia::Random(),"lleg6");

  model.addBody(model.getBodyId("root"),JointModelRX(),SE3::Random(),Inertia::Random(),"torso1");
  model.addBody(model.getBodyId("torso1"),JointModelRX(),SE3::Random(),Inertia::Random(),"torso2");
  model.addBody(model.getBodyId("torso2"),JointModelRX(),SE3::Random(),Inertia::Random(),"torso3");
  model.addBody(model.getBodyId("torso3"),JointModelRX(),SE3::Random(),Inertia::Random(),"neck1");
  model.addBody(model.getBodyId("neck1"),JointModelRX(),SE3::Random(),Inertia::Random(),"neck2");
  model.addBody(model.getBodyId("neck2"),JointModelRX(),SE3::Random(),Inertia::Random(),"neck3");

  model.addBody(model.getBodyId("torso3"),JointModelRX(),SE3::Random(),Inertia::Random(),"rarm1");
  model.addBody(model.getBodyId("rarm1"),JointModelRX(),SE3::Random(),Inertia::Random(),"rarm2");
  model.addBody(model.getBodyId("rarm2"),JointModelRX(),SE3::Random(),Inertia::Random(),"rarm3");
  model.addBody(model.getBodyId("rarm3"),JointModelRX(),SE3::Random(),Inertia::Random(),"rarm4");
  model.addBody(model.getBodyId("rarm4"),JointModelRX(),SE3::Random(),Inertia::Random(),"rarm5");
  model.addBody(model.getBodyId("rarm5"),JointModelRX(),SE3::Random(),Inertia::Random(),"rarm6");
  model.addBody(model.getBodyId("rarm6"),JointModelRX(),SE3::Random(),Inertia::Random(),"rgrip");

  model.addBody(model.getBodyId("torso3"),JointModelRX(),SE3::Random(),Inertia::Random(),"larm1");
  model.addBody(model.getBodyId("larm1"),JointModelRX(),SE3::Random(),Inertia::Random(),"larm2");
  model.addBody(model.getBodyId("larm2"),JointModelRX(),SE3::Random(),Inertia::Random(),"larm3");
  model.addBody(model.getBodyId("larm3"),JointModelRX(),SE3::Random(),Inertia::Random(),"larm4");
  model.addBody(model.getBodyId("larm4"),JointModelRX(),SE3::Random(),Inertia::Random(),"larm5");
  model.addBody(model.getBodyId("larm5"),JointModelRX(),SE3::Random(),Inertia::Random(),"larm6");
  model.addBody(model.getBodyId("larm6"),JointModelRX(),SE3::Random(),Inertia::Random(),"lgrip");

  se3::Data data(model);

  VectorXd q = VectorXd::Random(model.nq);
  VectorXd v = VectorXd::Random(model.nv);
  VectorXd a = VectorXd::Random(model.nv);

  data.v[0] = Motion::Zero();
  data.a[0] = -model.gravity;
 
  StackTicToc timer(StackTicToc::US); timer.tic();
  SMOOTH(1000)
    {
  for( int i=1;i<model.nbody;++i )
    {
      JointModelRX & jmodel = model.joints[i];
      JointDataRX & jdata = data.joints[i];
      jmodel.calc(jdata,q,v);
      VectorBlock<VectorXd> qdd = a.segment(jmodel.idx_v,jmodel.nv);

      const Model::Index & parent = model.parents[i];
      const SE3 & liMi = data.liMi[i] = model.jointPlacements[i]*jdata.M;
      
      if(parent>0) data.oMi[i] = data.oMi[parent]*liMi;
      else         data.oMi[i] = liMi;

      data.v[i] = jdata.v;
      if(parent>0) data.v[i] += liMi.actInv(data.v[parent]);

      data.a[i] =  Motion(jdata.S*qdd) + jdata.c + data.v[i].cross(jdata.v); 
      if(parent>0) data.a[i] += liMi.actInv(data.a[parent]);

      data.f[i] = model.inertias[i]*data.a[i] + model.inertias[i].vxiv(data.v[i]); // -f_ext
    }

  for( int i=model.nbody-1;i>0;--i )
    {
      const Model::Index & parent = model.parents[i];
      JointModelRX & jmodel = model.joints[i];
      
      data.tau.segment(jmodel.idx_v,jmodel.nv) = data.joints[i].S.transpose()*data.f[i].toVector();
      if(parent>0) data.f[parent] += data.liMi[i].act(data.f[i]);
      
    }
    }
  timer.toc(std::cout,1000);

  return 0;
}
