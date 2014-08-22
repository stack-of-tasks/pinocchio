#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/multibody/joint.hpp"
#include "pinocchio/multibody/model.hpp"

#include <iostream>

#include "pinocchio/tools/timer.hpp"

template<typename JointModel>
void rneaForwardStep(const se3::Model& model,
		     se3::Data& data,
		     const se3::JointModelBase<JointModel> & jmodel,
		     se3::JointDataBase<typename JointModel::JointData> & jdata,
		     int i,
		     const Eigen::VectorXd & q,
		     const Eigen::VectorXd & v,
		     const Eigen::VectorXd & a)
{
  using namespace Eigen;
  using namespace se3;

  jmodel.calc(jdata.derived(),q,v);
  
  const Model::Index & parent = model.parents[i];
  data.liMi[i] = model.jointPlacements[i]*jdata.M();
  
  if(parent>0) data.oMi[i] = data.oMi[parent]*data.liMi[i];
  else         data.oMi[i] = data.liMi[i];
  
  data.v[i] = jdata.v();
  if(parent>0) data.v[i] += data.liMi[i].actInv(data.v[parent]);

  jmodel.jointMotion(a);
  data.a[i] =  jdata.S()*jmodel.jointMotion(a) + jdata.c() + (data.v[i] ^ jdata.v()) ; 
  if(parent>0) data.a[i] += data.liMi[i].actInv(data.a[parent]);
  
  data.f[i] = model.inertias[i]*data.a[i] + model.inertias[i].vxiv(data.v[i]); // -f_ext
}

template<typename JointModel>
void rneaBackwardStep(const se3::Model& model,
		      se3::Data& data,
		      const se3::JointModelBase<JointModel> & jmodel,
		      se3::JointDataBase<typename JointModel::JointData> & jdata,
		      int i)
{
  using namespace Eigen;
  using namespace se3;
  
  const Model::Index & parent  = model.parents[i];      
  jmodel.jointForce(data.tau)  = jdata.S().transpose()*data.f[i];
  if(parent>0) data.f[parent] += data.liMi[i].act(data.f[i]);
}



struct RneaForwardStepVisitor : public boost::static_visitor<>
{
  const se3::Model& model;
  se3::Data& data;
  se3::JointDataVariant & jdata;
  int i;
  const Eigen::VectorXd & q;
  const Eigen::VectorXd & v;
  const Eigen::VectorXd & a;

  RneaForwardStepVisitor( const se3::Model& model,
			  se3::Data& data,
			  se3::JointDataVariant & jdata,
			  int i,
			  const Eigen::VectorXd & q,
			  const Eigen::VectorXd & v,
			  const Eigen::VectorXd & a)
    : model(model),data(data),jdata(jdata),i(i),q(q),v(v),a(a) {}


  template <typename D>
  void operator()(const se3::JointModelBase<D> & jmodel) const
  {
    rneaForwardStep(model,data,jmodel,boost::get<typename D::JointData&>(jdata),i,q,v,a);
  }

  static void run( const se3::Model& model,
		  se3::Data& data,
		  const se3::JointModelVariant & jmodel, 
		  se3::JointDataVariant & jdata,
		  int i,
		  const Eigen::VectorXd & q,
		  const Eigen::VectorXd & v,
		  const Eigen::VectorXd & a)
  {  boost::apply_visitor( RneaForwardStepVisitor(model,data,jdata,i,q,v,a), jmodel ); }

};

struct RneaBackwardStepVisitor : public boost::static_visitor<>
{
  const se3::Model& model;
  se3::Data& data;
  se3::JointDataVariant & jdata;
  int i;

  RneaBackwardStepVisitor( const se3::Model& model,
			  se3::Data& data,
			  se3::JointDataVariant & jdata,
			  int i)
    : model(model),data(data),jdata(jdata),i(i) {}


  template <typename D>
  void operator()(const se3::JointModelBase<D> & jmodel) const
  {
    rneaBackwardStep(model,data,jmodel,boost::get<typename D::JointData&>(jdata),i);
  }

  static void run( const se3::Model& model,
		  se3::Data& data,
		  const se3::JointModelVariant & jmodel, 
		  se3::JointDataVariant & jdata,
		  int i)
  {  boost::apply_visitor( RneaBackwardStepVisitor(model,data,jdata,i), jmodel ); }

};

//#define __SSE3__
#include <fenv.h>
#ifdef __SSE3__
#include <pmmintrin.h>
#endif

int main()
{
#ifdef __SSE3__
_MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
#endif


  using namespace Eigen;
  using namespace se3;


  se3::Model model;
  model.addBody(model.getBodyId("universe"),JointModelFreeFlyer(),SE3::Random(),Inertia::Random(),"root");

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
  //model.addBody(model.getBodyId("torso2"),JointModelRX(),SE3::Random(),Inertia::Random(),"torso3");
  model.addBody(model.getBodyId("torso2"),JointModelRX(),SE3::Random(),Inertia::Random(),"neck1");
  model.addBody(model.getBodyId("neck1"),JointModelRX(),SE3::Random(),Inertia::Random(),"neck2");
  //model.addBody(model.getBodyId("neck2"),JointModelRX(),SE3::Random(),Inertia::Random(),"neck3");

  model.addBody(model.getBodyId("torso2"),JointModelRX(),SE3::Random(),Inertia::Random(),"rarm1");
  model.addBody(model.getBodyId("rarm1"),JointModelRX(),SE3::Random(),Inertia::Random(),"rarm2");
  model.addBody(model.getBodyId("rarm2"),JointModelRX(),SE3::Random(),Inertia::Random(),"rarm3");
  model.addBody(model.getBodyId("rarm3"),JointModelRX(),SE3::Random(),Inertia::Random(),"rarm4");
  model.addBody(model.getBodyId("rarm4"),JointModelRX(),SE3::Random(),Inertia::Random(),"rarm5");
  model.addBody(model.getBodyId("rarm5"),JointModelRX(),SE3::Random(),Inertia::Random(),"rarm6");
  model.addBody(model.getBodyId("rarm6"),JointModelRX(),SE3::Random(),Inertia::Random(),"rgrip");

  model.addBody(model.getBodyId("torso2"),JointModelRX(),SE3::Random(),Inertia::Random(),"larm1");
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
      //rneaForwardStep(model,data,model.joints[i],data.joints[i],i,q,v,a);

      //rneaForwardStep(model,data,
      //	      boost::get<const se3::JointModelRX&>(model.joints[i]),
      //	      boost::get< se3::JointDataRX&>(data.joints[i]),
      //	      i,q,v,a);
      RneaForwardStepVisitor::run(model,data,model.joints[i],data.joints[i],i,q,v,a);
    }

  for( int i=model.nbody-1;i>0;--i )
    {
      //rneaBackwardStep(model,data,model.joints[i],data.joints[i],i);
      RneaBackwardStepVisitor::run(model,data,model.joints[i],data.joints[i],i);
    }
    }
  timer.toc(std::cout,1000);

  return 0;
}
