#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/multibody/joint.hpp"
#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/crba2.hpp"
#include "pinocchio/algorithm/cholesky.hpp"
//#include "pinocchio/multibody/parser/urdf.hpp"

#include <iostream>

#include "pinocchio/tools/timer.hpp"


//#define __SSE3__
#include <fenv.h>
#ifdef __SSE3__
#include <pmmintrin.h>
#endif

int main(int argc, const char ** argv)
{
#ifdef __SSE3__
  _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
#endif


  using namespace Eigen;
  using namespace se3;

  SE3::Matrix3 I3 = SE3::Matrix3::Identity();

  se3::Model model;

  // std::string filename = "/home/nmansard/src/metapod/data/simple_arm.urdf";
  // if(argc>1) filename = argv[1];
  // model = se3::buildModel(filename,false);


  model.addBody(model.getBodyId("universe"),JointModelRX(),SE3::Random(),Inertia::Random(),"ff1");
  model.addBody(model.getBodyId("ff1"),JointModelRX(),SE3::Random(),Inertia::Random(),"root");

  model.addBody(model.getBodyId("root"),JointModelRX(),SE3::Random(),Inertia::Random(),"lleg1");
  model.addBody(model.getBodyId("lleg1"),JointModelRX(),SE3::Random(),Inertia::Random(),"lleg2");

  model.addBody(model.getBodyId("root"),JointModelRX(),SE3::Random(),Inertia::Random(),"rleg1");
  model.addBody(model.getBodyId("rleg1"),JointModelRX(),SE3::Random(),Inertia::Random(),"rleg2");

  model.addBody(model.getBodyId("root"),JointModelRX(),SE3::Random(),Inertia::Random(),"torso1");
  model.addBody(model.getBodyId("torso1"),JointModelRX(),SE3::Random(),Inertia::Random(),"chest");

  model.addBody(model.getBodyId("chest"),JointModelRX(),SE3::Random(),Inertia::Random(),"rarm1");
  model.addBody(model.getBodyId("rarm1"),JointModelRX(),SE3::Random(),Inertia::Random(),"rarm2");
  model.addBody(model.getBodyId("rarm2"),JointModelRX(),SE3::Random(),Inertia::Random(),"rarm3");

  model.addBody(model.getBodyId("chest"),JointModelRX(),SE3::Random(),Inertia::Random(),"larm1");
  model.addBody(model.getBodyId("larm1"),JointModelRX(),SE3::Random(),Inertia::Random(),"larm2");
  model.addBody(model.getBodyId("larm2"),JointModelRX(),SE3::Random(),Inertia::Random(),"larm3");

  std::cout << model << std::endl;

  se3::Data data(model);
  VectorXd q = VectorXd::Zero(model.nq);
  crba2(model,data,q);
  
  std::cout << "M = [\n" << data.M << "];" << std::endl;
  for(int i=0;i<model.nv;++i)
    for(int j=i;j<model.nv;++j)
      {
	data.M(i,j) = round(data.M(i,j))+1;
	data.M(j,i) = data.M(i,j);
      }
  std::cout << "M = [\n" << data.M << "];" << std::endl;
	
  //cholesky(model,data)

  return 0;
}
