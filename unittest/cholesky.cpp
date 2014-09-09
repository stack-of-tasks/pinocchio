#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/multibody/joint.hpp"
#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/cholesky.hpp"

#include <iostream>
#ifdef NDEBUG
#  include <Eigen/Cholesky>
#endif

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

  if( 1 ) // Without FF
    {
       model.addBody(model.getBodyId("universe"),JointModelRX(),SE3::Random(),Inertia::Random(),"ff1");
       model.addBody(model.getBodyId("ff1"),JointModelRX(),SE3::Random(),Inertia::Random(),"ff2");
       model.addBody(model.getBodyId("ff2"),JointModelRX(),SE3::Random(),Inertia::Random(),"ff3");
       model.addBody(model.getBodyId("ff3"),JointModelRX(),SE3::Random(),Inertia::Random(),"ff4");
       model.addBody(model.getBodyId("ff4"),JointModelRX(),SE3::Random(),Inertia::Random(),"ff5");
       model.addBody(model.getBodyId("ff5"),JointModelRX(),SE3::Random(),Inertia::Random(),"root");
    }
  else
    model.addBody(model.getBodyId("universe"),JointModelFreeFlyer(),SE3::Random(),Inertia::Random(),"root");

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

  // model.addBody(model.getBodyId("universe"),JointModelRX(),SE3::Random(),Inertia::Random(),"ff1");
  // model.addBody(model.getBodyId("ff1"),JointModelRX(),SE3::Random(),Inertia::Random(),"root");

  // model.addBody(model.getBodyId("root"),JointModelRX(),SE3::Random(),Inertia::Random(),"lleg1");
  // model.addBody(model.getBodyId("lleg1"),JointModelRX(),SE3::Random(),Inertia::Random(),"lleg2");

  // model.addBody(model.getBodyId("root"),JointModelRX(),SE3::Random(),Inertia::Random(),"rleg1");
  // model.addBody(model.getBodyId("rleg1"),JointModelRX(),SE3::Random(),Inertia::Random(),"rleg2");

  // model.addBody(model.getBodyId("root"),JointModelRX(),SE3::Random(),Inertia::Random(),"torso1");
  // model.addBody(model.getBodyId("torso1"),JointModelRX(),SE3::Random(),Inertia::Random(),"chest");

  // model.addBody(model.getBodyId("chest"),JointModelRX(),SE3::Random(),Inertia::Random(),"rarm1");
  // model.addBody(model.getBodyId("rarm1"),JointModelRX(),SE3::Random(),Inertia::Random(),"rarm2");

  // model.addBody(model.getBodyId("chest"),JointModelRX(),SE3::Random(),Inertia::Random(),"larm1");
  // model.addBody(model.getBodyId("larm1"),JointModelRX(),SE3::Random(),Inertia::Random(),"larm2");

  // std::cout << model << std::endl;

  se3::Data data(model);
  VectorXd q = VectorXd::Zero(model.nq);
  crba(model,data,q);
  	
  StackTicToc timer(StackTicToc::US); timer.tic();
#ifdef NDEBUG
  SMOOTH(1000)
#endif
    {
      cholesky(model,data);
    }
  timer.toc(std::cout,1000);


  for(int i=0;i<model.nv;++i)
    for(int j=0;j<model.nv;++j)
      {
	if(isnan(data.M(i,j))) data.M(i,j) = 0;
	data.M(j,i) = data.M(i,j);
      }


#ifdef NDEBUG
  Eigen::Matrix<double,33,33> M33 = data.M;
  timer.tic();
  SMOOTH(1000)
    {
      M33 = data.M;
      //Eigen::LDLT <Eigen::MatrixXd> Mchol(data.M);
      CholeskyOuterLoopStep::udut<33>(M33);
    }
  std::cout << "\t\t"; timer.toc(std::cout,1000);

  timer.tic();
  SMOOTH(1000)
    {
      Eigen::LDLT <Eigen::MatrixXd> Mchol(data.M);
    }
  std::cout << "\t\t"; timer.toc(std::cout,1000);
#endif


  data.U.triangularView<Eigen::StrictlyLower>().fill(0);
  data.U.diagonal().fill(1);

#ifndef NDEBUG
  std::cout << "M = [\n" << data.M << "];" << std::endl;
  std::cout << "U = [\n" << data.U << "];" << std::endl;
  std::cout << "D = [\n" << data.D.transpose() << "];" << std::endl;
  // std::cout << "UDU = [\n" << (data.U*data.D.asDiagonal()*data.U.transpose()) << "];" << std::endl;
#endif
      
  assert((data.U*data.D.asDiagonal()*data.U.transpose()).isApprox(data.M));


  return 0;
}
