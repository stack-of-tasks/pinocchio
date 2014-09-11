#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/multibody/joint.hpp"
#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/multibody/parser/urdf.hpp"

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

  std::string filename = "/home/nmansard/src/metapod/data/simple_arm.urdf";
  if(argc>1) filename = argv[1];
  model = se3::buildModel(filename,argc>1);

  se3::Data data(model);
  VectorXd q = VectorXd::Zero(model.nq);
 
  StackTicToc timer(StackTicToc::US); timer.tic();
  SMOOTH(1000*100)
    {
      crba(model,data,q);
    }
  timer.toc(std::cout,1000*100);

#ifndef NDEBUG
  std::cout << "Mcrb = [ " << data.M << "  ];" << std::endl;

#ifdef __se3_rnea_hpp__    
  /* Joint inertia from iterative crba. */
  {
    Eigen::MatrixXd M(model.nv,model.nv);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);
    const Eigen::VectorXd bias = rnea(model,data,q,v,a);

    for(int i=0;i<model.nv;++i)
      { 
	M.col(i) = rnea(model,data,q,v,Eigen::VectorXd::Unit(model.nv,i)) - bias;
      }
    std::cout << "Mrne = [  " << M << " ]; " << std::endl;
  }	
#endif // ifdef __se3_rnea_hpp__    
#endif // ifndef NDEBUG


  return 0;
}
