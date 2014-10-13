#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/multibody/joint.hpp"
#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/cholesky.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/multibody/parser/urdf.hpp"
#include "pinocchio/multibody/parser/sample-models.hpp"

#include <iostream>

#include "pinocchio/tools/timer.hpp"


int main(int argc, const char ** argv)
{
  using namespace Eigen;
  using namespace se3;
  
  se3::Model model;

  std::string filename = "../models/simple_humanoid.urdf";
  if(argc>1) filename = argv[1];
  if( filename == "HS") 
    se3::buildModels::humanoidSimple(model,true);
  else if( filename == "H2" )
    se3::buildModels::humanoid2d(model);
  else
    model = se3::buildModel(filename,true);
  std::cout << "nq = " << model.nq << std::endl;

  se3::Data data(model);
  VectorXd q = VectorXd::Random(model.nq);
  VectorXd qdot = VectorXd::Random(model.nv);
  VectorXd qddot = VectorXd::Random(model.nv);
  
  double duration = 0.;
  int num_iterations = 1e5;
  StackTicToc timer(StackTicToc::US);
  
  
  
  for(int i = 0; i < num_iterations; i++)
  {
    q = VectorXd::Random(model.nq);
    qdot = VectorXd::Random(model.nv);
    qddot = VectorXd::Random(model.nv);
    
    timer.tic();
    rnea(model,data,q,qdot,qddot);
    duration += timer.toc (StackTicToc::US);
  }
  std::cout << "RNEA = \t\t" << duration / (double) num_iterations << " us\n";
 
  duration = 0.;
  for(int i = 0; i < num_iterations; i++)
  {
    q = VectorXd::Random(model.nq);
    
    timer.tic();
    crba(model,data,q);
    duration += timer.toc (StackTicToc::US);
  }
  std::cout << "CRBA = \t\t" << duration / (double) num_iterations << " us\n";
 
  duration = 0.;
  for(int i = 0; i < num_iterations; i++)
  {
    q = VectorXd::Random(model.nq);
    
    crba(model,data,q);
    
    timer.tic();
    cholesky::decompose(model,data);
    duration += timer.toc (StackTicToc::US);
  }
  std::cout << "Cholesky = \t\t" << duration / (double) num_iterations << " us\n";
 
  duration = 0.;
  for(int i = 0; i < num_iterations; i++)
  {
    q = VectorXd::Random(model.nq);
    
    timer.tic();
    computeJacobians(model,data,q);
    duration += timer.toc (StackTicToc::US);
  }
  std::cout << "Jacobian = \t\t" << duration / (double) num_iterations << " us\n";

  duration = 0.;
  for(int i = 0; i < num_iterations; i++)
  {
    q = VectorXd::Random(model.nq);
    
    timer.tic();
    jacobianCenterOfMass(model,data,q,false);
    duration += timer.toc (StackTicToc::US);
  }
  std::cout << "COM+Jcom = \t\t" << duration / (double) num_iterations << " us\n";

  std::cout << "--" << std::endl;
  return 0;
}
