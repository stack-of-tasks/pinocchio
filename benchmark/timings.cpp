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

  StackTicToc timer(StackTicToc::US);
  const int NBT = 1000*100;
  se3::Model model;

  std::string filename = "../models/simple_humanoid.urdf";
  if(argc>1) filename = argv[1];
  if( filename == "HS") 
    se3::urdf::buildModels::humanoidSimple(model,true);
  else if( filename == "H2" )
    se3::urdf::buildModels::humanoid2d(model);
  else
    model = se3::urdf::buildModel(filename,true);
  std::cout << "nq = " << model.nq << std::endl;

  se3::Data data(model);
  VectorXd q = VectorXd::Random(model.nq);
  VectorXd qdot = VectorXd::Random(model.nv);
  VectorXd qddot = VectorXd::Random(model.nv);
 
  timer.tic();
  SMOOTH(NBT)
    {
      rnea(model,data,q,qdot,qddot);
    }
  std::cout << "RNEA = \t\t"; timer.toc(std::cout,NBT);
 
  timer.tic();
  SMOOTH(NBT)
    {
      crba(model,data,q);
    }
  std::cout << "CRBA = \t\t"; timer.toc(std::cout,NBT);
 
  timer.tic();
  SMOOTH(NBT)
    {
      cholesky::decompose(model,data);
    }
  std::cout << "Cholesky = \t"; timer.toc(std::cout,NBT);
 
  timer.tic();
  SMOOTH(NBT)
    {
      computeJacobians(model,data,q);
    }
  std::cout << "Jacobian = \t"; timer.toc(std::cout,NBT);

  timer.tic();
  SMOOTH(NBT)
    {
      jacobianCenterOfMass(model,data,q,false);
    }
  std::cout << "COM+Jcom = \t"; timer.toc(std::cout,NBT);

  std::cout << "--" << std::endl;
  return 0;
}
