#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/multibody/joint.hpp"
#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/non-linear-effects.hpp"
#include "pinocchio/algorithm/cholesky.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/simulation/compute-all-terms.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/multibody/parser/urdf.hpp"
#include "pinocchio/multibody/parser/sample-models.hpp"

#include <iostream>

#include "pinocchio/tools/timer.hpp"

#include <Eigen/StdVector>
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::VectorXd)

int main(int argc, const char ** argv)
{
  using namespace Eigen;
  using namespace se3;

  StackTicToc timer(StackTicToc::US);
  #ifdef NDEBUG
  const int NBT = 1000*100;
  #else
    const int NBT = 1;
    std::cout << "(the time score in debug mode is not relevant) " << std::endl;
  #endif
    
  se3::Model model;

  std::string filename = PINOCCHIO_SOURCE_DIR"/models/simple_humanoid.urdf";
  if(argc>1) filename = argv[1];
  if( filename == "HS") 
    se3::buildModels::humanoidSimple(model,true);
  else if( filename == "H2" )
    se3::buildModels::humanoid2d(model);
  else
    model = se3::urdf::buildModel(filename,true);
  std::cout << "nq = " << model.nq << std::endl;

  se3::Data data(model);
  VectorXd q = VectorXd::Random(model.nq);
  VectorXd qdot = VectorXd::Random(model.nv);
  VectorXd qddot = VectorXd::Random(model.nv);

  std::vector<VectorXd> qs     (NBT);
  std::vector<VectorXd> qdots  (NBT);
  std::vector<VectorXd> qddots (NBT);
  for(size_t i=0;i<NBT;++i)
    {
      qs[i]     = Eigen::VectorXd::Random(model.nq);
      qs[i].segment<4>(3) /= qs[i].segment<4>(3).norm();
      qdots[i]  = Eigen::VectorXd::Random(model.nv);
      qddots[i] = Eigen::VectorXd::Random(model.nv);
    }

 
  timer.tic();
  SMOOTH(NBT)
    {
      rnea(model,data,qs[_smooth],qdots[_smooth],qddots[_smooth]);
    }
  std::cout << "RNEA = \t\t"; timer.toc(std::cout,NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    nonLinearEffects(model,data,qs[_smooth],qdots[_smooth]);
  }
  std::cout << "NLE = \t\t"; timer.toc(std::cout,NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    rnea(model,data,qs[_smooth],qdots[_smooth],Eigen::VectorXd::Zero(model.nv));
  }
  std::cout << "NLE via RNEA = \t\t"; timer.toc(std::cout,NBT);
 
  timer.tic();
  SMOOTH(NBT)
    {
      crba(model,data,qs[_smooth]);
    }
  std::cout << "CRBA = \t\t"; timer.toc(std::cout,NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    computeAllTerms(model,data,qs[_smooth],qdots[_smooth]);
  }
  std::cout << "computeAllTerms = \t\t"; timer.toc(std::cout,NBT);
  
  double total = 0;
  SMOOTH(NBT)
    {
      crba(model,data,qs[_smooth]);
      timer.tic();
      cholesky::decompose(model,data);
      total += timer.toc(timer.DEFAULT_UNIT);
    }
  std::cout << "Cholesky = \t" << (total/NBT) 
	    << " " << timer.unitName(timer.DEFAULT_UNIT) <<std::endl;
 
  timer.tic();
  SMOOTH(NBT)
    {
      computeJacobians(model,data,qs[_smooth]);
    }
  std::cout << "Jacobian = \t"; timer.toc(std::cout,NBT);

  timer.tic();
  SMOOTH(NBT)
    {
      jacobianCenterOfMass(model,data,qs[_smooth],false);
    }
  std::cout << "COM+Jcom = \t"; timer.toc(std::cout,NBT);
  
  timer.tic();
  SMOOTH(NBT)
  {
    centerOfMassAcceleration(model,data,qs[_smooth], qdots[_smooth], qddots[_smooth], false);
  }
  std::cout << "COM+vCOM+aCOM = \t"; timer.toc(std::cout,NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    geometry(model,data,qs[_smooth]);
  }
  std::cout << "Geometry = \t"; timer.toc(std::cout,NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    kinematics(model,data,qs[_smooth],qdots[_smooth]);
  }
  std::cout << "Kinematics = \t"; timer.toc(std::cout,NBT);
  
  timer.tic();
  SMOOTH(NBT)
  {
    dynamics(model,data,qs[_smooth],qdots[_smooth], qddots[_smooth]);
  }
  std::cout << "Dynamics = \t"; timer.toc(std::cout,NBT);

  std::cout << "--" << std::endl;
  return 0;
}
