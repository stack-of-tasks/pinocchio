//
// Copyright (c) 2018 CNRS
//

#include "pinocchio/codegen/cppadcg.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/cholesky.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/container/aligned-vector.hpp"

#include "pinocchio/codegen/code-generator-algo.hpp"

#include <iostream>

#include "pinocchio/utils/timer.hpp"

int main(int argc, const char ** argv)
{
  using namespace Eigen;
  using namespace pinocchio;

  PinocchioTicToc timer(PinocchioTicToc::US);
  #ifdef NDEBUG
  const int NBT = 1000*100;
  #else
    const int NBT = 1;
    std::cout << "(the time score in debug mode is not relevant) " << std::endl;
  #endif
    
  pinocchio::Model model;
  std::string filename = PINOCCHIO_MODEL_DIR + std::string("/simple_humanoid.urdf");
  if(argc>1) filename = argv[1];
  
  bool with_ff = true;
  if(argc>2)
  {
    const std::string ff_option = argv[2];
    if(ff_option == "-no-ff")
      with_ff = false;
  }
  
  if( filename == "H")
    pinocchio::buildModels::humanoidRandom(model,true);
  else
    if(with_ff)
      pinocchio::urdf::buildModel(filename,JointModelFreeFlyer(),model);
    else
      pinocchio::urdf::buildModel(filename,model);
  
  std::cout << "nq = " << model.nq << std::endl;
  std::cout << "nv = " << model.nv << std::endl;
  std::cout << "--" << std::endl;

  pinocchio::Data data(model);
  VectorXd qmax = Eigen::VectorXd::Ones(model.nq);
  
  CodeGenRNEA<double> rnea_code_gen(model);
  rnea_code_gen.initLib();
  rnea_code_gen.loadLib();
  
  CodeGenABA<double> aba_code_gen(model);
  aba_code_gen.initLib();
  aba_code_gen.loadLib();
  
  CodeGenCRBA<double> crba_code_gen(model);
  crba_code_gen.initLib();
  crba_code_gen.loadLib();
  
  CodeGenMinv<double> minv_code_gen(model);
  minv_code_gen.initLib();
  minv_code_gen.loadLib();
  
  CodeGenRNEADerivatives<double> rnea_derivatives_code_gen(model);
  rnea_derivatives_code_gen.initLib();
  rnea_derivatives_code_gen.loadLib();
  
  CodeGenABADerivatives<double> aba_derivatives_code_gen(model);
  aba_derivatives_code_gen.initLib();
  aba_derivatives_code_gen.loadLib();

  pinocchio::container::aligned_vector<VectorXd> qs     (NBT);
  pinocchio::container::aligned_vector<VectorXd> qdots  (NBT);
  pinocchio::container::aligned_vector<VectorXd> qddots (NBT);
  pinocchio::container::aligned_vector<VectorXd> taus (NBT);
  
  for(size_t i=0;i<NBT;++i)
  {
    qs[i]     = randomConfiguration(model,-qmax,qmax);
    qdots[i]  = Eigen::VectorXd::Random(model.nv);
    qddots[i] = Eigen::VectorXd::Random(model.nv);
    taus[i] = Eigen::VectorXd::Random(model.nv);
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
    rnea_code_gen.evalFunction(qs[_smooth],qdots[_smooth],qddots[_smooth]);
  }
  std::cout << "RNEA generated = \t\t"; timer.toc(std::cout,NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    rnea_code_gen.evalJacobian(qs[_smooth],qdots[_smooth],qddots[_smooth]);
  }
  std::cout << "RNEA partial derivatives auto diff + code gen = \t\t"; timer.toc(std::cout,NBT);
  
  timer.tic();
  SMOOTH(NBT)
  {
    rnea_derivatives_code_gen.evalFunction(qs[_smooth],qdots[_smooth],qddots[_smooth]);
  }
  std::cout << "RNEA partial derivatives code gen = \t\t"; timer.toc(std::cout,NBT);
  
  timer.tic();
  SMOOTH(NBT)
  {
    aba(model,data,qs[_smooth],qdots[_smooth],taus[_smooth]);
  }
  timer.toc();
  
  timer.tic();
  SMOOTH(NBT)
  {
    crba(model,data,qs[_smooth]);
  }
  std::cout << "CRBA = \t\t"; timer.toc(std::cout,NBT);
  
  timer.tic();
  SMOOTH(NBT)
  {
    crba_code_gen.evalFunction(qs[_smooth]);
  }
  std::cout << "CRBA generated = \t\t"; timer.toc(std::cout,NBT);
  
  timer.tic();
  SMOOTH(NBT)
  {
    computeMinverse(model,data,qs[_smooth]);
  }
  std::cout << "Minv = \t\t"; timer.toc(std::cout,NBT);
  
  timer.tic();
  SMOOTH(NBT)
  {
    minv_code_gen.evalFunction(qs[_smooth]);
  }
  std::cout << "Minv generated = \t\t"; timer.toc(std::cout,NBT);
  
  timer.tic();
  SMOOTH(NBT)
  {
    aba(model,data,qs[_smooth],qdots[_smooth],taus[_smooth]);
  }
  std::cout << "ABA = \t\t"; timer.toc(std::cout,NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    aba_code_gen.evalFunction(qs[_smooth],qdots[_smooth],taus[_smooth]);
  }
  std::cout << "ABA generated = \t\t"; timer.toc(std::cout,NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    aba_code_gen.evalJacobian(qs[_smooth],qdots[_smooth],taus[_smooth]);
  }
  std::cout << "ABA partial derivatives auto diff + code gen = \t\t"; timer.toc(std::cout,NBT);
  
  timer.tic();
  SMOOTH(NBT)
  {
    aba_derivatives_code_gen.evalFunction(qs[_smooth],qdots[_smooth],qddots[_smooth]);
  }
  std::cout << "ABA partial derivatives code gen = \t\t"; timer.toc(std::cout,NBT);
  
  return 0;
}
