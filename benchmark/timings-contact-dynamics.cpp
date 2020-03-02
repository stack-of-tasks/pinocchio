//
// Copyright (c) 2019-2020 INRIA
//

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/contact-dynamics.hpp"
#include "pinocchio/algorithm/cholesky.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/sample-models.hpp"

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
  
  // Build model
  Model model;
  
  std::string filename = PINOCCHIO_MODEL_DIR + std::string("/simple_humanoid.urdf");
  if(argc>1) filename = argv[1];
  bool with_ff = true;
  
  if(argc>2)
  {
    const std::string ff_option = argv[2];
    if(ff_option == "-no-ff")
      with_ff = false;
  }
  
  if( filename == "HS")
    buildModels::humanoidRandom(model,true);
  else
    if(with_ff)
      pinocchio::urdf::buildModel(filename,JointModelFreeFlyer(),model);
  //      pinocchio::urdf::buildModel(filename,JointModelRX(),model);
    else
      pinocchio::urdf::buildModel(filename,model);
  
  const std::string RA = "RARM_LINK6";
  const std::string LA = "LARM_LINK6";
  const std::string RF = "RLEG_LINK6";
  const std::string LF = "LLEG_LINK6";
  
  RigidContactModel ci_RF_6D(CONTACT_6D,model.getFrameId(RF),WORLD);
  RigidContactModel ci_RF_3D(CONTACT_3D,model.getFrameId(RF),WORLD);
  
  RigidContactModel ci_LF_6D(CONTACT_6D,model.getFrameId(LF),WORLD);
  RigidContactModel ci_LF_3D(CONTACT_3D,model.getFrameId(LF),WORLD);
  
  RigidContactModel ci_RA_3D(CONTACT_3D,model.getFrameId(RA),WORLD);
  RigidContactModel ci_LA_3D(CONTACT_3D,model.getFrameId(LA),WORLD);
  
  // Define contact infos structure
  const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_infos_empty;
  cholesky::ContactCholeskyDecomposition contact_chol_empty(model,contact_infos_empty);
  
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_infos_6D;
  contact_infos_6D.push_back(ci_RF_6D);
  cholesky::ContactCholeskyDecomposition contact_chol_6D(model,contact_infos_6D);
  
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_infos_6D6D;
  contact_infos_6D6D.push_back(ci_RF_6D);
  contact_infos_6D6D.push_back(ci_LF_6D);
  cholesky::ContactCholeskyDecomposition contact_chol_6D6D(model,contact_infos_6D6D);
  
  std::cout << "nq = " << model.nq << std::endl;
  std::cout << "nv = " << model.nv << std::endl;
  std::cout << "--" << std::endl;
  
  Data data(model);
  VectorXd qmax = Eigen::VectorXd::Ones(model.nq);
  
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(VectorXd) qs(NBT);
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(VectorXd) qdots(NBT);
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(VectorXd) qddots(NBT);
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(VectorXd) taus(NBT);
  
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
    aba(model,data,qs[_smooth],qdots[_smooth],taus[_smooth]);
  }
  std::cout << "ABA = \t\t"; timer.toc(std::cout,NBT);
  
  double total_time = 0;
  SMOOTH(NBT)
  {
    crba(model,data,qs[_smooth]);
    timer.tic();
    cholesky::decompose(model,data);
    total_time += timer.toc(timer.DEFAULT_UNIT);
  }
  std::cout << "Sparse Cholesky = \t\t" << (total_time/NBT)
  << " " << timer.unitName(timer.DEFAULT_UNIT) <<std::endl;
  
  total_time = 0;
  SMOOTH(NBT)
  {
    computeAllTerms(model,data,qs[_smooth],qdots[_smooth]);
    timer.tic();
    contact_chol_empty.compute(model,data,contact_infos_empty);
    total_time += timer.toc(timer.DEFAULT_UNIT);
  }
  std::cout << "contactCholesky {} = \t\t" << (total_time/NBT)
  << " " << timer.unitName(timer.DEFAULT_UNIT) <<std::endl;
  
  total_time = 0;
  MatrixXd H_inverse(contact_chol_empty.size(),contact_chol_empty.size());
  SMOOTH(NBT)
  {
    computeAllTerms(model,data,qs[_smooth],qdots[_smooth]);
    contact_chol_empty.compute(model,data,contact_infos_empty);
    timer.tic();
    contact_chol_empty.inverse(H_inverse);
    total_time += timer.toc(timer.DEFAULT_UNIT);
  }
  std::cout << "contactCholeskyInverse {} = \t\t" << (total_time/NBT)
  << " " << timer.unitName(timer.DEFAULT_UNIT) <<std::endl;
  
  initContactDynamics(model,data,contact_infos_empty);
  timer.tic();
  SMOOTH(NBT)
  {
    contactDynamics(model,data,qs[_smooth],qdots[_smooth],taus[_smooth],contact_infos_empty);
  }
  std::cout << "contactDynamics {} = \t\t"; timer.toc(std::cout,NBT);
  
  total_time = 0;
  SMOOTH(NBT)
  {
    computeAllTerms(model,data,qs[_smooth],qdots[_smooth]);
    timer.tic();
    contact_chol_6D.compute(model,data,contact_infos_6D);
    total_time += timer.toc(timer.DEFAULT_UNIT);
  }
  std::cout << "contactCholesky {6D} = \t\t" << (total_time/NBT)
  << " " << timer.unitName(timer.DEFAULT_UNIT) <<std::endl;
  
  total_time = 0;
  H_inverse.resize(contact_chol_6D.size(),contact_chol_6D.size());
  SMOOTH(NBT)
  {
    computeAllTerms(model,data,qs[_smooth],qdots[_smooth]);
    contact_chol_6D.compute(model,data,contact_infos_6D);
    timer.tic();
    contact_chol_6D.inverse(H_inverse);
    total_time += timer.toc(timer.DEFAULT_UNIT);
  }
  std::cout << "contactCholeskyInverse {6D} = \t\t" << (total_time/NBT)
  << " " << timer.unitName(timer.DEFAULT_UNIT) <<std::endl;
  
  MatrixXd J(contact_chol_6D.constraintDim(),model.nv);
  J.setZero();
  MatrixXd MJtJ_inv(model.nv+contact_chol_6D.constraintDim(),
                    model.nv+contact_chol_6D.constraintDim());
  MJtJ_inv.setZero();
  
  VectorXd gamma(contact_chol_6D.constraintDim());
  gamma.setZero();
  
  total_time = 0;
  SMOOTH(NBT)
  {
    computeAllTerms(model,data,qs[_smooth],qdots[_smooth]);
    timer.tic();
    getFrameJacobian(model,data,ci_RF_6D.frame_id,ci_RF_6D.reference_frame,J.middleRows<6>(0));
    total_time += timer.toc(timer.DEFAULT_UNIT);
    
    forwardDynamics(model,data,qs[_smooth], qdots[_smooth], taus[_smooth], J, gamma);
    
    timer.tic();
    cholesky::decompose(model,data);
    getKKTContactDynamicMatrixInverse(model,data,J,MJtJ_inv);
    total_time += timer.toc(timer.DEFAULT_UNIT);
  }
  std::cout << "KKTContactDynamicMatrixInverse {6D} = \t\t" << (total_time/NBT)
  << " " << timer.unitName(timer.DEFAULT_UNIT) <<std::endl;
  
  initContactDynamics(model,data,contact_infos_6D);
  timer.tic();
  SMOOTH(NBT)
  {
    contactDynamics(model,data,qs[_smooth],qdots[_smooth],taus[_smooth],contact_infos_6D);
  }
  std::cout << "contactDynamics {6D} = \t\t"; timer.toc(std::cout,NBT);
  
  total_time = 0;
  SMOOTH(NBT)
  {
    computeAllTerms(model,data,qs[_smooth],qdots[_smooth]);
    timer.tic();
    contact_chol_6D6D.compute(model,data,contact_infos_6D6D);
    total_time += timer.toc(timer.DEFAULT_UNIT);
  }
  std::cout << "contactCholesky {6D,6D} = \t\t" << (total_time/NBT)
  << " " << timer.unitName(timer.DEFAULT_UNIT) <<std::endl;
  
  total_time = 0;
  H_inverse.resize(contact_chol_6D6D.size(),contact_chol_6D6D.size());
  SMOOTH(NBT)
  {
    computeAllTerms(model,data,qs[_smooth],qdots[_smooth]);
    contact_chol_6D6D.compute(model,data,contact_infos_6D6D);
    timer.tic();
    contact_chol_6D6D.inverse(H_inverse);
    total_time += timer.toc(timer.DEFAULT_UNIT);
  }
  std::cout << "contactCholeskyInverse {6D,6D} = \t\t" << (total_time/NBT)
  << " " << timer.unitName(timer.DEFAULT_UNIT) <<std::endl;
  
  J.resize(contact_chol_6D6D.constraintDim(),model.nv);
  J.setZero();
  MJtJ_inv.resize(model.nv+contact_chol_6D6D.constraintDim(),
                  model.nv+contact_chol_6D6D.constraintDim());
  MJtJ_inv.setZero();
  
  gamma.resize(contact_chol_6D6D.constraintDim());
  gamma.setZero();
  
  total_time = 0;
  SMOOTH(NBT)
  {
    computeAllTerms(model,data,qs[_smooth],qdots[_smooth]);
    getFrameJacobian(model,data,ci_RF_6D.frame_id,ci_RF_6D.reference_frame,J.middleRows<6>(0));
    getFrameJacobian(model,data,ci_LF_6D.frame_id,ci_LF_6D.reference_frame,J.middleRows<6>(6));
    
    forwardDynamics(model,data,qs[_smooth], qdots[_smooth], taus[_smooth], J, gamma);
    
    timer.tic();
    cholesky::decompose(model,data);
    getKKTContactDynamicMatrixInverse(model,data,J,MJtJ_inv);
    total_time += timer.toc(timer.DEFAULT_UNIT);
  }
  std::cout << "KKTContactDynamicMatrixInverse {6D,6D} = \t\t" << (total_time/NBT)
  << " " << timer.unitName(timer.DEFAULT_UNIT) <<std::endl;
  
  initContactDynamics(model,data,contact_infos_6D6D);
  timer.tic();
  SMOOTH(NBT)
  {
    contactDynamics(model,data,qs[_smooth],qdots[_smooth],taus[_smooth],contact_infos_6D6D);
  }
  std::cout << "contactDynamics {6D,6D} = \t\t"; timer.toc(std::cout,NBT);
  
  J.setZero();
  timer.tic();
  SMOOTH(NBT)
  {
    computeAllTerms(model,data,qs[_smooth],qdots[_smooth]);
    getFrameJacobian(model,data,ci_RF_6D.frame_id,ci_RF_6D.reference_frame,J.middleRows<6>(0));
    getFrameJacobian(model,data,ci_LF_6D.frame_id,ci_LF_6D.reference_frame,J.middleRows<6>(6));
    forwardDynamics(model,data,taus[_smooth],J,gamma);
  }
  std::cout << "constrainedDynamics {6D,6D} = \t\t"; timer.toc(std::cout,NBT);
  
  std::cout << "--" << std::endl;
  
  return 0;
}

