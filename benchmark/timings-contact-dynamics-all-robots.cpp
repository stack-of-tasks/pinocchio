//
// Copyright (c) 2019-2020 INRIA
//

#include <example-robot-data/path.hpp>
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/contact-dynamics.hpp"
#include "pinocchio/algorithm/cholesky.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/sample-models.hpp"

#include <iostream>

#include "pinocchio/utils/timer.hpp"
#include "pinocchio/utils/file-io.hpp"

#define STDDEV(vec) std::sqrt(((vec - vec.mean())).square().sum() / (static_cast<double>(vec.size()) - 1))
#define AVG(vec) (vec.mean())

void print_benchmark(const std::string& model_name,
                     const pinocchio::Model& model,
                     const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidContactModel)& contact_models)
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

  /******************************* create csv file *******************************/
  const std::string csv_filename = "/tmp/" + model_name + "_" + std::to_string(model.nv) + "DoF.bench";
  CsvStream csv(csv_filename);
  csv << "fn_name"
      << "contacts"
      << "mean"
      << "stddev"
      << "max"
      << "min"
      << csv.endl;

  Eigen::ArrayXd duration(NBT);
  double avg;
  double stddev;
  /***********************************************************/

  
  for(std::size_t nc=0; nc<=contact_models.size(); nc++)
  {
    // Define contact infos structure
    PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_models_subset;
    PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) contact_datas_subset;
    std::string contact_info_string;
    int nconstraint = 0;
    for(std::size_t j=0;j<nc;j++)
    {
      contact_models_subset.push_back(contact_models[j]);
      contact_datas_subset.push_back(RigidContactData(contact_models[j]));
      contact_info_string += std::to_string(contact_models[j].size())+"D";
      nconstraint += contact_models[j].size();
    }
    cholesky::ContactCholeskyDecomposition contact_chol_subset(model,contact_models_subset);

    ProximalSettings prox_settings;
    prox_settings.max_iter = 10;
    prox_settings.mu = 1e8;
    double mu = 1e-5;

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
  
    duration.setZero();
    SMOOTH(NBT)
    {
      timer.tic();
      contactABA(model,data,qs[_smooth],qdots[_smooth],
                 taus[_smooth],contact_models_subset,contact_datas_subset);
      duration[_smooth] = timer.toc(timer.DEFAULT_UNIT);
    }
    avg = AVG(duration);
    stddev = STDDEV(duration);
    std::cout << "contact ABA"<<"{"<<contact_info_string <<"}"<<"= \t\t"<< avg
              << " " << timer.unitName(timer.DEFAULT_UNIT) <<std::endl;
    csv << "contactABA" << nconstraint
        << avg << stddev << duration.maxCoeff() << duration.minCoeff() << csv.endl;

    
    
    duration.setZero();
    SMOOTH(NBT)
    {
      computeAllTerms(model,data,qs[_smooth],qdots[_smooth]);
      timer.tic();
      contact_chol_subset.compute(model,data,contact_models_subset,
                                  contact_datas_subset, mu);
      duration[_smooth] = timer.toc(timer.DEFAULT_UNIT);
    }
    avg = AVG(duration);
    stddev = STDDEV(duration);
    std::cout << "contactCholesky {"<<contact_info_string<<"} = \t\t" << avg
              << " " << timer.unitName(timer.DEFAULT_UNIT) <<std::endl;

    csv << "contactCholesky" << nconstraint
        << avg << stddev << duration.maxCoeff() << duration.minCoeff() << csv.endl;

    
    duration.setZero();
    Eigen::LDLT<Eigen::MatrixXd> MJtJ_ldlt(contact_chol_subset.size());
    Eigen::MatrixXd MJtJ(Eigen::MatrixXd::Zero(contact_chol_subset.size(),
                                               contact_chol_subset.size()));
    SMOOTH(NBT)
    {
      computeAllTerms(model,data,qs[_smooth],qdots[_smooth]);
      contact_chol_subset.compute(model,data,contact_models_subset,contact_datas_subset, mu);
      contact_chol_subset.matrix(MJtJ);
      timer.tic();
      MJtJ_ldlt.compute(MJtJ);
      duration[_smooth] = timer.toc(timer.DEFAULT_UNIT);
    }
    avg = AVG(duration);
    stddev = STDDEV(duration);    
    std::cout << "Dense Eigen contactCholesky {"<<contact_info_string<<"} = \t" << avg
              << " " << timer.unitName(timer.DEFAULT_UNIT) <<std::endl;
    csv << "eigenCholesky" << nconstraint
        << avg << stddev << duration.maxCoeff() << duration.minCoeff() << csv.endl;

    
    duration.setZero();
    MatrixXd H_inverse(contact_chol_subset.size(),contact_chol_subset.size());
    SMOOTH(NBT)
    {
      computeAllTerms(model,data,qs[_smooth],qdots[_smooth]);
      contact_chol_subset.compute(model,data,contact_models_subset,contact_datas_subset,mu);
      timer.tic();
      contact_chol_subset.inverse(H_inverse);
      duration[_smooth] = timer.toc(timer.DEFAULT_UNIT);
    }
    avg = AVG(duration);
    stddev = STDDEV(duration);        
    std::cout << "contactCholeskyInverse {"<<contact_info_string<<"} = \t\t" << avg
              << " " << timer.unitName(timer.DEFAULT_UNIT) <<std::endl;
    csv << "contactCholeskyInverse" << nconstraint
        << avg << stddev << duration.maxCoeff() << duration.minCoeff() << csv.endl;
  
    MatrixXd J(contact_chol_subset.constraintDim(),model.nv);
    J.setZero();
    MatrixXd MJtJ_inv(model.nv+contact_chol_subset.constraintDim(),
                      model.nv+contact_chol_subset.constraintDim());
    MJtJ_inv.setZero();
  
    VectorXd gamma(contact_chol_subset.constraintDim());
    gamma.setZero();

    duration.setZero();
    SMOOTH(NBT)
    {
      computeAllTerms(model,data,qs[_smooth],qdots[_smooth]);
      std::size_t running_nc = 0;
      MatrixXd Jout(6,model.nv);
      for(std::size_t k=0;k<contact_models_subset.size();++k)
      {
        Jout.setZero();
        int contact_dim = contact_models[k].size();
        getJointJacobian(model,data,contact_models_subset[k].joint1_id,
                         contact_models_subset[k].reference_frame,
                         Jout);
        J.middleRows(running_nc,contact_dim) = Jout.topRows(contact_dim);
        running_nc +=contact_dim;
      }
      forwardDynamics(model,data,qs[_smooth], qdots[_smooth], taus[_smooth], J, gamma);
      
      timer.tic();
      cholesky::decompose(model,data);
      getKKTContactDynamicMatrixInverse(model,data,J,MJtJ_inv);
      duration[_smooth] = timer.toc(timer.DEFAULT_UNIT);
    }
    avg = AVG(duration);
    stddev = STDDEV(duration);
    std::cout << "KKTContactDynamicMatrixInverse {"<<contact_info_string<<"} = \t\t" << avg
              << " " << timer.unitName(timer.DEFAULT_UNIT) <<std::endl;    

    csv << "KKTcontactCholeskyInverse" << nconstraint
        << avg << stddev << duration.maxCoeff() << duration.minCoeff() << csv.endl;
    
    initContactDynamics(model,data,contact_models_subset);
    duration.setZero();
    SMOOTH(NBT)
    {
      timer.tic();
      contactDynamics(model,data,qs[_smooth],qdots[_smooth],
                      taus[_smooth],contact_models_subset,contact_datas_subset,mu);
      duration[_smooth] = timer.toc(timer.DEFAULT_UNIT);
    }
    avg = AVG(duration);
    stddev = STDDEV(duration);
    std::cout << "contactDynamics {"<<contact_info_string<<"} = \t\t" << avg
              << " " << timer.unitName(timer.DEFAULT_UNIT) <<std::endl;
    csv << "contactDynamics" << nconstraint
        << avg << stddev << duration.maxCoeff() << duration.minCoeff() << csv.endl;    


    J.setZero();
    duration.setZero();
    SMOOTH(NBT)
    {
      std::size_t running_nc = 0;
      MatrixXd Jout(6*contact_models.size(),model.nv);
      Jout.setZero();
      int contact_dim = 0;
      timer.tic();
      computeAllTerms(model,data,qs[_smooth],qdots[_smooth]);
      for(std::size_t k=0;k<contact_models_subset.size();++k)
      {
        contact_dim = contact_models[k].size();
        getJointJacobian(model,data,contact_models_subset[k].joint1_id,
                         contact_models_subset[k].reference_frame,
                         Jout.middleRows(6*k, 6));
        J.middleRows(running_nc,contact_dim) = Jout.middleRows(6*k, 6);
        running_nc +=contact_dim;
      }
      forwardDynamics(model,data,qs[_smooth], qdots[_smooth], taus[_smooth], J, gamma);
      duration[_smooth] = timer.toc(timer.DEFAULT_UNIT);
    }
    avg = AVG(duration);
    stddev = STDDEV(duration);
    std::cout << "constrainedDynamics {"<<contact_info_string<<"} = \t\t" << avg
              << " " << timer.unitName(timer.DEFAULT_UNIT) <<std::endl;
    csv << "constrainedDynamics" << nconstraint
        << avg << stddev << duration.maxCoeff() << duration.minCoeff() << csv.endl;

    std::cout << "--" << std::endl;
  }
}

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

  // Simple Humanoid Model-------------------------------------------
  Model model_simple;
  std::string filename = PINOCCHIO_MODEL_DIR + std::string("/simple_humanoid.urdf");
  pinocchio::urdf::buildModel(filename,JointModelFreeFlyer(),model_simple);

  std::string RF = "RLEG_LINK6";
  JointIndex RF_id = model_simple.frames[model_simple.getFrameId(RF)].parent;
  std::string LF = "LLEG_LINK6";
  JointIndex LF_id = model_simple.frames[model_simple.getFrameId(LF)].parent;

  RigidContactModel ci_RF_simple(CONTACT_6D,RF_id,LOCAL);  
  RigidContactModel ci_LF_simple(CONTACT_6D,LF_id,LOCAL);

  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_model_simple;
  contact_model_simple.push_back(ci_RF_simple);
  contact_model_simple.push_back(ci_LF_simple);
  std::cout<<"---------------- Simple Humanoid -------------"<<std::endl;
  print_benchmark("simple_humanoid", model_simple, contact_model_simple);

  // Quadruped Solo Benchmarks
  Model model_solo;
  filename = EXAMPLE_ROBOT_DATA_MODEL_DIR "/solo_description/robots/solo.urdf";
  pinocchio::urdf::buildModel(filename,JointModelFreeFlyer(),model_solo);

  RF = "FR_KFE";
  RF_id = model_solo.frames[model_solo.getFrameId(RF)].parent;
  LF = "FL_KFE";
  LF_id = model_solo.frames[model_solo.getFrameId(LF)].parent;

  std::string RA = "HR_KFE";
  JointIndex RA_id = model_solo.frames[model_solo.getFrameId(RF)].parent;
  std::string LA = "HL_KFE";
  JointIndex LA_id = model_solo.frames[model_solo.getFrameId(LF)].parent;

  
  RigidContactModel ci_RF_solo(CONTACT_3D,RF_id,LOCAL);  
  RigidContactModel ci_LF_solo(CONTACT_3D,LF_id,LOCAL);
  RigidContactModel ci_RA_solo(CONTACT_3D,RA_id,LOCAL);  
  RigidContactModel ci_LA_solo(CONTACT_3D,LA_id,LOCAL);
  
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_model_solo;
  contact_model_solo.push_back(ci_RF_solo);
  contact_model_solo.push_back(ci_LF_solo);
  contact_model_solo.push_back(ci_RA_solo);
  contact_model_solo.push_back(ci_LA_solo);
  
  std::cout << "********************Solo Quadruped******************" << std::endl;
  print_benchmark("solo", model_solo, contact_model_solo);


  // Quadruped Anymal Benchmarks
  Model model_anymal;
  filename = EXAMPLE_ROBOT_DATA_MODEL_DIR "/anymal_b_simple_description/robots/anymal.urdf";
  pinocchio::urdf::buildModel(filename,JointModelFreeFlyer(),model_anymal);

  RF = "RF_KFE";
  RF_id = model_anymal.frames[model_anymal.getFrameId(RF)].parent;
  LF = "LF_KFE";
  LF_id = model_anymal.frames[model_anymal.getFrameId(LF)].parent;

  RA = "RH_KFE";
  RA_id = model_anymal.frames[model_anymal.getFrameId(RF)].parent;
  LA = "LF_KFE";
  LA_id = model_anymal.frames[model_anymal.getFrameId(LF)].parent;
  
  RigidContactModel ci_RF_anymal(CONTACT_3D,RF_id,LOCAL);  
  RigidContactModel ci_LF_anymal(CONTACT_3D,LF_id,LOCAL);
  RigidContactModel ci_RA_anymal(CONTACT_3D,RA_id,LOCAL);  
  RigidContactModel ci_LA_anymal(CONTACT_3D,LA_id,LOCAL);
  
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_model_anymal;
  contact_model_anymal.push_back(ci_RF_anymal);
  contact_model_anymal.push_back(ci_LF_anymal);
  contact_model_anymal.push_back(ci_RA_anymal);
  contact_model_anymal.push_back(ci_LA_anymal);
  
  std::cout << "********************Anymal Quadruped******************" << std::endl;
  print_benchmark("anymal", model_anymal, contact_model_anymal);


  // Quadruped HyQ Benchmarks
  Model model_hyq;
  filename = EXAMPLE_ROBOT_DATA_MODEL_DIR "/hyq_description/robots/hyq_no_sensors.urdf";
  pinocchio::urdf::buildModel(filename,JointModelFreeFlyer(),model_hyq);

  RF = "rf_kfe_joint";
  RF_id = model_hyq.frames[model_hyq.getFrameId(RF)].parent;
  LF = "lf_kfe_joint";
  LF_id = model_hyq.frames[model_hyq.getFrameId(LF)].parent;

  RA = "rh_kfe_joint";
  RA_id = model_hyq.frames[model_hyq.getFrameId(RF)].parent;
  LA = "lh_kfe_joint";
  LA_id = model_hyq.frames[model_hyq.getFrameId(LF)].parent;
  
  RigidContactModel ci_RF_hyq(CONTACT_3D,RF_id,LOCAL);  
  RigidContactModel ci_LF_hyq(CONTACT_3D,LF_id,LOCAL);
  RigidContactModel ci_RA_hyq(CONTACT_3D,RA_id,LOCAL);  
  RigidContactModel ci_LA_hyq(CONTACT_3D,LA_id,LOCAL);
  
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_model_hyq;
  contact_model_hyq.push_back(ci_RF_hyq);
  contact_model_hyq.push_back(ci_LF_hyq);
  contact_model_hyq.push_back(ci_RA_hyq);
  contact_model_hyq.push_back(ci_LA_hyq);
  
  std::cout << "********************HyQ Quadruped******************" << std::endl;
  print_benchmark("hyq", model_hyq, contact_model_hyq);
  



  // Humanoid iCub Benchmarks
  Model model_icub;
  filename = EXAMPLE_ROBOT_DATA_MODEL_DIR "/icub_description/robots/icub_reduced.urdf";
  pinocchio::urdf::buildModel(filename,JointModelFreeFlyer(),model_icub);

  RF = "r_ankle_roll";
  RF_id = model_icub.frames[model_icub.getFrameId(RF)].parent;
  LF = "l_ankle_roll";
  LF_id = model_icub.frames[model_icub.getFrameId(LF)].parent;
  
  RigidContactModel ci_RF_icub(CONTACT_6D,RF_id,LOCAL);  
  RigidContactModel ci_LF_icub(CONTACT_6D,LF_id,LOCAL);
  
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_model_icub;
  contact_model_icub.push_back(ci_RF_icub);
  contact_model_icub.push_back(ci_LF_icub);
  
  std::cout << "********************Icub Humanoid******************" << std::endl;
  print_benchmark("icub", model_icub, contact_model_icub);

  // Humanoid Talos Benchmarks
  Model model_talos;
  filename = EXAMPLE_ROBOT_DATA_MODEL_DIR "/talos_data/robots/talos_reduced.urdf";
  pinocchio::urdf::buildModel(filename,JointModelFreeFlyer(),model_talos);

  RF = "leg_right_6_joint";
  RF_id = model_talos.frames[model_talos.getFrameId(RF)].parent;
  LF = "leg_left_6_joint";
  LF_id = model_talos.frames[model_talos.getFrameId(LF)].parent;
  
  RigidContactModel ci_RF_talos(CONTACT_6D,RF_id,LOCAL);  
  RigidContactModel ci_LF_talos(CONTACT_6D,LF_id,LOCAL);
  
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_model_talos;
  contact_model_talos.push_back(ci_RF_talos);
  contact_model_talos.push_back(ci_LF_talos);
  
  std::cout << "********************Talos Humanoid******************" << std::endl;
  print_benchmark("talos", model_talos, contact_model_talos);


  // Manip UR5 Benchmark
  Model model_ur5;
  filename = EXAMPLE_ROBOT_DATA_MODEL_DIR "/ur_description/urdf/ur5_gripper.urdf";
  pinocchio::urdf::buildModel(filename,model_ur5);

  RF = "ee_link";
  RF_id = model_ur5.frames[model_ur5.getFrameId(RF)].parent;
  
  RigidContactModel ci_RF_ur5(CONTACT_3D,RF_id,LOCAL);  
  
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_model_ur5;
  contact_model_ur5.push_back(ci_RF_ur5);
  
  std::cout << "********************UR5 Manipulator******************" << std::endl;
  print_benchmark("ur5", model_ur5, contact_model_ur5);

  
  return 0;
}
