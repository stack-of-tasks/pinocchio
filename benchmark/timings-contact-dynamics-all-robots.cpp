//
// Copyright (c) 2019-2020 INRIA
//

#include "pinocchio/codegen/cppadcg.hpp"
#include "pinocchio/codegen/code-generator-algo.hpp"
#include "pinocchio/autodiff/casadi-algo.hpp"

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
#include "pinocchio/algorithm/contact-dynamics.hpp"
#include <Eigen/SparseCholesky>
#include <Eigen/IterativeLinearSolvers>

#include <iostream>
#include "pinocchio/utils/timer.hpp"
#include "pinocchio/utils/timer2.hpp"
#include "pinocchio/utils/file-io.hpp"

#define STDDEV(vec) std::sqrt(((vec - vec.mean())).square().sum() / (static_cast<double>(vec.size()) - 1))
#define AVG(vec) (vec.mean())

void print_benchmark(const std::string& model_name,
                     const pinocchio::Model& model,
                     const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(pinocchio::RigidContactModel)& contact_models)
{
  using namespace Eigen;
  using namespace pinocchio;
  
  //PinocchioTicToc timer(PinocchioTicToc::US);
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
  pinocchio::Timer timer;
  
  for(std::size_t nc=0; nc<=contact_models.size(); nc++)
  {
    // Define contact infos structure
    PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_models_subset;
    PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) contact_datas_subset;
    std::string contact_info_string;
    int nconstraint = 0;

    if(model_name == "ur5" && (nc == contact_models.size())) {
      contact_models_subset.push_back(contact_models[nc-1]);
      contact_info_string += std::to_string(contact_models[nc-1].size())+"D";
      contact_datas_subset.push_back(RigidContactData(contact_models[nc-1]));
      nconstraint += contact_models[nc-1].size();
    }
    else {
      for(std::size_t j=0;j<nc;j++)
      {
        contact_models_subset.push_back(contact_models[j]);
        contact_datas_subset.push_back(RigidContactData(contact_models[j]));
        contact_info_string += std::to_string(contact_models[j].size())+"D";
        nconstraint += contact_models[j].size();
      }
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
      timer.reset();
      contactABA(model,data,qs[_smooth],qdots[_smooth],
                 taus[_smooth],contact_models_subset,contact_datas_subset);
      duration[_smooth] = timer.get_us_duration();
    }
    avg = AVG(duration);
    stddev = STDDEV(duration);
    std::cout << "contact ABA"<<"{"<<contact_info_string <<"}"<<"= \t\t"<< avg
              << " " << "us" <<std::endl;
    csv << "contactABA" << nconstraint
        << avg << stddev << duration.maxCoeff() << duration.minCoeff() << csv.endl;

    
    
    duration.setZero();
    SMOOTH(NBT)
    {
      computeAllTerms(model,data,qs[_smooth],qdots[_smooth]);
      timer.reset();
      contact_chol_subset.compute(model,data,contact_models_subset,
                                  contact_datas_subset, mu);
      duration[_smooth] = timer.get_us_duration();
    }
    avg = AVG(duration);
    stddev = STDDEV(duration);
    std::cout << "contactCholesky {"<<contact_info_string<<"} = \t\t" << avg
              << " " << "us" <<std::endl;

    csv << "contactCholesky" << nconstraint
        << avg << stddev << duration.maxCoeff() << duration.minCoeff() << csv.endl;


    duration.setZero();
    Eigen::MatrixXd osim_inv(Eigen::MatrixXd::Zero(contact_chol_subset.constraintDim(),
                                                   contact_chol_subset.constraintDim()));
    SMOOTH(NBT)
    {
      computeAllTerms(model,data,qs[_smooth],qdots[_smooth]);
      contact_chol_subset.compute(model,data,contact_models_subset,
                                  contact_datas_subset, mu);
      timer.reset();
      contact_chol_subset.getInverseOperationalSpaceInertiaMatrix(osim_inv);
      duration[_smooth] = timer.get_us_duration();
    }
    avg = AVG(duration);
    stddev = STDDEV(duration);
    std::cout << "osim_inv: {"<<contact_info_string<<"} = \t\t" << avg
              << " " << "us" <<std::endl;

    csv << "osimInv" << nconstraint
        << avg << stddev << duration.maxCoeff() << duration.minCoeff() << csv.endl;

    duration.setZero();
    Eigen::MatrixXd osim(Eigen::MatrixXd::Zero(contact_chol_subset.constraintDim(),
                                               contact_chol_subset.constraintDim()));
    SMOOTH(NBT)
    {
      computeAllTerms(model,data,qs[_smooth],qdots[_smooth]);
      contact_chol_subset.compute(model,data,contact_models_subset,
                                  contact_datas_subset, mu);
      timer.reset();
      contact_chol_subset.getOperationalSpaceInertiaMatrix(osim);
      duration[_smooth] = timer.get_us_duration();
    }
    avg = AVG(duration);
    stddev = STDDEV(duration);
    std::cout << "osim: {"<<contact_info_string<<"} = \t\t" << avg
              << " " << "us" <<std::endl;

    csv << "osim" << nconstraint
        << avg << stddev << duration.maxCoeff() << duration.minCoeff() << csv.endl;

    CodeGenContactDynamics<double> cg_contactDynamics(model, contact_models_subset,
                                                      "contactDynamics_fn_"
                                                      +contact_info_string,
                                                      model_name
                                                      +"contactDynamics_"
                                                      +contact_info_string);
    cg_contactDynamics.initLib();
    cg_contactDynamics.loadLib();
    duration.setZero();
    SMOOTH(NBT)
    {
      timer.reset();
      cg_contactDynamics.evalFunction(qs[_smooth],qdots[_smooth],
                                      taus[_smooth]);
      duration[_smooth] = timer.get_us_duration();
    }
    avg = AVG(duration);
    stddev = STDDEV(duration);
    std::cout << "cg_contactDyn: {"<<contact_info_string<<"} = \t\t" << avg
              << " " << "us" <<std::endl;
    csv << "cg_contactDyn" << nconstraint
        << avg << stddev << duration.maxCoeff() << duration.minCoeff() << csv.endl;

    duration.setZero();
    SMOOTH(NBT)
    {
      timer.reset();
      cg_contactDynamics.evalJacobian(qs[_smooth],qdots[_smooth],
                                      taus[_smooth]);
      duration[_smooth] = timer.get_us_duration();
    }
    avg = AVG(duration);
    stddev = STDDEV(duration);
    std::cout << "cg_contactDyn_jacobian: {"<<contact_info_string<<"} = \t\t" << avg
              << " " << "us" <<std::endl;
    csv << "cg_contactDyn_jacobian" << nconstraint
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
      timer.reset();
      MJtJ_ldlt.compute(MJtJ);
      duration[_smooth] = timer.get_us_duration();
    }
    avg = AVG(duration);
    stddev = STDDEV(duration);    
    std::cout << "Dense Eigen contactCholesky {"<<contact_info_string<<"} = \t" << avg
              << " " << "us" <<std::endl;
    csv << "eigenCholesky" << nconstraint
        << avg << stddev << duration.maxCoeff() << duration.minCoeff() << csv.endl;

    //-----------------Sparse Eigen Cholesky----------------------------
    duration.setZero();
    Eigen::SparseMatrix<double> MJtJ_sparse(contact_chol_subset.size(), contact_chol_subset.size());
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double> > simplical_solver;
    SMOOTH(NBT)
    {
      MJtJ.setZero();
      MJtJ_sparse.setZero();
      computeAllTerms(model,data,qs[_smooth],qdots[_smooth]);
      contact_chol_subset.compute(model,data,contact_models_subset,contact_datas_subset, mu);
      contact_chol_subset.matrix(MJtJ);
      for (Eigen::DenseIndex i=0;i<MJtJ.rows();++i)
      {
        for (Eigen::DenseIndex j=0;j<MJtJ.cols();++j)
        {
          if( MJtJ(i,j) != double(0.) )
          {
            MJtJ_sparse.insert(i,j) = MJtJ(i,j);
          }
        }
      }
      timer.reset();
      simplical_solver.compute(MJtJ_sparse);
      duration[_smooth] = timer.get_us_duration();
    }
    avg = AVG(duration);
    stddev = STDDEV(duration);    
    std::cout << "Simplicial Eigen contactCholesky {"<<contact_info_string<<"} = \t" << avg
              << " " << "us" <<std::endl;
    csv << "simplicialCholesky" << nconstraint
        << avg << stddev << duration.maxCoeff() << duration.minCoeff() << csv.endl;
    //-----------------------------------------------------------------
    


    //-----------------Sparse Eigen Cholesky----------------------------

    duration.setZero();
    Eigen::ConjugateGradient<Eigen::SparseMatrix<double>,Eigen::Lower|Eigen::Upper> conjugateGradient_solver;
    SMOOTH(NBT)
    {
      MJtJ.setZero();
      MJtJ_sparse.setZero();
      computeAllTerms(model,data,qs[_smooth],qdots[_smooth]);
      contact_chol_subset.compute(model,data,contact_models_subset,contact_datas_subset, mu);
      contact_chol_subset.matrix(MJtJ);
      for (Eigen::DenseIndex i=0;i<MJtJ.rows();++i)
      {
        for (Eigen::DenseIndex j=0;j<MJtJ.cols();++j)
        {
          if(MJtJ(i,j) != double(0.) )
          {
            MJtJ_sparse.insert(i,j) = MJtJ(i,j);
          }
        }
      }
      timer.reset();
      conjugateGradient_solver.compute(MJtJ_sparse);
      duration[_smooth] = timer.get_us_duration();
    }
    avg = AVG(duration);
    stddev = STDDEV(duration);
    std::cout << "Conjugate Gradient Eigen contactCholesky {"<<contact_info_string<<"} = \t" << avg
              << " " << "us" <<std::endl;
    csv << "ConjugateGradCholesky" << nconstraint
        << avg << stddev << duration.maxCoeff() << duration.minCoeff() << csv.endl;

    //-----------------------------------------------------------------

    
    duration.setZero();
    MatrixXd H_inverse(contact_chol_subset.size(),contact_chol_subset.size());
    SMOOTH(NBT)
    {
      computeAllTerms(model,data,qs[_smooth],qdots[_smooth]);
      contact_chol_subset.compute(model,data,contact_models_subset,contact_datas_subset,mu);
      timer.reset();
      contact_chol_subset.inverse(H_inverse);
      duration[_smooth] = timer.get_us_duration();
    }
    avg = AVG(duration);
    stddev = STDDEV(duration);        
    std::cout << "contactCholeskyInverse {"<<contact_info_string<<"} = \t\t" << avg
              << " " << "us" <<std::endl;
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
      
      timer.reset();
      cholesky::decompose(model,data);
      getKKTContactDynamicMatrixInverse(model,data,J,MJtJ_inv);
      duration[_smooth] = timer.get_us_duration();
    }
    avg = AVG(duration);
    stddev = STDDEV(duration);
    std::cout << "KKTContactDynamicMatrixInverse {"<<contact_info_string<<"} = \t\t" << avg
              << " " << "us" <<std::endl;    

    csv << "KKTcontactCholeskyInverse" << nconstraint
        << avg << stddev << duration.maxCoeff() << duration.minCoeff() << csv.endl;
    
    initContactDynamics(model,data,contact_models_subset);
    duration.setZero();
    SMOOTH(NBT)
    {
      timer.reset();
      contactDynamics(model,data,qs[_smooth],qdots[_smooth],
                      taus[_smooth],contact_models_subset,contact_datas_subset,mu);
      duration[_smooth] = timer.get_us_duration();
    }
    avg = AVG(duration);
    stddev = STDDEV(duration);
    std::cout << "contactDynamics {"<<contact_info_string<<"} = \t\t" << avg
              << " " << "us" <<std::endl;
    csv << "contactDynamics" << nconstraint
        << avg << stddev << duration.maxCoeff() << duration.minCoeff() << csv.endl;    


    //Contact Dynamics Derivatives
    duration.setZero();
    initContactDynamics(model,data,contact_models_subset);
    SMOOTH(NBT)
    {
      initContactDynamics(model,data,contact_models_subset);
      contactDynamics(model,data,qs[_smooth],qdots[_smooth],
                      taus[_smooth],contact_models_subset,contact_datas_subset,mu);
      timer.reset();      
      computeContactDynamicsDerivatives(model, data,
                                        contact_models_subset, contact_datas_subset);
      duration[_smooth] = timer.get_us_duration();
    }
    avg = AVG(duration);
    stddev = STDDEV(duration);
    std::cout << "contactDynamicsDerivs {"<<contact_info_string<<"} = \t\t" << avg
              << " " << "us" <<std::endl;
    csv << "contactDynamicsDerivs" << nconstraint
        << avg << stddev << duration.maxCoeff() << duration.minCoeff() << csv.endl;


    //Contact Dynamics Derivatives Finite Differences

    duration.setZero();
    initContactDynamics(model,data,contact_models_subset);
    SMOOTH(NBT)
    {
      //ddq/dq
      VectorXd v_eps(VectorXd::Zero(model.nv));      
      VectorXd q_plus(model.nq);
      VectorXd ddq_plus(model.nv);
      VectorXd lambda_plus(nconstraint);
      MatrixXd ddq_partial_dq_fd(model.nv,model.nv); ddq_partial_dq_fd.setZero();
      MatrixXd lambda_partial_dq_fd(nconstraint,model.nv);
      lambda_partial_dq_fd.setZero();

      //ddq/dv
      VectorXd v_plus(qdots[_smooth]);
      MatrixXd ddq_partial_dv_fd(model.nv,model.nv); ddq_partial_dv_fd.setZero();
      MatrixXd lambda_partial_dv_fd(nconstraint,model.nv);
      lambda_partial_dv_fd.setZero();

      //ddq/dtau
      VectorXd tau_plus(taus[_smooth]);
      MatrixXd ddq_partial_dtau_fd(model.nv,model.nv); ddq_partial_dtau_fd.setZero();      
      MatrixXd lambda_partial_dtau_fd(nconstraint,model.nv);
      lambda_partial_dtau_fd.setZero();
      
      const VectorXd ddq0 = contactDynamics(model,data,qs[_smooth],qdots[_smooth],
                                            taus[_smooth],contact_models_subset,
                                            contact_datas_subset,mu);
      const VectorXd lambda0 = data.lambda_c;
      const double alpha = 1e-8;

      timer.reset();
      for(int k = 0; k < model.nv; ++k)
      {
        v_eps[k] += alpha;
        q_plus = integrate(model,qs[_smooth],v_eps);
        ddq_plus = contactDynamics(model,data,q_plus,
                                   qdots[_smooth],taus[_smooth],
                                   contact_models_subset,contact_datas_subset,mu);
        ddq_partial_dq_fd.col(k) = (ddq_plus - ddq0)/alpha;
        lambda_partial_dq_fd.col(k) = (data.lambda_c - lambda0)/alpha;
        v_eps[k] = 0.;
      }
      for(int k = 0; k < model.nv; ++k)
      {
        v_plus[k] += alpha;
        ddq_plus = contactDynamics(model,data,qs[_smooth],v_plus,taus[_smooth],
                                   contact_models_subset,contact_datas_subset,mu);
        ddq_partial_dv_fd.col(k) = (ddq_plus - ddq0)/alpha;
        lambda_partial_dv_fd.col(k) = (data.lambda_c - lambda0)/alpha;
        v_plus[k] -= alpha;
      }
      for(int k = 0; k < model.nv; ++k)
      {
        tau_plus[k] += alpha;
        ddq_plus = contactDynamics(model,data,qs[_smooth],qdots[_smooth],tau_plus,
                                   contact_models_subset,contact_datas_subset,mu);
        ddq_partial_dtau_fd.col(k) = (ddq_plus - ddq0)/alpha;
        lambda_partial_dtau_fd.col(k) = (data.lambda_c - lambda0)/alpha;
        tau_plus[k] -= alpha;
      }
      duration[_smooth] = timer.get_us_duration();
    }
    avg = AVG(duration);
    stddev = STDDEV(duration);
    std::cout << "contactDynamicsDerivs_fd {"<<contact_info_string<<"} = \t\t" << avg
              << " " << "us" <<std::endl;
    csv << "contactDynamicsDerivs_fd" << nconstraint
        << avg << stddev << duration.maxCoeff() << duration.minCoeff() << csv.endl;

    
    CodeGenContactDynamicsDerivatives<double>
      cg_contactDynamicsDerivs(model,
                               contact_models_subset,
                               "contactDynamicsDerivs_fn_"+contact_info_string,
                               model_name+"contactDynamicsDerivs_"+contact_info_string);
    cg_contactDynamicsDerivs.initLib();
    cg_contactDynamicsDerivs.loadLib();
    duration.setZero();
    SMOOTH(NBT)
    {
      timer.reset();
      cg_contactDynamicsDerivs.evalFunction(qs[_smooth],qdots[_smooth],
                                            taus[_smooth]);
      duration[_smooth] = timer.get_us_duration();
    }
    avg = AVG(duration);
    stddev = STDDEV(duration);
    std::cout << "cg_contactDynDerivs: {"<<contact_info_string<<"} = \t\t" << avg
              << " " << "us" <<std::endl;
    csv << "cg_contactDynDerivs" << nconstraint
        << avg << stddev << duration.maxCoeff() << duration.minCoeff() << csv.endl;

    pinocchio::casadi::AutoDiffContactDynamics<double>
      casadi_contactDynamics_jacobian(model,
                                      contact_models_subset,
                                      model_name+"_casadi_contactDynamics_fn_"+contact_info_string,
                                      "lib"+model_name+"_casadi_contactDynamics_fn_"+contact_info_string);

    casadi_contactDynamics_jacobian.initLib();
    casadi_contactDynamics_jacobian.loadLib();

    duration.setZero();
    SMOOTH(NBT)
    {
      timer.reset();
      casadi_contactDynamics_jacobian.evalFunction(qs[_smooth],qdots[_smooth],
                                                   taus[_smooth]);
      duration[_smooth] = timer.get_us_duration();
    }
    avg = AVG(duration);
    stddev = STDDEV(duration);
    std::cout << "casadi contactDyn Function: {"<<contact_info_string<<"} = \t\t" << avg
              << " " << "us" <<std::endl;
    csv << "casadi_contactDyn_function" << nconstraint
        << avg << stddev << duration.maxCoeff() << duration.minCoeff() << csv.endl;
    duration.setZero();
    SMOOTH(NBT)
    {
      timer.reset();
      casadi_contactDynamics_jacobian.evalJacobian(qs[_smooth],qdots[_smooth],
                                                   taus[_smooth]);
      duration[_smooth] = timer.get_us_duration();
    }
    avg = AVG(duration);
    stddev = STDDEV(duration);
    std::cout << "casadi contactDyn Jacobian: {"<<contact_info_string<<"} = \t\t" << avg
              << " " << "us" <<std::endl;
    csv << "casadi_contactDyn_jacobian" << nconstraint
        << avg << stddev << duration.maxCoeff() << duration.minCoeff() << csv.endl;


    pinocchio::casadi::AutoDiffContactDynamicsDerivatives<double>
      casadi_contactDynamicsDerivatives(model,
                                        contact_models_subset,
                                        model_name+"_casadi_contactDynamicsDerivs_fn_"+contact_info_string,
                                        "lib"+model_name+"_casadi_contactDynamicsDerivs_fn_"+contact_info_string);
    casadi_contactDynamicsDerivatives.initLib();
    casadi_contactDynamicsDerivatives.loadLib();
    
    duration.setZero();
    SMOOTH(NBT)
    {
      timer.reset();
      casadi_contactDynamicsDerivatives.evalFunction(qs[_smooth],qdots[_smooth],
                                                     taus[_smooth]);
      duration[_smooth] = timer.get_us_duration();
    }
    avg = AVG(duration);
    stddev = STDDEV(duration);
    std::cout << "casadi contactDynDerivs: {"<<contact_info_string<<"} = \t\t" << avg
              << " " << "us" <<std::endl;
    csv << "casadi_contactDynDerivs" << nconstraint
        << avg << stddev << duration.maxCoeff() << duration.minCoeff() << csv.endl;

    J.setZero();
    duration.setZero();
    SMOOTH(NBT)
    {
      std::size_t running_nc = 0;
      MatrixXd Jout(6*contact_models.size(),model.nv);
      Jout.setZero();
      int contact_dim = 0;
      timer.reset();
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
      duration[_smooth] = timer.get_us_duration();
    }
    avg = AVG(duration);
    stddev = STDDEV(duration);
    std::cout << "constrainedDynamics {"<<contact_info_string<<"} = \t\t" << avg
              << " " << "us" <<std::endl;
    csv << "constrainedDynamics" << nconstraint
        << avg << stddev << duration.maxCoeff() << duration.minCoeff() << csv.endl;
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

  // Quadruped Solo Benchmarks--------------------------------------------------
  Model model_solo;
  filename = EXAMPLE_ROBOT_DATA_MODEL_DIR "/solo_description/robots/solo.urdf";
  pinocchio::urdf::buildModel(filename,JointModelFreeFlyer(),model_solo);

  RF = "FR_KFE";
  RF_id = model_solo.frames[model_solo.getFrameId(RF)].parent;
  LF = "FL_KFE";
  LF_id = model_solo.frames[model_solo.getFrameId(LF)].parent;

  std::string RA = "HR_KFE";
  JointIndex RA_id = model_solo.frames[model_solo.getFrameId(RA)].parent;
  std::string LA = "HL_KFE";
  JointIndex LA_id = model_solo.frames[model_solo.getFrameId(LA)].parent;

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


  // Quadruped Anymal Benchmarks--------------------------------------------------
  Model model_anymal;
  filename = EXAMPLE_ROBOT_DATA_MODEL_DIR "/anymal_b_simple_description/robots/anymal.urdf";
  pinocchio::urdf::buildModel(filename,JointModelFreeFlyer(),model_anymal);

  RF = "RF_KFE";
  RF_id = model_anymal.frames[model_anymal.getFrameId(RF)].parent;
  LF = "LF_KFE";
  LF_id = model_anymal.frames[model_anymal.getFrameId(LF)].parent;

  RA = "RH_KFE";
  RA_id = model_anymal.frames[model_anymal.getFrameId(RA)].parent;
  LA = "LH_KFE";
  LA_id = model_anymal.frames[model_anymal.getFrameId(LA)].parent;
  
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


  // Quadruped HyQ Benchmarks--------------------------------------------------
  Model model_hyq;
  filename = EXAMPLE_ROBOT_DATA_MODEL_DIR "/hyq_description/robots/hyq_no_sensors.urdf";
  pinocchio::urdf::buildModel(filename,JointModelFreeFlyer(),model_hyq);

  RF = "rf_kfe_joint";
  RF_id = model_hyq.frames[model_hyq.getFrameId(RF)].parent;
  LF = "lf_kfe_joint";
  LF_id = model_hyq.frames[model_hyq.getFrameId(LF)].parent;

  RA = "rh_kfe_joint";
  RA_id = model_hyq.frames[model_hyq.getFrameId(RA)].parent;
  LA = "lh_kfe_joint";
  LA_id = model_hyq.frames[model_hyq.getFrameId(LA)].parent;
  
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


  // Humanoid iCub Benchmarks--------------------------------------------------
  Model model_icub;
  filename = EXAMPLE_ROBOT_DATA_MODEL_DIR "/icub_description/robots/icub_reduced.urdf";
  pinocchio::urdf::buildModel(filename,JointModelFreeFlyer(),model_icub);

  RF = "r_ankle_roll";
  RF_id = model_icub.frames[model_icub.getFrameId(RF)].parent;
  LF = "l_ankle_roll";
  LF_id = model_icub.frames[model_icub.getFrameId(LF)].parent;

  RA = "r_gripper";
  RA_id = model_icub.frames[model_icub.getFrameId(RA)].parent;
  LA = "l_gripper";
  LA_id = model_icub.frames[model_icub.getFrameId(LA)].parent;

  
  RigidContactModel ci_RF_icub(CONTACT_6D,RF_id,LOCAL);  
  RigidContactModel ci_LF_icub(CONTACT_6D,LF_id,LOCAL);
  RigidContactModel ci_RA_icub(CONTACT_6D,RA_id,LOCAL);  
  RigidContactModel ci_LA_icub(CONTACT_6D,LA_id,LOCAL);

  
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_model_icub;
  contact_model_icub.push_back(ci_RF_icub);
  contact_model_icub.push_back(ci_LF_icub);
  contact_model_icub.push_back(ci_RA_icub);
  contact_model_icub.push_back(ci_LA_icub);  
  
  std::cout << "********************Icub Humanoid******************" << std::endl;
  print_benchmark("icub", model_icub, contact_model_icub);

  // Humanoid Talos Benchmarks--------------------------------------------------
  Model model_talos;
  filename = EXAMPLE_ROBOT_DATA_MODEL_DIR "/talos_data/robots/talos_reduced.urdf";
  pinocchio::urdf::buildModel(filename,JointModelFreeFlyer(),model_talos);

  RF = "leg_right_6_joint";
  RF_id = model_talos.frames[model_talos.getFrameId(RF)].parent;
  LF = "leg_left_6_joint";
  LF_id = model_talos.frames[model_talos.getFrameId(LF)].parent;

  RA = "gripper_right_base_link";
  RA_id = model_talos.frames[model_talos.getFrameId(RA)].parent;
  LA = "gripper_left_base_link";
  LA_id = model_talos.frames[model_talos.getFrameId(LA)].parent;

  
  RigidContactModel ci_RF_talos(CONTACT_6D,RF_id,LOCAL);  
  RigidContactModel ci_LF_talos(CONTACT_6D,LF_id,LOCAL);
  RigidContactModel ci_RA_talos(CONTACT_6D,RA_id,LOCAL);  
  RigidContactModel ci_LA_talos(CONTACT_6D,LA_id,LOCAL);
  
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_model_talos;
  contact_model_talos.push_back(ci_RF_talos);
  contact_model_talos.push_back(ci_LF_talos);
  contact_model_talos.push_back(ci_RA_talos);
  contact_model_talos.push_back(ci_LA_talos);
  
  std::cout << "********************Talos Humanoid******************" << std::endl;
  print_benchmark("talos", model_talos, contact_model_talos);


  // Manip UR5 Benchmark--------------------------------------------------
  Model model_ur5;
  filename = EXAMPLE_ROBOT_DATA_MODEL_DIR "/ur_description/urdf/ur5_gripper.urdf";
  pinocchio::urdf::buildModel(filename,model_ur5);

  RF = "ee_link";
  RF_id = model_ur5.frames[model_ur5.getFrameId(RF)].parent;
  
  RigidContactModel ci_RF_ur5_3d(CONTACT_3D,RF_id,LOCAL);
  RigidContactModel ci_RF_ur5_6d(CONTACT_6D,RF_id,LOCAL);  
  
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_model_ur5;
  contact_model_ur5.push_back(ci_RF_ur5_3d);
  contact_model_ur5.push_back(ci_RF_ur5_6d);
  
  std::cout << "********************UR5 Manipulator******************" << std::endl;
  print_benchmark("ur5", model_ur5, contact_model_ur5);

  return 0;
}
