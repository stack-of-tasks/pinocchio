//
// Copyright (c) 2018 CNRS
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/cholesky.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/container/aligned-vector.hpp"

#include <iostream>

#include "pinocchio/utils/timer.hpp"

template<typename _Scalar, typename _EvalScalar = _Scalar>
struct AutoDiffRNEA
{
  typedef _Scalar Scalar;
  typedef CppAD::AD<Scalar> ADScalar;
  typedef _EvalScalar EvalScalar;
  enum { Options = 0 };
  
  typedef se3::ModelTpl<Scalar,Options> Model;
  typedef se3::DataTpl<Scalar,Options> Data;
  typedef se3::ModelTpl<EvalScalar,Options> EvalModel;
  typedef se3::DataTpl<EvalScalar,Options> EvalData;
  typedef se3::ModelTpl<ADScalar,Options> ADModel;
  typedef se3::DataTpl<ADScalar,Options> ADData;
  
  typedef Eigen::Matrix<EvalScalar,Eigen::Dynamic,Eigen::Dynamic,Options> MatrixXs;
  typedef Eigen::Matrix<EvalScalar,Eigen::Dynamic,1,Options> VectorXs;
  typedef Eigen::Matrix<ADScalar,Eigen::Dynamic,1,Options> ADVectorXs;
  
  typedef typename ADModel::ConfigVectorType ADCongigVectorType;
  typedef typename ADModel::TangentVectorType ADTangentVectorType;
  
  typedef CppAD::ADFun<Scalar> ADFun;
  
  template<typename OtherScalar>
  AutoDiffRNEA(const se3::ModelTpl<OtherScalar,Options> & model)
  : drnea_dq(model.nv,model.nv)
  , drnea_dv(model.nv,model.nv)
  , drnea_da(model.nv,model.nv)
  , ad_model(model.template cast<ADScalar>())
  , ad_data(ad_model)
  {
    ad_q = ADCongigVectorType(model.nq);
    ad_q_plus = ADCongigVectorType(model.nq);
    ad_dq = ADTangentVectorType(model.nv); ad_dq.setZero();
    ad_v = ADTangentVectorType(model.nv); ad_v.setZero();
    ad_a = ADTangentVectorType(model.nv); ad_a.setZero();
    
    ad_X = ADVectorXs(model.nv*3);
    ad_Y = ADVectorXs(model.nv);
  }
  
  void compute()
  {
    ad_X.head(ad_model.nv).setZero();
    size_t abort_op_index = 0;
    bool   record_compare = false;
    CppAD::Independent(ad_X,abort_op_index,record_compare,ad_q);

    Eigen::DenseIndex it = 0;
    ad_dq = ad_X.head(ad_model.nv); it += ad_model.nv;
    ad_v = ad_X.segment(it,ad_model.nv); it += ad_model.nv;
    ad_a = ad_X.segment(it,ad_model.nv); it += ad_model.nv;
    
    ad_q_plus = se3::integrate(ad_model,ad_q,ad_dq);
    se3::rnea(ad_model,ad_data,ad_q_plus,ad_v,ad_a);
    
    ad_Y = ad_data.tau;
    
    ad_fun.Dependent(ad_X,ad_Y);
    ad_fun.optimize("no_compare_op");
  }
  
  template<typename ConfigVectorType, typename TangentVector1, typename TangentVector2>
  void eval(const Eigen::MatrixBase<ConfigVectorType> & q,
            const Eigen::MatrixBase<TangentVector1> & v,
            const Eigen::MatrixBase<TangentVector2> & a)
  {
    ad_fun.new_dynamic(q.derived());
    
    CPPAD_TESTVECTOR(EvalScalar) x((size_t)(ad_X.size()));
    Eigen::DenseIndex it = 0;
    Eigen::Map<typename Data::TangentVectorType>(x.data(),ad_model.nv,1).setZero();
    it += ad_model.nv;
    Eigen::Map<typename Data::TangentVectorType>(x.data()+it,ad_model.nv,1) = v;
    it += ad_model.nv;
    Eigen::Map<typename Data::TangentVectorType>(x.data()+it,ad_model.nv,1) = a;
    it += ad_model.nv;
    
    CPPAD_TESTVECTOR(EvalScalar) dy_dx = ad_fun.Jacobian(x);
    it = 0;
    drnea_dq = Eigen::Map<typename EIGEN_PLAIN_ROW_MAJOR_TYPE(typename EvalData::MatrixXs)>(dy_dx.data(),ad_model.nv,ad_model.nv);
    it += ad_model.nv * ad_model.nv;
    drnea_dv = Eigen::Map<typename EIGEN_PLAIN_ROW_MAJOR_TYPE(typename EvalData::MatrixXs)>(dy_dx.data()+it,ad_model.nv,ad_model.nv);
    it += ad_model.nv * ad_model.nv;
    drnea_da = Eigen::Map<typename EIGEN_PLAIN_ROW_MAJOR_TYPE(typename EvalData::MatrixXs)>(dy_dx.data()+it,ad_model.nv,ad_model.nv);
    it += ad_model.nv * ad_model.nv;
  }
  
  // Results of eval
  MatrixXs drnea_dq, drnea_dv, drnea_da;
  ADCongigVectorType ad_q, ad_q_plus;
  ADTangentVectorType ad_dq, ad_v, ad_a;
  
  ADVectorXs ad_X, ad_Y;
  
  ADFun ad_fun;
  
protected:
  
  ADModel ad_model;
  ADData ad_data;
};

template<typename _Scalar>
struct CodeGenRNEA : AutoDiffRNEA<CppAD::cg::CG<_Scalar>,_Scalar>
{
  typedef _Scalar Scalar;
  typedef CppAD::cg::CG<Scalar> CGScalar;
  typedef AutoDiffRNEA<CGScalar,Scalar> Base;
  
  typedef typename Base::ADScalar ADScalar;
  typedef typename Base::EvalScalar EvalScalar;
  
  enum { Options = Base::Options };
  
  typedef se3::ModelTpl<Scalar,Options> Model;
  
  using Base::eval;
  using Base::compute;
  
  CodeGenRNEA(const Model & model,
              const std::string & function_name = "rnea",
              const std::string & library_name = "cg_rnea")
  : Base(model.template cast<ADScalar>())
  , dynamicLib_ptr(nullptr)
  {
    compute();
    
    // generates source code
    cgen_ptr = std::unique_ptr<CppAD::cg::ModelCSourceGen<Scalar> >(new CppAD::cg::ModelCSourceGen<Scalar>(Base::ad_fun, function_name));
    cgen_ptr->setCreateJacobian(true);
    cgen_ptr->setCreateForwardZero(true);
//    cgen.setCreateForwardOne(true);
//    cgen.setCreateReverseOne(true);
//    cgen.setCreateReverseTwo(true);
    libcgen_ptr = std::unique_ptr<CppAD::cg::ModelLibraryCSourceGen<Scalar> >(new CppAD::cg::ModelLibraryCSourceGen<Scalar>(*cgen_ptr));
    
    dynamicLibManager_ptr
    = std::unique_ptr<CppAD::cg::DynamicModelLibraryProcessor<Scalar> >(new CppAD::cg::DynamicModelLibraryProcessor<Scalar>(*libcgen_ptr,library_name));
  }
  
  bool existLib() const
  {
    const std::string filename = dynamicLibManager_ptr->getLibraryName() + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION;
    std::ifstream file(filename.c_str());
    return file.good();
  }
  
  void generateLib()
  {
    CppAD::cg::GccCompiler<Scalar> compiler;
    dynamicLibManager_ptr->createDynamicLibrary(compiler,false);
  }
  
  void loadLib(const bool generate_if_not_exist = true)
  {
    if(not existLib() && generate_if_not_exist)
      generateLib();
    
    const auto it = dynamicLibManager_ptr->getOptions().find("dlOpenMode");
    if (it == dynamicLibManager_ptr->getOptions().end())
    {
      dynamicLib_ptr.reset(new CppAD::cg::LinuxDynamicLib<Scalar>(dynamicLibManager_ptr->getLibraryName() +  CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION));
    }
    else
    {
      int dlOpenMode = std::stoi(it->second);
      dynamicLib_ptr.reset(new CppAD::cg::LinuxDynamicLib<Scalar>(dynamicLibManager_ptr->getLibraryName() +  CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION, dlOpenMode));
    }
  }
  
protected:
  
  std::unique_ptr<CppAD::cg::ModelCSourceGen<Scalar> > cgen_ptr;
  std::unique_ptr<CppAD::cg::ModelLibraryCSourceGen<Scalar> > libcgen_ptr;
  std::unique_ptr<CppAD::cg::DynamicModelLibraryProcessor<Scalar> > dynamicLibManager_ptr;
  std::unique_ptr<CppAD::cg::DynamicLib<Scalar> > dynamicLib_ptr;
};

template<typename Matrix1, typename Matrix2, typename Matrix3>
void rnea_fd(const se3::Model & model, se3::Data & data_fd,
             const Eigen::VectorXd & q,
             const Eigen::VectorXd & v,
             const Eigen::VectorXd & a,
             const Eigen::MatrixBase<Matrix1> & _drnea_dq,
             const Eigen::MatrixBase<Matrix2> & _drnea_dv,
             const Eigen::MatrixBase<Matrix3> & _drnea_da)
{
  Matrix1 & drnea_dq = EIGEN_CONST_CAST(Matrix1,_drnea_dq);
  Matrix2 & drnea_dv = EIGEN_CONST_CAST(Matrix2,_drnea_dv);
  Matrix3 & drnea_da = EIGEN_CONST_CAST(Matrix3,_drnea_da);
  
  using namespace Eigen;
  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd tau_plus(model.nv);
  const double alpha = 1e-8;
  
  VectorXd tau0 = rnea(model,data_fd,q,v,a);
  
  // dRNEA/dq
  for(int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model,q,v_eps);
    tau_plus = rnea(model,data_fd,q_plus,v,a);
    
    drnea_dq.col(k) = (tau_plus - tau0)/alpha;
    v_eps[k] -= alpha;
  }
  
  // dRNEA/dv
  VectorXd v_plus(v);
  for(int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    tau_plus = rnea(model,data_fd,q,v_plus,a);
    
    drnea_dv.col(k) = (tau_plus - tau0)/alpha;
    v_plus[k] -= alpha;
  }
  
  // dRNEA/da
  drnea_da = crba(model,data_fd,q);
  drnea_da.template triangularView<Eigen::StrictlyLower>()
  = drnea_da.transpose().template triangularView<Eigen::StrictlyLower>();
  
}

void aba_fd(const se3::Model & model, se3::Data & data_fd,
            const Eigen::VectorXd & q,
            const Eigen::VectorXd & v,
            const Eigen::VectorXd & tau,
            Eigen::MatrixXd & daba_dq,
            Eigen::MatrixXd & daba_dv,
            se3::Data::RowMatrixXs & daba_dtau)
{
  using namespace Eigen;
  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd a_plus(model.nv);
  const double alpha = 1e-8;
  
  VectorXd a0 = aba(model,data_fd,q,v,tau);
  
  // dABA/dq
  for(int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model,q,v_eps);
    a_plus = aba(model,data_fd,q_plus,v,tau);
    
    daba_dq.col(k) = (a_plus - a0)/alpha;
    v_eps[k] -= alpha;
  }
  
  // dABA/dv
  VectorXd v_plus(v);
  for(int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    a_plus = aba(model,data_fd,q,v_plus,tau);
    
    daba_dv.col(k) = (a_plus - a0)/alpha;
    v_plus[k] -= alpha;
  }
  
  // dABA/dtau
  daba_dtau = computeMinverse(model,data_fd,q);
}

int main(int argc, const char ** argv)
{
  using namespace Eigen;
  using namespace se3;

  PinocchioTicToc timer(PinocchioTicToc::US);
  #ifdef NDEBUG
  const int NBT = 1000*100;
  #else
    const int NBT = 1;
    std::cout << "(the time score in debug mode is not relevant) " << std::endl;
  #endif
    
  Model model;

  std::string filename = PINOCCHIO_SOURCE_DIR"/models/simple_humanoid.urdf";
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
  else if( filename == "H2" )
    buildModels::humanoid2d(model);
  else
    if(with_ff)
      se3::urdf::buildModel(filename,JointModelFreeFlyer(),model);
//      se3::urdf::buildModel(filename,JointModelRX(),model);
    else
      se3::urdf::buildModel(filename,model);
  std::cout << "nq = " << model.nq << std::endl;
  std::cout << "nv = " << model.nv << std::endl;

  Data data(model);
  VectorXd q = VectorXd::Random(model.nq);
  VectorXd qdot = VectorXd::Random(model.nv);
  VectorXd qddot = VectorXd::Random(model.nv);

  container::aligned_vector<VectorXd> qs     (NBT);
  container::aligned_vector<VectorXd> qdots  (NBT);
  container::aligned_vector<VectorXd> qddots (NBT);
  container::aligned_vector<VectorXd> taus (NBT);
  
  for(size_t i=0;i<NBT;++i)
  {
    qs[i]     = Eigen::VectorXd::Random(model.nq);
    qs[i].segment<4>(3) /= qs[i].segment<4>(3).norm();
    qdots[i]  = Eigen::VectorXd::Random(model.nv);
    qddots[i] = Eigen::VectorXd::Random(model.nv);
    taus[i] = Eigen::VectorXd::Random(model.nv);
  }

//  AutoDiffRNEA<double> ad_rnea(model);
//  CodeGenRNEA<double> cg_rnea(model);
//  cg_rnea.compute();
//  cg_rnea.loadLib();
  
  EIGEN_PLAIN_ROW_MAJOR_TYPE(MatrixXd) drnea_dq(MatrixXd::Zero(model.nv,model.nv));
  EIGEN_PLAIN_ROW_MAJOR_TYPE(MatrixXd) drnea_dv(MatrixXd::Zero(model.nv,model.nv));
//  EIGEN_PLAIN_ROW_MAJOR_TYPE(MatrixXd)
  MatrixXd drnea_da(MatrixXd::Zero(model.nv,model.nv));
 
  MatrixXd daba_dq(MatrixXd::Zero(model.nv,model.nv));
  MatrixXd daba_dv(MatrixXd::Zero(model.nv,model.nv));
  Data::RowMatrixXs daba_dtau(Data::RowMatrixXs::Zero(model.nv,model.nv));
  
//  timer.tic();
//  SMOOTH(NBT)
//  {
//    forwardKinematics(model,data,qs[_smooth],qdots[_smooth],qddots[_smooth]);
//  }
//  std::cout << "FK= \t\t"; timer.toc(std::cout,NBT);
//
//  timer.tic();
//  SMOOTH(NBT)
//  {
//    computeForwardKinematicsDerivatives(model,data,qs[_smooth],qdots[_smooth],qddots[_smooth]);
//  }
//  std::cout << "FK derivatives= \t\t"; timer.toc(std::cout,NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    rnea(model,data,qs[_smooth],qdots[_smooth],qddots[_smooth]);
  }
  std::cout << "RNEA= \t\t"; timer.toc(std::cout,NBT);
  
  timer.tic();
  SMOOTH(NBT)
  {
    computeRNEADerivatives(model,data,qs[_smooth],qdots[_smooth],qddots[_smooth],
                           drnea_dq,drnea_dv,drnea_da);
  }
  std::cout << "RNEA derivatives= \t\t"; timer.toc(std::cout,NBT);
  
//  ad_rnea.compute();
//  timer.tic();
//  SMOOTH(NBT/100)
//  {
//    ad_rnea.eval(qs[_smooth],qdots[_smooth],qddots[_smooth]);
//  }
//  std::cout << "RNEA auto diff= \t\t"; timer.toc(std::cout,NBT/100);

  timer.tic();
  SMOOTH(NBT/100)
  {
    rnea_fd(model,data,qs[_smooth],qdots[_smooth],qddots[_smooth],
            drnea_dq,drnea_dv,drnea_da);
  }
  std::cout << "RNEA finite differences= \t\t"; timer.toc(std::cout,NBT/100);

  timer.tic();
  SMOOTH(NBT)
  {
    aba(model,data,qs[_smooth],qdots[_smooth],taus[_smooth]);
  }
  std::cout << "ABA= \t\t"; timer.toc(std::cout,NBT);
  
  timer.tic();
  SMOOTH(NBT)
  {
    computeABADerivatives(model,data,qs[_smooth],qdots[_smooth],taus[_smooth],
                          daba_dq,daba_dv,daba_dtau);
  }
  std::cout << "ABA derivatives= \t\t"; timer.toc(std::cout,NBT);
  
  timer.tic();
  SMOOTH(NBT)
  {
    aba_fd(model,data,qs[_smooth],qdots[_smooth],taus[_smooth],
           daba_dq,daba_dv,daba_dtau);
  }
  std::cout << "ABA finite differences= \t\t"; timer.toc(std::cout,NBT);
//
//  timer.tic();
//  SMOOTH(NBT)
//  {
//    computeMinverse(model,data,qs[_smooth]);
//  }
//  std::cout << "M.inverse() from ABA = \t\t"; timer.toc(std::cout,NBT);
//  
//  MatrixXd Minv(model.nv,model.nv); Minv.setZero();
//  timer.tic();
//  SMOOTH(NBT)
//  {
//    crba(model,data,qs[_smooth]);
//    cholesky::decompose(model,data);
//    cholesky::computeMinv(model,data,Minv);
//  }
//  std::cout << "Minv from Cholesky = \t\t"; timer.toc(std::cout,NBT);

  std::cout << "--" << std::endl;
  return 0;
}
