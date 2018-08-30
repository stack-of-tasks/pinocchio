//
// Copyright (c) 2015-2018 CNRS
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

#include <iostream>

#include "pinocchio/utils/timer.hpp"

template<typename _Scalar>
struct CodeGenBase
{
  typedef _Scalar Scalar;
  typedef CppAD::cg::CG<Scalar> CGScalar;
  typedef CppAD::AD<CGScalar> ADScalar;
  
  enum { Options = 0 };
  
  typedef se3::ModelTpl<Scalar,Options> Model;
  typedef se3::DataTpl<Scalar,Options> Data;
  typedef se3::ModelTpl<CGScalar,Options> CGModel;
  typedef se3::DataTpl<CGScalar,Options> CGData;
  typedef se3::ModelTpl<ADScalar,Options> ADModel;
  typedef se3::DataTpl<ADScalar,Options> ADData;
  
  typedef Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic,Options> MatrixXs;
  typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1,Options> VectorXs;
  typedef Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic,Options|Eigen::RowMajor> RowMatrixXs;
  typedef Eigen::Matrix<ADScalar,Eigen::Dynamic,1,Options> ADVectorXs;
  
  typedef typename Model::ConfigVectorType CongigVectorType;
  typedef typename Model::TangentVectorType TangentVectorType;
  
  typedef typename ADModel::ConfigVectorType ADCongigVectorType;
  typedef typename ADModel::TangentVectorType ADTangentVectorType;
  
  typedef CppAD::ADFun<CGScalar> ADFun;
  
  CodeGenBase(const Model & model,
              const Eigen::DenseIndex dim_input,
              const Eigen::DenseIndex dim_output,
              const std::string & function_name,
              const std::string & library_name)
  : ad_model(model.template cast<ADScalar>())
  , ad_data(ad_model)
  , function_name(function_name)
  , library_name(library_name)
  {
    ad_X = ADVectorXs(model.nq+model.nv*2);
    ad_Y = ADVectorXs(model.nv);
    
    x = VectorXs(ad_X.size());
    y = VectorXs(ad_Y.size());
    
    jac = MatrixXs(y.size(),x.size());
  }
  
  /// \brief build the mapping Y = f(X)
  virtual void buildMap() = 0;
  
  void generateLib()
  {
    buildMap();
    
    // generates source code
    cgen_ptr = std::unique_ptr<CppAD::cg::ModelCSourceGen<Scalar> >(new CppAD::cg::ModelCSourceGen<Scalar>(ad_fun, function_name));
    cgen_ptr->setCreateJacobian(true);
    cgen_ptr->setCreateForwardZero(true);
    libcgen_ptr = std::unique_ptr<CppAD::cg::ModelLibraryCSourceGen<Scalar> >(new CppAD::cg::ModelLibraryCSourceGen<Scalar>(*cgen_ptr));
    
    dynamicLibManager_ptr
    = std::unique_ptr<CppAD::cg::DynamicModelLibraryProcessor<Scalar> >(new CppAD::cg::DynamicModelLibraryProcessor<Scalar>(*libcgen_ptr,library_name));
    
    CppAD::cg::GccCompiler<Scalar> compiler;
    std::vector<std::string> compile_options = compiler.getCompileFlags();
    compile_options[0] = "-Ofast";
    compiler.setCompileFlags(compile_options);
    dynamicLibManager_ptr->createDynamicLibrary(compiler,false);
  }
  
  bool existLib() const
  {
    const std::string filename = dynamicLibManager_ptr->getLibraryName() + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION;
    std::ifstream file(filename.c_str());
    return file.good();
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
    
    generatedFun_ptr = dynamicLib_ptr->model(function_name.c_str());
  }
  
  template<typename Vector>
  void evalFunction(const Eigen::MatrixBase<Vector> & x)
  {
    generatedFun_ptr->ForwardZero(x,y);
  }
  
  template<typename Vector>
  void evalJacobian(const Eigen::MatrixBase<Vector> & x)
  {
    generatedFun_ptr->Jacobian(x,jac);
  }
  
  Eigen::DenseIndex inputDimension() const { return (size_t)ad_X.size(); }
  Eigen::DenseIndex outputDimension() const { return (size_t)ad_Y.size(); }
  
protected:
  
  ADModel ad_model;
  ADData ad_data;
  
  const std::string function_name;
  const std::string library_name;
  
  ADVectorXs ad_X, ad_Y;
  ADFun ad_fun;
  
  ADCongigVectorType ad_q, ad_q_plus;
  ADTangentVectorType ad_dq, ad_v, ad_a;
  
  VectorXs x,y;
  RowMatrixXs jac;

  std::unique_ptr<CppAD::cg::ModelCSourceGen<Scalar> > cgen_ptr;
  std::unique_ptr<CppAD::cg::ModelLibraryCSourceGen<Scalar> > libcgen_ptr;
  std::unique_ptr<CppAD::cg::DynamicModelLibraryProcessor<Scalar> > dynamicLibManager_ptr;
  std::unique_ptr<CppAD::cg::DynamicLib<Scalar> > dynamicLib_ptr;
  std::unique_ptr<CppAD::cg::GenericModel<Scalar> > generatedFun_ptr;
  
};

template<typename _Scalar>
struct CodeGenRNEA
{
  typedef _Scalar Scalar;
  typedef CppAD::cg::CG<Scalar> CGScalar;
  typedef CppAD::AD<CGScalar> ADScalar;
  
  enum { Options = 0 };
  
  typedef se3::ModelTpl<Scalar,Options> Model;
  typedef se3::DataTpl<Scalar,Options> Data;
  typedef se3::ModelTpl<CGScalar,Options> CGModel;
  typedef se3::DataTpl<CGScalar,Options> CGData;
  typedef se3::ModelTpl<ADScalar,Options> ADModel;
  typedef se3::DataTpl<ADScalar,Options> ADData;
  
  typedef Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic,Options> MatrixXs;
  typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1,Options> VectorXs;
  typedef Eigen::Matrix<ADScalar,Eigen::Dynamic,1,Options> ADVectorXs;
  
  typedef typename Model::ConfigVectorType CongigVectorType;
  typedef typename Model::TangentVectorType TangentVectorType;
  
  typedef typename ADModel::ConfigVectorType ADCongigVectorType;
  typedef typename ADModel::TangentVectorType ADTangentVectorType;
  
  typedef CppAD::ADFun<CGScalar> ADFun;
  
  CodeGenRNEA(const Model & model,
              const std::string & function_name = "rnea",
              const std::string & library_name = "cg_rnea_eval")
  : ad_model(model.template cast<ADScalar>())
  , ad_data(ad_model)
  , function_name(function_name)
  {
    // Init variables
    ad_q = ADCongigVectorType(model.nq);
//    ad_q_plus = ADCongigVectorType(model.nq);
//    ad_dq = ADTangentVectorType(model.nv); ad_dq.setZero();
    ad_v = ADTangentVectorType(model.nv); ad_v.setZero();
    ad_a = ADTangentVectorType(model.nv); ad_a.setZero();
    
    ad_X = ADVectorXs(model.nq+model.nv*2);
    ad_Y = ADVectorXs(model.nv);
    
    x = VectorXs(ad_X.size());
    
    // Create the mapping Y = f(X)
    compute();
    
    // generates source code
    cgen_ptr = std::unique_ptr<CppAD::cg::ModelCSourceGen<Scalar> >(new CppAD::cg::ModelCSourceGen<Scalar>(ad_fun, function_name));
    //        cgen_ptr->setCreateJacobian(true);
    cgen_ptr->setCreateForwardZero(true);
    //    cgen.setCreateForwardOne(true);
    //    cgen.setCreateReverseOne(true);
    //    cgen.setCreateReverseTwo(true);
    libcgen_ptr = std::unique_ptr<CppAD::cg::ModelLibraryCSourceGen<Scalar> >(new CppAD::cg::ModelLibraryCSourceGen<Scalar>(*cgen_ptr));
    
    dynamicLibManager_ptr
    = std::unique_ptr<CppAD::cg::DynamicModelLibraryProcessor<Scalar> >(new CppAD::cg::DynamicModelLibraryProcessor<Scalar>(*libcgen_ptr,library_name));
  }
  
  void compute()
  {
//    ad_X.head(ad_model.nv).setZero();
//    size_t abort_op_index = 0;
//    bool   record_compare = false;
//    CppAD::Independent(ad_X,abort_op_index,record_compare,ad_q);
    CppAD::Independent(ad_X);
    
    Eigen::DenseIndex it = 0;
    ad_q = ad_X.segment(it,ad_model.nq); it += ad_model.nq;
    ad_v = ad_X.segment(it,ad_model.nv); it += ad_model.nv;
    ad_a = ad_X.segment(it,ad_model.nv); it += ad_model.nv;
    
    //    ad_q_plus = se3::integrate(ad_model,ad_q,ad_dq);
    se3::rnea(ad_model,ad_data,ad_q,ad_v,ad_a);
    
    ad_Y = ad_data.tau;
    
    ad_fun.Dependent(ad_X,ad_Y);
    //    ad_fun.optimize("no_compare_op");
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
    std::vector<std::string> compile_options = compiler.getCompileFlags();
    compile_options[0] = "-Ofast";
    compiler.setCompileFlags(compile_options);
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
    
    generatedFun_ptr = dynamicLib_ptr->model(function_name.c_str());
  }
  
  template<typename ConfigVectorType, typename TangentVector1, typename TangentVector2>
  void eval(const Eigen::MatrixBase<ConfigVectorType> & q,
            const Eigen::MatrixBase<TangentVector1> & v,
            const Eigen::MatrixBase<TangentVector2> & a)
  {
    // fill x
    Eigen::DenseIndex it = 0;
    x.segment(it,ad_model.nq) = q; it += ad_model.nq;
    x.segment(it,ad_model.nv) = v; it += ad_model.nv;
    x.segment(it,ad_model.nv) = a; it += ad_model.nv;
    
    tau = generatedFun_ptr->ForwardZero(x);
//    ad_fun.new_dynamic(q.derived());
//
//    CPPAD_TESTVECTOR(EvalScalar) x((size_t)(ad_X.size()));
//    Eigen::DenseIndex it = 0;
//    Eigen::Map<typename Data::TangentVectorType>(x.data(),ad_model.nv,1).setZero();
//    it += ad_model.nv;
//    Eigen::Map<typename Data::TangentVectorType>(x.data()+it,ad_model.nv,1) = v;
//    it += ad_model.nv;
//    Eigen::Map<typename Data::TangentVectorType>(x.data()+it,ad_model.nv,1) = a;
//    it += ad_model.nv;
//
//    CPPAD_TESTVECTOR(EvalScalar) dy_dx = ad_fun.Jacobian(x);
//    it = 0;
//    drnea_dq = Eigen::Map<typename EIGEN_PLAIN_ROW_MAJOR_TYPE(typename EvalData::MatrixXs)>(dy_dx.data(),ad_model.nv,ad_model.nv);
//    it += ad_model.nv * ad_model.nv;
//    drnea_dv = Eigen::Map<typename EIGEN_PLAIN_ROW_MAJOR_TYPE(typename EvalData::MatrixXs)>(dy_dx.data()+it,ad_model.nv,ad_model.nv);
//    it += ad_model.nv * ad_model.nv;
//    drnea_da = Eigen::Map<typename EIGEN_PLAIN_ROW_MAJOR_TYPE(typename EvalData::MatrixXs)>(dy_dx.data()+it,ad_model.nv,ad_model.nv);
//    it += ad_model.nv * ad_model.nv;
  }
protected:
  
  ADModel ad_model;
  ADData ad_data;
  
  const std::string function_name;
  const std::string library_name;
  
  ADVectorXs ad_X, ad_Y;
  ADFun ad_fun;
  
  ADCongigVectorType ad_q, ad_q_plus;
  ADTangentVectorType ad_dq, ad_v, ad_a;
  
  VectorXs x,y,tau;
  MatrixXs jac;
  
  std::unique_ptr<CppAD::cg::ModelCSourceGen<Scalar> > cgen_ptr;
  std::unique_ptr<CppAD::cg::ModelLibraryCSourceGen<Scalar> > libcgen_ptr;
  std::unique_ptr<CppAD::cg::DynamicModelLibraryProcessor<Scalar> > dynamicLibManager_ptr;
  std::unique_ptr<CppAD::cg::DynamicLib<Scalar> > dynamicLib_ptr;
  std::unique_ptr<CppAD::cg::GenericModel<Scalar> > generatedFun_ptr;

};

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
    
  se3::Model model;

  std::string filename = PINOCCHIO_SOURCE_DIR"/models/simple_humanoid.urdf";
  if(argc>1) filename = argv[1];
  if( filename == "HS") 
    se3::buildModels::humanoidSimple(model,true);
  else if( filename == "H2" )
    se3::buildModels::humanoid2d(model);
  else
    se3::urdf::buildModel(filename,JointModelFreeFlyer(),model);
  std::cout << "nq = " << model.nq << std::endl;

  se3::Data data(model);
  
  CodeGenRNEA<double> rnea_code_gen(model);
  rnea_code_gen.loadLib();

  se3::container::aligned_vector<VectorXd> qs     (NBT);
  se3::container::aligned_vector<VectorXd> qdots  (NBT);
  se3::container::aligned_vector<VectorXd> qddots (NBT);
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
    rnea_code_gen.eval(qs[_smooth],qdots[_smooth],qddots[_smooth]);
  }
  std::cout << "RNEA generated = \t\t"; timer.toc(std::cout,NBT);
  
  return 0;
}
