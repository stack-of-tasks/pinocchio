//
// Copyright (c) 2018-2020 CNRS INRIA
//

#ifndef __pinocchio_utils_code_generator_base_hpp__
#define __pinocchio_utils_code_generator_base_hpp__

#include "pinocchio/codegen/cppadcg.hpp"

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

namespace pinocchio
{
  
  template<typename _Scalar>
  struct CodeGenBase
  {
    typedef _Scalar Scalar;
    typedef CppAD::cg::CG<Scalar> CGScalar;
    typedef CppAD::AD<CGScalar> ADScalar;
    
    enum { Options = 0 };
    
    typedef pinocchio::ModelTpl<Scalar,Options> Model;
    typedef pinocchio::DataTpl<Scalar,Options> Data;
    typedef pinocchio::ModelTpl<CGScalar,Options> CGModel;
    typedef pinocchio::DataTpl<CGScalar,Options> CGData;
    typedef pinocchio::ModelTpl<ADScalar,Options> ADModel;
    typedef pinocchio::DataTpl<ADScalar,Options> ADData;
    
    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic,Options> MatrixXs;
    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1,Options> VectorXs;
    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic,Options|Eigen::RowMajor> RowMatrixXs;
    typedef Eigen::Matrix<ADScalar,Eigen::Dynamic,1,Options> ADVectorXs;
    typedef Eigen::Matrix<ADScalar,Eigen::Dynamic,Eigen::Dynamic,Options> ADMatrixXs;
    
    typedef typename Model::ConfigVectorType ConfigVectorType;
    typedef typename Model::TangentVectorType TangentVectorType;
    
    typedef typename ADModel::ConfigVectorType ADConfigVectorType;
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
    , library_name(library_name + "_" + model.name)
    , build_forward(true)
    , build_jacobian(true)
    {
      ad_X = ADVectorXs(dim_input);
      ad_Y = ADVectorXs(dim_output);
      
      y = VectorXs(ad_Y.size());
      
      jac = RowMatrixXs(ad_Y.size(),ad_X.size());
    }
    
    /// \brief build the mapping Y = f(X)
    virtual void buildMap() = 0;
    
    void initLib()
    {
      buildMap();
      
      // generates source code
      cgen_ptr = std::unique_ptr<CppAD::cg::ModelCSourceGen<Scalar> >(new CppAD::cg::ModelCSourceGen<Scalar>(ad_fun, function_name));
      cgen_ptr->setCreateForwardZero(build_forward);
      cgen_ptr->setCreateJacobian(build_jacobian);
      libcgen_ptr = std::unique_ptr<CppAD::cg::ModelLibraryCSourceGen<Scalar> >(new CppAD::cg::ModelLibraryCSourceGen<Scalar>(*cgen_ptr));
      
      dynamicLibManager_ptr
      = std::unique_ptr<CppAD::cg::DynamicModelLibraryProcessor<Scalar> >(new CppAD::cg::DynamicModelLibraryProcessor<Scalar>(*libcgen_ptr,library_name));
    }
    
    CppAD::cg::ModelCSourceGen<Scalar> & codeGenerator()
    { return *cgen_ptr; }
    
    void compileLib()
    {
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
      if(!existLib() && generate_if_not_exist)
        compileLib();
      
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
      assert(build_forward);
      
      generatedFun_ptr->ForwardZero(PINOCCHIO_EIGEN_CONST_CAST(Vector,x),y);
    }
    
    template<typename Vector>
    void evalJacobian(const Eigen::MatrixBase<Vector> & x)
    {
      assert(build_jacobian);
      
      CppAD::cg::ArrayView<const Scalar> x_(PINOCCHIO_EIGEN_CONST_CAST(Vector,x).data(),(size_t)x.size());
      CppAD::cg::ArrayView<Scalar> jac_(jac.data(),(size_t)jac.size());
      generatedFun_ptr->Jacobian(x_,jac_);
    }
    
    /// \brief Dimension of the input vector
    Eigen::DenseIndex getInputDimension() const { return ad_X.size(); }
    /// \brief Dimension of the output vector
    Eigen::DenseIndex getOutputDimension() const { return ad_Y.size(); }
    
  protected:
    
    ADModel ad_model;
    ADData ad_data;
    
    /// \brief Name of the function
    const std::string function_name;
    /// \brief Name of the library
    const std::string library_name;
    
    /// \brief Options to generate or not the source code for the evaluation function
    bool build_forward;
    
    /// \brief Options to build or not the Jacobian of he function
    bool build_jacobian;
    
    ADVectorXs ad_X, ad_Y;
    ADFun ad_fun;
    
    ADConfigVectorType ad_q, ad_q_plus;
    ADTangentVectorType ad_dq, ad_v, ad_a;
    
    VectorXs y;
    RowMatrixXs jac;
    
    std::unique_ptr<CppAD::cg::ModelCSourceGen<Scalar> > cgen_ptr;
    std::unique_ptr<CppAD::cg::ModelLibraryCSourceGen<Scalar> > libcgen_ptr;
    std::unique_ptr<CppAD::cg::DynamicModelLibraryProcessor<Scalar> > dynamicLibManager_ptr;
    std::unique_ptr<CppAD::cg::DynamicLib<Scalar> > dynamicLib_ptr;
    std::unique_ptr<CppAD::cg::GenericModel<Scalar> > generatedFun_ptr;
    
  }; // struct CodeGenBase
  
} // namespace pinocchio

#endif // ifndef __pinocchio_utils_code_generator_base_hpp__
