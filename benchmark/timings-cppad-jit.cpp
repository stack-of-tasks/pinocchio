//
// Copyright (c) 2022 INRIA
//
#include "pinocchio/codegen/cppadcg.hpp"
#include "pinocchio/codegen/code-generator-algo.hpp"
#include "pinocchio/container/aligned-vector.hpp"

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include "pinocchio/parsers/sample-models.hpp"

#include <iostream>

#ifdef _WIN32
  #define DLL_EXT ".dll"
#else
  #define DLL_EXT ".so"
#endif

#include "pinocchio/utils/timer.hpp"

bool DELETE_GENERATED_LIBS_AFTER_TEST = true;

int main()
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

  using CppAD::AD;
  using CppAD::NearEqual;
  
  enum { Options = 0 };
  typedef double Scalar;
  typedef AD<Scalar> ADScalar;
  typedef CppAD::cg::CG<Scalar> CGScalar;
  typedef CppAD::AD<CGScalar> ADCGScalar;
  typedef CppAD::ADFun<CGScalar> ADCGFun;

  typedef Eigen::Matrix<ADScalar,Eigen::Dynamic,1> ADVectorXs;
  typedef Eigen::Matrix<ADCGScalar,Eigen::Dynamic,1,Options> ADCGVectorXs;
  typedef Eigen::Matrix<ADScalar,Eigen::Dynamic,Eigen::Dynamic> ADMatrixXs;

  typedef pinocchio::ModelTpl<Scalar> Model;
  typedef Model::Data Data;

  typedef pinocchio::ModelTpl<ADScalar> ADModel;
  typedef ADModel::Data ADData;

  typedef pinocchio::ModelTpl<ADCGScalar> ADCGModel;
  typedef ADCGModel::Data ADCGData;
  
  typedef Model::ConfigVectorType ConfigVectorType;
  typedef Model::TangentVectorType TangentVectorType;

  typedef ADModel::ConfigVectorType ADConfigVectorType;
  typedef ADModel::TangentVectorType ADTangentVectorType;

  typedef ADCGModel::ConfigVectorType ADCGConfigVectorType;
  typedef ADCGModel::TangentVectorType ADCGTangentVectorType;

  Model model;
  pinocchio::buildModels::humanoidRandom(model);
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  
  ConfigVectorType q(model.nq);
  q = pinocchio::randomConfiguration(model);
  TangentVectorType v(TangentVectorType::Random(model.nv));
  TangentVectorType a(TangentVectorType::Random(model.nv));
  
  {
    Data data(model);
    timer.tic();
    SMOOTH(NBT)
    {
      pinocchio::crba(model,data,q);
      data.M.triangularView<Eigen::StrictlyLower>()
      = data.M.transpose().triangularView<Eigen::StrictlyLower>();
    }
    std::cout << "Calculate data.M (JSIM) with crba = \t\t"; timer.toc(std::cout,NBT);
  }
  
  {
    ADModel ad_model = model.cast<ADScalar>();
    ADData ad_data(ad_model);
    
    ADConfigVectorType ad_q = q.cast<ADScalar>();
    
    CppAD::Independent(ad_q);
    pinocchio::crba(ad_model,ad_data,ad_q);
    ad_data.M.triangularView<Eigen::StrictlyLower>()
    = ad_data.M.transpose().triangularView<Eigen::StrictlyLower>();

    ADVectorXs M_vector = Eigen::Map<ADVectorXs>(ad_data.M.data(), ad_data.M.cols()*ad_data.M.rows());

    CppAD::ADFun<Scalar> ad_fun(ad_q, M_vector);
    ad_fun.function_name_set("ad_fun_crba");

    // create csrc_file
    std::string c_type    =  "double";
    std::string csrc_file = "jit_JSIM_crba.c";
    std::ofstream ofs;
    ofs.open(csrc_file , std::ofstream::out);
    ad_fun.to_csrc(ofs, c_type);
    ofs.close();

    // create dll_file
    std::string dll_file = "jit_JSIM_crba" DLL_EXT;
    CPPAD_TESTVECTOR( std::string) csrc_files(1);
    csrc_files[0] = csrc_file;
    std::map< std::string, std::string > options;
    std::string err_msg = CppAD::create_dll_lib(dll_file, csrc_files, options);
    if( err_msg != "" )
    {   
      std::cerr << "jit_JSIM_crba: err_msg = " << err_msg << "\n";
    }

    // dll_linker
    CppAD::link_dll_lib dll_linker(dll_file, err_msg);
    if( err_msg != "" )
    {   
      std::cerr << "jit_JSIM_crba: err_msg = " << err_msg << "\n";
    }

    std::string function_name = "cppad_jit_ad_fun_crba";
    void* void_ptr = dll_linker(function_name, err_msg);
    if( err_msg != "" )
    {   
      std::cerr << "jit_JSIM_crba: err_msg = " << err_msg << "\n";
    }

    using CppAD::jit_double;
    jit_double ad_fun_ptr =
        reinterpret_cast<jit_double>(void_ptr);

    CPPAD_TESTVECTOR(Scalar) x((size_t)model.nq);
    Eigen::Map<Data::TangentVectorType>(x.data(),model.nq,1) = q;

    size_t compare_change = 0, nx = (size_t)model.nq, nM_vector = (size_t)model.nv*(size_t)model.nv;
    CPPAD_TESTVECTOR(Scalar) M_vector_jit(nM_vector);
    
    timer.tic();
    SMOOTH(NBT)
    {
      ad_fun_ptr(nx, x.data(), nM_vector, M_vector_jit.data(), &compare_change);   
    }
    std::cout << "Calculate data.M (JSIM) with crba using cppad-jit = \t\t"; timer.toc(std::cout,NBT);
  }

  {
    ADCGModel ad_model(model.template cast<ADCGScalar>());
    ADCGData ad_data(ad_model);

    ADCGConfigVectorType ad_q = q.cast<ADCGScalar>();
    ad_q = q.cast<ADCGScalar>();

    std::unique_ptr<CppAD::cg::ModelCSourceGen<Scalar> > cgen_ptr;
    std::unique_ptr<CppAD::cg::ModelLibraryCSourceGen<Scalar> > libcgen_ptr;
    std::unique_ptr<CppAD::cg::DynamicModelLibraryProcessor<Scalar> > dynamicLibManager_ptr;
    std::unique_ptr<CppAD::cg::DynamicLib<Scalar> > dynamicLib_ptr;
    std::unique_ptr<CppAD::cg::GenericModel<Scalar> > generatedFun_ptr;

    const std::string & function_name = "crba";
    const std::string & library_name = "cg_crba_eval";
    const std::string & compile_options = "-Ofast";

    CppAD::Independent(ad_q);
    pinocchio::crba(ad_model,ad_data,ad_q);
    ad_data.M.triangularView<Eigen::StrictlyLower>()
    = ad_data.M.transpose().triangularView<Eigen::StrictlyLower>();

    ADCGVectorXs M_vector = Eigen::Map<ADCGVectorXs>(ad_data.M.data(), ad_data.M.cols()*ad_data.M.rows());

    ADCGFun ad_fun;
    ad_fun.Dependent(ad_q,M_vector);
    ad_fun.optimize("no_compare_op");

    // generates source code
    cgen_ptr = std::unique_ptr<CppAD::cg::ModelCSourceGen<Scalar> >(new CppAD::cg::ModelCSourceGen<Scalar>(ad_fun, function_name));
    cgen_ptr->setCreateForwardZero(true);
    cgen_ptr->setCreateJacobian(false);
    libcgen_ptr = std::unique_ptr<CppAD::cg::ModelLibraryCSourceGen<Scalar> >(new CppAD::cg::ModelLibraryCSourceGen<Scalar>(*cgen_ptr));
    
    dynamicLibManager_ptr
    = std::unique_ptr<CppAD::cg::DynamicModelLibraryProcessor<Scalar> >(new CppAD::cg::DynamicModelLibraryProcessor<Scalar>(*libcgen_ptr,library_name));
  
    CppAD::cg::GccCompiler<Scalar> compiler;
    std::vector<std::string> compile_flags = compiler.getCompileFlags();
    compile_flags[0] = compile_options;
    compiler.setCompileFlags(compile_flags);
    dynamicLibManager_ptr->createDynamicLibrary(compiler,false);

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

    CPPAD_TESTVECTOR(Scalar) x((size_t)model.nq);
    CPPAD_TESTVECTOR(Scalar) y((size_t)model.nv*(size_t)model.nv);
    Eigen::Map<TangentVectorType>(x.data(),model.nq,1) = q;

    timer.tic();
    SMOOTH(NBT)
    {
      generatedFun_ptr->ForwardZero(x,y);
    }
    std::cout << "Calculate data.M (JSIM) with crba using cppadcg = \t\t"; timer.toc(std::cout,NBT);
  }

  {
    ADModel ad_model = model.cast<ADScalar>();
    ADData ad_data(ad_model);

    ADConfigVectorType ad_q = q.cast<ADScalar>();
    ADTangentVectorType ad_v = v.cast<ADScalar>();
    ADTangentVectorType ad_a = a.cast<ADScalar>();
    
    CppAD::Independent(ad_a);
    pinocchio::rnea(ad_model,ad_data,ad_q,ad_v,ad_a);

    ADVectorXs ad_Y(model.nv);
    Eigen::Map<ADData::TangentVectorType>(ad_Y.data(),model.nv,1) = ad_data.tau;

    CppAD::ADFun<Scalar> f(ad_a,ad_Y);
    CppAD::ADFun<ADScalar,Scalar > af = f.base2ad();

    CppAD::Independent(ad_a);
    ADMatrixXs dtau_da = af.Jacobian(ad_a);
    ADVectorXs dtau_da_vector(model.nv*model.nv);
    dtau_da_vector = Eigen::Map<ADVectorXs>(dtau_da.data(), dtau_da.cols()*dtau_da.rows());
    CppAD::ADFun<double> ad_fun(ad_a, dtau_da_vector);

    ad_fun.function_name_set("ad_fun");

    // create csrc_file
    std::string c_type    =  "double";
    std::string csrc_file = "jit_JSIM.c";
    std::ofstream ofs;
    ofs.open(csrc_file , std::ofstream::out);
    ad_fun.to_csrc(ofs, c_type);
    ofs.close();

    // create dll_file
    std::string dll_file = "jit_JSIM" DLL_EXT;
    CPPAD_TESTVECTOR( std::string) csrc_files(1);
    csrc_files[0] = csrc_file;
    std::map< std::string, std::string > options;
    std::string err_msg = CppAD::create_dll_lib(dll_file, csrc_files, options);
    if( err_msg != "" )
    {   
      std::cerr << "jit_JSIM: err_msg = " << err_msg << "\n";
    }

    // dll_linker
    CppAD::link_dll_lib dll_linker(dll_file, err_msg);
    if( err_msg != "" )
    {   
      std::cerr << "jit_JSIM: err_msg = " << err_msg << "\n";
    }

    std::string function_name = "cppad_jit_ad_fun";
    void* void_ptr = dll_linker(function_name, err_msg);
    if( err_msg != "" )
    {   
      std::cerr << "jit_JSIM: err_msg = " << err_msg << "\n";
    }

    using CppAD::jit_double;
    jit_double ad_fun_ptr =
        reinterpret_cast<jit_double>(void_ptr);

    CPPAD_TESTVECTOR(Scalar) x((size_t)model.nv);
    Eigen::Map<Data::TangentVectorType>(x.data(),model.nv,1) = a;

    size_t compare_change = 0, nx = (size_t)model.nv, ndtau_da_ = (size_t)model.nv*(size_t)model.nv;
    std::vector<double> dtau_da_jit(ndtau_da_);
    
    timer.tic();
    SMOOTH(NBT)
    {
      ad_fun_ptr(nx, x.data(), ndtau_da_, dtau_da_jit.data(), &compare_change);   
    }
    std::cout << "Calculate dtau_da (JSIM) as rnea jacobian with cppad using jit  = \t\t"; timer.toc(std::cout,NBT);
  }

  {
    ADModel ad_model = model.cast<ADScalar>();
    ADData ad_data(ad_model);

    ADConfigVectorType ad_q = q.cast<ADScalar>();
    ADTangentVectorType ad_v = v.cast<ADScalar>();
    ADTangentVectorType ad_a = a.cast<ADScalar>();
    
    CppAD::Independent(ad_a);
    pinocchio::rnea(ad_model,ad_data,ad_q,ad_v,ad_a);

    ADVectorXs ad_Y(model.nv);
    Eigen::Map<ADData::TangentVectorType>(ad_Y.data(),model.nv,1) = ad_data.tau;

    CppAD::ADFun<Scalar> ad_fun(ad_a,ad_Y);

    CPPAD_TESTVECTOR(Scalar) x((size_t)model.nv);
    Eigen::Map<Data::TangentVectorType>(x.data(),model.nv,1) = a;
    timer.tic();
    SMOOTH(NBT)
    {
      CPPAD_TESTVECTOR(Scalar) dtau_da = ad_fun.Jacobian(x);
    }
    std::cout << "Calculate dtau_da (JSIM) as rnea jacobian with cppad = \t\t"; timer.toc(std::cout,NBT);
  }

  {
    ADCGModel ad_model(model.template cast<ADCGScalar>());
    ADCGData ad_data(ad_model);

    ADCGVectorXs ad_Y(model.nv);

    ADCGConfigVectorType ad_q = q.cast<ADCGScalar>();
    ADCGTangentVectorType ad_v = v.cast<ADCGScalar>();
    ADCGTangentVectorType ad_a = a.cast<ADCGScalar>();

    ad_q = q.cast<ADCGScalar>();
    ad_v = v.cast<ADCGScalar>();
    ad_a = a.cast<ADCGScalar>();

    ADCGFun ad_fun;
    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> RowMatrixXs;
    RowMatrixXs jac;

    std::unique_ptr<CppAD::cg::ModelCSourceGen<Scalar> > cgen_ptr;
    std::unique_ptr<CppAD::cg::ModelLibraryCSourceGen<Scalar> > libcgen_ptr;
    std::unique_ptr<CppAD::cg::DynamicModelLibraryProcessor<Scalar> > dynamicLibManager_ptr;
    std::unique_ptr<CppAD::cg::DynamicLib<Scalar> > dynamicLib_ptr;
    std::unique_ptr<CppAD::cg::GenericModel<Scalar> > generatedFun_ptr;

    const std::string & function_name = "rnea";
    const std::string & library_name = "cg_rnea_eval";
    const std::string & compile_options = "-Ofast";

    CppAD::Independent(ad_a);
    pinocchio::rnea(ad_model,ad_data,ad_q,ad_v,ad_a);
    ad_Y = ad_data.tau;    
    ad_fun.Dependent(ad_a,ad_Y);
    ad_fun.optimize("no_compare_op");
    jac = RowMatrixXs(ad_Y.size(),ad_a.size());

    // generates source code
    cgen_ptr = std::unique_ptr<CppAD::cg::ModelCSourceGen<Scalar> >(new CppAD::cg::ModelCSourceGen<Scalar>(ad_fun, function_name));
    cgen_ptr->setCreateForwardZero(true);
    cgen_ptr->setCreateJacobian(true);
    libcgen_ptr = std::unique_ptr<CppAD::cg::ModelLibraryCSourceGen<Scalar> >(new CppAD::cg::ModelLibraryCSourceGen<Scalar>(*cgen_ptr));
    
    dynamicLibManager_ptr
    = std::unique_ptr<CppAD::cg::DynamicModelLibraryProcessor<Scalar> >(new CppAD::cg::DynamicModelLibraryProcessor<Scalar>(*libcgen_ptr,library_name));
  
    CppAD::cg::GccCompiler<Scalar> compiler;
    std::vector<std::string> compile_flags = compiler.getCompileFlags();
    compile_flags[0] = compile_options;
    compiler.setCompileFlags(compile_flags);
    dynamicLibManager_ptr->createDynamicLibrary(compiler,false);

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

    CPPAD_TESTVECTOR(Scalar) x((size_t)model.nv);
    Eigen::Map<Data::TangentVectorType>(x.data(),model.nv,1) = a;
    CppAD::cg::ArrayView<const Scalar> x_(x.data(),(size_t)x.size());
    CppAD::cg::ArrayView<Scalar> jac_(jac.data(),(size_t)jac.size());

    timer.tic();
    SMOOTH(NBT)
    {
      generatedFun_ptr->Jacobian(x_,jac_);
    }
    std::cout << "Calculate dtau_da (JSIM) as rnea jacobian with cppad code gen = \t\t"; timer.toc(std::cout,NBT);
  }

  std::this_thread::sleep_for(std::chrono::seconds(2));
  if (DELETE_GENERATED_LIBS_AFTER_TEST){
    std::remove("cg_rnea_eval.dylib");
    std::remove("jit_JSIM.c");
    std::remove("jit_JSIM.so");
    std::remove("jit_JSIM_crba.c");
    std::remove("jit_JSIM_crba.so");
  }

  std::cout << "--" << std::endl;
  return 0;
}
