//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/autodiff/cppad.hpp"

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
  
  typedef double Scalar;
  typedef AD<Scalar> ADScalar;
  
  typedef pinocchio::ModelTpl<Scalar> Model;
  typedef Model::Data Data;

  typedef pinocchio::ModelTpl<ADScalar> ADModel;
  typedef ADModel::Data ADData;
  
  Model model;
  pinocchio::buildModels::humanoidRandom(model);
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  
  ADModel ad_model = model.cast<ADScalar>();
  ADData ad_data(ad_model);
  
  typedef Model::ConfigVectorType ConfigVectorType;
  typedef Model::TangentVectorType TangentVectorType;
  ConfigVectorType q(model.nq);
  q = pinocchio::randomConfiguration(model);

  TangentVectorType v(TangentVectorType::Random(model.nv));
  TangentVectorType a(TangentVectorType::Random(model.nv));
  
  typedef ADModel::ConfigVectorType ADConfigVectorType;
  typedef ADModel::TangentVectorType ADTangentVectorType;
  
  ADConfigVectorType ad_q = q.cast<ADScalar>();
  ADTangentVectorType ad_v = v.cast<ADScalar>();
  ADTangentVectorType ad_a = a.cast<ADScalar>();
  
  typedef Eigen::Matrix<ADScalar,Eigen::Dynamic,1> VectorXAD;
  typedef Eigen::Matrix<ADScalar,Eigen::Dynamic,Eigen::Dynamic> MatrixXAD;

  {
    CppAD::Independent(ad_a);
    pinocchio::rnea(ad_model,ad_data,ad_q,ad_v,ad_a);

    VectorXAD Y(model.nv);
    Eigen::Map<ADData::TangentVectorType>(Y.data(),model.nv,1) = ad_data.tau;

    CppAD::ADFun<Scalar> f(ad_a,Y);
    CppAD::ADFun<ADScalar,Scalar > af = f.base2ad();

    CppAD::Independent(ad_a);
    MatrixXAD dtau_da = af.Jacobian(ad_a);
    VectorXAD dtau_da_vector(model.nv*model.nv);
    dtau_da_vector = Eigen::Map<VectorXAD>(dtau_da.data(), dtau_da.cols()*dtau_da.rows());
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
    std::cout << "Calculate JSIM with cppad using jit  = \t"; timer.toc(std::cout,NBT);
  }

  {
    CppAD::Independent(ad_a);
    pinocchio::rnea(ad_model,ad_data,ad_q,ad_v,ad_a);

    VectorXAD Y(model.nv);
    Eigen::Map<ADData::TangentVectorType>(Y.data(),model.nv,1) = ad_data.tau;

    CppAD::ADFun<Scalar> ad_fun(ad_a,Y);

    CPPAD_TESTVECTOR(Scalar) x((size_t)model.nv);
    Eigen::Map<Data::TangentVectorType>(x.data(),model.nv,1) = a;
    timer.tic();
    SMOOTH(NBT)
    {
      CPPAD_TESTVECTOR(Scalar) dtau_da = ad_fun.Jacobian(x);
    }
    std::cout << "Calculate JSIM with cppad = \t"; timer.toc(std::cout,NBT);
  }

  {
    Data data(model);
    timer.tic();
    SMOOTH(NBT)
    {
      pinocchio::rnea(model,data,q,v,a);
    }
    std::cout << "Calculate JSIM with rnea = \t"; timer.toc(std::cout,NBT);
  }

  std::cout << "--" << std::endl;
  return 0;
}
