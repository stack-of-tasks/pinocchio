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

#include "pinocchio/multibody/sample-models.hpp"

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
  const int NBT = 1000 * 100;
#else
  const int NBT = 1;
  std::cout << "(the time score in debug mode is not relevant) " << std::endl;
#endif

  using CppAD::AD;
  using CppAD::NearEqual;

  enum
  {
    Options = 0
  };
  typedef double Scalar;
  typedef AD<Scalar> ADScalar;
  typedef CppAD::cg::CG<Scalar> CGScalar;
  typedef CppAD::AD<CGScalar> ADCGScalar;
  typedef CppAD::ADFun<CGScalar> ADCGFun;

  typedef Eigen::Matrix<ADScalar, Eigen::Dynamic, 1> ADVectorXs;
  typedef Eigen::Matrix<ADCGScalar, Eigen::Dynamic, 1, Options> ADCGVectorXs;
  typedef Eigen::Matrix<ADScalar, Eigen::Dynamic, Eigen::Dynamic> ADMatrixXs;
  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> RowMatrixXs;

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
  int nq = model.nq;
  int nv = model.nv;
  int nx = nq + 2 * nv;
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);

  ConfigVectorType q(nq);
  q = pinocchio::randomConfiguration(model);
  TangentVectorType v(TangentVectorType::Random(nv));
  TangentVectorType a(TangentVectorType::Random(nv));

  {
    Data data(model);

    std::vector<ConfigVectorType> qs;
    for (size_t it = 0; it < NBT; ++it)
    {
      q = pinocchio::randomConfiguration(model);
      qs.push_back(q);
    }

    timer.tic();
    SMOOTH(NBT)
    {
      pinocchio::crba(model, data, qs[_smooth], pinocchio::Convention::WORLD);
      data.M.triangularView<Eigen::StrictlyLower>() =
        data.M.transpose().triangularView<Eigen::StrictlyLower>();
    }
    std::cout << "Calculate data.M (JSIM) with crba = \t\t";
    timer.toc(std::cout, NBT);
  }

  {
    ADModel ad_model = model.cast<ADScalar>();
    ADData ad_data(ad_model);

    ADConfigVectorType ad_q = q.cast<ADScalar>();

    CppAD::Independent(ad_q);
    pinocchio::crba(ad_model, ad_data, ad_q, pinocchio::Convention::WORLD);
    ad_data.M.triangularView<Eigen::StrictlyLower>() =
      ad_data.M.transpose().triangularView<Eigen::StrictlyLower>();

    ADVectorXs M_vector =
      Eigen::Map<ADVectorXs>(ad_data.M.data(), ad_data.M.cols() * ad_data.M.rows());

    CppAD::ADFun<Scalar> ad_fun(ad_q, M_vector);
    ad_fun.function_name_set("ad_fun_crba");

    // create csrc_file
    std::string c_type = "double";
    std::string csrc_file = "./jit_JSIM_crba.c";
    std::ofstream ofs;
    ofs.open(csrc_file, std::ofstream::out);
    ad_fun.to_csrc(ofs, c_type);
    ofs.close();

    // create dll_file
    std::string dll_file = "./jit_JSIM_crba" DLL_EXT;
    CPPAD_TESTVECTOR(std::string) csrc_files(1);
    csrc_files[0] = csrc_file;
    std::map<std::string, std::string> options;
    std::string err_msg = CppAD::create_dll_lib(dll_file, csrc_files, options);
    if (err_msg != "")
    {
      std::cerr << "jit_JSIM_crba: err_msg = " << err_msg << "\n";
    }

    // dll_linker
    CppAD::link_dll_lib dll_linker(dll_file, err_msg);
    if (err_msg != "")
    {
      std::cerr << "jit_JSIM_crba: err_msg = " << err_msg << "\n";
    }

    std::string function_name = "cppad_jit_ad_fun_crba";
    void * void_ptr = dll_linker(function_name, err_msg);
    if (err_msg != "")
    {
      std::cerr << "jit_JSIM_crba: err_msg = " << err_msg << "\n";
    }

    using CppAD::jit_double;
    jit_double ad_fun_ptr = reinterpret_cast<jit_double>(void_ptr);

    CPPAD_TESTVECTOR(Scalar) x((size_t)nq);
    size_t compare_change = 0, nx = (size_t)nq, nM_vector = (size_t)nv * (size_t)nv;
    CPPAD_TESTVECTOR(Scalar) M_vector_jit(nM_vector);

    std::vector<CPPAD_TESTVECTOR(Scalar)> xs;
    for (size_t it = 0; it < NBT; ++it)
    {
      q = pinocchio::randomConfiguration(model);
      Eigen::Map<Data::ConfigVectorType>(x.data(), nq, 1) = q;
      xs.push_back(x);
    }

    timer.tic();
    SMOOTH(NBT)
    {
      ad_fun_ptr(nx, xs[_smooth].data(), nM_vector, M_vector_jit.data(), &compare_change);
    }
    std::cout << "Calculate data.M (JSIM) with crba using cppad-jit = \t\t";
    timer.toc(std::cout, NBT);
  }

  {
    ADCGModel ad_model(model.template cast<ADCGScalar>());
    ADCGData ad_data(ad_model);

    ADCGConfigVectorType ad_q = q.cast<ADCGScalar>();
    ad_q = q.cast<ADCGScalar>();

    std::unique_ptr<CppAD::cg::ModelCSourceGen<Scalar>> cgen_ptr;
    std::unique_ptr<CppAD::cg::ModelLibraryCSourceGen<Scalar>> libcgen_ptr;
    std::unique_ptr<CppAD::cg::DynamicModelLibraryProcessor<Scalar>> dynamicLibManager_ptr;
    std::unique_ptr<CppAD::cg::DynamicLib<Scalar>> dynamicLib_ptr;
    std::unique_ptr<CppAD::cg::GenericModel<Scalar>> generatedFun_ptr;

    const std::string & function_name = "crba";
    const std::string & library_name = "cg_crba_eval";
    const std::string & compile_options = "-Ofast";

    CppAD::Independent(ad_q);
    pinocchio::crba(ad_model, ad_data, ad_q, pinocchio::Convention::WORLD);
    ad_data.M.triangularView<Eigen::StrictlyLower>() =
      ad_data.M.transpose().triangularView<Eigen::StrictlyLower>();

    ADCGVectorXs M_vector =
      Eigen::Map<ADCGVectorXs>(ad_data.M.data(), ad_data.M.cols() * ad_data.M.rows());

    ADCGFun ad_fun;
    ad_fun.Dependent(ad_q, M_vector);
    ad_fun.optimize("no_compare_op");

    // generates source code
    cgen_ptr = std::unique_ptr<CppAD::cg::ModelCSourceGen<Scalar>>(
      new CppAD::cg::ModelCSourceGen<Scalar>(ad_fun, function_name));
    cgen_ptr->setCreateForwardZero(true);
    cgen_ptr->setCreateJacobian(false);
    libcgen_ptr = std::unique_ptr<CppAD::cg::ModelLibraryCSourceGen<Scalar>>(
      new CppAD::cg::ModelLibraryCSourceGen<Scalar>(*cgen_ptr));

    dynamicLibManager_ptr = std::unique_ptr<CppAD::cg::DynamicModelLibraryProcessor<Scalar>>(
      new CppAD::cg::DynamicModelLibraryProcessor<Scalar>(*libcgen_ptr, library_name));

    CppAD::cg::GccCompiler<Scalar> compiler(PINOCCHIO_CXX_COMPILER);
    std::vector<std::string> compile_flags = compiler.getCompileFlags();
    compile_flags[0] = compile_options;
    compiler.setCompileFlags(compile_flags);
    dynamicLibManager_ptr->createDynamicLibrary(compiler, false);

    const auto it = dynamicLibManager_ptr->getOptions().find("dlOpenMode");
    if (it == dynamicLibManager_ptr->getOptions().end())
    {
      dynamicLib_ptr.reset(new CppAD::cg::LinuxDynamicLib<Scalar>(
        dynamicLibManager_ptr->getLibraryName()
        + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION));
    }
    else
    {
      int dlOpenMode = std::stoi(it->second);
      dynamicLib_ptr.reset(new CppAD::cg::LinuxDynamicLib<Scalar>(
        dynamicLibManager_ptr->getLibraryName()
          + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION,
        dlOpenMode));
    }

    generatedFun_ptr = dynamicLib_ptr->model(function_name.c_str());

    CPPAD_TESTVECTOR(Scalar) x((size_t)nq);
    CPPAD_TESTVECTOR(Scalar) y((size_t)nv * (size_t)nv);

    std::vector<CPPAD_TESTVECTOR(Scalar)> xs;
    for (size_t it = 0; it < NBT; ++it)
    {
      q = pinocchio::randomConfiguration(model);
      Eigen::Map<Data::ConfigVectorType>(x.data(), nq, 1) = q;
      xs.push_back(x);
    }

    timer.tic();
    SMOOTH(NBT)
    {
      generatedFun_ptr->ForwardZero(xs[_smooth], y);
    }
    std::cout << "Calculate data.M (JSIM) with crba using cppadcg = \t\t";
    timer.toc(std::cout, NBT);
  }

  {
    Data data(model);

    std::vector<ConfigVectorType> qs, vs, as;
    for (size_t it = 0; it < NBT; ++it)
    {
      q = pinocchio::randomConfiguration(model);
      TangentVectorType v(TangentVectorType::Random(nv));
      TangentVectorType a(TangentVectorType::Random(nv));
      qs.push_back(q);
      vs.push_back(v);
      as.push_back(a);
    }

    timer.tic();
    SMOOTH(NBT)
    {
      computeRNEADerivatives(model, data, qs[_smooth], vs[_smooth], as[_smooth]);
    }
    std::cout << "Calculate dtau_dx using computeRNEADerivatives = \t\t";
    timer.toc(std::cout, NBT);
  }

  {
    ADModel ad_model = model.cast<ADScalar>();
    ADData ad_data(ad_model);

    ADConfigVectorType ad_q = q.cast<ADScalar>();
    ADTangentVectorType ad_v = v.cast<ADScalar>();
    ADTangentVectorType ad_a = a.cast<ADScalar>();
    ADConfigVectorType ad_X = ADConfigVectorType::Zero(nx);
    Eigen::DenseIndex i = 0;
    ad_X.segment(i, nq) = ad_q;
    i += nq;
    ad_X.segment(i, nv) = ad_v;
    i += nv;
    ad_X.segment(i, nv) = ad_a;
    i += nv;

    CppAD::Independent(ad_X);
    pinocchio::rnea(
      ad_model, ad_data, ad_X.segment(0, nq), ad_X.segment(nq, nv), ad_X.segment(nq + nv, nv));

    ADVectorXs ad_Y(nv);
    Eigen::Map<ADData::TangentVectorType>(ad_Y.data(), nv, 1) = ad_data.tau;

    CppAD::ADFun<Scalar> f(ad_X, ad_Y);
    CppAD::ADFun<ADScalar, Scalar> af = f.base2ad();

    CppAD::Independent(ad_X);
    ADMatrixXs dtau_dx = af.Jacobian(ad_X);
    ADVectorXs dtau_dx_vector(nv * nx);
    dtau_dx_vector = Eigen::Map<ADVectorXs>(dtau_dx.data(), dtau_dx.cols() * dtau_dx.rows());
    CppAD::ADFun<double> ad_fun(ad_X, dtau_dx_vector);

    ad_fun.function_name_set("ad_fun");

    // create csrc_file
    std::string c_type = "double";
    std::string csrc_file = "./jit_JSIM.c";
    std::ofstream ofs;
    ofs.open(csrc_file, std::ofstream::out);
    ad_fun.to_csrc(ofs, c_type);
    ofs.close();

    // create dll_file
    std::string dll_file = "./jit_JSIM" DLL_EXT;
    CPPAD_TESTVECTOR(std::string) csrc_files(1);
    csrc_files[0] = csrc_file;
    std::map<std::string, std::string> options;
    std::string err_msg = CppAD::create_dll_lib(dll_file, csrc_files, options);
    if (err_msg != "")
    {
      std::cerr << "jit_JSIM: err_msg = " << err_msg << "\n";
    }

    // dll_linker
    CppAD::link_dll_lib dll_linker(dll_file, err_msg);
    if (err_msg != "")
    {
      std::cerr << "jit_JSIM: err_msg = " << err_msg << "\n";
    }

    std::string function_name = "cppad_jit_ad_fun";
    void * void_ptr = dll_linker(function_name, err_msg);
    if (err_msg != "")
    {
      std::cerr << "jit_JSIM: err_msg = " << err_msg << "\n";
    }

    using CppAD::jit_double;
    jit_double ad_fun_ptr = reinterpret_cast<jit_double>(void_ptr);

    CPPAD_TESTVECTOR(Scalar) x((size_t)nx);
    std::vector<CPPAD_TESTVECTOR(Scalar)> xs;
    for (size_t it = 0; it < NBT; ++it)
    {
      q = pinocchio::randomConfiguration(model);
      TangentVectorType v(TangentVectorType::Random(nv));
      TangentVectorType a(TangentVectorType::Random(nv));
      ConfigVectorType _x = ConfigVectorType::Zero(nx);
      Eigen::DenseIndex i = 0;
      _x.segment(i, nq) = q;
      i += nq;
      _x.segment(i, nv) = v;
      i += nv;
      _x.segment(i, nv) = a;
      i += nv;

      Eigen::Map<Data::TangentVectorType>(x.data(), nx, 1) = _x;
      xs.push_back(x);
    }

    size_t compare_change = 0, nx_ = (size_t)nx, ndtau_dx = (size_t)nv * (size_t)nx;
    std::vector<double> dtau_dx_jit(ndtau_dx);

    timer.tic();
    SMOOTH(NBT)
    {
      ad_fun_ptr(nx_, xs[_smooth].data(), ndtau_dx, dtau_dx_jit.data(), &compare_change);
    }
    std::cout << "Calculate dtau_dx (rnea) with cppad using jit  = \t\t";
    timer.toc(std::cout, NBT);
  }

  {
    ADModel ad_model = model.cast<ADScalar>();
    ADData ad_data(ad_model);

    ADConfigVectorType ad_q = q.cast<ADScalar>();
    ADTangentVectorType ad_v = v.cast<ADScalar>();
    ADTangentVectorType ad_a = a.cast<ADScalar>();

    ADConfigVectorType ad_X = ADConfigVectorType::Zero(nq + 2 * nv);
    Eigen::DenseIndex i = 0;
    ad_X.segment(i, nq) = ad_q;
    i += nq;
    ad_X.segment(i, nv) = ad_v;
    i += nv;
    ad_X.segment(i, nv) = ad_a;
    i += nv;

    CppAD::Independent(ad_X);
    pinocchio::rnea(
      ad_model, ad_data, ad_X.segment(0, nq), ad_X.segment(nq, nv), ad_X.segment(nq + nv, nv));

    ADVectorXs ad_Y(nv);
    Eigen::Map<ADData::TangentVectorType>(ad_Y.data(), nv, 1) = ad_data.tau;

    CppAD::ADFun<Scalar> ad_fun(ad_X, ad_Y);

    CPPAD_TESTVECTOR(Scalar) x((size_t)nx);
    std::vector<CPPAD_TESTVECTOR(Scalar)> xs;
    for (size_t it = 0; it < NBT; ++it)
    {
      q = pinocchio::randomConfiguration(model);
      TangentVectorType v(TangentVectorType::Random(nv));
      TangentVectorType a(TangentVectorType::Random(nv));
      ConfigVectorType _x = ConfigVectorType::Zero(nq + 2 * nv);
      Eigen::DenseIndex i = 0;
      _x.segment(i, nq) = q;
      i += nq;
      _x.segment(i, nv) = v;
      i += nv;
      _x.segment(i, nv) = a;
      i += nv;

      Eigen::Map<Data::TangentVectorType>(x.data(), nx, 1) = _x;
      xs.push_back(x);
    }
    timer.tic();
    SMOOTH(NBT)
    {
      CPPAD_TESTVECTOR(Scalar) dtau_da = ad_fun.Jacobian(xs[_smooth]);
    }
    std::cout << "Calculate dtau_dx (rnea) with cppad = \t\t";
    timer.toc(std::cout, NBT);
  }

  {
    ADCGModel ad_model(model.template cast<ADCGScalar>());
    ADCGData ad_data(ad_model);

    ADCGVectorXs ad_Y(nv);

    ADCGConfigVectorType ad_q = q.cast<ADCGScalar>();
    ADCGTangentVectorType ad_v = v.cast<ADCGScalar>();
    ADCGTangentVectorType ad_a = a.cast<ADCGScalar>();
    ADCGConfigVectorType ad_X = ADCGConfigVectorType::Zero(nx);
    Eigen::DenseIndex i = 0;
    ad_X.segment(i, nq) = ad_q;
    i += nq;
    ad_X.segment(i, nv) = ad_v;
    i += nv;
    ad_X.segment(i, nv) = ad_a;
    i += nv;

    ADCGFun ad_fun;

    std::unique_ptr<CppAD::cg::ModelCSourceGen<Scalar>> cgen_ptr;
    std::unique_ptr<CppAD::cg::ModelLibraryCSourceGen<Scalar>> libcgen_ptr;
    std::unique_ptr<CppAD::cg::DynamicModelLibraryProcessor<Scalar>> dynamicLibManager_ptr;
    std::unique_ptr<CppAD::cg::DynamicLib<Scalar>> dynamicLib_ptr;
    std::unique_ptr<CppAD::cg::GenericModel<Scalar>> generatedFun_ptr;

    const std::string & function_name = "rnea";
    const std::string & library_name = "cg_rnea_eval";
    const std::string & compile_options = "-Ofast";

    CppAD::Independent(ad_X);
    pinocchio::rnea(
      ad_model, ad_data, ad_X.segment(0, nq), ad_X.segment(nq, nv), ad_X.segment(nq + nv, nv));
    ad_Y = ad_data.tau;
    ad_fun.Dependent(ad_X, ad_Y);
    ad_fun.optimize("no_compare_op");
    RowMatrixXs jac = RowMatrixXs::Zero(ad_Y.size(), ad_X.size());

    // generates source code
    cgen_ptr = std::unique_ptr<CppAD::cg::ModelCSourceGen<Scalar>>(
      new CppAD::cg::ModelCSourceGen<Scalar>(ad_fun, function_name));
    cgen_ptr->setCreateForwardZero(true);
    cgen_ptr->setCreateJacobian(true);
    libcgen_ptr = std::unique_ptr<CppAD::cg::ModelLibraryCSourceGen<Scalar>>(
      new CppAD::cg::ModelLibraryCSourceGen<Scalar>(*cgen_ptr));

    dynamicLibManager_ptr = std::unique_ptr<CppAD::cg::DynamicModelLibraryProcessor<Scalar>>(
      new CppAD::cg::DynamicModelLibraryProcessor<Scalar>(*libcgen_ptr, library_name));

    CppAD::cg::GccCompiler<Scalar> compiler(PINOCCHIO_CXX_COMPILER);
    std::vector<std::string> compile_flags = compiler.getCompileFlags();
    compile_flags[0] = compile_options;
    compiler.setCompileFlags(compile_flags);
    dynamicLibManager_ptr->createDynamicLibrary(compiler, false);

    const auto it = dynamicLibManager_ptr->getOptions().find("dlOpenMode");
    if (it == dynamicLibManager_ptr->getOptions().end())
    {
      dynamicLib_ptr.reset(new CppAD::cg::LinuxDynamicLib<Scalar>(
        dynamicLibManager_ptr->getLibraryName()
        + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION));
    }
    else
    {
      int dlOpenMode = std::stoi(it->second);
      dynamicLib_ptr.reset(new CppAD::cg::LinuxDynamicLib<Scalar>(
        dynamicLibManager_ptr->getLibraryName()
          + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION,
        dlOpenMode));
    }

    generatedFun_ptr = dynamicLib_ptr->model(function_name.c_str());

    CPPAD_TESTVECTOR(Scalar) x((size_t)nv);
    std::vector<size_t> xs;
    for (size_t it = 0; it < NBT; ++it)
    {
      ConfigVectorType q = pinocchio::randomConfiguration(model);
      TangentVectorType v(TangentVectorType::Random(nv));
      TangentVectorType a(TangentVectorType::Random(nv));

      ConfigVectorType x = ConfigVectorType::Zero(nq + 2 * nv);
      i = 0;
      x.segment(i, nq) = q;
      i += nq;
      x.segment(i, nv) = v;
      i += nv;
      x.segment(i, nv) = a;
      i += nv;

      xs.push_back(static_cast<size_t>(x.size()));
    }

    CppAD::cg::ArrayView<Scalar> jac_(jac.data(), (size_t)jac.size());

    timer.tic();
    SMOOTH(NBT)
    {
      generatedFun_ptr->Jacobian(CppAD::cg::ArrayView<const Scalar>(x.data(), xs[_smooth]), jac_);
    }
    std::cout << "Calculate dtau_dx (rnea) with cppad code gen = \t\t";
    timer.toc(std::cout, NBT);
  }

  if (DELETE_GENERATED_LIBS_AFTER_TEST)
  {
    std::remove("cg_rnea_eval.dylib");
    std::remove("jit_JSIM.c");
    std::remove("jit_JSIM.so");
    std::remove("jit_JSIM_crba.c");
    std::remove("jit_JSIM_crba.so");
  }

  std::cout << "--" << std::endl;
  return 0;
}
