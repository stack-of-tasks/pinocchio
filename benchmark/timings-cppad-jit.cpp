//
// Copyright (c) 2022-2025 INRIA
//

#include "model-fixture.hpp"

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

#include <cppad/core/to_csrc.hpp>
#include <iostream>

#include <benchmark/benchmark.h>

#ifdef _WIN32
  #define DLL_EXT ".dll"
#else
  #define DLL_EXT ".so"
#endif

enum
{
  Options = 0
};
typedef double Scalar;
typedef CppAD::AD<Scalar> ADScalar;
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

struct CPPADJITFixture : benchmark::Fixture
{
  void SetUp(benchmark::State &)
  {
    model = MODEL;
    data = Data(model);
    int nx = model.nq + 2 * model.nv;

    const Eigen::VectorXd qmax(Eigen::VectorXd::Ones(model.nq));
    q = randomConfiguration(model, -qmax, qmax);
    v = TangentVectorType::Random(model.nv);
    a = TangentVectorType::Random(model.nv);
    tau = TangentVectorType::Random(model.nv);
    X = ConfigVectorType::Zero(nx);
    X.segment(0, model.nq) = q;
    X.segment(model.nq, model.nv) = v;
    X.segment(model.nq + model.nv, model.nv) = a;

    ad_model = model.cast<ADScalar>();
    ad_data = ADData(ad_model);
    ad_q = q.cast<ADScalar>();
    ad_v = v.cast<ADScalar>();
    ad_a = a.cast<ADScalar>();
    ad_X = ADConfigVectorType::Zero(nx);
    ad_X.segment(0, model.nq) = ad_q;
    ad_X.segment(model.nq, model.nv) = ad_v;
    ad_X.segment(model.nq + model.nv, model.nv) = ad_a;

    ad_cg_model = model.cast<ADCGScalar>();
    ad_cg_data = ADCGData(ad_cg_model);
    ad_cg_q = q.cast<ADCGScalar>();
    ad_cg_v = v.cast<ADCGScalar>();
    ad_cg_a = a.cast<ADCGScalar>();
    ad_cg_X = ADCGConfigVectorType::Zero(nx);
    ad_cg_X.segment(0, model.nq) = ad_cg_q;
    ad_cg_X.segment(model.nq, model.nv) = ad_cg_v;
    ad_cg_X.segment(model.nq + model.nv, model.nv) = ad_cg_a;
  }

  void TearDown(benchmark::State &)
  {
  }

  CppAD::jit_double toJIT(CppAD::ADFun<Scalar> & ad_fun, const std::string & suffix)
  {
    // create csrc_file
    std::string c_type = "double";
    std::string name = std::string("jit_JSIM_") + suffix;
    std::string csrc_file = std::string("./") + name + std::string(".c");
    std::ofstream ofs;
    ofs.open(csrc_file, std::ofstream::out);
    ad_fun.to_csrc(ofs, c_type);
    ofs.close();

    // create dll_file
    std::string dll_file = std::string("./") + name + std::string(DLL_EXT);
    CPPAD_TESTVECTOR(std::string) csrc_files(1);
    csrc_files[0] = csrc_file;
    std::map<std::string, std::string> options;
    std::string err_msg = CppAD::create_dll_lib(dll_file, csrc_files, options);
    if (err_msg != "")
    {
      std::cerr << name << ": err_msg = " << err_msg << "\n";
    }

    // dll_linker
    dll_linker = std::make_unique<CppAD::link_dll_lib>(dll_file, err_msg);
    if (err_msg != "")
    {
      std::cerr << name << ": err_msg = " << err_msg << "\n";
    }

    std::string function_name = std::string("cppad_jit_ad_fun_") + suffix;
    void * void_ptr = (*dll_linker)(function_name, err_msg);
    if (err_msg != "")
    {
      std::cerr << name << ": err_msg = " << err_msg << "\n";
    }

    return reinterpret_cast<CppAD::jit_double>(void_ptr);
  }

  std::unique_ptr<CppAD::cg::GenericModel<Scalar>>
  toBIN(ADCGFun & ad_cg_fun, const std::string & suffix, bool create_jacobian = false)
  {
    const std::string function_name = suffix;
    const std::string library_name = std::string("cg_") + suffix + std::string("_eval");
    const std::string compile_options = "-Ofast";

    // generates source code
    CppAD::cg::ModelCSourceGen<Scalar> cgen(ad_cg_fun, function_name);
    cgen.setCreateForwardZero(true);
    cgen.setCreateJacobian(create_jacobian);
    CppAD::cg::ModelLibraryCSourceGen<Scalar> libcgen(cgen);

    CppAD::cg::DynamicModelLibraryProcessor<Scalar> dynamicLibManager(libcgen, library_name);

    CppAD::cg::GccCompiler<Scalar> compiler(PINOCCHIO_CXX_COMPILER);
    std::vector<std::string> compile_flags = compiler.getCompileFlags();
    compile_flags[0] = compile_options;
    compiler.setCompileFlags(compile_flags);
    dynamicLibManager.createDynamicLibrary(compiler, false);

    const auto it = dynamicLibManager.getOptions().find("dlOpenMode");
    if (it == dynamicLibManager.getOptions().end())
    {
      dynamicLib_ptr = std::make_unique<CppAD::cg::LinuxDynamicLib<Scalar>>(
        dynamicLibManager.getLibraryName()
        + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION);
    }
    else
    {
      int dlOpenMode = std::stoi(it->second);
      dynamicLib_ptr = std::make_unique<CppAD::cg::LinuxDynamicLib<Scalar>>(
        dynamicLibManager.getLibraryName() + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION,
        dlOpenMode);
    }

    return dynamicLib_ptr->model(function_name.c_str());
  }

  Model model;
  Data data;
  ConfigVectorType q;
  TangentVectorType v;
  TangentVectorType a;
  TangentVectorType tau;
  ConfigVectorType X;

  ADModel ad_model;
  ADData ad_data;
  ADConfigVectorType ad_q;
  ADTangentVectorType ad_v;
  ADTangentVectorType ad_a;
  ADConfigVectorType ad_X;

  ADCGModel ad_cg_model;
  ADCGData ad_cg_data;
  ADCGConfigVectorType ad_cg_q;
  ADCGTangentVectorType ad_cg_v;
  ADCGTangentVectorType ad_cg_a;
  ADCGConfigVectorType ad_cg_X;

  // Used in toJIT to keep alive dll_linker that own void_ptr.
  std::unique_ptr<CppAD::link_dll_lib> dll_linker;
  // Used in toBIN to keep alive dynamicLib_ptr that own the compiled function.
  std::unique_ptr<CppAD::cg::DynamicLib<Scalar>> dynamicLib_ptr;

  static void GlobalSetUp(const ExtraArgs &)
  {
    pinocchio::buildModels::humanoidRandom(MODEL);
    MODEL.lowerPositionLimit.head<3>().fill(-1.);
    MODEL.upperPositionLimit.head<3>().fill(1.);
    std::cout << "nq = " << MODEL.nq << std::endl;
    std::cout << "nv = " << MODEL.nv << std::endl;
    std::cout << "name = " << MODEL.name << std::endl;
    std::cout << "--" << std::endl;
  }

  static Model MODEL;
};
Model CPPADJITFixture::MODEL;

static void CustomArguments(benchmark::internal::Benchmark * b)
{
  b->MinWarmUpTime(3.);
}

// CRBA_WORLD

PINOCCHIO_DONT_INLINE static void
crbaWorldCall(const pinocchio::Model & model, pinocchio::Data & data, const Eigen::VectorXd & q)
{
  pinocchio::crba(model, data, q, pinocchio::Convention::WORLD);
  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.transpose().triangularView<Eigen::StrictlyLower>();
}
BENCHMARK_DEFINE_F(CPPADJITFixture, CRBA_WORLD)(benchmark::State & st)
{
  for (auto _ : st)
  {
    crbaWorldCall(model, data, q);
  }
}
BENCHMARK_REGISTER_F(CPPADJITFixture, CRBA_WORLD)->Apply(CustomArguments);

// CRBA_WORLD_AD

PINOCCHIO_DONT_INLINE static void crbaWorldADCall(
  CppAD::ADFun<Scalar> & ad_fun,
  const CPPAD_TESTVECTOR(Scalar) & x,
  CPPAD_TESTVECTOR(Scalar) & M_vector)
{
  M_vector = ad_fun.Forward(0, x);
}
BENCHMARK_DEFINE_F(CPPADJITFixture, CRBA_WORLD_AD)(benchmark::State & st)
{
  CppAD::Independent(ad_q);
  pinocchio::crba(ad_model, ad_data, ad_q);
  ad_data.M.triangularView<Eigen::StrictlyLower>() =
    ad_data.M.transpose().triangularView<Eigen::StrictlyLower>();
  ADVectorXs M_vector =
    Eigen::Map<ADVectorXs>(ad_data.M.data(), ad_data.M.cols() * ad_data.M.rows());

  CppAD::ADFun<Scalar> ad_fun(ad_q, M_vector);
  ad_fun.function_name_set("ad_fun_crba");

  CPPAD_TESTVECTOR(Scalar) x((size_t)model.nq);
  Eigen::Map<Data::ConfigVectorType>(x.data(), model.nq, 1) = q;
  CPPAD_TESTVECTOR(Scalar) res_M_vector;

  for (auto _ : st)
  {
    crbaWorldADCall(ad_fun, x, res_M_vector);
  }
}
BENCHMARK_REGISTER_F(CPPADJITFixture, CRBA_WORLD_AD)->Apply(CustomArguments);

// CRBA_WORLD_AD_JIT

BENCHMARK_DEFINE_F(CPPADJITFixture, CRBA_WORLD_AD_JIT)(benchmark::State & st)
{
  CppAD::Independent(ad_q);
  pinocchio::crba(ad_model, ad_data, ad_q);
  ad_data.M.triangularView<Eigen::StrictlyLower>() =
    ad_data.M.transpose().triangularView<Eigen::StrictlyLower>();
  ADVectorXs M_vector =
    Eigen::Map<ADVectorXs>(ad_data.M.data(), ad_data.M.cols() * ad_data.M.rows());

  CppAD::ADFun<Scalar> ad_fun(ad_q, M_vector);
  ad_fun.function_name_set("ad_fun_crba");

  CPPAD_TESTVECTOR(Scalar) x((size_t)model.nq);
  Eigen::Map<Data::ConfigVectorType>(x.data(), model.nq, 1) = q;
  size_t compare_change = 0, nx = (size_t)model.nq, nM_vector = (size_t)model.nv * (size_t)model.nv;
  CPPAD_TESTVECTOR(Scalar) M_vector_jit(nM_vector);

  auto ad_fun_ptr = toJIT(ad_fun, "crba");
  for (auto _ : st)
  {
    ad_fun_ptr(nx, x.data(), nM_vector, M_vector_jit.data(), &compare_change);
  }
}
BENCHMARK_REGISTER_F(CPPADJITFixture, CRBA_WORLD_AD_JIT)->Apply(CustomArguments);

// CRBA_WORLD_AD_CG

BENCHMARK_DEFINE_F(CPPADJITFixture, CRBA_WORLD_AD_CG)(benchmark::State & st)
{
  CppAD::Independent(ad_cg_q);
  pinocchio::crba(ad_cg_model, ad_cg_data, ad_cg_q, pinocchio::Convention::WORLD);
  ad_cg_data.M.triangularView<Eigen::StrictlyLower>() =
    ad_cg_data.M.transpose().triangularView<Eigen::StrictlyLower>();

  ADCGVectorXs M_vector =
    Eigen::Map<ADCGVectorXs>(ad_cg_data.M.data(), ad_cg_data.M.cols() * ad_cg_data.M.rows());

  ADCGFun ad_cg_fun;
  ad_cg_fun.Dependent(ad_cg_q, M_vector);
  ad_cg_fun.optimize("no_compare_op");

  CPPAD_TESTVECTOR(Scalar) x((size_t)model.nq);
  CPPAD_TESTVECTOR(Scalar) y((size_t)model.nv * (size_t)model.nv);
  Eigen::Map<Data::ConfigVectorType>(x.data(), model.nq, 1) = q;

  auto generatedFun_ptr = toBIN(ad_cg_fun, "crba");

  for (auto _ : st)
  {
    generatedFun_ptr->ForwardZero(x, y);
  }
}
BENCHMARK_REGISTER_F(CPPADJITFixture, CRBA_WORLD_AD_CG)->Apply(CustomArguments);

// COMPUTE_RNEA_DERIVATIVES

PINOCCHIO_DONT_INLINE static void computeRNEADerivativesCall(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & v,
  const Eigen::VectorXd & a)
{
  pinocchio::computeRNEADerivatives(model, data, q, v, a);
}
BENCHMARK_DEFINE_F(CPPADJITFixture, COMPUTE_RNEA_DERIVATIVES)(benchmark::State & st)
{
  for (auto _ : st)
  {
    computeRNEADerivativesCall(model, data, q, v, a);
  }
}
BENCHMARK_REGISTER_F(CPPADJITFixture, COMPUTE_RNEA_DERIVATIVES)->Apply(CustomArguments);

// COMPUTE_RNEA_DERIVATIVES_AD

PINOCCHIO_DONT_INLINE static void computeRNEADerivativesADCall(
  CppAD::ADFun<Scalar> & ad_fun,
  const CPPAD_TESTVECTOR(Scalar) & x,
  CPPAD_TESTVECTOR(Scalar) & dtau_da)
{
  dtau_da = ad_fun.Jacobian(x);
}
BENCHMARK_DEFINE_F(CPPADJITFixture, COMPUTE_RNEA_DERIVATIVES_AD)(benchmark::State & st)
{
  CppAD::Independent(ad_X);
  pinocchio::rnea(
    ad_model, ad_data, ad_X.segment(0, model.nq), ad_X.segment(model.nq, model.nv),
    ad_X.segment(model.nq + model.nv, model.nv));

  ADVectorXs ad_Y(model.nv);
  Eigen::Map<ADData::TangentVectorType>(ad_Y.data(), model.nv, 1) = ad_data.tau;

  CppAD::ADFun<Scalar> ad_fun(ad_X, ad_Y);

  int nx = model.nq + 2 * model.nv;
  CPPAD_TESTVECTOR(Scalar) x((size_t)nx);
  Eigen::Map<Data::TangentVectorType>(x.data(), nx, 1) = X;
  CPPAD_TESTVECTOR(Scalar) res_dtau_da;

  for (auto _ : st)
  {
    computeRNEADerivativesADCall(ad_fun, x, res_dtau_da);
  }
}
BENCHMARK_REGISTER_F(CPPADJITFixture, COMPUTE_RNEA_DERIVATIVES_AD)->Apply(CustomArguments);

// COMPUTE_RNEA_DERIVATIVES_AD_JIT

BENCHMARK_DEFINE_F(CPPADJITFixture, COMPUTE_RNEA_DERIVATIVES_AD_JIT)(benchmark::State & st)
{
  CppAD::Independent(ad_X);
  pinocchio::rnea(
    ad_model, ad_data, ad_X.segment(0, model.nq), ad_X.segment(model.nq, model.nv),
    ad_X.segment(model.nq + model.nv, model.nv));

  ADVectorXs ad_Y(model.nv);
  Eigen::Map<ADData::TangentVectorType>(ad_Y.data(), model.nv, 1) = ad_data.tau;

  CppAD::ADFun<Scalar> f(ad_X, ad_Y);
  CppAD::ADFun<ADScalar, Scalar> af = f.base2ad();

  int nx = model.nq + 2 * model.nv;

  CppAD::Independent(ad_X);
  ADMatrixXs dtau_dx = af.Jacobian(ad_X);
  ADVectorXs dtau_dx_vector(model.nv * nx);
  dtau_dx_vector = Eigen::Map<ADVectorXs>(dtau_dx.data(), dtau_dx.cols() * dtau_dx.rows());
  CppAD::ADFun<double> ad_fun(ad_X, dtau_dx_vector);
  ad_fun.function_name_set("ad_fun_rnea_derivatives");

  CPPAD_TESTVECTOR(Scalar) x((size_t)nx);
  Eigen::Map<Data::TangentVectorType>(x.data(), nx, 1) = X;
  size_t compare_change = 0, nx_ = (size_t)nx, ndtau_dx = (size_t)model.nv * (size_t)nx;
  std::vector<double> dtau_dx_jit(ndtau_dx);

  auto ad_fun_ptr = toJIT(ad_fun, "rnea_derivatives");
  for (auto _ : st)
  {
    ad_fun_ptr(nx_, x.data(), ndtau_dx, dtau_dx_jit.data(), &compare_change);
  }
}
BENCHMARK_REGISTER_F(CPPADJITFixture, COMPUTE_RNEA_DERIVATIVES_AD_JIT)->Apply(CustomArguments);

// COMPUTE_RNEA_DERIVATIVES_AD_CG

BENCHMARK_DEFINE_F(CPPADJITFixture, COMPUTE_RNEA_DERIVATIVES_AD_CG)(benchmark::State & st)
{
  const int nx = model.nq + 2 * model.nv;

  ADCGFun ad_cg_fun;

  CppAD::Independent(ad_cg_X);
  pinocchio::rnea(
    ad_cg_model, ad_cg_data, ad_cg_X.segment(0, model.nq), ad_cg_X.segment(model.nq, model.nv),
    ad_cg_X.segment(model.nq + model.nv, model.nv));
  ADCGVectorXs ad_cg_Y(model.nv);
  Eigen::Map<ADCGData::TangentVectorType>(ad_cg_Y.data(), model.nv, 1) = ad_cg_data.tau;
  ad_cg_fun.Dependent(ad_cg_X, ad_cg_Y);
  ad_cg_fun.optimize("no_compare_op");

  RowMatrixXs jac = RowMatrixXs::Zero(ad_cg_Y.size(), ad_cg_X.size());
  CPPAD_TESTVECTOR(Scalar) x((size_t)nx);
  Eigen::Map<Data::TangentVectorType>(x.data(), nx, 1) = X;

  auto generatedFun_ptr = toBIN(ad_cg_fun, "crba_derivatives", true);
  CppAD::cg::ArrayView<Scalar> jac_(jac.data(), (size_t)jac.size());

  for (auto _ : st)
  {
    generatedFun_ptr->Jacobian(x, jac_);
  }
}
BENCHMARK_REGISTER_F(CPPADJITFixture, COMPUTE_RNEA_DERIVATIVES_AD_CG)->Apply(CustomArguments);

PINOCCHIO_BENCHMARK_MAIN_WITH_SETUP(CPPADJITFixture::GlobalSetUp);
