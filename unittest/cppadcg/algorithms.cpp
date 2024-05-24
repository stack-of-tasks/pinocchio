//
// Copyright (c) 2018-2019 CNRS INRIA
//

#include "pinocchio/codegen/cppadcg.hpp"

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include "pinocchio/multibody/sample-models.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_crba_code_generation)
{
  typedef double Scalar;
  typedef CppAD::cg::CG<Scalar> CGScalar;
  typedef CppAD::AD<CGScalar> ADScalar;

  typedef Eigen::Matrix<ADScalar, Eigen::Dynamic, 1> ADVector;

  typedef pinocchio::ModelTpl<Scalar> Model;
  typedef Model::Data Data;

  typedef pinocchio::ModelTpl<ADScalar> ADModel;
  typedef ADModel::Data ADData;

  Model model;
  pinocchio::buildModels::humanoidRandom(model);
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  Data data(model);

  ADModel ad_model = model.cast<ADScalar>();
  ADData ad_data(ad_model);

  // Sample random configuration
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

  ADTangentVectorType & X = ad_a;
  CppAD::Independent(X);

  pinocchio::rnea(ad_model, ad_data, ad_q, ad_v, ad_a);
  ADVector Y(model.nv);
  Y = ad_data.tau;

  CppAD::ADFun<CGScalar> fun(X, Y);

  // generates source code
  CppAD::cg::ModelCSourceGen<Scalar> cgen(fun, "rnea");
  cgen.setCreateJacobian(true);
  cgen.setCreateForwardZero(true);
  cgen.setCreateForwardOne(true);
  cgen.setCreateReverseOne(true);
  cgen.setCreateReverseTwo(true);
  CppAD::cg::ModelLibraryCSourceGen<Scalar> libcgen(cgen);

  // compile source code
  CppAD::cg::DynamicModelLibraryProcessor<Scalar> p(libcgen);

  CppAD::cg::GccCompiler<Scalar> compiler(PINOCCHIO_CXX_COMPILER);
  std::unique_ptr<CppAD::cg::DynamicLib<Scalar>> dynamicLib = p.createDynamicLibrary(compiler);

  // save to files (not really required)
  CppAD::cg::SaveFilesModelLibraryProcessor<Scalar> p2(libcgen);
  p2.saveSources();

  // use the generated code
  std::unique_ptr<CppAD::cg::GenericModel<Scalar>> rnea_generated = dynamicLib->model("rnea");

  CPPAD_TESTVECTOR(Scalar) x((size_t)model.nv);
  Eigen::Map<TangentVectorType>(x.data(), model.nv, 1) = a;

  CPPAD_TESTVECTOR(Scalar) tau = rnea_generated->ForwardZero(x);

  Eigen::Map<TangentVectorType> tau_map(tau.data(), model.nv, 1);
  Data::TangentVectorType tau_ref = pinocchio::rnea(model, data, q, v, a);
  BOOST_CHECK(tau_map.isApprox(tau_ref));

  pinocchio::crba(model, data, q, pinocchio::Convention::WORLD);
  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.transpose().triangularView<Eigen::StrictlyLower>();

  CPPAD_TESTVECTOR(Scalar) dtau_da = rnea_generated->Jacobian(x);
  Eigen::Map<PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(Data::MatrixXs)> M_map(
    dtau_da.data(), model.nv, model.nv);
  BOOST_CHECK(M_map.isApprox(data.M));
}

BOOST_AUTO_TEST_CASE(test_crba_code_generation_pointer)
{
  typedef double Scalar;
  typedef CppAD::cg::CG<Scalar> CGScalar;
  typedef CppAD::AD<CGScalar> ADScalar;
  typedef CppAD::ADFun<CGScalar> ADCGFun;

  typedef Eigen::Matrix<ADScalar, Eigen::Dynamic, 1> ADVector;
  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> RowMatrixXs;

  typedef pinocchio::ModelTpl<Scalar> Model;
  typedef Model::Data Data;

  typedef pinocchio::ModelTpl<ADScalar> ADModel;
  typedef ADModel::Data ADData;

  Model model;
  pinocchio::buildModels::humanoidRandom(model);
  int nq = model.nq;
  int nv = model.nv;
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  Data data(model);

  ADModel ad_model = model.cast<ADScalar>();
  ADData ad_data(ad_model);

  // Sample random configuration
  typedef Model::ConfigVectorType ConfigVectorType;
  typedef Model::TangentVectorType TangentVectorType;
  ConfigVectorType q(nq);
  q = pinocchio::randomConfiguration(model);

  TangentVectorType v(TangentVectorType::Random(nv));
  TangentVectorType a(TangentVectorType::Random(nv));

  typedef ADModel::ConfigVectorType ADConfigVectorType;
  typedef ADModel::TangentVectorType ADTangentVectorType;

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
  ADVector ad_Y = ad_data.tau;
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

  CppAD::cg::ArrayView<Scalar> jac_(jac.data(), (size_t)jac.size());
  for (size_t it = 0; it < 3; ++it)
  {
    std::cout << "test " << it << std::endl;
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

    CppAD::cg::ArrayView<const Scalar> x_(x.data(), (size_t)x.size());
    generatedFun_ptr->Jacobian(x_, jac_);

    pinocchio::crba(model, data, q, pinocchio::Convention::WORLD);
    data.M.triangularView<Eigen::StrictlyLower>() =
      data.M.transpose().triangularView<Eigen::StrictlyLower>();
    BOOST_CHECK(jac.middleCols(nq + nv, nv).isApprox(data.M));
  }
}

BOOST_AUTO_TEST_SUITE_END()
