//
// Copyright (c) 2018-2022 CNRS INRIA
//

#include "pinocchio/autodiff/cppad.hpp"

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

#ifdef _WIN32
  #define DLL_EXT ".dll"
#else
  #define DLL_EXT ".so"
#endif

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_mass_matrix)
{
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

  typedef Eigen::Matrix<ADScalar, Eigen::Dynamic, 1> VectorXAD;
  pinocchio::crba(model, data, q, pinocchio::Convention::WORLD);
  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.transpose().triangularView<Eigen::StrictlyLower>();

  Data::TangentVectorType tau = pinocchio::rnea(model, data, q, v, a);

  {
    CppAD::Independent(ad_a);
    pinocchio::rnea(ad_model, ad_data, ad_q, ad_v, ad_a);

    VectorXAD Y(model.nv);
    Eigen::Map<ADData::TangentVectorType>(Y.data(), model.nv, 1) = ad_data.tau;

    CppAD::ADFun<Scalar> ad_fun(ad_a, Y);

    CPPAD_TESTVECTOR(Scalar) x((size_t)model.nv);
    Eigen::Map<Data::TangentVectorType>(x.data(), model.nv, 1) = a;

    CPPAD_TESTVECTOR(Scalar) tau = ad_fun.Forward(0, x);
    BOOST_CHECK(Eigen::Map<Data::TangentVectorType>(tau.data(), model.nv, 1).isApprox(data.tau));

    CPPAD_TESTVECTOR(Scalar) dtau_da = ad_fun.Jacobian(x);
    Data::MatrixXs M = Eigen::Map<PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(Data::MatrixXs)>(
      dtau_da.data(), model.nv, model.nv);
    BOOST_CHECK(M.isApprox(data.M));
  }

  ADTangentVectorType ad_tau = tau.cast<ADScalar>();

  pinocchio::computeMinverse(model, data, q);
  data.Minv.triangularView<Eigen::StrictlyLower>() =
    data.Minv.transpose().triangularView<Eigen::StrictlyLower>();

  pinocchio::aba(model, data, q, v, tau, pinocchio::Convention::WORLD);
  {
    CppAD::Independent(ad_tau);
    pinocchio::aba(ad_model, ad_data, ad_q, ad_v, ad_tau, pinocchio::Convention::WORLD);

    VectorXAD Y(model.nv);
    Eigen::Map<ADData::TangentVectorType>(Y.data(), model.nv, 1) = ad_data.ddq;

    CppAD::ADFun<Scalar> ad_fun(ad_tau, Y);

    CPPAD_TESTVECTOR(Scalar) x((size_t)model.nv);
    Eigen::Map<Data::TangentVectorType>(x.data(), model.nv, 1) = tau;

    CPPAD_TESTVECTOR(Scalar) ddq = ad_fun.Forward(0, x);
    BOOST_CHECK(Eigen::Map<Data::TangentVectorType>(ddq.data(), model.nv, 1).isApprox(a));

    CPPAD_TESTVECTOR(Scalar) dddq_da = ad_fun.Jacobian(x);
    Data::MatrixXs Minv = Eigen::Map<PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(Data::MatrixXs)>(
      dddq_da.data(), model.nv, model.nv);
    BOOST_CHECK(Minv.isApprox(data.Minv));
  }
}

BOOST_AUTO_TEST_CASE(test_kinematics_jacobian)
{
  using CppAD::AD;
  using CppAD::NearEqual;

  typedef double Scalar;
  typedef AD<Scalar> ADScalar;

  typedef pinocchio::ModelTpl<Scalar> Model;
  typedef Model::Data Data;
  typedef Model::Motion Motion;

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

  // Test if the jacobian of a precise link is given by dv_link/dv
  const std::string joint_name = "rarm5_joint";
  Model::JointIndex joint_id = model.getJointId(joint_name);
  pinocchio::computeJointJacobiansTimeVariation(model, data, q, v);
  pinocchio::forwardKinematics(model, data, q, v, a);

  Data::Matrix6x J_local(6, model.nv), J_global(6, model.nv);
  J_local.setZero();
  J_global.setZero();
  Data::Matrix6x dJ_local(6, model.nv), dJ_global(6, model.nv);
  dJ_local.setZero();
  dJ_global.setZero();
  pinocchio::getJointJacobian(model, data, joint_id, pinocchio::LOCAL, J_local);
  pinocchio::getJointJacobian(model, data, joint_id, pinocchio::WORLD, J_global);

  pinocchio::getJointJacobianTimeVariation(model, data, joint_id, pinocchio::LOCAL, dJ_local);
  pinocchio::getJointJacobianTimeVariation(model, data, joint_id, pinocchio::WORLD, dJ_global);

  const ADData::Motion & v_local = ad_data.v[joint_id];
  const ADData::Motion & a_local = ad_data.a[joint_id];

  typedef Eigen::Matrix<ADScalar, Eigen::Dynamic, 1> VectorXAD;

  {
    CppAD::Independent(ad_v);
    pinocchio::forwardKinematics(ad_model, ad_data, ad_q, ad_v, ad_a);

    const ADData::Motion v_global = ad_data.oMi[joint_id].act(v_local);

    VectorXAD Y(6 * 3);
    Eigen::DenseIndex current_id = 0;
    for (Eigen::DenseIndex k = 0; k < 3; ++k)
    {
      Y[current_id + k + Motion::LINEAR] = v_local.linear()[k];
      Y[current_id + k + Motion::ANGULAR] = v_local.angular()[k];
    }
    current_id += 6;

    for (Eigen::DenseIndex k = 0; k < 3; ++k)
    {
      Y[current_id + k + Motion::LINEAR] = v_global.linear()[k];
      Y[current_id + k + Motion::ANGULAR] = v_global.angular()[k];
    }
    current_id += 6;

    for (Eigen::DenseIndex k = 0; k < 3; ++k)
    {
      Y[current_id + k + Motion::LINEAR] = a_local.linear()[k];
      Y[current_id + k + Motion::ANGULAR] = a_local.angular()[k];
    }
    current_id += 6;

    CppAD::ADFun<Scalar> vjoint(ad_v, Y);

    CPPAD_TESTVECTOR(Scalar) x((size_t)model.nv);
    for (Eigen::DenseIndex k = 0; k < model.nv; ++k)
    {
      x[(size_t)k] = v[k];
    }

    CPPAD_TESTVECTOR(Scalar) y = vjoint.Forward(0, x);
    Scalar * y_ptr = y.data();
    BOOST_CHECK(data.v[joint_id].isApprox(Motion(Eigen::Map<Motion::Vector6>(y_ptr))));
    y_ptr += 6;
    BOOST_CHECK(data.oMi[joint_id]
                  .act(data.v[joint_id])
                  .isApprox(Motion(Eigen::Map<Motion::Vector6>(y_ptr))));
    y_ptr += 6;
    BOOST_CHECK(data.a[joint_id].isApprox(Motion(Eigen::Map<Motion::Vector6>(y_ptr))));
    y_ptr += 6;

    CPPAD_TESTVECTOR(Scalar) dY_dv = vjoint.Jacobian(x);

    Scalar * dY_dv_ptr = dY_dv.data();
    Data::Matrix6x ad_J_local =
      Eigen::Map<PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(Data::Matrix6x)>(dY_dv_ptr, 6, model.nv);
    dY_dv_ptr += ad_J_local.size();
    Data::Matrix6x ad_J_global =
      Eigen::Map<PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(Data::Matrix6x)>(dY_dv_ptr, 6, model.nv);
    dY_dv_ptr += ad_J_global.size();

    BOOST_CHECK(ad_J_local.isApprox(J_local));
    BOOST_CHECK(ad_J_global.isApprox(J_global));
  }

  {
    CppAD::Independent(ad_a);
    pinocchio::forwardKinematics(ad_model, ad_data, ad_q, ad_v, ad_a);

    VectorXAD Y(6 * 2);
    Eigen::DenseIndex current_id = 0;
    for (Eigen::DenseIndex k = 0; k < 3; ++k)
    {
      Y[current_id + k + Motion::LINEAR] = v_local.linear()[k];
      Y[current_id + k + Motion::ANGULAR] = v_local.angular()[k];
    }
    current_id += 6;

    for (Eigen::DenseIndex k = 0; k < 3; ++k)
    {
      Y[current_id + k + Motion::LINEAR] = a_local.linear()[k];
      Y[current_id + k + Motion::ANGULAR] = a_local.angular()[k];
    }
    current_id += 6;

    CppAD::ADFun<Scalar> ajoint(ad_a, Y);

    CPPAD_TESTVECTOR(Scalar) x((size_t)model.nv);
    for (Eigen::DenseIndex k = 0; k < model.nv; ++k)
    {
      x[(size_t)k] = a[k];
    }

    CPPAD_TESTVECTOR(Scalar) y = ajoint.Forward(0, x);
    Scalar * y_ptr = y.data() + 6;
    BOOST_CHECK(data.a[joint_id].isApprox(Motion(Eigen::Map<Motion::Vector6>(y_ptr))));
    y_ptr += 6;

    CPPAD_TESTVECTOR(Scalar) dY_da = ajoint.Jacobian(x);

    Scalar * dY_da_ptr = dY_da.data();
    Data::Matrix6x ad_dv_da =
      Eigen::Map<PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(Data::Matrix6x)>(dY_da_ptr, 6, model.nv);
    dY_da_ptr += ad_dv_da.size();
    Data::Matrix6x ad_J_local =
      Eigen::Map<PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(Data::Matrix6x)>(dY_da_ptr, 6, model.nv);
    dY_da_ptr += ad_J_local.size();

    BOOST_CHECK(ad_dv_da.isZero());
    BOOST_CHECK(ad_J_local.isApprox(J_local));
  }
}

BOOST_AUTO_TEST_CASE(test_JSIM_jit)
{
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

  typedef Eigen::Matrix<ADScalar, Eigen::Dynamic, 1> VectorXAD;
  typedef Eigen::Matrix<ADScalar, Eigen::Dynamic, Eigen::Dynamic> MatrixXAD;
  pinocchio::crba(model, data, q, pinocchio::Convention::WORLD);
  data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.transpose().triangularView<Eigen::StrictlyLower>();

  pinocchio::rnea(model, data, q, v, a);
  {
    CppAD::Independent(ad_a);
    pinocchio::rnea(ad_model, ad_data, ad_q, ad_v, ad_a);

    VectorXAD Y(model.nv);
    Eigen::Map<ADData::TangentVectorType>(Y.data(), model.nv, 1) = ad_data.tau;

    CppAD::ADFun<Scalar> f(ad_a, Y);
    CppAD::ADFun<ADScalar, Scalar> af = f.base2ad();

    CppAD::Independent(ad_a);
    MatrixXAD dtau_da = af.Jacobian(ad_a);
    VectorXAD dtau_da_vector(model.nv * model.nv);
    dtau_da_vector = Eigen::Map<VectorXAD>(dtau_da.data(), dtau_da.cols() * dtau_da.rows());
    CppAD::ADFun<double> ad_fun(ad_a, dtau_da_vector);

    ad_fun.function_name_set("ad_fun");

    // create csrc_file
    std::string c_type = "double";
    std::string csrc_file = "jit_JSIM.c";
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

    CPPAD_TESTVECTOR(Scalar) x((size_t)model.nv);
    Eigen::Map<Data::TangentVectorType>(x.data(), model.nv, 1) = a;

    size_t compare_change = 0, nx = (size_t)model.nv,
           ndtau_da_ = (size_t)model.nv * (size_t)model.nv;
    std::vector<double> dtau_da_jit(ndtau_da_);
    ad_fun_ptr(nx, x.data(), ndtau_da_, dtau_da_jit.data(), &compare_change);

    Data::MatrixXs M = Eigen::Map<PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(Data::MatrixXs)>(
      dtau_da_jit.data(), model.nv, model.nv);
    BOOST_CHECK(M.isApprox(data.M));
  }
}

BOOST_AUTO_TEST_SUITE_END()
