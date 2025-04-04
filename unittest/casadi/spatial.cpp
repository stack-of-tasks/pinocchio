//
// Copyright (c) 2019 INRIA
//

#include <pinocchio/autodiff/casadi.hpp>

#include <pinocchio/math/quaternion.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/motion.hpp>
#include "pinocchio/spatial/explog.hpp"

#include <pinocchio/spatial/explog.hpp>

#include <boost/variant.hpp> // to avoid C99 warnings

#include <iostream>
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_se3)
{
  typedef pinocchio::SE3Tpl<casadi::SX> SE3;
  SE3 M1 = SE3::Identity();
  SE3 M2 = SE3::Random();

  SE3 M3 = M2 * M1;
  SE3 M1inv = M1.inverse();

  ::casadi::SX trans;
  pinocchio::casadi::copy(M1.translation(), trans);

  const Eigen::DenseIndex col = 0;
  for (Eigen::DenseIndex k = 0; k < 3; ++k)
  {
    BOOST_CHECK(casadi::SX::is_equal(trans(k, col), M1.translation()[k]));
  }
}

BOOST_AUTO_TEST_CASE(test_motion)
{
  typedef pinocchio::MotionTpl<casadi::SX> Motion;
  Motion v1 = Motion::Zero();
  Motion v2 = Motion::Random();

  Motion v3 = v1 + v2;
}

BOOST_AUTO_TEST_CASE(test_quaternion)
{
  typedef pinocchio::SE3Tpl<casadi::SX> SE3AD;
  typedef pinocchio::SE3 SE3;

  SE3AD ad_M;
  SE3AD::Matrix3 & ad_rot = ad_M.rotation();

  casadi::SX cs_rot = casadi::SX::sym("rot", 3, 3);
  pinocchio::casadi::copy(cs_rot, ad_rot);

  SE3AD::Quaternion ad_quat;
  pinocchio::quaternion::assignQuaternion(ad_quat, ad_rot);

  casadi::SX cs_quat(4, 1);
  pinocchio::casadi::copy(ad_quat.coeffs(), cs_quat);

  casadi::Function eval_quat("eval_quat", casadi::SXVector{cs_rot}, casadi::SXVector{cs_quat});

  for (int k = 0; k < 1e4; ++k)
  {
    SE3 M(SE3::Random());
    SE3::Quaternion quat_ref(M.rotation());

    casadi::DM vec_rot(3, 3);
    pinocchio::casadi::copy(M.rotation(), vec_rot);

    casadi::DM quat_res = eval_quat(casadi::DMVector{vec_rot})[0];
    SE3::Quaternion quat_value;

    quat_value.coeffs() =
      Eigen::Map<Eigen::Vector4d>(static_cast<std::vector<double>>(quat_res).data());
    BOOST_CHECK(pinocchio::quaternion::defineSameRotation(quat_value, quat_ref));
    //    if(! quat_value.coeffs().isApprox(quat_ref.coeffs()))
    //    {
    //      std::cout << "quat_value: " << quat_value.coeffs().transpose() << std::endl;
    //      std::cout << "quat_ref: " << quat_ref.coeffs().transpose() << std::endl;
    //    }
  }
}

BOOST_AUTO_TEST_CASE(test_log3_firstorder_derivatives)
{
  typedef double Scalar;
  typedef casadi::SX ADScalar;

  typedef pinocchio::SE3 SE3;
  typedef SE3::Vector3 Vector3;
  typedef SE3::Matrix3 Matrix3;

  typedef pinocchio::SE3Tpl<ADScalar> SE3AD;
  typedef SE3AD::Vector3 Vector3AD;

  SE3::Matrix3 RTarget;
  pinocchio::toRotationMatrix(Vector3(SE3::Vector3::UnitX()), pinocchio::PI<Scalar>() / 4, RTarget);

  SE3::Quaternion quat0(0.707107, 0.707107, 0, 0);
  Matrix3 R0 = quat0.toRotationMatrix();
  Vector3 nu0 = pinocchio::log3(R0);

  casadi::SX cs_nu = casadi::SX::sym("nu", 3);

  Vector3AD nu_ad = Eigen::Map<Vector3AD>(static_cast<std::vector<ADScalar>>(cs_nu).data());
  auto log3_casadi_exp =
    pinocchio::log3(pinocchio::exp3(nu_ad).transpose() * RTarget.cast<ADScalar>());

  casadi::SX cs_res = casadi::SX::sym("res", 3);
  pinocchio::casadi::copy(log3_casadi_exp, cs_res);

  casadi::Function log3_casadi("log3_casadi", casadi::SXVector{cs_nu}, casadi::SXVector{cs_res});

  std::vector<double> log3_casadi_input(3);
  Eigen::Map<Vector3>(log3_casadi_input.data()) = nu0;
  casadi::DM log3_casadi_res = log3_casadi(casadi::DMVector{log3_casadi_input})[0];
  Vector3 res0 = Eigen::Map<Vector3>(static_cast<std::vector<double>>(log3_casadi_res).data());

  Vector3 res0_ref = pinocchio::log3(pinocchio::exp3(nu0).transpose() * RTarget);

  // Check first that the value matches
  BOOST_CHECK(res0 == res0);
  BOOST_CHECK(res0_ref == res0_ref);
  BOOST_CHECK(res0.isApprox(res0_ref));

  ADScalar log3_casadi_jacobian_exp = jacobian(cs_res, cs_nu);
  casadi::Function log3_casadi_jacobian(
    "log3_casadi_jacobian", casadi::SXVector{cs_nu}, casadi::SXVector{log3_casadi_jacobian_exp});

  casadi::DM log3_casadi_jacobian_res =
    log3_casadi_jacobian(casadi::DMVector{log3_casadi_input})[0];
  Matrix3 jac0 =
    Eigen::Map<Matrix3>(static_cast<std::vector<double>>(log3_casadi_jacobian_res).data());

  BOOST_CHECK(jac0 == jac0);

  ADScalar log3_casadi_gradient_exp = gradient(log3_casadi_exp[0], cs_nu);

  casadi::Function log3_casadi_gradient(
    "log3_casadi_jacobian", casadi::SXVector{cs_nu}, casadi::SXVector{log3_casadi_gradient_exp});

  casadi::DM log3_casadi_gradient_res =
    log3_casadi_gradient(casadi::DMVector{log3_casadi_input})[0];
  Vector3 grad0 =
    Eigen::Map<Vector3>(static_cast<std::vector<double>>(log3_casadi_gradient_res).data());

  BOOST_CHECK(grad0 == grad0);
}

BOOST_AUTO_TEST_SUITE_END()
