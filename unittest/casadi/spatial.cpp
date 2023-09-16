//
// Copyright (c) 2019 INRIA
//

#include <pinocchio/autodiff/casadi.hpp>

#include <pinocchio/math/quaternion.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/motion.hpp>
#include <pinocchio/spatial/explog-quaternion.hpp>

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
  pinocchio::casadi::copy(M1.translation(),trans);
  
  const Eigen::DenseIndex col = 0;
  for(Eigen::DenseIndex k = 0; k < 3; ++k)
  {
    BOOST_CHECK(casadi::SX::is_equal(trans(k,col),M1.translation()[k]));
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
  
  casadi::SX cs_rot = casadi::SX::sym("rot", 3,3);
  pinocchio::casadi::copy(cs_rot,ad_rot);
  
  SE3AD::Quaternion ad_quat;
  pinocchio::quaternion::assignQuaternion(ad_quat,ad_rot);
  
  casadi::SX cs_quat(4,1);
  pinocchio::casadi::copy(ad_quat.coeffs(),cs_quat);
  
  casadi::Function eval_quat("eval_quat",
                             casadi::SXVector {cs_rot},
                             casadi::SXVector {cs_quat});
  

  for(int k = 0; k < 1e4; ++k)
  {
    SE3 M(SE3::Random());
    SE3::Quaternion quat_ref(M.rotation());

    casadi::DM vec_rot(3,3);
    pinocchio::casadi::copy(M.rotation(),vec_rot);

    casadi::DM quat_res = eval_quat(casadi::DMVector {vec_rot})[0];
    SE3::Quaternion quat_value;
    
    quat_value.coeffs() = Eigen::Map<Eigen::Vector4d>(static_cast< std::vector<double> >(quat_res).data());
    BOOST_CHECK(pinocchio::quaternion::defineSameRotation(quat_value,quat_ref));
//    if(! quat_value.coeffs().isApprox(quat_ref.coeffs()))
//    {
//      std::cout << "quat_value: " << quat_value.coeffs().transpose() << std::endl;
//      std::cout << "quat_ref: " << quat_ref.coeffs().transpose() << std::endl;
//    }
  }
  
  
}

BOOST_AUTO_TEST_CASE(test_quaternion_normalize)
{
  Eigen::Quaternion<double> quat_double(1, 2, 3, 4);
  quat_double.normalize();

  Eigen::Quaternion<casadi::SX> quat_casadi(1, 2, 3, 4);
  pinocchio::quaternion::normalize(quat_casadi);

  for (int i = 0; i < 4; ++i) {
    BOOST_CHECK_CLOSE(double(quat_casadi.coeffs()[i]), quat_double.coeffs()[i], 1e-6);
  }
}


BOOST_AUTO_TEST_CASE(test_log3_autodiff_degenerative_case)
{
  auto quat_casadi = casadi::SX::sym("quat", 4);  // qx, qy, qz, qw

  // eigen quaternion eats scalar first
  Eigen::Quaternion<casadi::SX> quat(quat_casadi(3), quat_casadi(0),
                                     quat_casadi(1), quat_casadi(2));
  auto so3 = pinocchio::quaternion::log3(quat);
  auto so3_casadi = casadi::SX::sym("so3", 3);
  pinocchio::casadi::copy(so3, so3_casadi);
  auto jac = jacobian(so3_casadi, quat_casadi);

  auto so3_func = casadi::Function("log3_func", {quat_casadi}, {so3_casadi}, {"quat_in"}, {"so3_out"});
  auto jac_func = casadi::Function("jac_func", {quat_casadi}, {jac}, {"quat_in"}, {"jac_out"});

  auto so3_result = so3_func(casadi::DMDict{{"quat_in", {0, 0, 0, 1}}}).at("so3_out");
  auto jac_result = jac_func(casadi::DMDict{{"quat_in", {0, 0, 0, 1}}}).at("jac_out");

  Eigen::Vector<double, 3> so3_expected(0, 0, 0);
  Eigen::Matrix<double, 3, 4> jac_expected;
  jac_expected << 2, 0, 0, 0,
                  0, 2, 0, 0,
                  0, 0, 2, 0;

  for (int i = 0; i < 3; ++i)
  {
      for (int j = 0; j < 4; ++j)
      {
          BOOST_CHECK(double(jac_result(i, j)) == jac_expected(i, j));
      }
      BOOST_CHECK(double(so3_result(i)) == so3_expected[i]);
  }
}

BOOST_AUTO_TEST_SUITE_END()
