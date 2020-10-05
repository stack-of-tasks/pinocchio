//
// Copyright (c) 2019 INRIA
//

#include <pinocchio/autodiff/casadi.hpp>

#include <pinocchio/math/quaternion.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/motion.hpp>

#include <boost/variant.hpp> // to avoid C99 warnings

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

BOOST_AUTO_TEST_SUITE_END()
