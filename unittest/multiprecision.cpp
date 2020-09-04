//
// Copyright (c) 2020 INRIA
//

#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/parsers/sample-models.hpp"

#include "pinocchio/math/multiprecision.hpp"

#include <boost/multiprecision/cpp_dec_float.hpp>
#include <boost/math/special_functions/gamma.hpp>

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_basic)
{
  using namespace boost::multiprecision;

  // Operations at fixed precision and full numeric_limits support:
  cpp_dec_float_100 b = 2;
  std::cout << std::numeric_limits<cpp_dec_float_100>::digits << std::endl;
  std::cout << std::numeric_limits<cpp_dec_float_100>::digits10 << std::endl;
  // We can use any C++ std lib function, lets print all the digits as well:
  std::cout << std::setprecision(std::numeric_limits<cpp_dec_float_100>::max_digits10)
  << log(b) << std::endl; // print log(2)
                          // We can also use any function from Boost.Math:
  std::cout << boost::math::tgamma(b) << std::endl;
  // These even work when the argument is an expression template:
  std::cout << boost::math::tgamma(b * b) << std::endl;
  // And since we have an extended exponent range we can generate some really large
  // numbers here (4.0238726007709377354370243e+2564):
  std::cout << boost::math::tgamma(cpp_dec_float_100(1000)) << std::endl;
}

BOOST_AUTO_TEST_CASE(test_cast)
{
  typedef boost::multiprecision::cpp_dec_float_100 float_100;
  
  // Test Scalar cast
  double initial_value = boost::math::constants::pi<double>();
  float_100 value_100(initial_value);
  double value_cast = value_100.convert_to<double>();
  BOOST_CHECK(initial_value == value_cast);
  
  typedef Eigen::Matrix<float_100,Eigen::Dynamic,1> VectorFloat100;
  static const Eigen::DenseIndex dim = 100;
  Eigen::VectorXd initial_vec = Eigen::VectorXd::Random(dim);
  VectorFloat100 vec_float_100 = initial_vec.cast<float_100>();
  Eigen::VectorXd vec = vec_float_100.cast<double>();
  
  BOOST_CHECK(vec == initial_vec);
}

#define BOOST_CHECK_IS_APPROX(double_field,multires_field,Scalar) \
  BOOST_CHECK(double_field.isApprox(multires_field.cast<Scalar>()))

BOOST_AUTO_TEST_CASE(test_mutliprecision)
{
  using namespace pinocchio;

  Model model;
  pinocchio::buildModels::humanoidRandom(model);
  Data data(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill( 1.);
  
  typedef boost::multiprecision::cpp_dec_float_100 float_100;
  typedef ModelTpl<float_100> ModelMulti;
  typedef DataTpl<float_100> DataMulti;
  
  ModelMulti model_multi = model.cast<float_100>();
  DataMulti data_multi(model_multi);
  
  ModelMulti::ConfigVectorType q_multi = randomConfiguration(model_multi);
  ModelMulti::TangentVectorType v_multi = ModelMulti::TangentVectorType::Random(model_multi.nv);
  ModelMulti::TangentVectorType a_multi = ModelMulti::TangentVectorType::Random(model_multi.nv);
  ModelMulti::TangentVectorType tau_multi = ModelMulti::TangentVectorType::Random(model_multi.nv);
    
  //  Model::ConfigVectorType q = randomConfiguration(model);
  //  Model::TangentVectorType v = Model::TangentVectorType::Random(model.nv);
  //  Model::TangentVectorType a = Model::TangentVectorType::Random(model.nv);
  //  Model::TangentVectorType tau = Model::TangentVectorType::Random(model.nv);
  
  Model::ConfigVectorType q = q_multi.cast<double>();
  Model::TangentVectorType v = v_multi.cast<double>();
  Model::TangentVectorType a = a_multi.cast<double>();
  Model::TangentVectorType tau = tau_multi.cast<double>();
  
  forwardKinematics(model_multi,data_multi,q_multi,v_multi,a_multi);
  forwardKinematics(model,data,q,v,a);
  
  for(JointIndex joint_id = 1; joint_id < (JointIndex)model.njoints; ++joint_id)
  {
    BOOST_CHECK_IS_APPROX(data.oMi[joint_id],data_multi.oMi[joint_id],double);
    BOOST_CHECK_IS_APPROX(data.v[joint_id],data_multi.v[joint_id],double);
    BOOST_CHECK_IS_APPROX(data.a[joint_id],data_multi.a[joint_id],double);
  }
  
  // Jacobians
  computeJointJacobians(model_multi,data_multi,q_multi);
  computeJointJacobians(model,data,q);
  
  BOOST_CHECK_IS_APPROX(data.J,data_multi.J,double);
  
  // Inverse Dynamics
  rnea(model_multi,data_multi,q_multi,v_multi,a_multi);
  rnea(model,data,q,v,a);
  
  BOOST_CHECK_IS_APPROX(data.tau,data_multi.tau,double);
  
  // Forward Dynamics
  aba(model_multi,data_multi,q_multi,v_multi,tau_multi);
  aba(model,data,q,v,tau);
  
  BOOST_CHECK_IS_APPROX(data.ddq,data_multi.ddq,double);
  
  // Mass matrix
  crba(model_multi,data_multi,q_multi);
  data_multi.M.triangularView<Eigen::StrictlyLower>()
    = data_multi.M.transpose().triangularView<Eigen::StrictlyLower>();

  crba(model,data,q);
  data.M.triangularView<Eigen::StrictlyLower>()
    = data.M.transpose().triangularView<Eigen::StrictlyLower>();

  BOOST_CHECK_IS_APPROX(data.M,data_multi.M,double);
}

BOOST_AUTO_TEST_SUITE_END()
