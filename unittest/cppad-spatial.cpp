//
// Copyright (c) 2018-2019 CNRS INRIA
//

#include "pinocchio/autodiff/cppad.hpp"

#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/explog.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

template<typename Vector3Like>
Eigen::Matrix<typename Vector3Like::Scalar,3,3,0>
computeV(const Eigen::MatrixBase<Vector3Like> & v3)
{
  typedef typename Vector3Like::Scalar Scalar;
  typedef Eigen::Matrix<Scalar,3,3,0> ReturnType;
  typedef ReturnType Matrix3;
  
  Scalar t2 = v3.squaredNorm();
  const Scalar t = pinocchio::math::sqrt(t2);
  Scalar alpha, beta, zeta;
  
  if (t < 1e-4)
  {
    alpha = Scalar(1) + t2/Scalar(6) - t2*t2/Scalar(120);
    beta = Scalar(1)/Scalar(2) - t2/Scalar(24);
    zeta = Scalar(1)/Scalar(6) - t2/Scalar(120);
  }
  else
  {
    Scalar st,ct; pinocchio::SINCOS(t,&st,&ct);
    alpha = st/t;
    beta = (1-ct)/t2;
    zeta = (1 - alpha)/(t2);
  }
  
  Matrix3 V
  = alpha * Matrix3::Identity()
  + beta * pinocchio::skew(v3)
  + zeta * v3 * v3.transpose();
  
  return V;
}

template<typename Vector3Like>
Eigen::Matrix<typename Vector3Like::Scalar,3,3,0>
computeVinv(const Eigen::MatrixBase<Vector3Like> & v3)
{
  typedef typename Vector3Like::Scalar Scalar;
  typedef Eigen::Matrix<Scalar,3,3,0> ReturnType;
  typedef ReturnType Matrix3;
  
  Scalar t2 = v3.squaredNorm();
  const Scalar t = pinocchio::math::sqrt(t2);
  
  Scalar alpha, beta;
  if (t < 1e-4)
  {
    alpha = Scalar(1) - t2/Scalar(12) - t2*t2/Scalar(720);
    beta = Scalar(1)/Scalar(12) + t2/Scalar(720);
  }
  else
  {
    Scalar st,ct; pinocchio::SINCOS(t,&st,&ct);
    alpha = t*st/(Scalar(2)*(Scalar(1)-ct));
    beta = Scalar(1)/t2 - st/(Scalar(2)*t*(Scalar(1)-ct));
  }
  
  Matrix3 Vinv
  = alpha * Matrix3::Identity()
  - 0.5 * pinocchio::skew(v3)
  + beta * v3 * v3.transpose();
  
  return Vinv;
}

BOOST_AUTO_TEST_CASE(test_log3)
{
  using CppAD::AD;
  using CppAD::NearEqual;

  typedef double Scalar;
  typedef AD<Scalar> ADScalar;

  typedef pinocchio::SE3Tpl<Scalar> SE3;
  typedef pinocchio::MotionTpl<Scalar> Motion;
  typedef pinocchio::SE3Tpl<ADScalar> ADSE3;
  typedef pinocchio::MotionTpl<ADScalar> ADMotion;

  Motion v(Motion::Zero());
  SE3 M(SE3::Random()); M.translation().setZero();

  SE3::Matrix3 rot_next = M.rotation() * pinocchio::exp3(v.angular());

  SE3::Matrix3 Jlog3;
  pinocchio::Jlog3(M.rotation(), Jlog3);

  typedef Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic> Matrix;
  typedef Eigen::Matrix<ADScalar,Eigen::Dynamic,1> ADVector;

  ADMotion ad_v(v.cast<ADScalar>());
  ADSE3 ad_M(M.cast<ADScalar>());
  ADSE3::Matrix3 rot = ad_M.rotation();

  ADVector X(3);

  X = ad_v.angular();

  CppAD::Independent(X);
  ADMotion::Vector3 X_ = X;

  ADSE3::Matrix3 ad_rot_next = rot * pinocchio::exp3(X_);
  ADMotion::Vector3 log_R_next = pinocchio::log3(ad_rot_next);

  ADVector Y(3);
  Y = log_R_next;

  CppAD::ADFun<Scalar> map(X,Y);

  CPPAD_TESTVECTOR(Scalar) x(3);
  Eigen::Map<Motion::Vector3>(x.data()).setZero();

  CPPAD_TESTVECTOR(Scalar) nu_next_vec = map.Forward(0,x);
  Motion::Vector3 nu_next(Eigen::Map<Motion::Vector3>(nu_next_vec.data()));

  SE3::Matrix3 rot_next_from_map = pinocchio::exp3(nu_next);

  CPPAD_TESTVECTOR(Scalar) jac = map.Jacobian(x);

  Matrix jacobian = Eigen::Map<PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(Matrix)>(jac.data(),3,3);

  BOOST_CHECK(rot_next_from_map.isApprox(rot_next));
  BOOST_CHECK(jacobian.isApprox(Jlog3));
}

BOOST_AUTO_TEST_CASE(test_explog_translation)
{
  using CppAD::AD;
  using CppAD::NearEqual;
  
  typedef double Scalar;
  typedef AD<Scalar> ADScalar;
  
  typedef pinocchio::SE3Tpl<Scalar> SE3;
  typedef pinocchio::MotionTpl<Scalar> Motion;
  typedef pinocchio::SE3Tpl<ADScalar> ADSE3;
  typedef pinocchio::MotionTpl<ADScalar> ADMotion;
  
  Motion v(Motion::Zero());
  SE3 M(SE3::Random()); //M.rotation().setIdentity();
  
  {
    Motion::Vector3 v3_test; v3_test.setRandom();
    SE3::Matrix3 V = computeV(v3_test);
    SE3::Matrix3 Vinv = computeVinv(v3_test);
    
    BOOST_CHECK((V*Vinv).isIdentity());
  }
  
  SE3 M_next = M * pinocchio::exp6(v);
//  BOOST_CHECK(M_next.rotation().isIdentity());
  
  typedef Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic> Matrix;
  typedef Eigen::Matrix<ADScalar,Eigen::Dynamic,1> ADVector;
  
  ADMotion ad_v(v.cast<ADScalar>());
  ADSE3 ad_M(M.cast<ADScalar>());
  
  ADVector X(6);
  
  X = ad_v.toVector();
  
  CppAD::Independent(X);
  ADMotion::Vector6 X_ = X;
  
  pinocchio::MotionRef<ADMotion::Vector6> ad_v_ref(X_);
  ADSE3 ad_M_next = ad_M * pinocchio::exp6(ad_v_ref);

  ADVector Y(6);
  Y.head<3>() = ad_M_next.translation();
  Y.tail<3>() = ad_M.translation() + ad_M.rotation() * computeV(ad_v_ref.angular()) * ad_v_ref.linear();
  
  CppAD::ADFun<Scalar> map(X,Y);
  
  CPPAD_TESTVECTOR(Scalar) x((size_t)X.size());
  Eigen::Map<Motion::Vector6>(x.data()).setZero();
  
  CPPAD_TESTVECTOR(Scalar) translation_vec = map.Forward(0,x);
  Motion::Vector3 translation1(Eigen::Map<Motion::Vector3>(translation_vec.data()));
  Motion::Vector3 translation2(Eigen::Map<Motion::Vector3>(translation_vec.data()+3));
  BOOST_CHECK(translation1.isApprox(M_next.translation()));
  BOOST_CHECK(translation2.isApprox(M_next.translation()));
  
  CPPAD_TESTVECTOR(Scalar) jac = map.Jacobian(x);
  
  Matrix jacobian = Eigen::Map<PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(Matrix)>(jac.data(),Y.size(),X.size());
  
  BOOST_CHECK(jacobian.topLeftCorner(3,3).isApprox(M.rotation()));
  
}


BOOST_AUTO_TEST_CASE(test_explog)
{
  using CppAD::AD;
  using CppAD::NearEqual;

  typedef double Scalar;
  typedef AD<Scalar> ADScalar;

  typedef pinocchio::SE3Tpl<Scalar> SE3;
  typedef pinocchio::MotionTpl<Scalar> Motion;
  typedef pinocchio::SE3Tpl<ADScalar> ADSE3;
  typedef pinocchio::MotionTpl<ADScalar> ADMotion;

  Motion v(Motion::Zero());
  SE3 M(SE3::Random()); //M.translation().setZero();

  SE3::Matrix6 Jlog6;
  pinocchio::Jlog6(M, Jlog6);

  typedef Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic> Matrix;
  typedef Eigen::Matrix<ADScalar,Eigen::Dynamic,1> ADVector;

  ADMotion ad_v(v.cast<ADScalar>());
  ADSE3 ad_M(M.cast<ADScalar>());

  ADVector X(6);

  X.segment<3>(Motion::LINEAR) = ad_v.linear();
  X.segment<3>(Motion::ANGULAR) = ad_v.angular();

  CppAD::Independent(X);
  ADMotion::Vector6 X_ = X;
  pinocchio::MotionRef< ADMotion::Vector6> v_X(X_);

  ADSE3 ad_M_next = ad_M * pinocchio::exp6(v_X);
  ADMotion ad_log_M_next = pinocchio::log6(ad_M_next);

  ADVector Y(6);
  Y.segment<3>(Motion::LINEAR) = ad_log_M_next.linear();
  Y.segment<3>(Motion::ANGULAR) = ad_log_M_next.angular();

  CppAD::ADFun<Scalar> map(X,Y);

  CPPAD_TESTVECTOR(Scalar) x(6);
  Eigen::Map<Motion::Vector6>(x.data()).setZero();

  CPPAD_TESTVECTOR(Scalar) nu_next_vec = map.Forward(0,x);
  Motion nu_next(Eigen::Map<Motion::Vector6>(nu_next_vec.data()));

  CPPAD_TESTVECTOR(Scalar) jac = map.Jacobian(x);

  Matrix jacobian = Eigen::Map<PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(Matrix)>(jac.data(),6,6);

  // Check using finite differencies
  Motion dv(Motion::Zero());
  typedef Eigen::Matrix<Scalar,6,6> Matrix6;
  Matrix6 Jlog6_fd(Matrix6::Zero());
  Motion v_plus, v0(log6(M));

  const Scalar eps = 1e-8;
  for(int k = 0; k < 6; ++k)
  {
    dv.toVector()[k] = eps;
    SE3 M_plus = M * exp6(dv);
    v_plus = log6(M_plus);
    Jlog6_fd.col(k) = (v_plus-v0).toVector()/eps;
    dv.toVector()[k] = 0;
  }
  
  SE3::Matrix6 Jlog6_analytic;
  pinocchio::Jlog6(M, Jlog6_analytic);
  
  BOOST_CHECK(Jlog6.isApprox(Jlog6_analytic));
  BOOST_CHECK(Jlog6_fd.isApprox(Jlog6,pinocchio::math::sqrt(eps)));
}

BOOST_AUTO_TEST_SUITE_END()
