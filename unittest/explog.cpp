//
// Copyright (c) 2016,2018 CNRS
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.

#include "utils/macros.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

#include "pinocchio/spatial/explog.hpp"

using namespace se3;

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE(exp)
{
  SE3 M(SE3::Random());
  Motion v(Motion::Random()); v.linear().setZero();
  
  SE3::Matrix3 R = exp3(v.angular());
  BOOST_CHECK(R.isApprox(Eigen::AngleAxis<double>(v.angular().norm(), v.angular().normalized()).matrix()));
  
  SE3::Matrix3 R0 = exp3(SE3::Vector3::Zero());
  BOOST_CHECK(R0.isIdentity());
  
  M = exp6(v);
  
  BOOST_CHECK(R.isApprox(M.rotation()));
  
  R = exp3(SE3::Vector3::Zero());
  BOOST_CHECK(R.isIdentity());
  
  // Quaternion
  Eigen::Quaterniond quat;
  quaternion::exp3(v.angular(),quat);
  BOOST_CHECK(quat.toRotationMatrix().isApprox(M.rotation()));

  quaternion::exp3(SE3::Vector3::Zero(),quat);
  BOOST_CHECK(quat.toRotationMatrix().isIdentity());
  BOOST_CHECK(quat.vec().isZero() && quat.coeffs().tail<1>().isOnes());
  
  // Check QuaternionMap
  Eigen::Vector4d vec4;
  Eigen::QuaternionMapd quat_map(vec4.data());
  quaternion::exp3(v.angular(),quat_map);
  BOOST_CHECK(quat_map.toRotationMatrix().isApprox(M.rotation()));
}

BOOST_AUTO_TEST_CASE(log)
{
  SE3 M(SE3::Identity());
  Motion v(Motion::Random()); v.linear().setZero();
  
  SE3::Vector3 omega = log3(M.rotation());
  BOOST_CHECK(omega.isZero());
  
  M.setRandom();
  M.translation().setZero();
  
  v = log6(M);
  omega = log3(M.rotation());
  BOOST_CHECK(omega.isApprox(v.angular()));
  
  // Quaternion
  Eigen::Quaterniond quat(SE3::Matrix3::Identity());
  omega = quaternion::log3(quat);
  BOOST_CHECK(omega.isZero());

  for(int k = 0; k < 1e3; ++k)
  {
    quat = M.rotation();
    BOOST_CHECK(quat.toRotationMatrix().isApprox(M.rotation()));
    double theta;
    omega = quaternion::log3(quat,theta);
    const double PI_value = PI<double>();
    BOOST_CHECK(omega.norm() <= PI_value);
    double theta_ref;
    SE3::Vector3 omega_ref = log3(quat.toRotationMatrix(),theta_ref);
    
    BOOST_CHECK(omega.isApprox(omega_ref));
  }


  // Check QuaternionMap
  Eigen::Vector4d vec4;
  Eigen::QuaternionMapd quat_map(vec4.data());
  quat_map = SE3::Random().rotation();
  BOOST_CHECK(quaternion::log3(quat_map).isApprox(log3(quat_map.toRotationMatrix())));
}

BOOST_AUTO_TEST_CASE(explog3)
{
  SE3 M(SE3::Random());
  SE3::Matrix3 M_res = exp3(log3(M.rotation()));
  BOOST_CHECK(M_res.isApprox(M.rotation()));
  
  Motion::Vector3 v; v.setRandom();
  Motion::Vector3 v_res = log3(exp3(v));
  BOOST_CHECK(v_res.isApprox(v));
}

BOOST_AUTO_TEST_CASE(explog3_quaternion)
{
  SE3 M(SE3::Random());
  Eigen::Quaterniond quat;
  quat = M.rotation();
  Eigen::Quaterniond quat_res;
  quaternion::exp3(quaternion::log3(quat),quat_res);
  BOOST_CHECK(quat_res.isApprox(quat) || quat_res.coeffs().isApprox(-quat.coeffs()));
  
  Motion::Vector3 v; v.setRandom();
  quaternion::exp3(v,quat);
  BOOST_CHECK(quaternion::log3(quat).isApprox(v));
  
  SE3::Matrix3 R_next = M.rotation() * exp3(v);
  Motion::Vector3 v_est = log3(M.rotation().transpose() * R_next);
  BOOST_CHECK(v_est.isApprox(v));
  
  SE3::Quaternion quat_v;
  quaternion::exp3(v,quat_v);
  SE3::Quaternion quat_next = quat * quat_v;
  v_est = quaternion::log3(quat.conjugate() * quat_next);
  BOOST_CHECK(v_est.isApprox(v));
}

BOOST_AUTO_TEST_CASE(Jlog3_fd)
{
  SE3 M(SE3::Random());
  SE3::Matrix3 R (M.rotation());
  
  SE3::Matrix3 Jfd, Jlog;
  Jlog3 (R, Jlog);
  Jfd.setZero();

  Motion::Vector3 dR; dR.setZero();
  const double eps = 1e-8;
  for (int i = 0; i < 3; ++i)
  {
    dR[i] = eps;
    SE3::Matrix3 R_dR = R * exp3(dR);
    Jfd.col(i) = (log3(R_dR) - log3(R)) / eps;
    dR[i] = 0;
  }
  BOOST_CHECK(Jfd.isApprox(Jlog, std::sqrt(eps)));
}

BOOST_AUTO_TEST_CASE(Jexp3_fd)
{
  SE3 M(SE3::Random());
  SE3::Matrix3 R (M.rotation());

  Motion::Vector3 v = log3(R);

  SE3::Matrix3 Jexp_fd, Jexp;
  Jexp3(v, Jexp);

  Motion::Vector3 dv; dv.setZero();
  const double eps = 1e-8;
  for (int i = 0; i < 3; ++i)
  {
    dv[i] = eps;
    SE3::Matrix3 R_next = exp3(v+dv);
    Jexp_fd.col(i) = log3(R.transpose()*R_next) / eps;
    dv[i] = 0;
  }
  BOOST_CHECK(Jexp_fd.isApprox(Jexp, std::sqrt(eps)));
}

template<typename QuaternionLike, typename Matrix43Like>
void Jexp3QuatLocal(const Eigen::QuaternionBase<QuaternionLike> & quat,
                    const Eigen::MatrixBase<Matrix43Like> & Jexp)
{
  Matrix43Like & Jout = EIGEN_CONST_CAST(Matrix43Like,Jexp);
  
  skew(0.5 * quat.vec(),Jout.template topRows<3>());
  Jout.template topRows<3>().diagonal().array() += 0.5 * quat.w();
  Jout.template bottomRows<1>() = -0.5 * quat.vec().transpose();
}

BOOST_AUTO_TEST_CASE(Jexp3_quat_fd)
{
  typedef double Scalar;
  SE3::Vector3 w; w.setRandom();
  SE3::Quaternion quat; quaternion::exp3(w,quat);
  
  typedef Eigen::Matrix<Scalar,4,3> Matrix43;
  Matrix43 Jexp3, Jexp3_fd;
  quaternion::Jexp3CoeffWise(w,Jexp3);
  SE3::Vector3 dw; dw.setZero();
  const double eps = 1e-8;
  
  for(int i = 0; i < 3; ++i)
  {
    dw[i] = eps;
    SE3::Quaternion quat_plus; quaternion::exp3(w + dw,quat_plus);
    Jexp3_fd.col(i) = (quat_plus.coeffs() - quat.coeffs()) / eps;
    dw[i] = 0;
  }
  BOOST_CHECK(Jexp3.isApprox(Jexp3_fd,sqrt(eps)));
  
  SE3::Matrix3 Jlog;
  se3::Jlog3(quat.toRotationMatrix(),Jlog);
  
  Matrix43 Jexp_quat_local;
  Jexp3QuatLocal(quat,Jexp_quat_local);
  
  
  Matrix43 Jcompositon = Jexp3 * Jlog;
  BOOST_CHECK(Jcompositon.isApprox(Jexp_quat_local));
//  std::cout << "Jcompositon\n" << Jcompositon << std::endl;
//  std::cout << "Jexp_quat_local\n" << Jexp_quat_local << std::endl;
  
  // Arount zero
  w.setZero();
  w.fill(1e-4);
  quaternion::exp3(w,quat);
  quaternion::Jexp3CoeffWise(w,Jexp3);
  for(int i = 0; i < 3; ++i)
  {
    dw[i] = eps;
    SE3::Quaternion quat_plus; quaternion::exp3(w + dw,quat_plus);
    Jexp3_fd.col(i) = (quat_plus.coeffs() - quat.coeffs()) / eps;
    dw[i] = 0;
  }
  BOOST_CHECK(Jexp3.isApprox(Jexp3_fd,sqrt(eps)));

}

BOOST_AUTO_TEST_CASE(Jexp3_quat)
{
  SE3 M(SE3::Random());
  SE3::Quaternion quat(M.rotation());
  
  Motion dv(Motion::Zero());
  const double eps = 1e-8;
  
  typedef Eigen::Matrix<double,7,6> Matrix76;
  Matrix76 Jexp6_fd, Jexp6_quat; Jexp6_quat.setZero();
  typedef Eigen::Matrix<double,4,3> Matrix43;
  Matrix43 Jexp3_quat; Jexp3QuatLocal(quat,Jexp3_quat);
  SE3 M_next;
  
  Jexp6_quat.middleRows<3>(Motion::LINEAR).middleCols<3>(Motion::LINEAR) = M.rotation();
  Jexp6_quat.middleRows<4>(Motion::ANGULAR).middleCols<3>(Motion::ANGULAR) = Jexp3_quat;// * Jlog6_SE3.middleRows<3>(Motion::ANGULAR);
  for(int i = 0; i < 6; ++i)
  {
    dv.toVector()[i] = eps;
    M_next = M * exp6(dv);
    const SE3::Quaternion quat_next(M_next.rotation());
    Jexp6_fd.middleRows<3>(Motion::LINEAR).col(i) = (M_next.translation() - M.translation())/eps;
    Jexp6_fd.middleRows<4>(Motion::ANGULAR).col(i) = (quat_next.coeffs() - quat.coeffs())/eps;
    dv.toVector()[i] = 0.;
  }
  
  BOOST_CHECK(Jexp6_quat.isApprox(Jexp6_fd,sqrt(eps)));
}

BOOST_AUTO_TEST_CASE(Jexplog3)
{
  Motion v(Motion::Random());
  
  Eigen::Matrix3d R (exp3(v.angular())),
    Jexp, Jlog;
  Jexp3 (v.angular(), Jexp);
  Jlog3 (R          , Jlog);
  
  BOOST_CHECK((Jlog * Jexp).isIdentity());

  SE3 M(SE3::Random());
  R = M.rotation();
  v.angular() = log3(R);
  Jlog3 (R          , Jlog);
  Jexp3 (v.angular(), Jexp);
  
  BOOST_CHECK((Jexp * Jlog).isIdentity());
}

BOOST_AUTO_TEST_CASE(Jlog3_quat)
{
  SE3::Vector3 w; w.setRandom();
  SE3::Quaternion quat; quaternion::exp3(w,quat);
  
  SE3::Matrix3 R(quat.toRotationMatrix());
  
  SE3::Matrix3 res, res_ref;
  quaternion::Jlog3(quat,res);
  Jlog3(R,res_ref);
  
  BOOST_CHECK(res.isApprox(res_ref));
}

BOOST_AUTO_TEST_CASE(explog6)
{
  SE3 M(SE3::Random());
  SE3 M_res = exp6(log6(M));
  BOOST_CHECK(M_res.isApprox(M));
  
  Motion v(Motion::Random());
  Motion v_res = log6(exp6(v));
  BOOST_CHECK(v_res.toVector().isApprox(v.toVector()));
}

BOOST_AUTO_TEST_CASE(Jlog6_fd)
{
  SE3 M(SE3::Random());

  SE3::Matrix6 Jfd, Jlog;
  Jlog6 (M, Jlog);
  Jfd.setZero();

  Motion dM; dM.setZero();
  double step = 1e-8;
  for (int i = 0; i < 6; ++i)
  {
    dM.toVector()[i] = step;
    SE3 M_dM = M * exp6(dM);
    Jfd.col(i) = (log6(M_dM).toVector() - log6(M).toVector()) / step;
    dM.toVector()[i] = 0;
  }

  BOOST_CHECK(Jfd.isApprox(Jlog, sqrt(step)));
}

BOOST_AUTO_TEST_CASE(Jexplog6)
{
  Motion v(Motion::Random());
  
  SE3 M (exp6(v));
  SE3::Matrix6 Jexp, Jlog;
  Jexp6 (v, Jexp);
  Jlog6 (M, Jlog);

  BOOST_CHECK((Jlog * Jexp).isIdentity());

  M.setRandom();
  
  v = log6(M);
  Jlog6 (M, Jlog);
  Jexp6 (v, Jexp);

  BOOST_CHECK((Jexp * Jlog).isIdentity());
}

BOOST_AUTO_TEST_CASE (test_basic)
{
  typedef se3::SE3::Vector3 Vector3;
  typedef se3::SE3::Matrix3 Matrix3;
  typedef Eigen::Matrix4d Matrix4;
  typedef se3::Motion::Vector6 Vector6;
  
  const double EPSILON = 1e-12;
  
  // exp3 and log3.
  Vector3 v3(Vector3::Random());
  Matrix3 R(se3::exp3(v3));
  BOOST_CHECK(R.transpose().isApprox(R.inverse(), EPSILON));
  BOOST_CHECK_SMALL(R.determinant() - 1.0, EPSILON);
  Vector3 v3FromLog(se3::log3(R));
  BOOST_CHECK(v3.isApprox(v3FromLog, EPSILON));
  
  // exp6 and log6.
  se3::Motion nu = se3::Motion::Random();
  se3::SE3 m = se3::exp6(nu);
  BOOST_CHECK(m.rotation().transpose().isApprox(m.rotation().inverse(),
                                                EPSILON));
  BOOST_CHECK_SMALL(m.rotation().determinant() - 1.0, EPSILON);
  se3::Motion nuFromLog(se3::log6(m));
  BOOST_CHECK(nu.linear().isApprox(nuFromLog.linear(), EPSILON));
  BOOST_CHECK(nu.angular().isApprox(nuFromLog.angular(), EPSILON));
  
  Vector6 v6(Vector6::Random());
  se3::SE3 m2(se3::exp6(v6));
  BOOST_CHECK(m2.rotation().transpose().isApprox(m2.rotation().inverse(),
                                                 EPSILON));
  BOOST_CHECK_SMALL(m2.rotation().determinant() - 1.0, EPSILON);
  Matrix4 M = m2.toHomogeneousMatrix();
  se3::Motion nu2FromLog(se3::log6(M));
  Vector6 v6FromLog(nu2FromLog.toVector());
  BOOST_CHECK(v6.isApprox(v6FromLog, EPSILON));
}

BOOST_AUTO_TEST_SUITE_END()
