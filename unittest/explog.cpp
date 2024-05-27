//
// Copyright (c) 2016-2021 CNRS INRIA
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#include "pinocchio/spatial/explog.hpp"

#include <iostream>
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

#include "utils/macros.hpp"

using namespace pinocchio;

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(exp)
{
  SE3 M(SE3::Random());
  Motion v(Motion::Random());
  v.linear().setZero();

  SE3::Matrix3 R = exp3(v.angular());
  BOOST_CHECK(
    R.isApprox(Eigen::AngleAxis<double>(v.angular().norm(), v.angular().normalized()).matrix()));

  SE3::Matrix3 R0 = exp3(SE3::Vector3::Zero());
  BOOST_CHECK(R0.isIdentity());

// check the NAN case
#ifdef NDEBUG
  Motion::Vector3 vec3_nan(Motion::Vector3::Constant(NAN));
  SE3::Matrix3 R_nan = exp3(vec3_nan);
  BOOST_CHECK(R_nan != R_nan);
#endif

  M = exp6(v);

  BOOST_CHECK(R.isApprox(M.rotation()));

  Eigen::Matrix<double, 7, 1> q0;
  q0 << 0., 0., 0., 0., 0., 0., 1.;
  Eigen::Matrix<double, 7, 1> q = quaternion::exp6(v);
  Eigen::Quaterniond quat0(q.tail<4>());
  BOOST_CHECK(R.isApprox(quat0.matrix()));

// check the NAN case
#ifdef NDEBUG
  Motion::Vector6 vec6_nan(Motion::Vector6::Constant(NAN));
  SE3 M_nan = exp6(vec6_nan);
  BOOST_CHECK(M_nan != M_nan);
#endif

  R = exp3(SE3::Vector3::Zero());
  BOOST_CHECK(R.isIdentity());

  // Quaternion
  Eigen::Quaterniond quat;
  quaternion::exp3(v.angular(), quat);
  BOOST_CHECK(quat.toRotationMatrix().isApprox(M.rotation()));

  quaternion::exp3(SE3::Vector3::Zero(), quat);
  BOOST_CHECK(quat.toRotationMatrix().isIdentity());
  BOOST_CHECK(quat.vec().isZero() && quat.coeffs().tail<1>().isOnes());

  // Check QuaternionMap
  Eigen::Vector4d vec4;
  Eigen::QuaternionMapd quat_map(vec4.data());
  quaternion::exp3(v.angular(), quat_map);
  BOOST_CHECK(quat_map.toRotationMatrix().isApprox(M.rotation()));
}

BOOST_AUTO_TEST_CASE(renorm_rotation)
{
  SE3 M0, M1;
  SE3::Matrix3 R0, R1;
  SE3::Matrix3 R_normed;
  SE3::Matrix3 Id(SE3::Matrix3::Identity());
  SE3::Vector3 vals;
  double tr0, tr;
  const size_t num_tries = 20;

  for (size_t i = 0; i < num_tries; i++)
  {
    M0.setRandom();
    M1 = M0.actInv(M0);
    R1 = M1.rotation();
    R_normed = pinocchio::renormalize_rotation_matrix(R1);
    BOOST_CHECK((R_normed.transpose() * R_normed).isApprox(Id));
    tr0 = R1.trace();

    tr = R_normed.trace();
    vals = 2. * R_normed.diagonal().array() - tr + 1.;
  }
}

BOOST_AUTO_TEST_CASE(log)
{
  SE3 M(SE3::Identity());
  Motion v(Motion::Random());
  v.linear().setZero();

  SE3::Vector3 omega = log3(M.rotation());
  BOOST_CHECK(omega.isZero());

  // check the NAN case
#ifdef NDEBUG
  SE3::Matrix3 mat3_nan(SE3::Matrix3::Constant(NAN));
  SE3::Vector3 omega_nan = log3(mat3_nan);
  BOOST_CHECK(omega_nan != omega_nan);
#endif

  M.setRandom();
  M.translation().setZero();

  v = log6(M);
  omega = log3(M.rotation());
  BOOST_CHECK(omega.isApprox(v.angular()));

// check the NAN case
#ifdef NDEBUG
  SE3::Matrix4 mat4_nan(SE3::Matrix4::Constant(NAN));
  Motion::Vector6 v_nan = log6(mat4_nan);
  BOOST_CHECK(v_nan != v_nan);
#endif

  // Quaternion
  Eigen::Quaterniond quat(SE3::Matrix3::Identity());
  omega = quaternion::log3(quat);
  BOOST_CHECK(omega.isZero());

  for (int k = 0; k < 1e3; ++k)
  {
    SE3::Vector3 w;
    w.setRandom();
    quaternion::exp3(w, quat);
    SE3::Matrix3 rot = exp3(w);

    BOOST_CHECK(quat.toRotationMatrix().isApprox(rot));
    double theta;
    omega = quaternion::log3(quat, theta);
    const double PI_value = PI<double>();
    BOOST_CHECK(omega.norm() <= PI_value);
    double theta_ref;
    SE3::Vector3 omega_ref = log3(quat.toRotationMatrix(), theta_ref);

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

  Motion::Vector3 v;
  v.setRandom();
  Motion::Vector3 v_res = log3(exp3(v));
  BOOST_CHECK(v_res.isApprox(v));
}

BOOST_AUTO_TEST_CASE(explog3_quaternion)
{
  SE3 M(SE3::Random());
  Eigen::Quaterniond quat;
  quat = M.rotation();
  Eigen::Quaterniond quat_res;
  quaternion::exp3(quaternion::log3(quat), quat_res);
  BOOST_CHECK(quat_res.isApprox(quat) || quat_res.coeffs().isApprox(-quat.coeffs()));

  Motion::Vector3 v;
  v.setRandom();
  quaternion::exp3(v, quat);
  BOOST_CHECK(quaternion::log3(quat).isApprox(v));

  SE3::Matrix3 R_next = M.rotation() * exp3(v);
  Motion::Vector3 v_est = log3(M.rotation().transpose() * R_next);
  BOOST_CHECK(v_est.isApprox(v));

  SE3::Quaternion quat_v;
  quaternion::exp3(v, quat_v);
  SE3::Quaternion quat_next = quat * quat_v;
  v_est = quaternion::log3(quat.conjugate() * quat_next);
  BOOST_CHECK(v_est.isApprox(v));
}

BOOST_AUTO_TEST_CASE(Jlog3_fd)
{
  SE3 M(SE3::Random());
  SE3::Matrix3 R(M.rotation());

  PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
  PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_MAYBE_UNINITIALIZED
  SE3::Matrix3 Jfd, Jlog;
  Jlog3(R, Jlog);
  PINOCCHIO_COMPILER_DIAGNOSTIC_POP
  Jfd.setZero();

  Motion::Vector3 dR;
  dR.setZero();
  const double eps = 1e-8;
  for (int i = 0; i < 3; ++i)
  {
    dR[i] = eps;
    SE3::Matrix3 R_dR_plus = R * exp3(dR);
    SE3::Matrix3 R_dR_minus = R * exp3(-dR);
    Jfd.col(i) = (log3(R_dR_plus) - log3(R_dR_minus)) / (2 * eps);
    dR[i] = 0;
  }
  BOOST_CHECK(Jfd.isApprox(Jlog, std::sqrt(eps)));
}

BOOST_AUTO_TEST_CASE(Jexp3_fd)
{
  SE3 M(SE3::Random());
  SE3::Matrix3 R(M.rotation());

  Motion::Vector3 v = log3(R);

  PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
  PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_MAYBE_UNINITIALIZED
  SE3::Matrix3 Jexp_fd, Jexp;

  Jexp3(Motion::Vector3::Zero(), Jexp);
  BOOST_CHECK(Jexp.isIdentity());

  Jexp3(v, Jexp);
  PINOCCHIO_COMPILER_DIAGNOSTIC_POP

  Motion::Vector3 dv;
  dv.setZero();
  const double eps = 1e-3;
  for (int i = 0; i < 3; ++i)
  {
    dv[i] = eps;
    SE3::Matrix3 R_next = exp3(v + dv);
    SE3::Matrix3 R_prev = exp3(v - dv);
    SE3::Matrix3 Rpn = R_prev.transpose() * R_next;
    Jexp_fd.col(i) = log3(Rpn) / (2. * eps);
    dv[i] = 0;
  }
  BOOST_CHECK(Jexp_fd.isApprox(Jexp, std::sqrt(eps)));
}

template<typename QuaternionLike, typename Matrix43Like>
void Jexp3QuatLocal(
  const Eigen::QuaternionBase<QuaternionLike> & quat, const Eigen::MatrixBase<Matrix43Like> & Jexp)
{
  Matrix43Like & Jout = PINOCCHIO_EIGEN_CONST_CAST(Matrix43Like, Jexp);

  skew(0.5 * quat.vec(), Jout.template topRows<3>());
  Jout.template topRows<3>().diagonal().array() += 0.5 * quat.w();
  Jout.template bottomRows<1>() = -0.5 * quat.vec().transpose();
}

BOOST_AUTO_TEST_CASE(Jexp3_quat_fd)
{
  typedef double Scalar;
  SE3::Vector3 w;
  w.setRandom();
  SE3::Quaternion quat;
  quaternion::exp3(w, quat);

  typedef Eigen::Matrix<Scalar, 4, 3> Matrix43;
  PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
  PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_MAYBE_UNINITIALIZED
  Matrix43 Jexp3, Jexp3_fd;
  quaternion::Jexp3CoeffWise(w, Jexp3);
  PINOCCHIO_COMPILER_DIAGNOSTIC_POP
  SE3::Vector3 dw;
  dw.setZero();
  const double eps = 1e-8;

  for (int i = 0; i < 3; ++i)
  {
    dw[i] = eps;
    SE3::Quaternion quat_plus;
    quaternion::exp3(w + dw, quat_plus);
    Jexp3_fd.col(i) = (quat_plus.coeffs() - quat.coeffs()) / eps;
    dw[i] = 0;
  }
  BOOST_CHECK(Jexp3.isApprox(Jexp3_fd, sqrt(eps)));

  PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
  PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_MAYBE_UNINITIALIZED
  SE3::Matrix3 Jlog;
  pinocchio::Jlog3(quat.toRotationMatrix(), Jlog);

  Matrix43 Jexp_quat_local;
  Jexp3QuatLocal(quat, Jexp_quat_local);
  PINOCCHIO_COMPILER_DIAGNOSTIC_POP

  Matrix43 Jcompositon = Jexp3 * Jlog;
  BOOST_CHECK(Jcompositon.isApprox(Jexp_quat_local));
  //  std::cout << "Jcompositon\n" << Jcompositon << std::endl;
  //  std::cout << "Jexp_quat_local\n" << Jexp_quat_local << std::endl;

  // Arount zero
  w.setZero();
  w.fill(1e-6);
  quaternion::exp3(w, quat);
  quaternion::Jexp3CoeffWise(w, Jexp3);
  for (int i = 0; i < 3; ++i)
  {
    dw[i] = eps;
    SE3::Quaternion quat_plus;
    quaternion::exp3(w + dw, quat_plus);
    Jexp3_fd.col(i) = (quat_plus.coeffs() - quat.coeffs()) / eps;
    dw[i] = 0;
  }
  BOOST_CHECK(Jexp3.isApprox(Jexp3_fd, sqrt(eps)));
}

BOOST_AUTO_TEST_CASE(Jexp3_quat)
{
  SE3 M(SE3::Random());
  SE3::Quaternion quat(M.rotation());

  Motion dv(Motion::Zero());
  const double eps = 1e-8;

  typedef Eigen::Matrix<double, 7, 6> Matrix76;
  Matrix76 Jexp6_fd, Jexp6_quat;
  Jexp6_quat.setZero();
  typedef Eigen::Matrix<double, 4, 3> Matrix43;
  PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
  PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_MAYBE_UNINITIALIZED
  Matrix43 Jexp3_quat;
  Jexp3QuatLocal(quat, Jexp3_quat);
  PINOCCHIO_COMPILER_DIAGNOSTIC_POP
  SE3 M_next;

  Jexp6_quat.middleRows<3>(Motion::LINEAR).middleCols<3>(Motion::LINEAR) = M.rotation();
  Jexp6_quat.middleRows<4>(Motion::ANGULAR).middleCols<3>(Motion::ANGULAR) =
    Jexp3_quat; // * Jlog6_SE3.middleRows<3>(Motion::ANGULAR);
  for (int i = 0; i < 6; ++i)
  {
    dv.toVector()[i] = eps;
    M_next = M * exp6(dv);
    const SE3::Quaternion quat_next(M_next.rotation());
    Jexp6_fd.middleRows<3>(Motion::LINEAR).col(i) = (M_next.translation() - M.translation()) / eps;
    Jexp6_fd.middleRows<4>(Motion::ANGULAR).col(i) = (quat_next.coeffs() - quat.coeffs()) / eps;
    dv.toVector()[i] = 0.;
  }

  BOOST_CHECK(Jexp6_quat.isApprox(Jexp6_fd, sqrt(eps)));
}

BOOST_AUTO_TEST_CASE(Jexplog3)
{
  Motion v(Motion::Random());

  PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
  PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_MAYBE_UNINITIALIZED
  Eigen::Matrix3d R(exp3(v.angular())), Jexp, Jlog;
  Jexp3(v.angular(), Jexp);
  Jlog3(R, Jlog);
  PINOCCHIO_COMPILER_DIAGNOSTIC_POP

  BOOST_CHECK((Jlog * Jexp).isIdentity());

  SE3 M(SE3::Random());
  R = M.rotation();
  v.angular() = log3(R);
  Jlog3(R, Jlog);
  Jexp3(v.angular(), Jexp);

  BOOST_CHECK((Jexp * Jlog).isIdentity());
}

BOOST_AUTO_TEST_CASE(Jlog3_quat)
{
  SE3::Vector3 w;
  w.setRandom();
  SE3::Quaternion quat;
  quaternion::exp3(w, quat);

  SE3::Matrix3 R(quat.toRotationMatrix());

  PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
  PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_MAYBE_UNINITIALIZED
  SE3::Matrix3 res, res_ref;
  quaternion::Jlog3(quat, res);
  Jlog3(R, res_ref);
  PINOCCHIO_COMPILER_DIAGNOSTIC_POP

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

  PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
  PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_MAYBE_UNINITIALIZED
  SE3::Matrix6 Jfd, Jlog;
  Jlog6(M, Jlog);
  PINOCCHIO_COMPILER_DIAGNOSTIC_POP
  Jfd.setZero();

  Motion dM;
  dM.setZero();
  double step = 1e-8;
  for (int i = 0; i < 6; ++i)
  {
    dM.toVector()[i] = step;
    SE3 M_dM = M * exp6(dM);
    Jfd.col(i) = (log6(M_dM).toVector() - log6(M).toVector()) / step;
    dM.toVector()[i] = 0;
  }

  BOOST_CHECK(Jfd.isApprox(Jlog, sqrt(step)));

  SE3::Matrix6 Jlog2 = Jlog6(M);
  BOOST_CHECK(Jlog2.isApprox(Jlog));
}

BOOST_AUTO_TEST_CASE(Jlog6_singular)
{
  for (size_t i = 0; i < 15; i++)
  {
    SE3 M0;
    if (i == 0)
    {
      M0 = SE3::Identity();
    }
    else
    {
      M0 = SE3::Random();
    }
    SE3 dM(M0.actInv(M0));

    BOOST_CHECK(dM.isApprox(SE3::Identity()));
    SE3::Matrix6 J0(SE3::Matrix6::Identity());

    SE3::Matrix6 J_val = Jlog6(dM);
    BOOST_CHECK(J0.isApprox(J_val));
  }
}

BOOST_AUTO_TEST_CASE(Jexp6_fd)
{
  SE3 M(SE3::Random());

  const Motion v = log6(M);

  SE3::Matrix6 Jexp_fd, Jexp;

  Jexp6(Motion::Zero(), Jexp);
  BOOST_CHECK(Jexp.isIdentity());

  Jexp6(v, Jexp);

  Motion::Vector6 dv;
  dv.setZero();
  const double eps = 1e-8;
  for (int i = 0; i < 6; ++i)
  {
    dv[i] = eps;
    SE3 M_next = exp6(v + Motion(dv));
    Jexp_fd.col(i) = log6(M.actInv(M_next)).toVector() / eps;
    dv[i] = 0;
  }
  BOOST_CHECK(Jexp_fd.isApprox(Jexp, std::sqrt(eps)));

  SE3::Matrix6 Jexp2 = Jexp6(v);
  BOOST_CHECK(Jexp2.isApprox(Jexp));
}

BOOST_AUTO_TEST_CASE(Jlog6_of_product_fd)
{
  SE3 Ma(SE3::Random());
  SE3 Mb(SE3::Random());

  PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
  PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_MAYBE_UNINITIALIZED
  SE3::Matrix6 Jlog, Ja, Jb, Jfda, Jfdb;
  Jlog6(Ma.inverse() * Mb, Jlog);
  PINOCCHIO_COMPILER_DIAGNOSTIC_POP
  Ja = -Jlog * (Ma.inverse() * Mb).toActionMatrixInverse();
  Jb = Jlog;
  Jfda.setZero();
  Jfdb.setZero();

  Motion dM;
  dM.setZero();
  double step = 1e-8;
  for (int i = 0; i < 6; ++i)
  {
    dM.toVector()[i] = step;
    Jfda.col(i) =
      (log6((Ma * exp6(dM)).inverse() * Mb).toVector() - log6(Ma.inverse() * Mb).toVector()) / step;
    Jfdb.col(i) =
      (log6(Ma.inverse() * Mb * exp6(dM)).toVector() - log6(Ma.inverse() * Mb).toVector()) / step;
    dM.toVector()[i] = 0;
  }

  BOOST_CHECK(Jfda.isApprox(Ja, sqrt(step)));
  BOOST_CHECK(Jfdb.isApprox(Jb, sqrt(step)));
}

BOOST_AUTO_TEST_CASE(Jexplog6)
{
  Motion v(Motion::Random());

  SE3 M(exp6(v));
  {
    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> Jexp, Jlog;
    Jexp6(v, Jexp);
    Jlog6(M, Jlog);

    BOOST_CHECK((Jlog * Jexp).isIdentity());
  }

  M.setRandom();

  v = log6(M);
  {
    Eigen::Matrix<double, 6, 6> Jexp, Jlog;
    Jlog6(M, Jlog);
    Jexp6(v, Jexp);

    BOOST_CHECK((Jexp * Jlog).isIdentity());
  }
}

BOOST_AUTO_TEST_CASE(Hlog3_fd)
{
  typedef SE3::Vector3 Vector3;
  typedef SE3::Matrix3 Matrix3;
  SE3::Quaternion q;
  quaternion::uniformRandom(q);
  Matrix3 R(q.matrix());

  // Hlog3(R, v) returns the matrix H * v
  // We check that H * e_k matches the finite difference of Hlog3(R, e_k)
  Vector3 dR;
  dR.setZero();
  double step = 1e-8;
  for (int k = 0; k < 3; ++k)
  {
    Vector3 e_k(Vector3::Zero());
    e_k[k] = 1.;

    Matrix3 Hlog_e_k;
    Hlog3(R, e_k, Hlog_e_k);

    Matrix3 R_dR = R * exp3(step * e_k);
    Matrix3 Jlog_R, Jlog_R_dR;
    Jlog3(R, Jlog_R);
    Jlog3(R_dR, Jlog_R_dR);

    Matrix3 Hlog_e_k_fd = (Jlog_R_dR - Jlog_R) / step;

    BOOST_CHECK(Hlog_e_k.isApprox(Hlog_e_k_fd, sqrt(step)));
  }
}

BOOST_AUTO_TEST_CASE(test_basic)
{
  typedef pinocchio::SE3::Vector3 Vector3;
  typedef pinocchio::SE3::Matrix3 Matrix3;
  typedef Eigen::Matrix4d Matrix4;
  typedef pinocchio::Motion::Vector6 Vector6;

  const double EPSILON = 1e-12;

  // exp3 and log3.
  Vector3 v3(Vector3::Random());
  Matrix3 R(pinocchio::exp3(v3));
  BOOST_CHECK(R.transpose().isApprox(R.inverse(), EPSILON));
  BOOST_CHECK_SMALL(R.determinant() - 1.0, EPSILON);
  Vector3 v3FromLog(pinocchio::log3(R));
  BOOST_CHECK(v3.isApprox(v3FromLog, EPSILON));

  // exp6 and log6.
  pinocchio::Motion nu = pinocchio::Motion::Random();
  pinocchio::SE3 m = pinocchio::exp6(nu);
  BOOST_CHECK(m.rotation().transpose().isApprox(m.rotation().inverse(), EPSILON));
  BOOST_CHECK_SMALL(m.rotation().determinant() - 1.0, EPSILON);
  pinocchio::Motion nuFromLog(pinocchio::log6(m));
  BOOST_CHECK(nu.linear().isApprox(nuFromLog.linear(), EPSILON));
  BOOST_CHECK(nu.angular().isApprox(nuFromLog.angular(), EPSILON));

  Vector6 v6(Vector6::Random());
  pinocchio::SE3 m2(pinocchio::exp6(v6));
  BOOST_CHECK(m2.rotation().transpose().isApprox(m2.rotation().inverse(), EPSILON));
  BOOST_CHECK_SMALL(m2.rotation().determinant() - 1.0, EPSILON);
  Matrix4 M = m2.toHomogeneousMatrix();
  pinocchio::Motion nu2FromLog(pinocchio::log6(M));
  Vector6 v6FromLog(nu2FromLog.toVector());
  BOOST_CHECK(v6.isApprox(v6FromLog, EPSILON));
}

BOOST_AUTO_TEST_CASE(test_SE3_interpolate)
{
  SE3 A = SE3::Random();
  SE3 B = SE3::Random();

  SE3 A_bis = SE3::Interpolate(A, B, 0.);
  BOOST_CHECK(A_bis.isApprox(A));
  SE3 B_bis = SE3::Interpolate(A, B, 1.);
  BOOST_CHECK(B_bis.isApprox(B));

  A_bis = SE3::Interpolate(A, A, 1.);
  BOOST_CHECK(A_bis.isApprox(A));

  B_bis = SE3::Interpolate(B, B, 1.);
  BOOST_CHECK(B_bis.isApprox(B));

  SE3 C = SE3::Interpolate(A, B, 0.5);
  SE3 D = SE3::Interpolate(B, A, 0.5);
  BOOST_CHECK(D.isApprox(C));
}

BOOST_AUTO_TEST_CASE(test_Jlog6_robustness)
{
  const int num_tests = 1e1;
  for (int k = 0; k < num_tests; ++k)
  {
    const SE3 M = SE3::Random();
    SE3::Matrix6 res = Jlog6(M.actInv(M));

    BOOST_CHECK(res.isIdentity());
  }
}

BOOST_AUTO_TEST_SUITE_END()
