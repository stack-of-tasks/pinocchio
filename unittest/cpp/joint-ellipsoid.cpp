//
// Copyright (c) 2025 INRIA
//

#define BOOST_TEST_MODULE joint_ellipsoid

#include "pinocchio/spatial.hpp"
#include "pinocchio/multibody.hpp"
#include "pinocchio/multibody/joint.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

#include <boost/test/unit_test.hpp>

using namespace pinocchio;
template<typename D>
void addJointAndBody(
  Model & model,
  const JointModelBase<D> & jmodel,
  const Model::JointIndex parent_id,
  const SE3 & joint_placement,
  const std::string & joint_name,
  const Inertia & Y)
{
  Model::JointIndex idx;

  idx = model.addJoint(parent_id, jmodel, joint_placement, joint_name);
  model.appendBodyToJoint(idx, Y);
}

/// \brief Compute motion subspace derivative Sdot analytically for JointModelEllipsoid
Eigen::Matrix<double, 6, 3> computeMotionSubspaceDerivative(
  const JointModelEllipsoid & jmodel, const Eigen::VectorXd & qs, const Eigen::VectorXd & vs)
{
  double c0, s0;
  SINCOS(qs(0), &s0, &c0);
  double c1, s1;
  SINCOS(qs(1), &s1, &c1);
  double c2, s2;
  SINCOS(qs(2), &s2, &c2);

  // Extract velocities
  double qdot0 = vs(0);
  double qdot1 = vs(1);
  double qdot2 = vs(2);

  // Get radii
  const double radius_x = jmodel.radius_x;
  const double radius_y = jmodel.radius_y;
  const double radius_z = jmodel.radius_z;

  // Derivatives of normal vector components w.r.t. velocities
  double dndotx_dqdot1 = c1;
  double dndoty_dqdot0 = -c0 * c1;
  double dndoty_dqdot1 = s0 * s1;
  double dndotz_dqdot0 = -c1 * s0;
  double dndotz_dqdot1 = -c0 * s1;

  // Second derivatives (derivatives of dndot w.r.t. configuration)
  double d_dndotx_dqdot1_dq1 = -s1;
  double d_dndoty_dqdot0_dq0 = s0 * c1;
  double d_dndoty_dqdot0_dq1 = c0 * s1;
  double d_dndoty_dqdot1_dq1 = s0 * c1;
  double d_dndotz_dqdot0_dq0 = -c1 * c0;
  double d_dndotz_dqdot0_dq1 = s0 * s1;
  double d_dndotz_dqdot1_dq1 = -c0 * c1;

  // Translational part (rows 1-3)
  double
    Sdot_11 =
      qdot0
        * (-dndoty_dqdot0 * radius_y * (-c0 * c2 * s1 + s0 * s2) + dndotz_dqdot0 * radius_z * (c0 * s2 + c2 * s0 * s1) + radius_y * (c0 * s2 + c2 * s0 * s1) * d_dndoty_dqdot0_dq0 + radius_z * (-c0 * c2 * s1 + s0 * s2) * d_dndotz_dqdot0_dq0)
      + qdot1 * (dndoty_dqdot0 * radius_y * c1 * c2 * s0 - dndotz_dqdot0 * radius_z * c0 * c1 * c2 + radius_y * (c0 * s2 + c2 * s0 * s1) * d_dndoty_dqdot0_dq1 + radius_z * (-c0 * c2 * s1 + s0 * s2) * d_dndotz_dqdot0_dq1)
      - qdot2
          * (dndoty_dqdot0 * radius_y * (-c0 * c2 + s0 * s1 * s2) - dndotz_dqdot0 * radius_z * (c0 * s1 * s2 + c2 * s0));

  double
    Sdot_12 =
      qdot0 * (-dndoty_dqdot1 * radius_y * (-c0 * c2 * s1 + s0 * s2) + dndotz_dqdot1 * radius_z * (c0 * s2 + c2 * s0 * s1) + radius_y * (c0 * s2 + c2 * s0 * s1) * d_dndoty_dqdot0_dq1 + radius_z * (-c0 * c2 * s1 + s0 * s2) * d_dndotz_dqdot0_dq1) + qdot1 * (-dndotx_dqdot1 * radius_x * c2 * s1 + dndoty_dqdot1 * radius_y * c1 * c2 * s0 - dndotz_dqdot1 * radius_z * c0 * c1 * c2 + radius_x * c1 * c2 * d_dndotx_dqdot1_dq1 + radius_y * (c0 * s2 + c2 * s0 * s1) * d_dndoty_dqdot1_dq1 + radius_z * (-c0 * c2 * s1 + s0 * s2) * d_dndotz_dqdot1_dq1)
      - qdot2
          * (dndotx_dqdot1 * radius_x * c1 * s2 + dndoty_dqdot1 * radius_y * (-c0 * c2 + s0 * s1 * s2) - dndotz_dqdot1 * radius_z * (c0 * s1 * s2 + c2 * s0));

  double
    Sdot_21 = -qdot0 * (dndoty_dqdot0 * radius_y * (c0 * s1 * s2 + c2 * s0) + dndotz_dqdot0 * radius_z * (-c0 * c2 + s0 * s1 * s2) + radius_y * (-c0 * c2 + s0 * s1 * s2) * d_dndoty_dqdot0_dq0 - radius_z * (c0 * s1 * s2 + c2 * s0) * d_dndotz_dqdot0_dq0) - qdot1 * (dndoty_dqdot0 * radius_y * c1 * s0 * s2 - dndotz_dqdot0 * radius_z * c0 * c1 * s2 + radius_y * (-c0 * c2 + s0 * s1 * s2) * d_dndoty_dqdot0_dq1 - radius_z * (c0 * s1 * s2 + c2 * s0) * d_dndotz_dqdot0_dq1)
              - qdot2
                  * (dndoty_dqdot0 * radius_y * (c0 * s2 + c2 * s0 * s1) + dndotz_dqdot0 * radius_z * (-c0 * c2 * s1 + s0 * s2));

  double
    Sdot_22 =
      -qdot0 * (dndoty_dqdot1 * radius_y * (c0 * s1 * s2 + c2 * s0) + dndotz_dqdot1 * radius_z * (-c0 * c2 + s0 * s1 * s2) + radius_y * (-c0 * c2 + s0 * s1 * s2) * d_dndoty_dqdot0_dq1 - radius_z * (c0 * s1 * s2 + c2 * s0) * d_dndotz_dqdot0_dq1) + qdot1 * (dndotx_dqdot1 * radius_x * s1 * s2 - dndoty_dqdot1 * radius_y * c1 * s0 * s2 + dndotz_dqdot1 * radius_z * c0 * c1 * s2 - radius_x * c1 * s2 * d_dndotx_dqdot1_dq1 - radius_y * (-c0 * c2 + s0 * s1 * s2) * d_dndoty_dqdot1_dq1 + radius_z * (c0 * s1 * s2 + c2 * s0) * d_dndotz_dqdot1_dq1)
      - qdot2
          * (dndotx_dqdot1 * radius_x * c1 * c2 + dndoty_dqdot1 * radius_y * (c0 * s2 + c2 * s0 * s1) + dndotz_dqdot1 * radius_z * (-c0 * c2 * s1 + s0 * s2));

  double Sdot_31 =
    -qdot0 * c1
      * (dndoty_dqdot0 * radius_y * c0 + dndotz_dqdot0 * radius_z * s0 + radius_y * s0 * d_dndoty_dqdot0_dq0 - radius_z * c0 * d_dndotz_dqdot0_dq0)
    + qdot1
        * (-c1 * (radius_y * s0 * d_dndoty_dqdot0_dq1 - radius_z * c0 * d_dndotz_dqdot0_dq1) + s1 * (dndoty_dqdot0 * radius_y * s0 - dndotz_dqdot0 * radius_z * c0));

  double Sdot_32 =
    -qdot0 * c1
      * (dndoty_dqdot1 * radius_y * c0 + dndotz_dqdot1 * radius_z * s0 + radius_y * s0 * d_dndoty_dqdot0_dq1 - radius_z * c0 * d_dndotz_dqdot0_dq1)
    + qdot1
        * (dndotx_dqdot1 * radius_x * c1 + dndoty_dqdot1 * radius_y * s0 * s1 - dndotz_dqdot1 * radius_z * c0 * s1 + radius_x * s1 * d_dndotx_dqdot1_dq1 - radius_y * c1 * s0 * d_dndoty_dqdot1_dq1 + radius_z * c0 * c1 * d_dndotz_dqdot1_dq1);

  // Angular part (rows 4-6)
  double Sdot_41 = -(qdot1 * c2 * s1 + qdot2 * c1 * s2);
  double Sdot_51 = qdot1 * s1 * s2 - qdot2 * c1 * c2;
  double Sdot_61 = qdot1 * c1;

  double Sdot_42 = qdot2 * c2;
  double Sdot_52 = -qdot2 * s2;

  // Build and return 6x3 matrix
  Eigen::Matrix<double, 6, 3> Sdot;
  Sdot << Sdot_11, Sdot_12, 0.0, Sdot_21, Sdot_22, 0.0, Sdot_31, Sdot_32, 0.0, Sdot_41, Sdot_42,
    0.0, Sdot_51, Sdot_52, 0.0, Sdot_61, 0.0, 0.0;

  return Sdot;
}

/// \brief Compute Sdot (motion subspace derivative) via finite differences
template<typename JointModel>
Eigen::Matrix<double, 6, JointModel::NV> finiteDiffSdot(
  const JointModel & jmodel,
  typename JointModel::JointDataDerived & jdata,
  const typename JointModel::ConfigVector_t & q,
  const typename JointModel::TangentVector_t & v,
  double eps = 1e-8)
{
  typedef typename LieGroup<JointModel>::type LieGroupType;
  typedef typename JointModel::ConfigVector_t ConfigVector_t;
  typedef typename JointModel::TangentVector_t TangentVector_t;

  const Eigen::DenseIndex nv = jmodel.nv();

  Eigen::Matrix<double, 6, JointModel::NV> Sdot_fd;
  Sdot_fd.setZero();

  ConfigVector_t q_integrated(q);
  TangentVector_t v_integrate(nv);
  v_integrate.setZero();

  for (Eigen::DenseIndex k = 0; k < nv; ++k)
  {
    // Integrate along kth direction
    v_integrate[k] = eps;
    q_integrated = LieGroupType().integrate(q, v_integrate);

    // Compute S at q + eps * e_k
    jmodel.calc(jdata, q_integrated);
    const Eigen::Matrix<double, 6, JointModel::NV> S_plus = jdata.S.matrix();

    // Integrate in negative direction
    v_integrate[k] = -eps;
    q_integrated = LieGroupType().integrate(q, v_integrate);

    // Compute S at q - eps * e_k
    jmodel.calc(jdata, q_integrated);
    const Eigen::Matrix<double, 6, JointModel::NV> S_minus = jdata.S.matrix();

    // Compute dS/dq_k via central differences
    Eigen::Matrix<double, 6, JointModel::NV> dS_dqk = (S_plus - S_minus) / (2.0 * eps);

    // Accumulate: Sdot += (dS/dq_k) * v_k
    Sdot_fd += dS_dqk * v[k];

    // Reset
    v_integrate[k] = 0.;
  }

  return Sdot_fd;
}

SE3::Vector3 computeTranslations(const JointModelEllipsoid & jmodel, const Eigen::VectorXd & qs)
{
  double c0, s0;
  SINCOS(qs(0), &s0, &c0);
  double c1, s1;
  SINCOS(qs(1), &s1, &c1);

  double nx, ny, nz;
  nx = s1;
  ny = -s0 * c1;
  nz = c0 * c1;

  return SE3::Vector3(jmodel.radius_x * nx, jmodel.radius_y * ny, jmodel.radius_z * nz);
}

SE3::Vector3 computeTranslationVelocities(
  const JointModelEllipsoid & jmodel, const Eigen::VectorXd & qs, const Eigen::VectorXd & vs)
{
  double c0, s0;
  SINCOS(qs(0), &s0, &c0);
  double c1, s1;
  SINCOS(qs(1), &s1, &c1);

  SE3::Vector3 v;
  v(0) = jmodel.radius_x * c1 * vs(1);
  v(1) = jmodel.radius_y * (-c0 * c1 * vs(0) + s0 * s1 * vs(1));
  v(2) = jmodel.radius_z * (-s0 * c1 * vs(0) - c0 * s1 * vs(1));
  return v;
}

SE3::Vector3 computeTranslationAccelerations(
  const JointModelEllipsoid & jmodel,
  const Eigen::VectorXd & qs,
  const Eigen::VectorXd & vs,
  const Eigen::VectorXd & as)
{
  double c0, s0;
  SINCOS(qs(0), &s0, &c0);
  double c1, s1;
  SINCOS(qs(1), &s1, &c1);
  SE3::Vector3 a;
  a(0) = jmodel.radius_x * (-s1 * vs(1) * vs(1) + c1 * as(1));
  a(1) =
    jmodel.radius_y
    * (s0 * c1 * vs(0) * vs(0) + c0 * s1 * vs(0) * vs(1) - c0 * c1 * as(0) + c0 * s1 * vs(1) * vs(0) + s0 * c1 * vs(1) * vs(1) + s0 * s1 * as(1));
  a(2) =
    jmodel.radius_z
    * (-c0 * c1 * vs(0) * vs(0) + s0 * s1 * vs(0) * vs(1) - s0 * c1 * as(0) + s0 * s1 * vs(1) * vs(0) - c0 * c1 * vs(1) * vs(1) - c0 * s1 * as(1));
  return a;
}

/// @brief Test the rotationnal equivalence between JointModelEllipsoid and JointModelSphericalZYX
BOOST_AUTO_TEST_CASE(vsSphericalZYX)
{
  using namespace pinocchio;
  typedef SE3::Vector3 Vector3;
  typedef SE3::Matrix3 Matrix3;

  Inertia inertia(1., Vector3(0.5, 0., 0.0), Matrix3::Identity());
  SE3 pos(1);
  pos.translation() = SE3::LinearType(1., 0., 0.);

  // Create both models with the same structure
  Model modelEllipsoid, modelSphericalZYX;
  addJointAndBody(modelEllipsoid, JointModelEllipsoid(0, 0, 0), 0, pos, "ellipsoid", inertia);
  addJointAndBody(modelSphericalZYX, JointModelSphericalZYX(), 0, pos, "spherical", inertia);

  Data dataEllipsoid(modelEllipsoid);
  Data dataSphericalZYX(modelSphericalZYX);

  // Start with ZYX angles
  Eigen::VectorXd q_s(3);
  q_s << 0.5, 1.2, -0.8; // Z=0.5, Y=1.2, X=-0.8
  Eigen::VectorXd qd_s(3);
  qd_s << 0.1, -0.3, 0.7; // ZYX angle velocities
  Eigen::VectorXd qdotdot_s(3);
  qdotdot_s << 0.2, 0.1, -0.1; // ZYX angle accelerations
  // Compute the rotation matrix from ZYX angles
  forwardKinematics(modelSphericalZYX, dataSphericalZYX, q_s, qd_s);
  const Matrix3 & R = dataSphericalZYX.oMi[1].rotation();
  const Motion & spatial_vel_zyx = dataSphericalZYX.v[1];
  PINOCCHIO_UNUSED_VARIABLE(spatial_vel_zyx);

  // Extract XYZ Euler angles from the rotation matrix
  // For XYZ convention: R = Rx(x) * Ry(y) * Rz(z)
  // We need to solve for x, y, z
  Eigen::Vector3d q_e;
  q_e = R.eulerAngles(0, 1, 2); // XYZ convention

  // Get the motion subspace matrices (which give us the Jacobians)
  JointModelSphericalZYX jmodel_s;
  jmodel_s.setIndexes(0, 0, 0);

  JointDataSphericalZYX jdata_s = jmodel_s.createData();
  jmodel_s.calc(jdata_s, q_s);

  JointModelEllipsoid jmodel_e(0, 0, 0);
  jmodel_e.setIndexes(0, 0, 0);

  JointDataEllipsoid jdata_e = jmodel_e.createData();
  jmodel_e.calc(jdata_e, q_e);

  // The motion subspace S gives us: omega = S * v
  Matrix3 S_s = jdata_s.S.matrix().bottomRows<3>(); // Angular part
  Matrix3 S_e = jdata_e.S.matrix().bottomRows<3>(); // Angular part

  Eigen::Vector3d qd_e = S_e.inverse() * S_s * qd_s;

  Eigen::Vector3d w_s = S_s * qd_s;
  Eigen::Vector3d w_e = S_e * qd_e;

  BOOST_CHECK(w_s.isApprox(w_e));

  // Compute forward kinematics with the converted configurations
  forwardKinematics(modelEllipsoid, dataEllipsoid, q_e, qd_e);

  // Getting S with q_e from the three calcs
  JointDataEllipsoid jDataEllipsoidFK = jmodel_e.createData();
  JointDataEllipsoid jDataEllipsoidFK2 = jmodel_e.createData();
  JointDataEllipsoid jDataEllipsoidFK3 = jmodel_e.createData();

  jmodel_e.calc(jDataEllipsoidFK, q_e, qd_e);
  jmodel_e.calc(jDataEllipsoidFK2, q_e);
  jmodel_e.calc(jDataEllipsoidFK3, q_e);
  jmodel_e.calc(jDataEllipsoidFK3, Blank(), qd_e);

  BOOST_CHECK(jDataEllipsoidFK.S.matrix().isApprox(jDataEllipsoidFK2.S.matrix()));
  BOOST_CHECK(jDataEllipsoidFK.S.matrix().isApprox(jDataEllipsoidFK3.S.matrix()));

  JointDataSphericalZYX jDataSphereFK = jmodel_s.createData();
  jmodel_s.calc(jDataSphereFK, q_s, qd_s);

  // Joint-frame velocities (S * v)
  Eigen::Matrix<double, 6, 1> joint_vel_e = jDataEllipsoidFK.S.matrix() * qd_e;
  Eigen::Matrix<double, 6, 1> manual_vel = jDataEllipsoidFK.S.matrix() * jDataEllipsoidFK.joint_v;
  Eigen::Matrix<double, 6, 1> joint_vel_s = jDataSphereFK.S.matrix() * qd_s;
  PINOCCHIO_UNUSED_VARIABLE(joint_vel_e);
  PINOCCHIO_UNUSED_VARIABLE(manual_vel);
  PINOCCHIO_UNUSED_VARIABLE(joint_vel_s);

  BOOST_CHECK(dataEllipsoid.v[1].toVector().isApprox(dataSphericalZYX.v[1].toVector()));
  BOOST_CHECK(dataEllipsoid.oMi[1].isApprox(dataSphericalZYX.oMi[1]));

  Eigen::Vector3d c_e = jDataEllipsoidFK.c.angular(); // Sdot * qd_e

  // The acceleration conversion formula: wdot_s = wdot_e
  // S_s * qdotdot_s + c_s.angular() = Sdot_e * qd_e + S_e * qdotdot_e
  // Solving for qdotdot_e:
  // S_e * qdotdot_e =  c_s.angular()+ S_s * qdotdot_s - Sdot_e * qd_e
  Eigen::Vector3d qdotdot_e = S_e.inverse() * (S_s * qdotdot_s + jDataSphereFK.c.angular() - c_e);

  // Verify angular accelerations match
  Eigen::Vector3d wdot_s = jDataSphereFK.c.angular() + S_s * qdotdot_s;
  Eigen::Vector3d wdot_e = c_e + S_e * qdotdot_e;
  BOOST_CHECK(wdot_s.isApprox(wdot_e));

  forwardKinematics(modelEllipsoid, dataEllipsoid, q_e, qd_e, qdotdot_e);
  forwardKinematics(modelSphericalZYX, dataSphericalZYX, q_s, qd_s, qdotdot_s);

  BOOST_CHECK(dataEllipsoid.a[1].toVector().isApprox(dataSphericalZYX.a[1].toVector()));

  // Test RNEA (Recursive Newton-Euler Algorithm) - spatial forces should match with both joints
  rnea(modelEllipsoid, dataEllipsoid, q_e, qd_e, qdotdot_e);
  rnea(modelSphericalZYX, dataSphericalZYX, q_s, qd_s, qdotdot_s);

  BOOST_CHECK(dataEllipsoid.f[1].isApprox(dataSphericalZYX.f[1]));
}

/// @brief Test the equivalence between JointModelEllipsoid and a Composite joint (Tx, Ty, Tz, Rx,
/// Ry, Rz)
BOOST_AUTO_TEST_CASE(vsCompositeTxTyTzRxRyRz)
{
  using namespace pinocchio;

  // Ellipsoid parameters
  double radius_x = 2.0;
  double radius_y = 1.5;
  double radius_z = 1.0;

  Inertia inertia = Inertia::Identity();
  SE3 pos = SE3::Identity();

  // Create Ellipsoid model
  Model modelEllipsoid;
  JointModelEllipsoid jointModelEllipsoid(radius_x, radius_y, radius_z);
  addJointAndBody(modelEllipsoid, jointModelEllipsoid, 0, pos, "ellipsoid", inertia);

  // Create Composite model (Tx, Ty, Tz, Rx, Ry, Rz)
  Model modelComposite;
  JointModelComposite jComposite;
  jComposite.addJoint(JointModelPX());
  jComposite.addJoint(JointModelPY());
  jComposite.addJoint(JointModelPZ());
  jComposite.addJoint(JointModelRX());
  jComposite.addJoint(JointModelRY());
  jComposite.addJoint(JointModelRZ());
  addJointAndBody(modelComposite, jComposite, 0, pos, "composite", inertia);

  Data dataEllipsoid(modelEllipsoid);
  Data dataComposite(modelComposite);

  // Test positions of ellispoid vs composite
  Eigen::VectorXd q_ellipsoid(3);
  q_ellipsoid << 1.0, 2.0, 3.0; // rx, ry, rz
  Eigen::Vector3d t = computeTranslations(jointModelEllipsoid, q_ellipsoid);
  Eigen::VectorXd q_composite(6);
  q_composite << t, q_ellipsoid;

  forwardKinematics(modelEllipsoid, dataEllipsoid, q_ellipsoid);
  forwardKinematics(modelComposite, dataComposite, q_composite);

  BOOST_CHECK(dataEllipsoid.oMi[1].isApprox(dataComposite.oMi[1]));

  // Velocity test
  Eigen::VectorXd qdot_ellipsoid(3);
  qdot_ellipsoid << 0.1, 0.2, 0.3;

  Eigen::Vector3d v_linear =
    computeTranslationVelocities(jointModelEllipsoid, q_ellipsoid, qdot_ellipsoid);
  Eigen::VectorXd qdot_composite(6);
  qdot_composite << v_linear, qdot_ellipsoid;

  forwardKinematics(modelEllipsoid, dataEllipsoid, q_ellipsoid, qdot_ellipsoid);
  forwardKinematics(modelComposite, dataComposite, q_composite, qdot_composite);

  BOOST_CHECK(dataEllipsoid.v[1].toVector().isApprox(dataComposite.v[1].toVector()));

  // Acceleration test
  Eigen::VectorXd qddot_ellipsoid(3);
  qddot_ellipsoid << 0.01, 0.02, 0.03;
  Eigen::Vector3d a_linear = computeTranslationAccelerations(
    jointModelEllipsoid, q_ellipsoid, qdot_ellipsoid, qddot_ellipsoid);
  Eigen::VectorXd qddot_composite(6);
  qddot_composite << a_linear, qddot_ellipsoid;

  forwardKinematics(modelEllipsoid, dataEllipsoid, q_ellipsoid, qdot_ellipsoid, qddot_ellipsoid);
  forwardKinematics(modelComposite, dataComposite, q_composite, qdot_composite, qddot_composite);

  BOOST_CHECK(dataEllipsoid.a[1].toVector().isApprox(dataComposite.a[1].toVector()));

  // Test RNEA - spatial forces and torques should match
  rnea(modelEllipsoid, dataEllipsoid, q_ellipsoid, qdot_ellipsoid, qddot_ellipsoid);
  rnea(modelComposite, dataComposite, q_composite, qdot_composite, qddot_composite);

  BOOST_CHECK(dataEllipsoid.f[1].isApprox(dataComposite.f[1]));

  // Need joint data to get both motion subspaces S_comp and S_ell
  JointDataComposite jdata_c = jComposite.createData();
  jComposite.setIndexes(0, 0, 0);
  jComposite.calc(jdata_c, q_composite, qdot_composite);

  JointDataEllipsoid jdata_e = jointModelEllipsoid.createData();
  jointModelEllipsoid.setIndexes(0, 0, 0);
  jointModelEllipsoid.calc(jdata_e, q_ellipsoid, qdot_ellipsoid);

  const Eigen::Matrix<double, 6, 6> S_comp = jdata_c.S.matrix(); // 6x6 (Tx,Ty,Tz,Rx,Ry,Rz)
  const Eigen::Matrix<double, 6, 3> S_ell = jdata_e.S.matrix();  // 6x3 (ellipsoid)

  // Compute the Jacobian mapping from ellipsoid generalized velocities to composite ones:
  // qdot_comp = J * qdot_ell with J = (S_comp^T * S_comp)^-1 * S_comp^T * S_ell
  Eigen::MatrixXd J = (S_comp.transpose() * S_comp).ldlt().solve(S_comp.transpose() * S_ell);

  // Now map the torques back (dual mapping)
  // τ_ell = J^T * τ_comp
  Eigen::VectorXd tau_proj = J.transpose() * dataComposite.tau;

  // Check the numerical match
  BOOST_CHECK_MESSAGE(
    dataEllipsoid.tau.isApprox(tau_proj, 1e-6),
    "Projected composite torques do not match ellipsoid torques.\n"
      << "Expected: " << dataEllipsoid.tau.transpose() << "\nGot: " << tau_proj.transpose());
}

/// @brief Test RNEA vs ABA with multiple random configurations
BOOST_AUTO_TEST_CASE(RNEAvsABA)
{
  using namespace pinocchio;
  typedef SE3::Vector3 Vector3;
  typedef SE3::Matrix3 Matrix3;

  double radius_x = 2.5;
  double radius_y = 1.8;
  double radius_z = 1.2;

  Inertia inertia(1.0, Vector3::Zero(), Matrix3::Identity());
  SE3 pos = SE3::Identity();

  Model model;
  JointModelEllipsoid jointModel(radius_x, radius_y, radius_z);
  addJointAndBody(model, jointModel, 0, pos, "ellipsoid", inertia);

  Data data(model);
  Data data_aba(model);

  // Test with multiple random configurations
  for (int trial = 0; trial < 10; ++trial)
  {
    Eigen::VectorXd q = Eigen::VectorXd::Random(model.nq);
    Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
    Eigen::VectorXd a = Eigen::VectorXd::Random(model.nv);

    // RNEA
    rnea(model, data, q, v, a);
    Eigen::VectorXd tau_rnea = data.tau;

    // ABA
    aba(model, data_aba, q, v, tau_rnea, Convention::WORLD);

    BOOST_CHECK_MESSAGE(
      a.isApprox(data_aba.ddq, 1e-9), "RNEA and ABA inconsistent at trial "
                                        << trial << "\n"
                                        << "Configuration: " << q.transpose() << "\n"
                                        << "Expected: " << a.transpose() << "\n"
                                        << "Got: " << data_aba.ddq.transpose());
  }
}

/// @brief Test Sdot via finite differences
BOOST_AUTO_TEST_CASE(testSdotFiniteDifferences)
{
  using namespace pinocchio;

  // Ellipsoid parameters
  double radius_x = 2.0;
  double radius_y = 1.5;
  double radius_z = 1.0;

  JointModelEllipsoid jmodel(radius_x, radius_y, radius_z);
  jmodel.setIndexes(0, 0, 0);

  JointDataEllipsoid jdata = jmodel.createData();

  // Test configuration and velocity
  typedef JointModelEllipsoid::ConfigVector_t ConfigVector_t;
  typedef JointModelEllipsoid::TangentVector_t TangentVector_t;
  typedef LieGroup<JointModelEllipsoid>::type LieGroupType;

  ConfigVector_t q = LieGroupType().random();
  TangentVector_t v = TangentVector_t::Random();

  // Compute analytical Sdot
  const double eps = 1e-8;

  // Reference Sdot with analytical formula
  const Eigen::Matrix<double, 6, 3> Sdot_ref = computeMotionSubspaceDerivative(jmodel, q, v);

  // Compute Sdot via finite differences using helper function
  const Eigen::Matrix<double, 6, 3> Sdot_fd = finiteDiffSdot(jmodel, jdata, q, v, eps);

  BOOST_CHECK(Sdot_ref.isApprox(Sdot_fd, sqrt(eps)));
}

/// @brief Test that biais term equals Sdot * v
BOOST_AUTO_TEST_CASE(testBiaisVsSdotTimesVelocity)
{
  using namespace pinocchio;

  // Ellipsoid parameters
  double radius_x = 2.0;
  double radius_y = 1.5;
  double radius_z = 1.0;

  JointModelEllipsoid jmodel(radius_x, radius_y, radius_z);
  jmodel.setIndexes(0, 0, 0);

  JointDataEllipsoid jdata = jmodel.createData();

  // Test configuration and velocity
  typedef JointModelEllipsoid::ConfigVector_t ConfigVector_t;
  typedef JointModelEllipsoid::TangentVector_t TangentVector_t;
  typedef LieGroup<JointModelEllipsoid>::type LieGroupType;

  ConfigVector_t q = LieGroupType().random();
  TangentVector_t v = TangentVector_t::Random();
  jmodel.calc(jdata, q, v);

  jmodel.computeBiais(jdata, q, v);

  const Eigen::Matrix<double, 6, 3> Sdot = computeMotionSubspaceDerivative(jmodel, q, v);

  BOOST_CHECK(jdata.c.toVector().isApprox(Sdot * v, 1e-12));
}
