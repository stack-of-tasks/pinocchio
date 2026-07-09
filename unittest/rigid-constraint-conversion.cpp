//
// Copyright (c) 2026 INRIA
//

#define BOOST_TEST_MODULE rigid_constraint_conversion

#include "pinocchio/spatial.hpp"
#include "pinocchio/constraints.hpp"
#include "pinocchio/multibody/sample-models.hpp"

#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include <boost/utility/binary.hpp>

#include <boost/test/unit_test.hpp>

using namespace pinocchio;
using namespace Eigen;

/// Verify that a PointAnchorConstraintModel converts to a CONTACT_3D RigidConstraintModel
/// and that the kinematic structure (joint IDs, placements, reference frame) is preserved.
BOOST_AUTO_TEST_CASE(convert_point_anchor_structure)
{
  Model model;
  buildModels::humanoidRandom(model, true);

  const std::string RF = "rleg6_joint";
  const std::string LF = "lleg6_joint";
  const JointIndex rf_id = model.getJointId(RF);
  const JointIndex lf_id = model.getJointId(LF);

  const SE3 placement1 = SE3::Random();
  const SE3 placement2 = SE3::Random();

  // Single-joint constraint
  {
    const PointAnchorConstraintModel cm(model, rf_id, placement1);
    const RigidConstraintModel rcm = convertToRigidConstraintModel(model, cm);

    BOOST_CHECK(rcm.type == CONTACT_3D);
    BOOST_CHECK(rcm.residualSize() == 3);
    BOOST_CHECK(rcm.joint1_id == cm.joint1_id);
    BOOST_CHECK(rcm.joint2_id == cm.joint2_id);
    BOOST_CHECK(rcm.joint1_placement.isApprox(cm.joint1_placement));
    BOOST_CHECK(rcm.joint2_placement.isApprox(cm.joint2_placement));
    BOOST_CHECK(rcm.reference_frame == LOCAL); // default
  }

  // Two-joint constraint
  {
    const PointAnchorConstraintModel cm(model, rf_id, placement1, lf_id, placement2);
    const RigidConstraintModel rcm = convertToRigidConstraintModel(model, cm);

    BOOST_CHECK(rcm.type == CONTACT_3D);
    BOOST_CHECK(rcm.joint1_id == rf_id);
    BOOST_CHECK(rcm.joint2_id == lf_id);
    BOOST_CHECK(rcm.joint1_placement.isApprox(placement1));
    BOOST_CHECK(rcm.joint2_placement.isApprox(placement2));
  }

  // Explicit reference frames
  {
    const PointAnchorConstraintModel cm(model, rf_id, placement1);

    const RigidConstraintModel rcm_local = convertToRigidConstraintModel(model, cm, LOCAL);
    BOOST_CHECK(rcm_local.reference_frame == LOCAL);

    const RigidConstraintModel rcm_lwa =
      convertToRigidConstraintModel(model, cm, LOCAL_WORLD_ALIGNED);
    BOOST_CHECK(rcm_lwa.reference_frame == LOCAL_WORLD_ALIGNED);
  }
}

/// Verify that a FrameAnchorConstraintModel converts to a CONTACT_6D RigidConstraintModel
/// and that the kinematic structure (joint IDs, placements, reference frame) is preserved.
BOOST_AUTO_TEST_CASE(convert_frame_anchor_structure)
{
  Model model;
  buildModels::humanoidRandom(model, true);

  const std::string RF = "rleg6_joint";
  const std::string LF = "lleg6_joint";
  const JointIndex rf_id = model.getJointId(RF);
  const JointIndex lf_id = model.getJointId(LF);

  const SE3 placement1 = SE3::Random();
  const SE3 placement2 = SE3::Random();

  // Single-joint constraint
  {
    const FrameAnchorConstraintModel cm(model, rf_id, placement1);
    const RigidConstraintModel rcm = convertToRigidConstraintModel(model, cm);

    BOOST_CHECK(rcm.type == CONTACT_6D);
    BOOST_CHECK(rcm.residualSize() == 6);
    BOOST_CHECK(rcm.joint1_id == cm.joint1_id);
    BOOST_CHECK(rcm.joint2_id == cm.joint2_id);
    BOOST_CHECK(rcm.joint1_placement.isApprox(cm.joint1_placement));
    BOOST_CHECK(rcm.joint2_placement.isApprox(cm.joint2_placement));
    BOOST_CHECK(rcm.reference_frame == LOCAL); // default
  }

  // Two-joint constraint
  {
    const FrameAnchorConstraintModel cm(model, rf_id, placement1, lf_id, placement2);
    const RigidConstraintModel rcm = convertToRigidConstraintModel(model, cm);

    BOOST_CHECK(rcm.type == CONTACT_6D);
    BOOST_CHECK(rcm.joint1_id == rf_id);
    BOOST_CHECK(rcm.joint2_id == lf_id);
    BOOST_CHECK(rcm.joint1_placement.isApprox(placement1));
    BOOST_CHECK(rcm.joint2_placement.isApprox(placement2));
  }

  // Explicit reference frames
  {
    const FrameAnchorConstraintModel cm(model, rf_id, placement1);

    const RigidConstraintModel rcm_local = convertToRigidConstraintModel(model, cm, LOCAL);
    BOOST_CHECK(rcm_local.reference_frame == LOCAL);

    const RigidConstraintModel rcm_lwa =
      convertToRigidConstraintModel(model, cm, LOCAL_WORLD_ALIGNED);
    BOOST_CHECK(rcm_lwa.reference_frame == LOCAL_WORLD_ALIGNED);
  }
}

/// Verify that the desired fields of a PointAnchorConstraintModel map correctly
/// to the desired_contact_placement, desired_contact_velocity, and
/// desired_contact_acceleration of the converted RigidConstraintModel.
///
/// Expected mapping (independently stated):
///   desired_contact_placement  = SE3(Matrix3d::Identity(), desired_constraint_offset)
///   desired_contact_velocity   = Motion(desired_constraint_velocity,  Vector3d::Zero())
///   desired_contact_acceleration = Motion(desired_constraint_acceleration, Vector3d::Zero())
BOOST_AUTO_TEST_CASE(convert_point_anchor_desired_fields)
{
  Model model;
  buildModels::humanoidRandom(model, true);
  const JointIndex rf_id = model.getJointId("rleg6_joint");

  // Zero desired fields -> identity placement and zero motions
  {
    const PointAnchorConstraintModel cm(model, rf_id, SE3::Identity());
    // desired_constraint_* members are zero-initialised
    const RigidConstraintModel rcm = convertToRigidConstraintModel(model, cm);

    BOOST_CHECK(rcm.desired_contact_placement.isIdentity());
    BOOST_CHECK(rcm.desired_contact_velocity.isZero());
    BOOST_CHECK(rcm.desired_contact_acceleration.isZero());
  }

  // Non-zero desired fields
  {
    const Vector3d t_offset(1., 2., 3.);
    const Vector3d v_desired(4., 5., 6.);
    const Vector3d a_desired(7., 8., 9.);

    PointAnchorConstraintModel cm(model, rf_id, SE3::Identity());
    cm.desired_constraint_offset = t_offset;
    cm.desired_constraint_velocity = v_desired;
    cm.desired_constraint_acceleration = a_desired;

    const RigidConstraintModel rcm = convertToRigidConstraintModel(model, cm);

    // Build expected values independently using SE3 and Motion constructors
    const SE3 expected_placement(Matrix3d::Identity(), t_offset);
    BOOST_CHECK(rcm.desired_contact_placement.isApprox(expected_placement));

    const Motion expected_vel(v_desired, Vector3d::Zero());
    BOOST_CHECK(rcm.desired_contact_velocity.isApprox(expected_vel));

    const Motion expected_acc(a_desired, Vector3d::Zero());
    BOOST_CHECK(rcm.desired_contact_acceleration.isApprox(expected_acc));
  }
}

/// Verify that the desired fields of a FrameAnchorConstraintModel map correctly
/// to the desired_contact_placement, desired_contact_velocity, and
/// desired_contact_acceleration of the converted RigidConstraintModel.
///
/// Expected mapping (independently stated):
///   desired_contact_placement    = exp6(Motion(desired_constraint_offset))
///   desired_contact_velocity     = Motion(desired_constraint_velocity)
///   desired_contact_acceleration = Motion(desired_constraint_acceleration)
BOOST_AUTO_TEST_CASE(convert_frame_anchor_desired_fields)
{
  Model model;
  buildModels::humanoidRandom(model, true);
  const JointIndex rf_id = model.getJointId("rleg6_joint");

  // Zero desired fields -> identity placement and zero motions
  {
    const FrameAnchorConstraintModel cm(model, rf_id, SE3::Identity());
    const RigidConstraintModel rcm = convertToRigidConstraintModel(model, cm);

    BOOST_CHECK(rcm.desired_contact_placement.isIdentity());
    BOOST_CHECK(rcm.desired_contact_velocity.isZero());
    BOOST_CHECK(rcm.desired_contact_acceleration.isZero());
  }

  // Non-zero desired fields (small values to keep exp6 well-behaved)
  {
    typedef RigidConstraintModel::Motion Motion;

    Motion::Vector6 v6_offset, v6_vel, v6_acc;
    v6_offset << 0.1, 0.2, 0.3, 0.01, 0.02, 0.03;
    v6_vel << 1., 2., 3., 0.1, 0.2, 0.3;
    v6_acc << 4., 5., 6., 0.4, 0.5, 0.6;

    FrameAnchorConstraintModel cm(model, rf_id, SE3::Identity());
    cm.desired_constraint_offset = v6_offset;
    cm.desired_constraint_velocity = v6_vel;
    cm.desired_constraint_acceleration = v6_acc;

    const RigidConstraintModel rcm = convertToRigidConstraintModel(model, cm);

    // Build expected values independently
    const SE3 expected_placement = exp6(Motion(v6_offset));
    BOOST_CHECK(rcm.desired_contact_placement.isApprox(expected_placement));

    const Motion expected_vel(v6_vel);
    BOOST_CHECK(rcm.desired_contact_velocity.isApprox(expected_vel));

    const Motion expected_acc(v6_acc);
    BOOST_CHECK(rcm.desired_contact_acceleration.isApprox(expected_acc));
  }
}

/// At zero constraint position error the A-matrices of PointAnchorConstraintModel and
/// RigidConstraintModel(CONTACT_3D) differ only by a global sign, so their constraint
/// Jacobians satisfy:  J_rigid == -J_point
///
/// The sign difference reflects the opposite convention for "which frame is subtracted
/// from which": PointAnchorConstraintModel measures (c2 - c1) while
/// RigidConstraintModel measures (c1 - c2) in the 3D case.
BOOST_AUTO_TEST_CASE(convert_point_anchor_jacobian_at_zero_error)
{
  Model model;
  buildModels::humanoidRandom(model, true);
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);

  const JointIndex rf_id = model.getJointId("rleg6_joint");

  // Choose a configuration
  const VectorXd q = randomConfiguration(model);

  // Compute forward kinematics to obtain the world pose of the joint
  Data data(model);
  forwardKinematics(model, data, q);
  computeJointJacobians(model, data, q);

  // Set joint1_placement so that oMc1 == SE3::Identity() at configuration q,
  // making the constraint position error exactly zero (joint2 is the universe frame).
  const SE3 joint1_placement = data.oMi[rf_id].inverse();

  const PointAnchorConstraintModel cm_point(model, rf_id, joint1_placement);
  const RigidConstraintModel cm_rigid = convertToRigidConstraintModel(model, cm_point);

  // Compute constraint Jacobians for both
  PointAnchorConstraintData cd_point(cm_point);
  cm_point.calc(model, data, cd_point);
  // Verify the error is indeed zero at this configuration
  BOOST_CHECK(cd_point.constraint_position_error.isZero(1e-10));

  RigidConstraintData cd_rigid(cm_rigid);
  cm_rigid.calc(model, data, cd_rigid);

  MatrixXd J_point(3, model.nv);
  J_point.setZero();
  getConstraintJacobian(model, data, cm_point, cd_point, J_point);

  MatrixXd J_rigid(3, model.nv);
  J_rigid.setZero();
  getConstraintJacobian(model, data, cm_rigid, cd_rigid, J_rigid);

  BOOST_CHECK(J_rigid.isApprox(-J_point));
}

/// At zero constraint position error the Jacobians of FrameAnchorConstraintModel and
/// RigidConstraintModel(CONTACT_6D) satisfy:  J_rigid == -J_frame
BOOST_AUTO_TEST_CASE(convert_frame_anchor_jacobian_at_zero_error)
{
  Model model;
  buildModels::humanoidRandom(model, true);
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);

  const JointIndex rf_id = model.getJointId("rleg6_joint");

  const VectorXd q = randomConfiguration(model);

  Data data(model);
  forwardKinematics(model, data, q);
  computeJointJacobians(model, data, q);

  // Set joint1_placement so that oMc1 == SE3::Identity() at configuration q
  const SE3 joint1_placement = data.oMi[rf_id].inverse();

  const FrameAnchorConstraintModel cm_frame(model, rf_id, joint1_placement);
  const RigidConstraintModel cm_rigid = convertToRigidConstraintModel(model, cm_frame);

  FrameAnchorConstraintData cd_frame(cm_frame);
  cm_frame.calc(model, data, cd_frame);

  RigidConstraintData cd_rigid(cm_rigid);
  cm_rigid.calc(model, data, cd_rigid);

  MatrixXd J_frame(6, model.nv);
  J_frame.setZero();
  getConstraintJacobian(model, data, cm_frame, cd_frame, J_frame);

  MatrixXd J_rigid(6, model.nv);
  J_rigid.setZero();
  getConstraintJacobian(model, data, cm_rigid, cd_rigid, J_rigid);

  BOOST_CHECK(J_rigid.isApprox(-J_frame));
}
