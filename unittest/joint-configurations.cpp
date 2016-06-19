//
// Copyright (c) 2016 CNRS
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

/*
 * Compare the value obtained with the RNEA with the values obtained from
 * RBDL. The test is not complete. It only validates the RNEA for the revolute
 * joints. The free-flyer is not tested. It should be extended to account for
 * the free flyer and for the other algorithms.
 *
 * Additionnal notes: the RNEA is an algorithm that can be used to validate
 * many others (in particular, the mass matrix (CRBA) can be numerically
 * validated from the RNEA, then the center-of-mass jacobian can be validated
 * from the mass matrix, etc.
 *
 */

#include <iostream>
#include <iomanip>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/parser/sample-models.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/math/quaternion.hpp"

#include <vector>


bool configurations_are_equals(const Eigen::VectorXd & conf1, const Eigen::VectorXd & conf2)
{
  long size = conf1.size();
  if ( ! conf1.segment<3>(0).isApprox(conf2.segment<3>(0)) )
    return false;
  if( ! se3::defineSameRotation(Eigen::Quaterniond(conf1.segment<4>(3)), Eigen::Quaterniond(conf2.segment<4>(3))))
    return false;
  if( ! se3::defineSameRotation(Eigen::Quaterniond(conf1.segment<4>(7)), Eigen::Quaterniond(conf2.segment<4>(7))))
    return false;
  if ( ! conf1.segment(11, size-11).isApprox(conf2.segment(11, size-11)) )
    return false;
  return true;
}

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE JointConfigurationsTest
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>


BOOST_AUTO_TEST_SUITE ( JointConfigurationsTest )

BOOST_AUTO_TEST_CASE ( integration_test )
{
  se3::Model model;
  
  using namespace se3;

  model.addJointAndBody(model.getJointId("universe"),JointModelFreeFlyer(),SE3::Identity(),Inertia::Random(),
                "freeflyer_joint", "freeflyer_body");
  model.addJointAndBody(model.getJointId("freeflyer_joint"),JointModelSpherical(),SE3::Identity(),Inertia::Random(),
                "spherical_joint", "spherical_body");
  model.addJointAndBody(model.getJointId("spherical_joint"),JointModelRX(),SE3::Identity(),Inertia::Random(),
                "revolute_joint", "revolute_body");
  model.addJointAndBody(model.getJointId("revolute_joint"),JointModelPX(),SE3::Identity(),Inertia::Random(),
                "px_joint", "px_body");
  model.addJointAndBody(model.getJointId("px_joint"),JointModelPrismaticUnaligned(Eigen::Vector3d(1,0,0)),SE3::Identity(),Inertia::Random(),
                "pu_joint", "pu_body");
  model.addJointAndBody(model.getJointId("pu_joint"),JointModelRevoluteUnaligned(Eigen::Vector3d(0,0,1)),SE3::Identity(),Inertia::Random(),
                "ru_joint", "ru_body");
  model.addJointAndBody(model.getJointId("ru_joint"),JointModelSphericalZYX(),SE3::Identity(),Inertia::Random(),
                "sphericalZYX_joint", "sphericalZYX_body");
  model.addJointAndBody(model.getJointId("sphericalZYX_joint"),JointModelTranslation(),SE3::Identity(),Inertia::Random(),
                "translation_joint", "translation_body");
  model.addJointAndBody(model.getJointId("translation_joint"),JointModelPlanar(),SE3::Identity(),Inertia::Random(),
                "planar_joint", "planar_body");
  
  se3::Data data(model);

  Eigen::VectorXd q(Eigen::VectorXd::Ones(model.nq));
  q.segment<4>(3) /= q.segment<4>(3).norm(); // quaternion of freeflyer
  q.segment<4>(7) /= q.segment<4>(7).norm(); // quaternion of spherical joint
  Eigen::VectorXd q_dot(Eigen::VectorXd::Random(model.nv));
  Eigen::Quaterniond quat_ff(q[6],q[3],q[4],q[5]); Eigen::Vector3d omega_ff(q_dot[3],q_dot[4],q_dot[5]); Eigen::Vector3d transl_ff(q[0],q[1],q[2]);
  Eigen::Quaterniond quat_spherical(q[10],q[7],q[8],q[9]); Eigen::Vector3d omega_spherical(q_dot[6],q_dot[7],q_dot[8]);


  Eigen::VectorXd expected(model.nq);

  Eigen::Quaterniond v_ff(Eigen::AngleAxisd(omega_ff.norm(), omega_ff/omega_ff.norm()));
  Eigen::Quaterniond quat_ff__int( v_ff * quat_ff);
  Eigen::Quaterniond v_spherical(Eigen::AngleAxisd(omega_spherical.norm(), omega_spherical/omega_spherical.norm()));
  Eigen::Quaterniond quat_spherical_int( v_spherical * quat_spherical);

  expected.head<3>() = q.head<3>() + q_dot.head<3>();
  expected[3] = quat_ff__int.x();expected[4] = quat_ff__int.y(); expected[5] = quat_ff__int.z(); expected[6] = quat_ff__int.w(); 
  expected[7] = quat_spherical_int.x();expected[8] = quat_spherical_int.y(); expected[9] = quat_spherical_int.z(); expected[10] = quat_spherical_int.w(); 
  expected.tail<13>() = q.tail<13>() + q_dot.tail<13>();

  Eigen::VectorXd result(integrate(model,q,q_dot));

  BOOST_CHECK_MESSAGE(result.isApprox(expected, 1e-12), "integration of freeflyer joint - wrong results");
}

BOOST_AUTO_TEST_CASE ( interpolation_test )
{
  se3::Model model;
  
  using namespace se3;

  model.addJointAndBody(model.getJointId("universe"),JointModelFreeFlyer(),SE3::Identity(),Inertia::Random(),
                "freeflyer_joint", "freeflyer_body");
  model.addJointAndBody(model.getJointId("freeflyer_joint"),JointModelSpherical(),SE3::Identity(),Inertia::Random(),
                "spherical_joint", "spherical_body");
  model.addJointAndBody(model.getJointId("spherical_joint"),JointModelRX(),SE3::Identity(),Inertia::Random(),
                "revolute_joint", "revolute_body");
  model.addJointAndBody(model.getJointId("revolute_joint"),JointModelPX(),SE3::Identity(),Inertia::Random(),
                "px_joint", "px_body");
  model.addJointAndBody(model.getJointId("px_joint"),JointModelPrismaticUnaligned(Eigen::Vector3d(1,0,0)),SE3::Identity(),Inertia::Random(),
                "pu_joint", "pu_body");
  model.addJointAndBody(model.getJointId("pu_joint"),JointModelRevoluteUnaligned(Eigen::Vector3d(0,0,1)),SE3::Identity(),Inertia::Random(),
                "ru_joint", "ru_body");
  model.addJointAndBody(model.getJointId("ru_joint"),JointModelSphericalZYX(),SE3::Identity(),Inertia::Random(),
                "sphericalZYX_joint", "sphericalZYX_body");
  model.addJointAndBody(model.getJointId("sphericalZYX_joint"),JointModelTranslation(),SE3::Identity(),Inertia::Random(),
                "translation_joint", "translation_body");
  model.addJointAndBody(model.getJointId("translation_joint"),JointModelPlanar(),SE3::Identity(),Inertia::Random(),
                "planar_joint", "planar_body");
  
  se3::Data data(model);
  
  Eigen::VectorXd q1(Eigen::VectorXd::Zero(model.nq));
  Eigen::VectorXd q2(Eigen::VectorXd::Ones(model.nq));
  
  Eigen::Vector3d axis (Eigen::Vector3d::Ones()); axis.normalize();
  const double angle (0.5);
  Eigen::AngleAxis<double> aa1 (0., axis);
  Eigen::AngleAxis<double> aa2 (angle, axis);
  
  q1.segment<4>(3) = Eigen::Quaterniond(aa1).coeffs();
  q1.segment<4>(7) = Eigen::Quaterniond(aa1).coeffs();
  
  q2.segment<4>(3) = Eigen::Quaterniond(aa2).coeffs();
  q2.segment<4>(7) = Eigen::Quaterniond(aa2).coeffs();
  double u = 0.1;
  
  Eigen::Quaterniond quat_ff_1(q1[6],q1[3],q1[4],q1[5]);
  Eigen::Quaterniond quat_spherical_1(q1[10],q1[7],q1[8],q1[9]);
  Eigen::Quaterniond quat_ff_2(q2[6],q2[3],q2[4],q2[5]);
  Eigen::Quaterniond quat_spherical_2(q2[10],q2[7],q2[8],q2[9]);

  Eigen::VectorXd expected(model.nq);

  // u between 0 and 1
  Eigen::Quaterniond quat_ff__int = quat_ff_1.slerp(u, quat_ff_2);
  Eigen::Quaterniond quat_spherical_int = quat_spherical_1.slerp(u, quat_spherical_2);

  expected.head<3>() = (1-u) * q1.head<3>() + u*q2.head<3>();
  expected[3] = quat_ff__int.x();expected[4] = quat_ff__int.y(); expected[5] = quat_ff__int.z(); expected[6] = quat_ff__int.w(); 
  expected[7] = quat_spherical_int.x();expected[8] = quat_spherical_int.y(); expected[9] = quat_spherical_int.z(); expected[10] = quat_spherical_int.w(); 
  expected.tail<13>() = (1-u)* q1.tail<13>() + u*q2.tail<13>();

  Eigen::VectorXd result(interpolate(model,q1,q2,u));

  BOOST_CHECK_MESSAGE(result.isApprox(expected, 1e-12), "interpolation full model for u = 0.1 - wrong results");
  
  // u = 0 -> q_interpolate = q1
  u = 0;
  quat_ff__int = Eigen::Quaterniond((1-u) * quat_ff_1.w() + u*quat_ff_2.w(),
                                    (1-u) * quat_ff_1.x() + u*quat_ff_2.x(),
                                    (1-u) * quat_ff_1.y() + u*quat_ff_2.y(),
                                    (1-u) * quat_ff_1.z() + u*quat_ff_2.z()
                                    );
  quat_ff__int.normalize();
  quat_spherical_int = Eigen::Quaterniond((1-u) * quat_spherical_1.w() + u*quat_spherical_2.w(),
                                          (1-u) * quat_spherical_1.x() + u*quat_spherical_2.x(),
                                          (1-u) * quat_spherical_1.y() + u*quat_spherical_2.y(),
                                          (1-u) * quat_spherical_1.z() + u*quat_spherical_2.z()
                                          );
  quat_spherical_int.normalize();

  expected.head<3>() = (1-u) * q1.head<3>() + u*q2.head<3>();
  expected[3] = quat_ff__int.x();expected[4] = quat_ff__int.y(); expected[5] = quat_ff__int.z(); expected[6] = quat_ff__int.w(); 
  expected[7] = quat_spherical_int.x();expected[8] = quat_spherical_int.y(); expected[9] = quat_spherical_int.z(); expected[10] = quat_spherical_int.w(); 
  expected.tail<13>() = (1-u)* q1.tail<13>() + u*q2.tail<13>();

  result = interpolate(model,q1,q2,u);

  BOOST_CHECK_MESSAGE(result.isApprox(q1, 1e-12), "interpolation with u = 0 - wrong results");
  
  // u = 1 -> q_interpolate = q2
  u = 1;
  quat_ff__int = Eigen::Quaterniond((1-u) * quat_ff_1.w() + u*quat_ff_2.w(),
                                    (1-u) * quat_ff_1.x() + u*quat_ff_2.x(),
                                    (1-u) * quat_ff_1.y() + u*quat_ff_2.y(),
                                    (1-u) * quat_ff_1.z() + u*quat_ff_2.z()
                                    );
  quat_ff__int.normalize();
  quat_spherical_int = Eigen::Quaterniond((1-u) * quat_spherical_1.w() + u*quat_spherical_2.w(),
                                          (1-u) * quat_spherical_1.x() + u*quat_spherical_2.x(),
                                          (1-u) * quat_spherical_1.y() + u*quat_spherical_2.y(),
                                          (1-u) * quat_spherical_1.z() + u*quat_spherical_2.z()
                                          );
  quat_spherical_int.normalize();

  expected.head<3>() = (1-u) * q1.head<3>() + u*q2.head<3>();
  expected[3] = quat_ff__int.x();expected[4] = quat_ff__int.y(); expected[5] = quat_ff__int.z(); expected[6] = quat_ff__int.w(); 
  expected[7] = quat_spherical_int.x();expected[8] = quat_spherical_int.y(); expected[9] = quat_spherical_int.z(); expected[10] = quat_spherical_int.w(); 
  expected.tail<13>() = (1-u)* q1.tail<13>() + u*q2.tail<13>();

  result = interpolate(model,q1,q2,u);

  BOOST_CHECK_MESSAGE(configurations_are_equals(result, q2), "interpolation with u = 1 - wrong results");
}

BOOST_AUTO_TEST_CASE ( differentiation_test )
{
  se3::Model model;
  
  using namespace se3;

  model.addJointAndBody(model.getJointId("universe"),JointModelFreeFlyer(),SE3::Identity(),Inertia::Random(),
                "freeflyer_joint", "freeflyer_body");
  model.addJointAndBody(model.getJointId("freeflyer_joint"),JointModelSpherical(),SE3::Identity(),Inertia::Random(),
                "spherical_joint", "spherical_body");
  model.addJointAndBody(model.getJointId("spherical_joint"),JointModelRX(),SE3::Identity(),Inertia::Random(),
                "revolute_joint", "revolute_body");
  model.addJointAndBody(model.getJointId("revolute_joint"),JointModelPX(),SE3::Identity(),Inertia::Random(),
                "px_joint", "px_body");
  model.addJointAndBody(model.getJointId("px_joint"),JointModelPrismaticUnaligned(Eigen::Vector3d(1,0,0)),SE3::Identity(),Inertia::Random(),
                "pu_joint", "pu_body");
  model.addJointAndBody(model.getJointId("pu_joint"),JointModelRevoluteUnaligned(Eigen::Vector3d(0,0,1)),SE3::Identity(),Inertia::Random(),
                "ru_joint", "ru_body");
  model.addJointAndBody(model.getJointId("ru_joint"),JointModelSphericalZYX(),SE3::Identity(),Inertia::Random(),
                "sphericalZYX_joint", "sphericalZYX_body");
  model.addJointAndBody(model.getJointId("sphericalZYX_joint"),JointModelTranslation(),SE3::Identity(),Inertia::Random(),
                "translation_joint", "translation_body");
  model.addJointAndBody(model.getJointId("translation_joint"),JointModelPlanar(),SE3::Identity(),Inertia::Random(),
                "planar_joint", "planar_body");
  
  se3::Data data(model);

  Eigen::VectorXd q1(Eigen::VectorXd::Zero(model.nq));
  Eigen::VectorXd q2(Eigen::VectorXd::Ones(model.nq));
  
  Eigen::Vector3d axis (Eigen::Vector3d::Ones()); axis.normalize();
  const double angle (0.5);
  Eigen::AngleAxis<double> aa1 (0., axis);
  Eigen::AngleAxis<double> aa2 (angle, axis);
  
  q1.segment<4>(3) = Eigen::Quaterniond(aa1).coeffs();
  q1.segment<4>(7) = Eigen::Quaterniond(aa1).coeffs();
  
  q2.segment<4>(3) = Eigen::Quaterniond(aa2).coeffs();
  q2.segment<4>(7) = Eigen::Quaterniond(aa2).coeffs();
  
  Eigen::Quaterniond quat_ff_1(q1[6],q1[3],q1[4],q1[5]);
  Eigen::Quaterniond quat_spherical_1(q1[10],q1[7],q1[8],q1[9]);
  Eigen::Quaterniond quat_ff_2(q2[6],q2[3],q2[4],q2[5]);
  Eigen::Quaterniond quat_spherical_2(q2[10],q2[7],q2[8],q2[9]);

  Eigen::VectorXd expected(model.nv);

  // Quaternion freeflyer
  // Compute rotation vector between q2 and q1.
  Motion::Quaternion_t p_ff_1 (quat_ff_1);
  Motion::Quaternion_t p_ff_2 (quat_ff_2);

  Motion::Quaternion_t p_ff (p_ff_2*p_ff_1.conjugate());
  Eigen::AngleAxis<double> angle_axis_ff(p_ff);

  Eigen::Vector3d quat_ff__diff( angle_axis_ff.angle() * angle_axis_ff.axis());

  // Quaternion spherical
  Motion::Quaternion_t p_spherical_1 (quat_spherical_1);
  Motion::Quaternion_t p_spherical_2 (quat_spherical_2);

  Motion::Quaternion_t p_spherical (p_spherical_2*p_spherical_1.conjugate());
  Eigen::AngleAxis<double> angle_axis_spherical(p_spherical);

  Eigen::Vector3d quat_spherical_diff( angle_axis_spherical.angle() * angle_axis_spherical.axis());

  expected.head<3>() = q2.head<3>() - q1.head<3>();
  expected[3] = quat_ff__diff[0];expected[4] = quat_ff__diff[1]; expected[5] = quat_ff__diff[2]; 
  expected[6] = quat_spherical_diff[0];expected[7] = quat_spherical_diff[1]; expected[8] = quat_spherical_diff[2];
  expected.tail<13>() = q2.tail<13>() - q1.tail<13>();

  Eigen::VectorXd result(differentiate(model,q1,q2));
  
  BOOST_CHECK_MESSAGE(result.isApprox(expected, 1e-12), "Differentiation of full model - wrong results");
}

BOOST_AUTO_TEST_CASE ( distance_computation_test )
{
  se3::Model model;
  
  using namespace se3;

  model.addJointAndBody(model.getJointId("universe"),JointModelFreeFlyer(),SE3::Identity(),Inertia::Random(),
                "freeflyer_joint", "freeflyer_body");
  model.addJointAndBody(model.getJointId("freeflyer_joint"),JointModelSpherical(),SE3::Identity(),Inertia::Random(),
                "spherical_joint", "spherical_body");
  model.addJointAndBody(model.getJointId("spherical_joint"),JointModelRX(),SE3::Identity(),Inertia::Random(),
                "revolute_joint", "revolute_body");
  model.addJointAndBody(model.getJointId("revolute_joint"),JointModelPX(),SE3::Identity(),Inertia::Random(),
                "px_joint", "px_body");
  model.addJointAndBody(model.getJointId("px_joint"),JointModelPrismaticUnaligned(Eigen::Vector3d(1,0,0)),SE3::Identity(),Inertia::Random(),
                "pu_joint", "pu_body");
  model.addJointAndBody(model.getJointId("pu_joint"),JointModelRevoluteUnaligned(Eigen::Vector3d(0,0,1)),SE3::Identity(),Inertia::Random(),
                "ru_joint", "ru_body");
  model.addJointAndBody(model.getJointId("ru_joint"),JointModelSphericalZYX(),SE3::Identity(),Inertia::Random(),
                "sphericalZYX_joint", "sphericalZYX_body");
  model.addJointAndBody(model.getJointId("sphericalZYX_joint"),JointModelTranslation(),SE3::Identity(),Inertia::Random(),
                "translation_joint", "translation_body");
  model.addJointAndBody(model.getJointId("translation_joint"),JointModelPlanar(),SE3::Identity(),Inertia::Random(),
                "planar_joint", "planar_body");
  
  se3::Data data(model);
  
  Eigen::VectorXd q1(Eigen::VectorXd::Zero(model.nq));
  Eigen::VectorXd q2(Eigen::VectorXd::Ones(model.nq));
  
  Eigen::Vector3d axis (Eigen::Vector3d::Ones()); axis.normalize();
  const double angle (0.5);
  Eigen::AngleAxis<double> aa1 (0., axis);
  Eigen::AngleAxis<double> aa2 (angle, axis);
  
  q1.segment<4>(3) = Eigen::Quaterniond(aa1).coeffs();
  q1.segment<4>(7) = Eigen::Quaterniond(aa1).coeffs();
  
  q2.segment<4>(3) = Eigen::Quaterniond(aa2).coeffs();
  q2.segment<4>(7) = Eigen::Quaterniond(aa2).coeffs();
  
  Eigen::Quaterniond quat_ff_1(q1[6],q1[3],q1[4],q1[5]);
  Eigen::Quaterniond quat_spherical_1(q1[10],q1[7],q1[8],q1[9]);
  Eigen::Quaterniond quat_ff_2(q2[6],q2[3],q2[4],q2[5]);
  Eigen::Quaterniond quat_spherical_2(q2[10],q2[7],q2[8],q2[9]);

  Eigen::VectorXd expected(model.nbody-1);

  // Quaternion freeflyer
  // Compute rotation vector between q2 and q1.
  Motion::Quaternion_t p_ff_1 (quat_ff_1);
  Motion::Quaternion_t p_ff_2 (quat_ff_2);

  double angle_quat_ff = 2*angleBetweenQuaternions(p_ff_1, p_ff_2);
  double dist_ff = sqrt(pow((q2.head<3>() - q1.head<3>()).norm(),2) + pow(angle_quat_ff,2) );
  // Other ways to compute quaternion angle :
  // 1/
  // double angle_acos = 2 * acos(p_ff.w());
  // 2/
  // double angatan = 2*atan2(p_ff.vec().norm(), p_ff.w());

  // Quaternion spherical
  Motion::Quaternion_t p_spherical_1 (quat_spherical_1);
  Motion::Quaternion_t p_spherical_2 (quat_spherical_2);

  double dist_quat_spherical = 2*angleBetweenQuaternions(p_spherical_1, p_spherical_2);

  expected << dist_ff,
              dist_quat_spherical,
              q2[11] - q1[11],
              q2[12] - q1[12],
              q2[13] - q1[13],
              q2[14] - q1[14],
              (q2.segment<3>(15) - q1.segment<3>(15)).norm(),
              (q2.segment<3>(18) - q1.segment<3>(18)).norm(),
              (q2.segment<3>(21) - q1.segment<3>(21)).norm();

  Eigen::VectorXd result(distance(model,q1,q2));

  BOOST_CHECK_MESSAGE(result.isApprox(expected, 1e-12), "Distance between two configs of full model - wrong results");
}


BOOST_AUTO_TEST_CASE ( uniform_sampling_test )
{
  se3::Model model;
  
  using namespace se3;

  model.addJointAndBody(model.getJointId("universe"),JointModelFreeFlyer(),SE3::Identity(),Inertia::Random(),
                Eigen::VectorXd::Zero(6), 1e3 * (Eigen::VectorXd::Random(6).array() + 1),
                1e3 * (Eigen::VectorXd::Random(7).array() - 1),
                1e3 * (Eigen::VectorXd::Random(7).array() + 1),
                "freeflyer_joint", "freeflyer_body");
  model.addJointAndBody(model.getJointId("freeflyer_joint"),JointModelSpherical(),SE3::Identity(),Inertia::Random(),
                Eigen::VectorXd::Zero(3), 1e3 * (Eigen::VectorXd::Random(3).array() + 1),
                1e3 * (Eigen::VectorXd::Random(4).array() - 1),
                1e3 * (Eigen::VectorXd::Random(4).array() + 1),
                "spherical_joint", "spherical_body");
  model.addJointAndBody(model.getJointId("spherical_joint"),JointModelRX(),SE3::Identity(),Inertia::Random(),
                Eigen::VectorXd::Random(1).array() + 1, Eigen::VectorXd::Random(1).array() + 1,
                Eigen::VectorXd::Random(1).array() - 1, Eigen::VectorXd::Random(1).array() + 1,
                "revolute_joint", "revolute_body");
  model.addJointAndBody(model.getJointId("revolute_joint"),JointModelPX(),SE3::Identity(),Inertia::Random(),
                Eigen::VectorXd::Random(1).array() + 1, Eigen::VectorXd::Random(1).array() + 1,
                Eigen::VectorXd::Random(1).array() - 1, Eigen::VectorXd::Random(1).array() + 1,
                "px_joint", "px_body");
  model.addJointAndBody(model.getJointId("px_joint"),JointModelPrismaticUnaligned(Eigen::Vector3d(1,0,0)),SE3::Identity(),Inertia::Random(),
                Eigen::VectorXd::Random(1).array() + 1, Eigen::VectorXd::Random(1).array() + 1,
                Eigen::VectorXd::Random(1).array() - 1, Eigen::VectorXd::Random(1).array() + 1,
                "pu_joint", "pu_body");
  model.addJointAndBody(model.getJointId("pu_joint"),JointModelRevoluteUnaligned(Eigen::Vector3d(0,0,1)),SE3::Identity(),Inertia::Random(),
                Eigen::VectorXd::Random(1).array() + 1, Eigen::VectorXd::Random(1).array() + 1,
                Eigen::VectorXd::Random(1).array() - 1, Eigen::VectorXd::Random(1).array() + 1,
                "ru_joint", "ru_body");
  model.addJointAndBody(model.getJointId("ru_joint"),JointModelSphericalZYX(),SE3::Identity(),Inertia::Random(),
                Eigen::VectorXd::Random(3).array() + 1, Eigen::VectorXd::Random(3).array() + 1,
                Eigen::VectorXd::Random(3).array() - 1, Eigen::VectorXd::Random(3).array() + 1,
                "sphericalZYX_joint", "sphericalZYX_body");
  model.addJointAndBody(model.getJointId("sphericalZYX_joint"),JointModelTranslation(),SE3::Identity(),Inertia::Random(),
                Eigen::VectorXd::Random(3).array() + 1, Eigen::VectorXd::Random(3).array() + 1,
                Eigen::VectorXd::Random(3).array() - 1, Eigen::VectorXd::Random(3).array() + 1,
                "translation_joint", "translation_body");
  model.addJointAndBody(model.getJointId("translation_joint"),JointModelPlanar(),SE3::Identity(),Inertia::Random(),
                Eigen::VectorXd::Random(3).array() + 1, Eigen::VectorXd::Random(3).array() + 1,
                Eigen::VectorXd::Random(3).array() - 1, Eigen::VectorXd::Random(3).array() + 1,
                "planar_joint", "planar_body");
  

  se3::Data data(model);

  Eigen::VectorXd q1(randomConfiguration(model));
  
  for (int i = 0; i < q1.size(); ++i)
  {
    BOOST_CHECK_MESSAGE(q1[i] >= model.lowerPositionLimit[i] && q1[i] <= model.upperPositionLimit[i], " UniformlySample : Generated config not in bounds");
  }

}

BOOST_AUTO_TEST_CASE ( integrate_difference_test )
{
  se3::Model model;
  
  using namespace se3;

  model.addJointAndBody(model.getJointId("universe"),JointModelFreeFlyer(),SE3::Identity(),Inertia::Random(),
                Eigen::VectorXd::Zero(6), 1e3 * (Eigen::VectorXd::Random(6).array() + 1),
                1e3 * (Eigen::VectorXd::Random(7).array() - 1),
                1e3 * (Eigen::VectorXd::Random(7).array() + 1),
                "freeflyer_joint", "freeflyer_body");
  model.addJointAndBody(model.getJointId("freeflyer_joint"),JointModelSpherical(),SE3::Identity(),Inertia::Random(),
                Eigen::VectorXd::Zero(3), 1e3 * (Eigen::VectorXd::Random(3).array() + 1),
                1e3 * (Eigen::VectorXd::Random(4).array() - 1),
                1e3 * (Eigen::VectorXd::Random(4).array() + 1),
                "spherical_joint", "spherical_body");
  model.addJointAndBody(model.getJointId("spherical_joint"),JointModelRX(),SE3::Identity(),Inertia::Random(),
                Eigen::VectorXd::Random(1).array() + 1, Eigen::VectorXd::Random(1).array() + 1,
                Eigen::VectorXd::Random(1).array() - 1, Eigen::VectorXd::Random(1).array() + 1,
                "revolute_joint", "revolute_body");
  model.addJointAndBody(model.getJointId("revolute_joint"),JointModelPX(),SE3::Identity(),Inertia::Random(),
                Eigen::VectorXd::Random(1).array() + 1, Eigen::VectorXd::Random(1).array() + 1,
                Eigen::VectorXd::Random(1).array() - 1, Eigen::VectorXd::Random(1).array() + 1,
                "px_joint", "px_body");
  model.addJointAndBody(model.getJointId("px_joint"),JointModelPrismaticUnaligned(Eigen::Vector3d(1,0,0)),SE3::Identity(),Inertia::Random(),
                Eigen::VectorXd::Random(1).array() + 1, Eigen::VectorXd::Random(1).array() + 1,
                Eigen::VectorXd::Random(1).array() - 1, Eigen::VectorXd::Random(1).array() + 1,
                "pu_joint", "pu_body");
  model.addJointAndBody(model.getJointId("pu_joint"),JointModelRevoluteUnaligned(Eigen::Vector3d(0,0,1)),SE3::Identity(),Inertia::Random(),
                Eigen::VectorXd::Random(1).array() + 1, Eigen::VectorXd::Random(1).array() + 1,
                Eigen::VectorXd::Random(1).array() - 1, Eigen::VectorXd::Random(1).array() + 1,
                "ru_joint", "ru_body");
  model.addJointAndBody(model.getJointId("ru_joint"),JointModelSphericalZYX(),SE3::Identity(),Inertia::Random(),
                Eigen::VectorXd::Random(3).array() + 1, Eigen::VectorXd::Random(3).array() + 1,
                Eigen::VectorXd::Random(3).array() - 1, Eigen::VectorXd::Random(3).array() + 1,
                "sphericalZYX_joint", "sphericalZYX_body");
  model.addJointAndBody(model.getJointId("sphericalZYX_joint"),JointModelTranslation(),SE3::Identity(),Inertia::Random(),
                Eigen::VectorXd::Random(3).array() + 1, Eigen::VectorXd::Random(3).array() + 1,
                Eigen::VectorXd::Random(3).array() - 1, Eigen::VectorXd::Random(3).array() + 1,
                "translation_joint", "translation_body");
  model.addJointAndBody(model.getJointId("translation_joint"),JointModelPlanar(),SE3::Identity(),Inertia::Random(),
                Eigen::VectorXd::Random(3).array() + 1, Eigen::VectorXd::Random(3).array() + 1,
                Eigen::VectorXd::Random(3).array() - 1, Eigen::VectorXd::Random(3).array() + 1,
                "planar_joint", "planar_body");
  

  se3::Data data(model);

  Eigen::VectorXd q1(randomConfiguration(model, -1 * Eigen::VectorXd::Ones(model.nq), Eigen::VectorXd::Ones(model.nq) ));
  Eigen::VectorXd q2(randomConfiguration(model, -1 * Eigen::VectorXd::Ones(model.nq), Eigen::VectorXd::Ones(model.nq) ));

  BOOST_CHECK_MESSAGE(configurations_are_equals(integrate(model, q1, differentiate(model, q1,q2)), q2), "relation between integrate and differentiate");

}

BOOST_AUTO_TEST_SUITE_END ()
