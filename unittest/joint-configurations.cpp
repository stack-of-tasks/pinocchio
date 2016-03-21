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
#include "pinocchio/multibody/parser/urdf.hpp"
#include "pinocchio/spatial/explog.hpp"

#include <vector>


#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE JointConfigurationsTest
#include <boost/test/unit_test.hpp>


BOOST_AUTO_TEST_SUITE ( JointConfigurationsTest )

BOOST_AUTO_TEST_CASE ( integration_test )
{
  se3::Model model;
  
  using namespace se3;

  model.addBody(model.getBodyId("universe"),JointModelFreeFlyer(),SE3::Identity(),Inertia::Random(),
                "freeflyer_joint", "freeflyer_body");
  model.addBody(model.getBodyId("freeflyer_body"),JointModelSpherical(),SE3::Identity(),Inertia::Random(),
                "spherical_joint", "spherical_body");
  model.addBody(model.getBodyId("spherical_body"),JointModelRX(),SE3::Identity(),Inertia::Random(),
                "revolute_joint", "revolute_body");
  model.addBody(model.getBodyId("revolute_body"),JointModelPX(),SE3::Identity(),Inertia::Random(),
                "px_joint", "px_body");
  model.addBody(model.getBodyId("px_body"),JointModelPrismaticUnaligned(Eigen::Vector3d(1,0,0)),SE3::Identity(),Inertia::Random(),
                "pu_joint", "pu_body");
  model.addBody(model.getBodyId("pu_body"),JointModelRevoluteUnaligned(Eigen::Vector3d(0,0,1)),SE3::Identity(),Inertia::Random(),
                "ru_joint", "ru_body");
  model.addBody(model.getBodyId("ru_body"),JointModelSphericalZYX(),SE3::Identity(),Inertia::Random(),
                "sphericalZYX_joint", "sphericalZYX_body");
  model.addBody(model.getBodyId("sphericalZYX_body"),JointModelTranslation(),SE3::Identity(),Inertia::Random(),
                "translation_joint", "translation_body");
  model.addBody(model.getBodyId("translation_body"),JointModelPlanar(),SE3::Identity(),Inertia::Random(),
                "planar_joint", "planar_body");
  
  se3::Data data(model);

  Eigen::VectorXd q(Eigen::VectorXd::Random(model.nq));
  q.segment<4>(3) /= q.segment<4>(3).norm(); // quaternion of freeflyer
  q.segment<4>(7) /= q.segment<4>(7).norm(); // quaternion of spherical joint
  Eigen::VectorXd q_dot(Eigen::VectorXd::Random(model.nv));
  Eigen::Quaterniond quat_ff(q[6],q[3],q[4],q[5]); Eigen::Vector3d omega_ff(q_dot[3],q_dot[4],q_dot[5]); Eigen::Vector3d transl_ff(q[0],q[1],q[2]);
  Eigen::Quaterniond quat_spherical(q[10],q[7],q[8],q[9]); Eigen::Vector3d omega_spherical(q_dot[6],q_dot[7],q_dot[8]);

  Eigen::VectorXd result(model.nq);


  Eigen::VectorXd expected(model.nq);

  Eigen::Quaterniond v_ff(Eigen::AngleAxisd(omega_ff.norm(), omega_ff/omega_ff.norm()));
  Eigen::Quaterniond quat_ff__int( v_ff * quat_ff);
  Eigen::Quaterniond v_spherical(Eigen::AngleAxisd(omega_spherical.norm(), omega_spherical/omega_spherical.norm()));
  Eigen::Quaterniond quat_spherical_int( v_spherical * quat_spherical);

  expected.head<3>() = q.head<3>() + q_dot.head<3>();
  expected[3] = quat_ff__int.x();expected[4] = quat_ff__int.y(); expected[5] = quat_ff__int.z(); expected[6] = quat_ff__int.w(); 
  expected[7] = quat_spherical_int.x();expected[8] = quat_spherical_int.y(); expected[9] = quat_spherical_int.z(); expected[10] = quat_spherical_int.w(); 
  expected.tail<13>() = q.tail<13>() + q_dot.tail<13>();

  integrate(model, data,q,q_dot,result);

  assert(result.isApprox(expected) && "integration of freeflyer joint - wrong results");
}

BOOST_AUTO_TEST_CASE ( interpolation_test )
{
  se3::Model model;
  
  using namespace se3;

  model.addBody(model.getBodyId("universe"),JointModelFreeFlyer(),SE3::Identity(),Inertia::Random(),
                "freeflyer_joint", "freeflyer_body");
  model.addBody(model.getBodyId("freeflyer_body"),JointModelSpherical(),SE3::Identity(),Inertia::Random(),
                "spherical_joint", "spherical_body");
  model.addBody(model.getBodyId("spherical_body"),JointModelRX(),SE3::Identity(),Inertia::Random(),
                "revolute_joint", "revolute_body");
  model.addBody(model.getBodyId("revolute_body"),JointModelPX(),SE3::Identity(),Inertia::Random(),
                "px_joint", "px_body");
  model.addBody(model.getBodyId("px_body"),JointModelPrismaticUnaligned(Eigen::Vector3d(1,0,0)),SE3::Identity(),Inertia::Random(),
                "pu_joint", "pu_body");
  model.addBody(model.getBodyId("pu_body"),JointModelRevoluteUnaligned(Eigen::Vector3d(0,0,1)),SE3::Identity(),Inertia::Random(),
                "ru_joint", "ru_body");
  model.addBody(model.getBodyId("ru_body"),JointModelSphericalZYX(),SE3::Identity(),Inertia::Random(),
                "sphericalZYX_joint", "sphericalZYX_body");
  model.addBody(model.getBodyId("sphericalZYX_body"),JointModelTranslation(),SE3::Identity(),Inertia::Random(),
                "translation_joint", "translation_body");
  model.addBody(model.getBodyId("translation_body"),JointModelPlanar(),SE3::Identity(),Inertia::Random(),
                "planar_joint", "planar_body");
  
  se3::Data data(model);

  Eigen::VectorXd q1(Eigen::VectorXd::Random(model.nq));
  Eigen::VectorXd q2(Eigen::VectorXd::Random(model.nq));
  double u = 0.1;
  q1.segment<4>(3) /= q1.segment<4>(3).norm(); q2.segment<4>(3) /= q2.segment<4>(3).norm();// normalize quaternion of freeflyer
  q1.segment<4>(7) /= q1.segment<4>(7).norm(); q2.segment<4>(7) /= q2.segment<4>(7).norm();// normalize quaternion of spherical joint
  Eigen::Quaterniond quat_ff_1(q1[6],q1[3],q1[4],q1[5]);
  Eigen::Quaterniond quat_spherical_1(q1[10],q1[7],q1[8],q1[9]);
  Eigen::Quaterniond quat_ff_2(q2[6],q2[3],q2[4],q2[5]);
  Eigen::Quaterniond quat_spherical_2(q2[10],q2[7],q2[8],q2[9]);

  Eigen::VectorXd result(model.nq);
  Eigen::VectorXd expected(model.nq);


  // u between 0 and 1
  Eigen::Quaterniond quat_ff__int = quat_ff_1.slerp(u, quat_ff_2);
  Eigen::Quaterniond quat_spherical_int = quat_spherical_1.slerp(u, quat_spherical_2);

  expected.head<3>() = (1-u) * q1.head<3>() + u*q2.head<3>();
  expected[3] = quat_ff__int.x();expected[4] = quat_ff__int.y(); expected[5] = quat_ff__int.z(); expected[6] = quat_ff__int.w(); 
  expected[7] = quat_spherical_int.x();expected[8] = quat_spherical_int.y(); expected[9] = quat_spherical_int.z(); expected[10] = quat_spherical_int.w(); 
  expected.tail<13>() = (1-u)* q1.tail<13>() + u*q2.tail<13>();

  interpolate(model, data,q1,q2,u,result);

  assert(result.isApprox(expected) && "interpolation full model for u = 0.1 - wrong results");
  
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

  interpolate(model, data,q1,q2,u,result);

  assert(result.isApprox(q1) && "interpolation with u = 0 - wrong results");
  
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

  interpolate(model, data,q1,q2,u,result);

  assert(result.isApprox(q2) && "interpolation with u = 1 - wrong results");
}

BOOST_AUTO_TEST_CASE ( differentiation_test )
{
  se3::Model model;
  
  using namespace se3;

  model.addBody(model.getBodyId("universe"),JointModelFreeFlyer(),SE3::Identity(),Inertia::Random(),
                "freeflyer_joint", "freeflyer_body");
  model.addBody(model.getBodyId("freeflyer_body"),JointModelSpherical(),SE3::Identity(),Inertia::Random(),
                "spherical_joint", "spherical_body");
  model.addBody(model.getBodyId("spherical_body"),JointModelRX(),SE3::Identity(),Inertia::Random(),
                "revolute_joint", "revolute_body");
  model.addBody(model.getBodyId("revolute_body"),JointModelPX(),SE3::Identity(),Inertia::Random(),
                "px_joint", "px_body");
  model.addBody(model.getBodyId("px_body"),JointModelPrismaticUnaligned(Eigen::Vector3d(1,0,0)),SE3::Identity(),Inertia::Random(),
                "pu_joint", "pu_body");
  model.addBody(model.getBodyId("pu_body"),JointModelRevoluteUnaligned(Eigen::Vector3d(0,0,1)),SE3::Identity(),Inertia::Random(),
                "ru_joint", "ru_body");
  model.addBody(model.getBodyId("ru_body"),JointModelSphericalZYX(),SE3::Identity(),Inertia::Random(),
                "sphericalZYX_joint", "sphericalZYX_body");
  model.addBody(model.getBodyId("sphericalZYX_body"),JointModelTranslation(),SE3::Identity(),Inertia::Random(),
                "translation_joint", "translation_body");
  model.addBody(model.getBodyId("translation_body"),JointModelPlanar(),SE3::Identity(),Inertia::Random(),
                "planar_joint", "planar_body");
  
  se3::Data data(model);

  Eigen::VectorXd q1(Eigen::VectorXd::Random(model.nq));
  Eigen::VectorXd q2(Eigen::VectorXd::Random(model.nq));
  q1.segment<4>(3) /= q1.segment<4>(3).norm(); q2.segment<4>(3) /= q2.segment<4>(3).norm();// normalize quaternion of freeflyer
  q1.segment<4>(7) /= q1.segment<4>(7).norm(); q2.segment<4>(7) /= q2.segment<4>(7).norm();// normalize quaternion of spherical joint
  Eigen::Quaterniond quat_ff_1(q1[6],q1[3],q1[4],q1[5]);
  Eigen::Quaterniond quat_spherical_1(q1[10],q1[7],q1[8],q1[9]);
  Eigen::Quaterniond quat_ff_2(q2[6],q2[3],q2[4],q2[5]);
  Eigen::Quaterniond quat_spherical_2(q2[10],q2[7],q2[8],q2[9]);

  Eigen::VectorXd result(model.nv);
  Eigen::VectorXd expected(model.nv);

  // Quaternion freeflyer
  // Compute rotation vector between q2 and q1.
  Motion::Quaternion_t p_ff_1 (quat_ff_1);
  Motion::Quaternion_t p_ff_2 (quat_ff_2);

  Motion::Quaternion_t p_ff (p_ff_1*p_ff_2.conjugate());
  Eigen::AngleAxis<double> angle_axis_ff(p_ff);

  Eigen::Vector3d quat_ff__diff( angle_axis_ff.angle() * angle_axis_ff.axis());

  // Quaternion spherical
  Motion::Quaternion_t p_spherical_1 (quat_spherical_1);
  Motion::Quaternion_t p_spherical_2 (quat_spherical_2);

  Motion::Quaternion_t p_spherical (p_spherical_1*p_spherical_2.conjugate());
  Eigen::AngleAxis<double> angle_axis_spherical(p_spherical);

  Eigen::Vector3d quat_spherical_diff( angle_axis_spherical.angle() * angle_axis_spherical.axis());

  expected.head<3>() = q1.head<3>() - q2.head<3>();
  expected[3] = quat_ff__diff[0];expected[4] = quat_ff__diff[1]; expected[5] = quat_ff__diff[2]; 
  expected[6] = quat_spherical_diff[0];expected[7] = quat_spherical_diff[1]; expected[8] = quat_spherical_diff[2];
  expected.tail<13>() = q1.tail<13>() - q2.tail<13>();

  differentiate(model, data,q1,q2,result);

  assert(result.isApprox(expected) && "Differentiation of full model - wrong results");
}

BOOST_AUTO_TEST_CASE ( distance_computation_test )
{
  se3::Model model;
  
  using namespace se3;

  model.addBody(model.getBodyId("universe"),JointModelFreeFlyer(),SE3::Identity(),Inertia::Random(),
                "freeflyer_joint", "freeflyer_body");
  model.addBody(model.getBodyId("freeflyer_body"),JointModelSpherical(),SE3::Identity(),Inertia::Random(),
                "spherical_joint", "spherical_body");
  model.addBody(model.getBodyId("spherical_body"),JointModelRX(),SE3::Identity(),Inertia::Random(),
                "revolute_joint", "revolute_body");
  model.addBody(model.getBodyId("revolute_body"),JointModelPX(),SE3::Identity(),Inertia::Random(),
                "px_joint", "px_body");
  model.addBody(model.getBodyId("px_body"),JointModelPrismaticUnaligned(Eigen::Vector3d(1,0,0)),SE3::Identity(),Inertia::Random(),
                "pu_joint", "pu_body");
  model.addBody(model.getBodyId("pu_body"),JointModelRevoluteUnaligned(Eigen::Vector3d(0,0,1)),SE3::Identity(),Inertia::Random(),
                "ru_joint", "ru_body");
  model.addBody(model.getBodyId("ru_body"),JointModelSphericalZYX(),SE3::Identity(),Inertia::Random(),
                "sphericalZYX_joint", "sphericalZYX_body");
  model.addBody(model.getBodyId("sphericalZYX_body"),JointModelTranslation(),SE3::Identity(),Inertia::Random(),
                "translation_joint", "translation_body");
  model.addBody(model.getBodyId("translation_body"),JointModelPlanar(),SE3::Identity(),Inertia::Random(),
                "planar_joint", "planar_body");
  
  se3::Data data(model);

  Eigen::VectorXd q1(Eigen::VectorXd::Random(model.nq));
  Eigen::VectorXd q2(Eigen::VectorXd::Random(model.nq));
  q1.segment<4>(3) /= q1.segment<4>(3).norm(); q2.segment<4>(3) /= q2.segment<4>(3).norm();// normalize quaternion of freeflyer
  q1.segment<4>(7) /= q1.segment<4>(7).norm(); q2.segment<4>(7) /= q2.segment<4>(7).norm();// normalize quaternion of spherical joint
  Eigen::Quaterniond quat_ff_1(q1[6],q1[3],q1[4],q1[5]);
  Eigen::Quaterniond quat_spherical_1(q1[10],q1[7],q1[8],q1[9]);
  Eigen::Quaterniond quat_ff_2(q2[6],q2[3],q2[4],q2[5]);
  Eigen::Quaterniond quat_spherical_2(q2[10],q2[7],q2[8],q2[9]);

  Eigen::VectorXd result(model.nbody-1);
  Eigen::VectorXd expected(model.nbody-1);

  // Quaternion freeflyer
  // Compute rotation vector between q2 and q1.
  Motion::Quaternion_t p_ff_1 (quat_ff_1);
  Motion::Quaternion_t p_ff_2 (quat_ff_2);

  Motion::Quaternion_t p_ff (p_ff_1*p_ff_2.conjugate());
  Eigen::AngleAxis<double> angle_axis_ff(p_ff);
  double angle_quat_ff = angle_axis_ff.angle();
  double dist_ff = sqrt(pow((q1.head<3>() - q2.head<3>()).norm(),2) + pow(angle_quat_ff,2) );
  // Other ways to compute quaternion angle :
  // 1/
  // double angle_acos = 2 * acos(p_ff.w());
  // 2/
  // double angatan = 2*atan2(p_ff.vec().norm(), p_ff.w());

  // Quaternion spherical
  Motion::Quaternion_t p_spherical_1 (quat_spherical_1);
  Motion::Quaternion_t p_spherical_2 (quat_spherical_2);

  Motion::Quaternion_t p_spherical (p_spherical_1*p_spherical_2.conjugate());
  double dist_quat_spherical = 2*atan2(p_spherical.vec().norm(), p_spherical.w());;

  expected << dist_ff,
              dist_quat_spherical,
              q1[11] - q2[11],
              q1[12] - q2[12],
              q1[13] - q2[13],
              q1[14] - q2[14],
              (q1.segment<3>(15) - q2.segment<3>(15)).norm(),
              (q1.segment<3>(18) - q2.segment<3>(18)).norm(),
              (q1.segment<3>(21) - q2.segment<3>(21)).norm();
  distance(model, data,q1,q2,result);

  assert(result.isApprox(expected) && "Distance between two configs of full model - wrong results");
}

BOOST_AUTO_TEST_CASE ( randomization_test )
{
  se3::Model model;
  
  using namespace se3;

  model.addBody(model.getBodyId("universe"),JointModelFreeFlyer(),SE3::Identity(),Inertia::Random(),
                "freeflyer_joint", "freeflyer_body");
  model.addBody(model.getBodyId("freeflyer_body"),JointModelSpherical(),SE3::Identity(),Inertia::Random(),
                "spherical_joint", "spherical_body");
  model.addBody(model.getBodyId("spherical_body"),JointModelRX(),SE3::Identity(),Inertia::Random(),
                "revolute_joint", "revolute_body");
  model.addBody(model.getBodyId("revolute_body"),JointModelPX(),SE3::Identity(),Inertia::Random(),
                "px_joint", "px_body");
  model.addBody(model.getBodyId("px_body"),JointModelPrismaticUnaligned(Eigen::Vector3d(1,0,0)),SE3::Identity(),Inertia::Random(),
                "pu_joint", "pu_body");
  model.addBody(model.getBodyId("pu_body"),JointModelRevoluteUnaligned(Eigen::Vector3d(0,0,1)),SE3::Identity(),Inertia::Random(),
                "ru_joint", "ru_body");
  model.addBody(model.getBodyId("ru_body"),JointModelSphericalZYX(),SE3::Identity(),Inertia::Random(),
                "sphericalZYX_joint", "sphericalZYX_body");
  model.addBody(model.getBodyId("sphericalZYX_body"),JointModelTranslation(),SE3::Identity(),Inertia::Random(),
                "translation_joint", "translation_body");
  model.addBody(model.getBodyId("translation_body"),JointModelPlanar(),SE3::Identity(),Inertia::Random(),
                "planar_joint", "planar_body");
  
  se3::Data data(model);

  Eigen::VectorXd q1(model.nq);
  
  random(model, data,q1);

}

BOOST_AUTO_TEST_SUITE_END ()
