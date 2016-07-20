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


#include <iostream>
#include <iomanip>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/math/quaternion.hpp"

#include <vector>

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE JointConfigurationsTest
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

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

se3::Model createModelWithAllJoints()
{
  using namespace se3;

  Model model;

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

  return model;
}

se3::Model createBoundedModelWithAllJoints()
{
  using namespace se3;
 
  Model model;

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

  return model;
}



BOOST_AUTO_TEST_SUITE ( JointConfigurationsTest )

BOOST_AUTO_TEST_CASE ( integration_test )
{
  
  using namespace se3;

  // Creating the Model and Data
  Model model = createModelWithAllJoints();  
  se3::Data data(model);


  std::vector<Eigen::VectorXd> qs(2);
  std::vector<Eigen::VectorXd> qdots(2);
  std::vector<Eigen::VectorXd> results(2);

  //
  // Test Case 0 : Integration of a config with zero velocity
  //
  qs[0] = Eigen::VectorXd::Ones(model.nq);
  qs[0].segment<4>(3) /= qs[0].segment<4>(3).norm(); // quaternion of freeflyer
  qs[0].segment<4>(7) /= qs[0].segment<4>(7).norm(); // quaternion of spherical joint
 
  qdots[0] = Eigen::VectorXd::Zero(model.nv);
  results[0] = integrate(model,qs[0],qdots[0]);

  BOOST_CHECK_MESSAGE(results[0].isApprox(qs[0], 1e-12), "integration of full body with zero velocity - wrong results");

  //
  // Test Case 1 : Integration of a config with a non-zero velocity
  //
  qs[1] = Eigen::VectorXd::Ones(model.nq);
  qdots[1] = Eigen::VectorXd::Random(model.nv);

  Eigen::VectorXd expected(model.nq);
  Eigen::Quaterniond quat_ff(qs[1][6],qs[1][3],qs[1][4],qs[1][5]); Eigen::Vector3d omega_ff(qdots[1][3],qdots[1][4],qdots[1][5]); Eigen::Vector3d transl_ff(qs[1][0],qs[1][1],qs[1][2]);
  Eigen::Quaterniond quat_spherical(qs[1][10],qs[1][7],qs[1][8],qs[1][9]); Eigen::Vector3d omega_spherical(qdots[1][6],qdots[1][7],qdots[1][8]);


  Eigen::Quaterniond v_ff(Eigen::AngleAxisd(omega_ff.norm(), omega_ff/omega_ff.norm()));
  Eigen::Quaterniond quat_ff__int( v_ff * quat_ff);
  Eigen::Quaterniond v_spherical(Eigen::AngleAxisd(omega_spherical.norm(), omega_spherical/omega_spherical.norm()));
  Eigen::Quaterniond quat_spherical_int( v_spherical * quat_spherical);

  expected.head<3>() = qs[1].head<3>() + qdots[1].head<3>();
  expected[3] = quat_ff__int.x();expected[4] = quat_ff__int.y(); expected[5] = quat_ff__int.z(); expected[6] = quat_ff__int.w(); 
  expected[7] = quat_spherical_int.x();expected[8] = quat_spherical_int.y(); expected[9] = quat_spherical_int.z(); expected[10] = quat_spherical_int.w(); 
  expected.tail<13>() = qs[1].tail<13>() + qdots[1].tail<13>();

  results[1] = integrate(model,qs[1],qdots[1]);

  BOOST_CHECK_MESSAGE(results[1].isApprox(expected, 1e-12), "integration of full body with non zero velocity - wrong results");
}

BOOST_AUTO_TEST_CASE ( interpolation_test )
{
  using namespace se3;

  // Creating the Model and Data
  Model model = createModelWithAllJoints();  
  se3::Data data(model);

  std::vector<Eigen::VectorXd> qs1(3);
  std::vector<Eigen::VectorXd> qs2(3);
  std::vector<double> us(3);
  std::vector<Eigen::VectorXd> results(3);
  std::vector<Eigen::VectorXd> expecteds(3);
  
  //
  // Test Case 0 : u between 0 and 1
  //
  qs1[0] = Eigen::VectorXd::Zero(model.nq);
  qs2[0] = Eigen::VectorXd::Ones(model.nq);
  
  Eigen::Vector3d axis (Eigen::Vector3d::Ones()); axis.normalize();
  const double angle (0.5);
  Eigen::AngleAxis<double> aa1 (0., axis);
  Eigen::AngleAxis<double> aa2 (angle, axis);
  
  qs1[0].segment<4>(3) = Eigen::Quaterniond(aa1).coeffs();
  qs1[0].segment<4>(7) = Eigen::Quaterniond(aa1).coeffs();
  
  qs2[0].segment<4>(3) = Eigen::Quaterniond(aa2).coeffs();
  qs2[0].segment<4>(7) = Eigen::Quaterniond(aa2).coeffs();
  us[0] = 0.1; double & u = us[0];
  
  Eigen::Quaterniond quat_ff_1(qs1[0][6],qs1[0][3],qs1[0][4],qs1[0][5]);
  Eigen::Quaterniond quat_spherical_1(qs1[0][10],qs1[0][7],qs1[0][8],qs1[0][9]);
  Eigen::Quaterniond quat_ff_2(qs2[0][6],qs2[0][3],qs2[0][4],qs2[0][5]);
  Eigen::Quaterniond quat_spherical_2(qs2[0][10],qs2[0][7],qs2[0][8],qs2[0][9]);

  Eigen::VectorXd & expected = expecteds[0];
  expected.resize(model.nq);


  // filling expected.
  Eigen::Quaterniond quat_ff__int = quat_ff_1.slerp(u, quat_ff_2);
  Eigen::Quaterniond quat_spherical_int = quat_spherical_1.slerp(u, quat_spherical_2);

  expected.head<3>() = (1-u) * qs1[0].head<3>() + u*qs2[0].head<3>();
  expected[3] = quat_ff__int.x();expected[4] = quat_ff__int.y(); expected[5] = quat_ff__int.z(); expected[6] = quat_ff__int.w(); 
  expected[7] = quat_spherical_int.x();expected[8] = quat_spherical_int.y(); expected[9] = quat_spherical_int.z(); expected[10] = quat_spherical_int.w(); 
  expected.tail<13>() = (1-u)* qs1[0].tail<13>() + u*qs2[0].tail<13>();

  Eigen::VectorXd & result = results[0];
  result = interpolate(model,qs1[0],qs2[0],us[0]);

  BOOST_CHECK_MESSAGE(result.isApprox(expected, 1e-12), "interpolation full model for u = 0.1 - wrong results");
  
  //
  // Test Case 1 : u = 0 -> expected = qs1[1]
  //
  us[1] = 0; u = us[1];
  qs1[1] = qs1[0];
  qs2[1] = qs2[0];
  result = results[1];
  expected = expecteds[1]; expected.resize(model.nq);

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

  expected.head<3>() = (1-u) * qs1[1].head<3>() + u*qs2[1].head<3>();
  expected[3] = quat_ff__int.x();expected[4] = quat_ff__int.y(); expected[5] = quat_ff__int.z(); expected[6] = quat_ff__int.w(); 
  expected[7] = quat_spherical_int.x();expected[8] = quat_spherical_int.y(); expected[9] = quat_spherical_int.z(); expected[10] = quat_spherical_int.w(); 
  expected.tail<13>() = (1-u)* qs1[1].tail<13>() + u*qs2[1].tail<13>();

  result = interpolate(model,qs1[1],qs2[1],u);

  BOOST_CHECK_MESSAGE(result.isApprox(qs1[1], 1e-12), "interpolation with u = 0 - wrong results");
  
  //
  // u = 1 -> q_interpolate = q2
  //
  us[2] = 1; u = us[2];
  result = results[2];
  qs1[2] = qs1[0];
  qs2[2] = qs2[0];
  expected = expecteds[2]; expected.resize(model.nq);


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

  expected.head<3>() = (1-u) * qs1[2].head<3>() + u*qs2[2].head<3>();
  expected[3] = quat_ff__int.x();expected[4] = quat_ff__int.y(); expected[5] = quat_ff__int.z(); expected[6] = quat_ff__int.w(); 
  expected[7] = quat_spherical_int.x();expected[8] = quat_spherical_int.y(); expected[9] = quat_spherical_int.z(); expected[10] = quat_spherical_int.w(); 
  expected.tail<13>() = (1-u)* qs1[2].tail<13>() + u*qs2[2].tail<13>();

  result = interpolate(model,qs1[2],qs2[2],u);

  BOOST_CHECK_MESSAGE(configurations_are_equals(result, qs2[2]), "interpolation with u = 1 - wrong results");
}

BOOST_AUTO_TEST_CASE ( differentiation_test )
{
  using namespace se3;

  // Creating the Model and Data
  Model model = createModelWithAllJoints();  
  se3::Data data(model);


  //
  // Test Case 0 : Difference between two configs
  //
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

  //
  // Test Case 1 :  Difference between same zero configs
  //
  Eigen::VectorXd q0(Eigen::VectorXd::Zero(model.nq));
  expected = Eigen::VectorXd::Zero(model.nv);
  result = differentiate(model,q0,q0);
  BOOST_CHECK_MESSAGE(result.isApprox(expected, 1e-12), "Differentiation of full model with same zero configs - wrong results");

  //
  // Test Case 2 : Difference between same configs non zero
  //
  expected = Eigen::VectorXd::Zero(model.nv);
  result = differentiate(model,q1,q1);
  BOOST_CHECK_MESSAGE(result.isApprox(expected, 1e-12), "Differentiation of full model with same configs - wrong results");

}

BOOST_AUTO_TEST_CASE ( distance_computation_test )
{
  using namespace se3;

  // Creating the Model and Data
  Model model = createModelWithAllJoints();  
  se3::Data data(model);
  
  //
  // Test Case 0 : distance between two confis
  //

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

  //
  // Test Case 1 : Distance between two zero configs
  //
  Eigen::VectorXd q_zero(Eigen::VectorXd::Zero(model.nq));
  expected = Eigen::VectorXd::Zero(model.nbody-1);
  result = distance(model,q_zero,q_zero);
  BOOST_CHECK_MESSAGE(result.isApprox(expected, 1e-12), "Distance between two zero configs of full model - wrong results");

  //
  // Test Case 2 : Distance between two same configs
  //
  expected = Eigen::VectorXd::Zero(model.nbody-1);
  result = distance(model,q1,q1);
  BOOST_CHECK_MESSAGE(result.isApprox(expected, 1e-12), "Distance between two same configs of full model - wrong results");

  //
  // Test Case 3 : distance between q1 and q2 == distance between q2 and q1
  //
  BOOST_CHECK_MESSAGE(distance(model, q1, q2) == distance(model, q2, q1), "Distance q1 -> q2 != Distance q2 -> q1");
}

BOOST_AUTO_TEST_CASE ( neutral_configuration_test )
{
  using namespace se3;

  // Creating the Model and Data
  Model model = createModelWithAllJoints();  
  

  Eigen::VectorXd expected(model.nq);
  expected << 0,0,0,0,0,0,1,
              0,0,0,1,
              0,
              0,
              0,
              0,
              0,0,0,
              0,0,0,
              0,0,0;


  BOOST_CHECK_MESSAGE(model.neutralConfigurations.isApprox(expected, 1e-12), "neutral configurations - wrong results");
}

BOOST_AUTO_TEST_CASE ( uniform_sampling_test )
{
  using namespace se3;

  // Creating the Model and Data
  Model model = createBoundedModelWithAllJoints();  
  se3::Data data(model);

  Eigen::VectorXd q1(randomConfiguration(model));
  
  for (int i = 0; i < q1.size(); ++i)
  {
    BOOST_CHECK_MESSAGE(q1[i] >= model.lowerPositionLimit[i] && q1[i] <= model.upperPositionLimit[i], " UniformlySample : Generated config not in bounds");
  }

}

BOOST_AUTO_TEST_CASE ( integrate_difference_test )
{
  using namespace se3;

  // Creating the Model and Data
  Model model = createBoundedModelWithAllJoints();  
  se3::Data data(model);

  Eigen::VectorXd q1(randomConfiguration(model, -1 * Eigen::VectorXd::Ones(model.nq), Eigen::VectorXd::Ones(model.nq) ));
  Eigen::VectorXd q2(randomConfiguration(model, -1 * Eigen::VectorXd::Ones(model.nq), Eigen::VectorXd::Ones(model.nq) ));

  BOOST_CHECK_MESSAGE(configurations_are_equals(integrate(model, q1, differentiate(model, q1,q2)), q2), "relation between integrate and differentiate");

}

BOOST_AUTO_TEST_CASE ( normalized_test )
{
  using namespace se3;

  // Creating the Model and Data
  Model model = createModelWithAllJoints();
  se3::Data data(model);

  Eigen::VectorXd q = Eigen::VectorXd::Ones(model.nq);

  Eigen::VectorXd qn = se3::normalized(model, q);

  BOOST_CHECK(fabs(qn.segment<4>(3).norm() - 1) < Eigen::NumTraits<double>::epsilon()); // quaternion of freeflyer
  BOOST_CHECK(fabs(qn.segment<4>(7).norm() - 1) < Eigen::NumTraits<double>::epsilon()); // quaternion of spherical joint
}

BOOST_AUTO_TEST_SUITE_END ()
