//
// Copyright (c) 2015 CNRS
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

BOOST_AUTO_TEST_CASE ( integration )
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

  integrateModel(model, data,q,q_dot,result);

  std::cout << "-- full model -- " << std::endl;
  std::cout << "result : \n " << result << std::endl;
  std::cout << "expected : \n " << expected << std::endl << std::endl;
  assert(result.isApprox(expected) && "integration of freeflyer joint - wrong results");
}
BOOST_AUTO_TEST_SUITE_END ()
