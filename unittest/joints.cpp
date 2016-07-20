//
// Copyright (c) 2015 CNRS
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

#include <iostream>

#include "pinocchio/math/fwd.hpp"
#include "pinocchio/spatial/force.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/multibody/joint/joint-revolute.hpp"
#include "pinocchio/multibody/joint/joint-revolute-unaligned.hpp"
#include "pinocchio/multibody/joint/joint-spherical.hpp"
#include "pinocchio/multibody/joint/joint-spherical-ZYX.hpp"
#include "pinocchio/multibody/joint/joint-prismatic.hpp"
#include "pinocchio/multibody/joint/joint-prismatic-unaligned.hpp"
#include "pinocchio/multibody/joint/joint-translation.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE JointsTest
#include <boost/test/unit_test.hpp>

//#define VERBOSE

template <typename JoinData_t>
void printOutJointData (
#ifdef VERBOSE
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & q_dot,
  const JoinData_t & joint_data
#else
  const Eigen::VectorXd & ,
  const Eigen::VectorXd & ,
  const JoinData_t & 
#endif
                        )
{
  using namespace std;
  using namespace se3;

#ifdef VERBOSE
  cout << "q: " << q.transpose () << endl;
  cout << "q_dot: " << q_dot.transpose () << endl;
  cout << "Joint configuration:" << endl << joint_data.M << endl;
  cout << "v_J:\n" << (Motion) joint_data.v << endl;
  cout << "c_J:\n" << (Motion) joint_data.c << endl;
#endif
}


BOOST_AUTO_TEST_SUITE (JointRevoluteUnaligned)

BOOST_AUTO_TEST_CASE (vsRX)
{
  using namespace se3;
  typedef Eigen::Matrix <double, 3, 1> Vector3;
  typedef Eigen::Matrix <double, 6, 1> Vector6;
  typedef Eigen::Matrix <double, 3, 3> Matrix3;

  Eigen::Vector3d axis;
  axis << 1.0, 0.0, 0.0;

  Model modelRX, modelRevoluteUnaligned;

  Inertia inertia (1., Vector3 (0.5, 0., 0.0), Matrix3::Identity ());
  SE3 pos(1); pos.translation() = SE3::Linear_t(1.,0.,0.);

  JointModelRevoluteUnaligned joint_model_RU(axis);
  modelRX.addJointAndBody (0, JointModelRX (), pos, inertia, "rx");
  modelRevoluteUnaligned.addJointAndBody(0, joint_model_RU ,pos, inertia, "revolute-unaligne");

  Data dataRX(modelRX);
  Data dataRevoluteUnaligned(modelRevoluteUnaligned);


  Eigen::VectorXd q = Eigen::VectorXd::Ones (modelRX.nq);
  Eigen::VectorXd v = Eigen::VectorXd::Ones (modelRX.nv);
  Eigen::VectorXd tauRX = Eigen::VectorXd::Ones (modelRX.nv);       Eigen::VectorXd tauRevoluteUnaligned = Eigen::VectorXd::Ones (modelRevoluteUnaligned.nv);
  Eigen::VectorXd aRX = Eigen::VectorXd::Ones (modelRX.nv);         Eigen::VectorXd aRevoluteUnaligned(aRX);
  


  forwardKinematics(modelRX, dataRX, q, v);
  forwardKinematics(modelRevoluteUnaligned, dataRevoluteUnaligned, q, v);

  computeAllTerms(modelRX, dataRX, q, v);
  computeAllTerms(modelRevoluteUnaligned, dataRevoluteUnaligned, q, v);

  BOOST_CHECK(dataRevoluteUnaligned.oMi[1].isApprox(dataRX.oMi[1]));
  BOOST_CHECK(dataRevoluteUnaligned.liMi[1].isApprox(dataRX.liMi[1]));
  BOOST_CHECK(dataRevoluteUnaligned.Ycrb[1].matrix().isApprox(dataRX.Ycrb[1].matrix()));
  BOOST_CHECK(dataRevoluteUnaligned.f[1].toVector().isApprox(dataRX.f[1].toVector()));
  
  BOOST_CHECK(dataRevoluteUnaligned.nle.isApprox(dataRX.nle));
  BOOST_CHECK(dataRevoluteUnaligned.com[0].isApprox(dataRX.com[0]));



  // InverseDynamics == rnea
  tauRX = rnea(modelRX, dataRX, q, v, aRX);
  tauRevoluteUnaligned = rnea(modelRevoluteUnaligned, dataRevoluteUnaligned, q, v, aRevoluteUnaligned);

  BOOST_CHECK(tauRX.isApprox(tauRevoluteUnaligned));

  // ForwardDynamics == aba
  Eigen::VectorXd aAbaRX = aba(modelRX,dataRX, q, v, tauRX);
  Eigen::VectorXd aAbaRevoluteUnaligned = aba(modelRevoluteUnaligned,dataRevoluteUnaligned, q, v, tauRevoluteUnaligned);


  BOOST_CHECK(aAbaRX.isApprox(aAbaRevoluteUnaligned));

  // crba
  crba(modelRX, dataRX,q);
  crba(modelRevoluteUnaligned, dataRevoluteUnaligned, q);

  BOOST_CHECK(dataRX.M.isApprox(dataRevoluteUnaligned.M));
   
  // Jacobian
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobianRX;jacobianRX.resize(6,1); jacobianRX.setZero();
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobianRevoluteUnaligned;jacobianRevoluteUnaligned.resize(6,1);jacobianRevoluteUnaligned.setZero();
  computeJacobians(modelRX, dataRX, q);
  computeJacobians(modelRevoluteUnaligned, dataRevoluteUnaligned, q);
  getJacobian<true>(modelRX, dataRX, 1, jacobianRX);
  getJacobian<true>(modelRevoluteUnaligned, dataRevoluteUnaligned, 1, jacobianRevoluteUnaligned);


  BOOST_CHECK(jacobianRX.isApprox(jacobianRevoluteUnaligned));


}
BOOST_AUTO_TEST_SUITE_END ()

BOOST_AUTO_TEST_SUITE (JointPrismaticUnaligned)

BOOST_AUTO_TEST_CASE (vsPX)
{
  using namespace se3;
  typedef Eigen::Matrix <double, 3, 1> Vector3;
  typedef Eigen::Matrix <double, 6, 1> Vector6;
  typedef Eigen::Matrix <double, 3, 3> Matrix3;

  Eigen::Vector3d axis;
  axis << 1.0, 0.0, 0.0;

  Model modelPX, modelPrismaticUnaligned;

  Inertia inertia (1., Vector3 (0.5, 0., 0.0), Matrix3::Identity ());
  SE3 pos(1); pos.translation() = SE3::Linear_t(1.,0.,0.);

  JointModelPrismaticUnaligned joint_model_PU(axis);
  modelPX.addJointAndBody (0, JointModelPX (), pos, inertia, "px");
  modelPrismaticUnaligned.addJointAndBody(0, joint_model_PU ,pos, inertia, "prismatic-unaligne");

  Data dataPX(modelPX);
  Data dataPrismaticUnaligned(modelPrismaticUnaligned);


  Eigen::VectorXd q = Eigen::VectorXd::Ones (modelPX.nq);
  Eigen::VectorXd v = Eigen::VectorXd::Ones (modelPX.nv);
  Eigen::VectorXd tauPX = Eigen::VectorXd::Ones (modelPX.nv);       Eigen::VectorXd tauPrismaticUnaligned = Eigen::VectorXd::Ones (modelPrismaticUnaligned.nv);
  Eigen::VectorXd aPX = Eigen::VectorXd::Ones (modelPX.nv);         Eigen::VectorXd aPrismaticUnaligned(aPX);
  


  forwardKinematics(modelPX, dataPX, q, v);
  forwardKinematics(modelPrismaticUnaligned, dataPrismaticUnaligned, q, v);

  computeAllTerms(modelPX, dataPX, q, v);
  computeAllTerms(modelPrismaticUnaligned, dataPrismaticUnaligned, q, v);

  BOOST_CHECK(dataPrismaticUnaligned.oMi[1].isApprox(dataPX.oMi[1]));
  BOOST_CHECK(dataPrismaticUnaligned.liMi[1].isApprox(dataPX.liMi[1]));
  BOOST_CHECK(dataPrismaticUnaligned.Ycrb[1].matrix().isApprox(dataPX.Ycrb[1].matrix()));
  BOOST_CHECK(dataPrismaticUnaligned.f[1].toVector().isApprox(dataPX.f[1].toVector()));
  
  BOOST_CHECK(dataPrismaticUnaligned.nle.isApprox(dataPX.nle));
  BOOST_CHECK(dataPrismaticUnaligned.com[0].isApprox(dataPX.com[0]));



  // InverseDynamics == rnea
  tauPX = rnea(modelPX, dataPX, q, v, aPX);
  tauPrismaticUnaligned = rnea(modelPrismaticUnaligned, dataPrismaticUnaligned, q, v, aPrismaticUnaligned);

  BOOST_CHECK(tauPX.isApprox(tauPrismaticUnaligned));

  // ForwardDynamics == aba
  Eigen::VectorXd aAbaPX = aba(modelPX,dataPX, q, v, tauPX);
  Eigen::VectorXd aAbaPrismaticUnaligned = aba(modelPrismaticUnaligned,dataPrismaticUnaligned, q, v, tauPrismaticUnaligned);


  BOOST_CHECK(aAbaPX.isApprox(aAbaPrismaticUnaligned));

  // crba
  crba(modelPX, dataPX,q);
  crba(modelPrismaticUnaligned, dataPrismaticUnaligned, q);

  BOOST_CHECK(dataPX.M.isApprox(dataPrismaticUnaligned.M));
   
  // Jacobian
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobianPX;jacobianPX.resize(6,1); jacobianPX.setZero();
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobianPrismaticUnaligned;jacobianPrismaticUnaligned.resize(6,1);jacobianPrismaticUnaligned.setZero();
  computeJacobians(modelPX, dataPX, q);
  computeJacobians(modelPrismaticUnaligned, dataPrismaticUnaligned, q);
  getJacobian<true>(modelPX, dataPX, 1, jacobianPX);
  getJacobian<true>(modelPrismaticUnaligned, dataPrismaticUnaligned, 1, jacobianPrismaticUnaligned);


  BOOST_CHECK(jacobianPX.isApprox(jacobianPrismaticUnaligned));


}
BOOST_AUTO_TEST_SUITE_END ()

BOOST_AUTO_TEST_SUITE (JointSpherical)

BOOST_AUTO_TEST_CASE (vsFreeFlyer)
{
  using namespace se3;
  typedef Eigen::Matrix <double, 3, 1> Vector3;
  typedef Eigen::Matrix <double, 6, 1> Vector6;
  typedef Eigen::Matrix <double, 7, 1> VectorFF;
  typedef Eigen::Matrix <double, 3, 3> Matrix3;

  Model modelSpherical, modelFreeflyer;

  Inertia inertia (1., Vector3 (0.5, 0., 0.0), Matrix3::Identity ());
  SE3 pos(1); pos.translation() = SE3::Linear_t(1.,0.,0.);


  modelSpherical.addJointAndBody (0, JointModelSpherical (), pos, inertia, "spherical");
  modelFreeflyer.addJointAndBody(0, JointModelFreeFlyer(),pos, inertia, "ff");

  Data dataSpherical(modelSpherical);
  Data dataFreeFlyer(modelFreeflyer);


  Eigen::VectorXd q = Eigen::VectorXd::Ones (modelSpherical.nq);q.normalize();  VectorFF qff; qff << 0, 0, 0, q[0], q[1], q[2], q[3]; 
  Eigen::VectorXd v = Eigen::VectorXd::Ones (modelSpherical.nv);               Vector6 vff; vff << 0, 0, 0, 1, 1, 1;
  Eigen::VectorXd tauSpherical = Eigen::VectorXd::Ones (modelSpherical.nv);       Eigen::VectorXd tauff; tauff.resize(7); tauff << 0,0,0,1,1,1,1;
  Eigen::VectorXd aSpherical = Eigen::VectorXd::Ones (modelSpherical.nv);         Eigen::VectorXd aff(vff);
  


  forwardKinematics(modelSpherical, dataSpherical, q, v);
  forwardKinematics(modelFreeflyer, dataFreeFlyer, qff, vff);

  computeAllTerms(modelSpherical, dataSpherical, q, v);
  computeAllTerms(modelFreeflyer, dataFreeFlyer, qff, vff);

  BOOST_CHECK(dataFreeFlyer.oMi[1].isApprox(dataSpherical.oMi[1]));
  BOOST_CHECK(dataFreeFlyer.liMi[1].isApprox(dataSpherical.liMi[1]));
  BOOST_CHECK(dataFreeFlyer.Ycrb[1].matrix().isApprox(dataSpherical.Ycrb[1].matrix()));
  BOOST_CHECK(dataFreeFlyer.f[1].toVector().isApprox(dataSpherical.f[1].toVector()));
  
  Eigen::VectorXd nle_expected_ff(3); nle_expected_ff << dataFreeFlyer.nle[3],
                                                         dataFreeFlyer.nle[4],
                                                         dataFreeFlyer.nle[5]
                                                         ;
  BOOST_CHECK(nle_expected_ff.isApprox(dataSpherical.nle));
  BOOST_CHECK(dataFreeFlyer.com[0].isApprox(dataSpherical.com[0]));



  // InverseDynamics == rnea
  tauSpherical = rnea(modelSpherical, dataSpherical, q, v, aSpherical);
  tauff = rnea(modelFreeflyer, dataFreeFlyer, qff, vff, aff);

  Vector3 tau_expected; tau_expected << tauff(3), tauff(4), tauff(5);
  BOOST_CHECK(tauSpherical.isApprox(tau_expected));

  // ForwardDynamics == aba
  Eigen::VectorXd aAbaSpherical = aba(modelSpherical,dataSpherical, q, v, tauSpherical);
  Eigen::VectorXd aAbaFreeFlyer = aba(modelFreeflyer,dataFreeFlyer, qff, vff, tauff);
  Vector3 a_expected; a_expected << aAbaFreeFlyer[3],
                                    aAbaFreeFlyer[4],
                                    aAbaFreeFlyer[5]
                                    ;
  BOOST_CHECK(aAbaSpherical.isApprox(a_expected));

  // crba
  crba(modelSpherical, dataSpherical,q);
  crba(modelFreeflyer, dataFreeFlyer, qff);

  Eigen::Matrix<double, 3, 3> M_expected(dataFreeFlyer.M.bottomRightCorner<3,3>());

  BOOST_CHECK(dataSpherical.M.isApprox(M_expected));
   
  // Jacobian
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_planar;jacobian_planar.resize(6,3); jacobian_planar.setZero();
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_ff;jacobian_ff.resize(6,6);jacobian_ff.setZero();
  computeJacobians(modelSpherical, dataSpherical, q);
  computeJacobians(modelFreeflyer, dataFreeFlyer, qff);
  getJacobian<true>(modelSpherical, dataSpherical, 1, jacobian_planar);
  getJacobian<true>(modelFreeflyer, dataFreeFlyer, 1, jacobian_ff);


  Eigen::Matrix<double, 6, 3> jacobian_expected; jacobian_expected << jacobian_ff.col(3),
                                                                      jacobian_ff.col(4),
                                                                      jacobian_ff.col(5)
                                                                      ;

  BOOST_CHECK(jacobian_planar.isApprox(jacobian_expected));

}
BOOST_AUTO_TEST_SUITE_END ()


BOOST_AUTO_TEST_SUITE (JointSphericalZYX)

BOOST_AUTO_TEST_CASE (vsFreeFlyer)
{
  // WARNIG : Dynamic algorithm's results cannot be compared to FreeFlyer's ones because 
  // of the representation of the rotation and the ConstraintSubspace difference.
  using namespace se3;
  typedef Eigen::Matrix <double, 3, 1> Vector3;
  typedef Eigen::Matrix <double, 6, 1> Vector6;
  typedef Eigen::Matrix <double, 7, 1> VectorFF;
  typedef Eigen::Matrix <double, 3, 3> Matrix3;

  Model modelSphericalZYX, modelFreeflyer;

  Inertia inertia (1., Vector3 (0.5, 0., 0.0), Matrix3::Identity ());
  SE3 pos(1); pos.translation() = SE3::Linear_t(1.,0.,0.);


  modelSphericalZYX.addJointAndBody (0, JointModelSphericalZYX (), pos, inertia, "spherical");
    modelFreeflyer.addJointAndBody(0, JointModelFreeFlyer(),pos, inertia, "ff");

  Data dataSphericalZYX(modelSphericalZYX);
  Data dataFreeFlyer(modelFreeflyer);

  Eigen::AngleAxisd rollAngle(1, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd yawAngle(1, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd pitchAngle(1, Eigen::Vector3d::UnitX());
  Eigen::Quaterniond q_sph = rollAngle * yawAngle * pitchAngle;
  
  Eigen::VectorXd q = Eigen::VectorXd::Ones (modelSphericalZYX.nq);              VectorFF qff; qff << 0, 0, 0, q_sph.x(), q_sph.y(), q_sph.z(), q_sph.w(); 
  Eigen::VectorXd v = Eigen::VectorXd::Ones (modelSphericalZYX.nv);               Vector6 vff; vff << 0, 0, 0, 1, 1, 1;
  Eigen::VectorXd tauSpherical = Eigen::VectorXd::Ones (modelSphericalZYX.nv);       Eigen::VectorXd tauff; tauff.resize(6); tauff << 0,0,0,1,1,1;
  Eigen::VectorXd aSpherical = Eigen::VectorXd::Ones (modelSphericalZYX.nv);         Eigen::VectorXd aff(vff);
  


  forwardKinematics(modelSphericalZYX, dataSphericalZYX, q, v);
  forwardKinematics(modelFreeflyer, dataFreeFlyer, qff, vff);

  computeAllTerms(modelSphericalZYX, dataSphericalZYX, q, v);
  computeAllTerms(modelFreeflyer, dataFreeFlyer, qff, vff);

  BOOST_CHECK(dataFreeFlyer.oMi[1].isApprox(dataSphericalZYX.oMi[1]));
  BOOST_CHECK(dataFreeFlyer.liMi[1].isApprox(dataSphericalZYX.liMi[1]));
  BOOST_CHECK(dataFreeFlyer.Ycrb[1].matrix().isApprox(dataSphericalZYX.Ycrb[1].matrix()));

  BOOST_CHECK(dataFreeFlyer.com[0].isApprox(dataSphericalZYX.com[0]));

}

BOOST_AUTO_TEST_CASE ( test_rnea )
{
  using namespace se3;
  typedef Eigen::Matrix <double, 3, 1> Vector3;
  typedef Eigen::Matrix <double, 3, 3> Matrix3;

  Model model;
  Inertia inertia (1., Vector3 (0.5, 0., 0.0), Matrix3::Identity ());

  model.addJointAndBody (model.getJointId("universe"), JointModelSphericalZYX (), SE3::Identity (), inertia, "root");

  Data data (model);

  Eigen::VectorXd q = Eigen::VectorXd::Zero (model.nq);
  Eigen::VectorXd v = Eigen::VectorXd::Zero (model.nv);
  Eigen::VectorXd a = Eigen::VectorXd::Zero (model.nv);

  rnea (model, data, q, v, a);
  Vector3 tau_expected (0., -4.905, 0.);

  BOOST_CHECK (tau_expected.isApprox(data.tau, 1e-14));

  q = Eigen::VectorXd::Ones (model.nq);
  v = Eigen::VectorXd::Ones (model.nv);
  a = Eigen::VectorXd::Ones (model.nv);

  rnea (model, data, q, v, a);
  tau_expected << -0.53611600195085, -0.74621832606188, -0.38177329067604;

  BOOST_CHECK (tau_expected.isApprox(data.tau, 1e-12));

  q << 3, 2, 1;
  v = Eigen::VectorXd::Ones (model.nv);
  a = Eigen::VectorXd::Ones (model.nv);

  rnea (model, data, q, v, a);
  tau_expected << 0.73934458094049,  2.7804530848031, 0.50684940972146;

  BOOST_CHECK (tau_expected.isApprox(data.tau, 1e-12));
}

BOOST_AUTO_TEST_CASE ( test_crba )
{
  using namespace se3;
  using namespace std;
  typedef Eigen::Matrix <double, 3, 1> Vector3;
  typedef Eigen::Matrix <double, 3, 3> Matrix3;

  Model model;
  Inertia inertia (1., Vector3 (0.5, 0., 0.0), Matrix3::Identity ());

  model.addJointAndBody (model.getJointId("universe"), JointModelSphericalZYX (), SE3::Identity (), inertia, "root");

  Data data (model);

  Eigen::VectorXd q (Eigen::VectorXd::Zero (model.nq));
  Eigen::MatrixXd M_expected (model.nv,model.nv);

  crba (model, data, q);
  M_expected <<
  1.25,    0,    0,
  0, 1.25,    0,
  0,    0,    1;

  BOOST_CHECK (M_expected.isApprox(data.M, 1e-14));

  q = Eigen::VectorXd::Ones (model.nq);

  crba (model, data, q);
  M_expected <<
  1.0729816454316, -5.5511151231258e-17,     -0.8414709848079,
  -5.5511151231258e-17,                 1.25,                    0,
  -0.8414709848079,                    0,                    1;

  BOOST_CHECK (M_expected.isApprox(data.M, 1e-12));

  q << 3, 2, 1;

  crba (model, data, q);
  M_expected <<
  1.043294547392, 2.7755575615629e-17,   -0.90929742682568,
  0,                1.25,                   0,
  -0.90929742682568,                   0,                  1;

  BOOST_CHECK (M_expected.isApprox(data.M, 1e-10));
}

BOOST_AUTO_TEST_SUITE_END ()

BOOST_AUTO_TEST_SUITE ( JointPrismatic )

BOOST_AUTO_TEST_CASE ( test_kinematics )
{
  using namespace se3;


  Motion expected_v_J (Motion::Zero ());
  Motion expected_c_J (Motion::Zero ());

  SE3 expected_configuration (SE3::Identity ());

  JointDataPX joint_data;
  JointModelPX joint_model;

  joint_model.setIndexes (0, 0, 0);

  Eigen::VectorXd q (Eigen::VectorXd::Zero (1));
  Eigen::VectorXd q_dot (Eigen::VectorXd::Zero (1));

  // -------
  q << 0. ;
  q_dot << 0.;

  joint_model.calc (joint_data, q, q_dot);

  printOutJointData <JointDataPX> (q, q_dot, joint_data);

  BOOST_CHECK (expected_configuration.rotation ().isApprox(joint_data.M.rotation(), 1e-12));
  BOOST_CHECK (expected_configuration.translation ().isApprox(joint_data.M.translation (), 1e-12));
  BOOST_CHECK (expected_v_J.toVector ().isApprox(((Motion) joint_data.v).toVector(), 1e-12));
  BOOST_CHECK (expected_c_J.toVector ().isApprox(((Motion) joint_data.c).toVector(), 1e-12));

  // -------
  q << 1.;
  q_dot << 1.;


  joint_model.calc (joint_data, q, q_dot);

  printOutJointData <JointDataPX> (q, q_dot, joint_data);

  expected_configuration.translation () << 1, 0, 0;

  expected_v_J.linear () << 1., 0., 0.;

  BOOST_CHECK (expected_configuration.rotation ().isApprox(joint_data.M.rotation(), 1e-12));
  BOOST_CHECK (expected_configuration.translation ().isApprox(joint_data.M.translation (), 1e-12));
  BOOST_CHECK (expected_v_J.toVector ().isApprox(((Motion) joint_data.v).toVector(), 1e-12));
  BOOST_CHECK (expected_c_J.toVector ().isApprox(((Motion) joint_data.c).toVector(), 1e-12));
}

BOOST_AUTO_TEST_CASE ( test_rnea )
{
  using namespace se3;
  typedef Eigen::Matrix <double, 3, 1> Vector3;
  typedef Eigen::Matrix <double, 3, 3> Matrix3;

  Model model;
  Inertia inertia (1., Vector3 (0.5, 0., 0.0), Matrix3::Identity ());

  model.addJointAndBody (model.getJointId("universe"), JointModelPX(), SE3::Identity (), inertia, "root");

  Data data (model);

  Eigen::VectorXd q (Eigen::VectorXd::Zero (model.nq));
  Eigen::VectorXd v (Eigen::VectorXd::Zero (model.nv));
  Eigen::VectorXd a (Eigen::VectorXd::Zero (model.nv));

  rnea (model, data, q, v, a);

  Eigen::VectorXd tau_expected (Eigen::VectorXd::Zero (model.nq));
  tau_expected  << 0;

  BOOST_CHECK (tau_expected.isApprox(data.tau, 1e-14));

  // -----
  q = Eigen::VectorXd::Ones (model.nq);
  v = Eigen::VectorXd::Ones (model.nv);
  a = Eigen::VectorXd::Ones (model.nv);

  rnea (model, data, q, v, a);
  tau_expected << 1;

  BOOST_CHECK (tau_expected.isApprox(data.tau, 1e-12));

  q << 3;
  v = Eigen::VectorXd::Ones (model.nv);
  a = Eigen::VectorXd::Ones (model.nv);

  rnea (model, data, q, v, a);
  tau_expected << 1;

  BOOST_CHECK (tau_expected.isApprox(data.tau, 1e-12));
}

BOOST_AUTO_TEST_CASE ( test_crba )
{
  using namespace se3;
  using namespace std;
  typedef Eigen::Matrix <double, 3, 1> Vector3;
  typedef Eigen::Matrix <double, 3, 3> Matrix3;

  Model model;
  Inertia inertia (1., Vector3 (0.5, 0., 0.0), Matrix3::Identity ());

  model.addJointAndBody (model.getJointId("universe"), JointModelPX (), SE3::Identity (), inertia, "root");

  Data data (model);

  Eigen::VectorXd q (Eigen::VectorXd::Zero (model.nq));
  Eigen::MatrixXd M_expected (model.nv,model.nv);

  crba (model, data, q);
  M_expected << 1.0;

  BOOST_CHECK (M_expected.isApprox(data.M, 1e-14));

  q = Eigen::VectorXd::Ones (model.nq);

  crba (model, data, q);

  BOOST_CHECK (M_expected.isApprox(data.M, 1e-12));

  q << 3;

  crba (model, data, q);
  
  BOOST_CHECK (M_expected.isApprox(data.M, 1e-10));
}

BOOST_AUTO_TEST_SUITE_END ()



BOOST_AUTO_TEST_SUITE ( JointDense )

BOOST_AUTO_TEST_CASE ( toJointModelDense )
{
  using namespace se3;


  JointModelRX jmodel;
  jmodel.setIndexes (2, 0, 0);

  JointModelDense<JointModelBase<JointModelRX>::NQ, JointModelBase<JointModelRX>::NV> jmd(jmodel.id(), jmodel.idx_q(), jmodel.idx_v());
  JointModelDense<JointModelBase<JointModelRX>::NQ, JointModelBase<JointModelRX>::NV> jmd2 = jmodel.toDense();
  (void)jmd; (void)jmd2;

  BOOST_CHECK_MESSAGE(jmd.idx_q() == jmodel.idx_q() , "The comparison of the joint index in configuration space failed");
  BOOST_CHECK_MESSAGE(jmd.idx_q() == jmd2.idx_q() , "The comparison of the joint index in  configuration space failed");

  BOOST_CHECK_MESSAGE(jmd.idx_v() == jmodel.idx_v() , "The comparison of the joint index in velocity space failed");
  BOOST_CHECK_MESSAGE(jmd.idx_v() == jmd2.idx_v() , "The comparison of the joint index in  velocity space failed");

  BOOST_CHECK_MESSAGE(jmd.id() == jmodel.id() , "The comparison of the joint index in model's kinematic tree failed");
  BOOST_CHECK_MESSAGE(jmd.id() == jmd2.id() , "The comparison of the joint index in model's kinematic tree failed");

}

BOOST_AUTO_TEST_CASE ( toJointDataDense )
{
  using namespace se3;

  JointModelRX jmodel;
  jmodel.setIndexes (2, 0, 0);

  JointDataRX jdata = jmodel.createData();

  JointDataDense< JointDataBase<JointModelRX::JointDataDerived>::NQ,
                  JointDataBase<JointModelRX::JointDataDerived>::NV
                  > jdd = jdata.toDense();

  BOOST_CHECK(ConstraintXd(jdata.S).matrix().isApprox(jdd.S.matrix()));

}
BOOST_AUTO_TEST_SUITE_END ()

BOOST_AUTO_TEST_SUITE (JointPlanar)

BOOST_AUTO_TEST_CASE (vsFreeFlyer)
{
  using namespace se3;
  typedef Eigen::Matrix <double, 3, 1> Vector3;
  typedef Eigen::Matrix <double, 6, 1> Vector6;
  typedef Eigen::Matrix <double, 7, 1> VectorFF;
  typedef Eigen::Matrix <double, 3, 3> Matrix3;

  Model modelPlanar, modelFreeflyer;

  Inertia inertia (1., Vector3 (0.5, 0., 0.0), Matrix3::Identity ());
  SE3 pos(1); pos.translation() = SE3::Linear_t(1.,0.,0.);


  modelPlanar.addJointAndBody (0, JointModelPlanar (), pos, inertia, "planar");
  modelFreeflyer.addJointAndBody(0, JointModelFreeFlyer(),pos, inertia, "ff");

  Data dataPlanar(modelPlanar);
  Data dataFreeFlyer(modelFreeflyer);


  Eigen::VectorXd q = Eigen::VectorXd::Ones (modelPlanar.nq);q[2] = PI /2;  VectorFF qff; qff << 1, 1, 0, 0, 0, sqrt(2)/2, sqrt(2)/2 ; 
  Eigen::VectorXd v = Eigen::VectorXd::Ones (modelPlanar.nv);               Vector6 vff; vff << 1, 1, 0, 0, 0, 1;
  Eigen::VectorXd tauPlanar = Eigen::VectorXd::Ones (modelPlanar.nv);       Eigen::VectorXd tauff = Eigen::VectorXd::Ones (modelFreeflyer.nv);
  Eigen::VectorXd aPlanar = Eigen::VectorXd::Ones (modelPlanar.nv);         Eigen::VectorXd aff(vff);
  


  forwardKinematics(modelPlanar, dataPlanar, q, v);
  forwardKinematics(modelFreeflyer, dataFreeFlyer, qff, vff);

  computeAllTerms(modelPlanar, dataPlanar, q, v);
  computeAllTerms(modelFreeflyer, dataFreeFlyer, qff, vff);

  BOOST_CHECK(dataFreeFlyer.oMi[1].isApprox(dataPlanar.oMi[1]));
  BOOST_CHECK(dataFreeFlyer.liMi[1].isApprox(dataPlanar.liMi[1]));
  BOOST_CHECK(dataFreeFlyer.Ycrb[1].matrix().isApprox(dataPlanar.Ycrb[1].matrix()));
  BOOST_CHECK(dataFreeFlyer.f[1].toVector().isApprox(dataPlanar.f[1].toVector()));
  
  Eigen::VectorXd nle_expected_ff(3); nle_expected_ff << dataFreeFlyer.nle[0],
                                                         dataFreeFlyer.nle[1],
                                                         dataFreeFlyer.nle[5]
                                                         ;
  BOOST_CHECK(nle_expected_ff.isApprox(dataPlanar.nle));
  BOOST_CHECK(dataFreeFlyer.com[0].isApprox(dataPlanar.com[0]));



  // InverseDynamics == rnea
  tauPlanar = rnea(modelPlanar, dataPlanar, q, v, aPlanar);
  tauff = rnea(modelFreeflyer, dataFreeFlyer, qff, vff, aff);

  Vector3 tau_expected; tau_expected << tauff(0), tauff(1), tauff(5);
  BOOST_CHECK(tauPlanar.isApprox(tau_expected));

  // ForwardDynamics == aba
  Eigen::VectorXd aAbaPlanar = aba(modelPlanar,dataPlanar, q, v, tauPlanar);
  Eigen::VectorXd aAbaFreeFlyer = aba(modelFreeflyer,dataFreeFlyer, qff, vff, tauff);
  Vector3 a_expected; a_expected << aAbaFreeFlyer[0],
                                    aAbaFreeFlyer[1],
                                    aAbaFreeFlyer[5]
                                    ;
  BOOST_CHECK(aAbaPlanar.isApprox(a_expected));

  // crba
  crba(modelPlanar, dataPlanar,q);
  crba(modelFreeflyer, dataFreeFlyer, qff);

  Eigen::Matrix<double, 3, 3> M_expected;
  M_expected.block<2,2>(0,0) = dataFreeFlyer.M.block<2,2>(0,0);
  M_expected.block<1,2>(2,0) = dataFreeFlyer.M.block<1,2>(5,0);
  M_expected.block<2,1>(0,2) = dataFreeFlyer.M.col(5).head<2>();
  M_expected.block<1,1>(2,2) = dataFreeFlyer.M.col(5).tail<1>();

  BOOST_CHECK(dataPlanar.M.isApprox(M_expected));
   
  // Jacobian
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_planar;jacobian_planar.resize(6,3); jacobian_planar.setZero();
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_ff;jacobian_ff.resize(6,6);jacobian_ff.setZero();
  computeJacobians(modelPlanar, dataPlanar, q);
  computeJacobians(modelFreeflyer, dataFreeFlyer, qff);
  getJacobian<true>(modelPlanar, dataPlanar, 1, jacobian_planar);
  getJacobian<true>(modelFreeflyer, dataFreeFlyer, 1, jacobian_ff);


  Eigen::Matrix<double, 6, 3> jacobian_expected; jacobian_expected << jacobian_ff.col(0),
                                                                      jacobian_ff.col(1),
                                                                      jacobian_ff.col(5)
                                                                      ;

  BOOST_CHECK(jacobian_planar.isApprox(jacobian_expected));


}
BOOST_AUTO_TEST_SUITE_END ()

BOOST_AUTO_TEST_SUITE (JointTranslation)

BOOST_AUTO_TEST_CASE (vsFreeFlyer)
{
  using namespace se3;
  typedef Eigen::Matrix <double, 3, 1> Vector3;
  typedef Eigen::Matrix <double, 6, 1> Vector6;
  typedef Eigen::Matrix <double, 7, 1> VectorFF;
  typedef Eigen::Matrix <double, 3, 3> Matrix3;

  Model modelTranslation, modelFreeflyer;

  Inertia inertia (1., Vector3 (0.5, 0., 0.0), Matrix3::Identity ());
  SE3 pos(1); pos.translation() = SE3::Linear_t(1.,0.,0.);


  modelTranslation.addJointAndBody (0, JointModelTranslation (), pos, inertia, "translation");
  modelFreeflyer.addJointAndBody(0, JointModelFreeFlyer(),pos, inertia, "ff");

  Data dataTranslation(modelTranslation);
  Data dataFreeFlyer(modelFreeflyer);


  Eigen::VectorXd q = Eigen::VectorXd::Ones (modelTranslation.nq);               VectorFF qff; qff << 1, 1, 1, 0, 0, 0, 1 ; 
  Eigen::VectorXd v = Eigen::VectorXd::Ones (modelTranslation.nv);               Vector6 vff; vff << 1, 1, 1, 0, 0, 0;
  Eigen::VectorXd tauTranslation = Eigen::VectorXd::Ones (modelTranslation.nv);       Eigen::VectorXd tauff(6); tauff << 1, 1, 1, 0, 0, 0;
  Eigen::VectorXd aTranslation = Eigen::VectorXd::Ones (modelTranslation.nv);         Eigen::VectorXd aff(vff);
  


  forwardKinematics(modelTranslation, dataTranslation, q, v);
  forwardKinematics(modelFreeflyer, dataFreeFlyer, qff, vff);

  computeAllTerms(modelTranslation, dataTranslation, q, v);
  computeAllTerms(modelFreeflyer, dataFreeFlyer, qff, vff);

  BOOST_CHECK(dataFreeFlyer.oMi[1].isApprox(dataTranslation.oMi[1]));
  BOOST_CHECK(dataFreeFlyer.liMi[1].isApprox(dataTranslation.liMi[1]));
  BOOST_CHECK(dataFreeFlyer.Ycrb[1].matrix().isApprox(dataTranslation.Ycrb[1].matrix()));
  BOOST_CHECK(dataFreeFlyer.f[1].toVector().isApprox(dataTranslation.f[1].toVector()));
  
  Eigen::VectorXd nle_expected_ff(3); nle_expected_ff << dataFreeFlyer.nle[0],
                                                         dataFreeFlyer.nle[1],
                                                         dataFreeFlyer.nle[2]
                                                         ;
  BOOST_CHECK(nle_expected_ff.isApprox(dataTranslation.nle));
  BOOST_CHECK(dataFreeFlyer.com[0].isApprox(dataTranslation.com[0]));



  // InverseDynamics == rnea
  tauTranslation = rnea(modelTranslation, dataTranslation, q, v, aTranslation);
  tauff = rnea(modelFreeflyer, dataFreeFlyer, qff, vff, aff);

  Vector3 tau_expected; tau_expected << tauff(0), tauff(1), tauff(2);
  BOOST_CHECK(tauTranslation.isApprox(tau_expected));

  // ForwardDynamics == aba
  Eigen::VectorXd aAbaTranslation = aba(modelTranslation,dataTranslation, q, v, tauTranslation);
  Eigen::VectorXd aAbaFreeFlyer = aba(modelFreeflyer,dataFreeFlyer, qff, vff, tauff);
  Vector3 a_expected; a_expected << aAbaFreeFlyer[0],
                                    aAbaFreeFlyer[1],
                                    aAbaFreeFlyer[2]
                                    ;
  BOOST_CHECK(aAbaTranslation.isApprox(a_expected));

  // crba
  crba(modelTranslation, dataTranslation,q);
  crba(modelFreeflyer, dataFreeFlyer, qff);

  Eigen::Matrix<double, 3, 3> M_expected(dataFreeFlyer.M.topLeftCorner<3,3>());

  BOOST_CHECK(dataTranslation.M.isApprox(M_expected));
   
  // Jacobian
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_planar;jacobian_planar.resize(6,3); jacobian_planar.setZero();
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_ff;jacobian_ff.resize(6,6);jacobian_ff.setZero();
  computeJacobians(modelTranslation, dataTranslation, q);
  computeJacobians(modelFreeflyer, dataFreeFlyer, qff);
  getJacobian<true>(modelTranslation, dataTranslation, 1, jacobian_planar);
  getJacobian<true>(modelFreeflyer, dataFreeFlyer, 1, jacobian_ff);


  Eigen::Matrix<double, 6, 3> jacobian_expected; jacobian_expected << jacobian_ff.col(0),
                                                                      jacobian_ff.col(1),
                                                                      jacobian_ff.col(2)
                                                                      ;

  BOOST_CHECK(jacobian_planar.isApprox(jacobian_expected));


}
BOOST_AUTO_TEST_SUITE_END ()
