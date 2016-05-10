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

#include "pinocchio/tools/timer.hpp"

#include "pinocchio/multibody/joint/joint-accessor.hpp"
#include "pinocchio/multibody/joint/joint-composite.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"

#include <iostream>

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE JointCompositeTest
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE ( JointCompositeTest)

BOOST_AUTO_TEST_CASE ( JointModel)
{
  using namespace Eigen;
  using namespace se3;

  // Constructor from one type
  JointModelComposite jmodel_composite((JointModelRX()));

  BOOST_CHECK_MESSAGE(jmodel_composite.isFull(), " Constructor with one  RX did not fullfilled the joint composite");
  BOOST_CHECK_MESSAGE(!jmodel_composite.isEmpty(), " Constructor with one  RX did not fullfilled the joint composite");
  BOOST_CHECK_MESSAGE(jmodel_composite.nq() == 1, " Dimension nq is not the same");
  BOOST_CHECK_MESSAGE(jmodel_composite.nv() == 1, " Dimension nv is not the same");

  // Constructor whith one int
  // Check is empty
  // Add a joint
  // Check not empty, check full
  // check that it does nothing to add a joint if joint composite is full
  // comparison
  JointModelComposite jmodel_composite_one_joint(1);
  BOOST_CHECK_MESSAGE(jmodel_composite_one_joint.isEmpty(), " Constructor with one derived type to be added did not fullfilled the joint composite");
  jmodel_composite_one_joint.addJointModel(JointModelRX());
  BOOST_CHECK_MESSAGE(!jmodel_composite_one_joint.isEmpty(), " Added a joint. but stating that the joint composite is empty");
  BOOST_CHECK_MESSAGE(jmodel_composite_one_joint.isFull(), " Added a joint to composite of max 1. but stating that the joint composite is not full");
  std::size_t nb_joints = jmodel_composite_one_joint.addJointModel(JointModelRY());
  BOOST_CHECK_MESSAGE(nb_joints == 1, " Added a 2nd joint to composite of max 1");


  /// Update components indexes
  /// Create joint composite with two joints,
  JointModelComposite jmodel_composite_two_rx(2);
  jmodel_composite_two_rx.addJointModel(JointModelRX());
  jmodel_composite_two_rx.addJointModel(JointModelRY());
  jmodel_composite_two_rx.setIndexes(3,8,7); // suppose it comes after a freeflyer, and universe
  jmodel_composite_two_rx.updateComponentsIndexes();
  JointModelVariant & first_rx = jmodel_composite_two_rx.joints[0];
  JointModelVariant & second_rx = jmodel_composite_two_rx.joints[1];

  BOOST_CHECK_MESSAGE(idx_q(first_rx) == 8, "");
  BOOST_CHECK_MESSAGE(idx_q(second_rx) == 9, "");
  BOOST_CHECK_MESSAGE(idx_v(first_rx) == 7, "");
  BOOST_CHECK_MESSAGE(idx_v(second_rx) == 8, "");
  BOOST_CHECK_MESSAGE(id(first_rx) == jmodel_composite_two_rx.id(), "");
  BOOST_CHECK_MESSAGE(id(second_rx) == jmodel_composite_two_rx.id(), "");


  /// Comparing computations to JointModelPlanar()

  JointModelComposite jmodel_composite_planar(3);
  jmodel_composite_planar.addJointModel(JointModelPX());
  jmodel_composite_planar.addJointModel(JointModelPY());
  jmodel_composite_planar.addJointModel(JointModelRZ());
  jmodel_composite_planar.setIndexes(1,0,0);
  jmodel_composite_planar.updateComponentsIndexes();

  JointModelPlanar jmodel_planar;
  jmodel_planar.setIndexes(1,0,0);


  Eigen::VectorXd q1(Eigen::VectorXd::Random(3));
  Eigen::VectorXd q1_dot(Eigen::VectorXd::Random(3));
  Eigen::VectorXd q2(Eigen::VectorXd::Random(3));
  double u = 0.3;

  BOOST_CHECK_MESSAGE( jmodel_planar.integrate(q1,q1_dot).isApprox(jmodel_composite_planar.integrate(q1,q1_dot))
                      ,"Joint Model Composite vs Planar - integrate error");
  BOOST_CHECK_MESSAGE( jmodel_planar.interpolate(q1,q2,u).isApprox(jmodel_composite_planar.interpolate(q1,q2,u))
                      ,"Joint Model Composite vs Planar - interpolate error");
  BOOST_CHECK_MESSAGE(jmodel_planar.randomConfiguration( -1 * Eigen::VectorXd::Ones(jmodel_planar.nq()),
                                                Eigen::VectorXd::Ones(jmodel_planar.nq())).size()
                                    == jmodel_composite_planar.randomConfiguration(-1 * Eigen::VectorXd::Ones(jmodel_composite_planar.nq()),
                                                              Eigen::VectorXd::Ones(jmodel_composite_planar.nq())).size()
                        ,"Joint Model Composite vs Planar - Dimensions of random configuration are not the same");

  BOOST_CHECK_MESSAGE( jmodel_planar.difference(q1,q2).isApprox(jmodel_composite_planar.difference(q1,q2))
                      ,"Joint Model Composite vs Planar - difference error");
  BOOST_CHECK_MESSAGE( jmodel_planar.distance(q1,q2) == jmodel_composite_planar.distance(q1,q2)
                      ,"Joint Model Composite vs Planar - distance error");
  
}

BOOST_AUTO_TEST_CASE ( JointData )
{
  using namespace Eigen;
  using namespace se3;

  JointModelComposite jmodel_composite_planar(3);
  jmodel_composite_planar.addJointModel(JointModelPX());
  jmodel_composite_planar.addJointModel(JointModelPY());
  jmodel_composite_planar.addJointModel(JointModelRZ());
  jmodel_composite_planar.setIndexes(1,0,0);
  jmodel_composite_planar.updateComponentsIndexes();


  JointDataComposite jdata_composite_planar = jmodel_composite_planar.createData();

  JointModelPlanar jmodel_planar;
  jmodel_planar.setIndexes(1,0,0);
  JointDataPlanar jdata_planar = jmodel_planar.createData();

  Eigen::VectorXd q1(Eigen::VectorXd::Random(3));
  Eigen::VectorXd q1_dot(Eigen::VectorXd::Random(3));

  /// 
  /// S
  /// 
  // BOOST_CHECK_MESSAGE(((ConstraintXd)jdata_planar.S).matrix().isApprox(jdata_composite_planar.S.matrix()), "JointComposite vs Planar - constraint not equal");

  /// 
  /// calc -> M
  /// 
  jmodel_planar.calc(jdata_planar,q1);
  jmodel_composite_planar.calc(jdata_composite_planar,q1);
  BOOST_CHECK_MESSAGE(jdata_planar.M == jdata_composite_planar.M, "Joint Composite vs Planar - calc_zero_order error");

  /// 
  /// calc -> M, (v)
  /// 
  jmodel_planar.calc(jdata_planar,q1, q1_dot);
  jmodel_composite_planar.calc(jdata_composite_planar,q1, q1_dot);
  BOOST_CHECK_MESSAGE(jdata_planar.M == jdata_composite_planar.M, "Joint Composite vs Planar - calc_first_order error for M");
  // BOOST_CHECK_MESSAGE(Motion(jdata_planar.v) == jdata_composite_planar.v, "Joint Composite vs Planar - calc_first_order error for v");
  

  /// 
  /// calc_aba -> (U, Dinv, UDinv)
  /// 
  Inertia::Matrix6 Ia(Inertia::Random().matrix());
  bool update_I = false;
  jmodel_planar.calc_aba(jdata_planar,Ia, update_I);
  jmodel_composite_planar.calc_aba(jdata_composite_planar,Ia, update_I);
  // BOOST_CHECK_MESSAGE(jdata_planar.U.isApprox(jdata_composite_planar.U), "Joint Composite vs Planar - calc_aba error for U");
  // BOOST_CHECK_MESSAGE(jdata_planar.Dinv.isApprox(jdata_composite_planar.Dinv), "Joint Composite vs Planar - calc_aba error for Dinv");
  // BOOST_CHECK_MESSAGE(jdata_planar.UDinv.isApprox(jdata_composite_planar.UDinv), "Joint Composite vs Planar - calc_aba error for UDinv");
  


}

BOOST_AUTO_TEST_CASE ( test_recursive_variant)
{
  using namespace Eigen;
  using namespace se3;


  /// Update components indexes
  /// Create joint composite with two joints,
  JointModelComposite jmodel_composite_two_rx(2);
  jmodel_composite_two_rx.addJointModel(JointModelRX());
  jmodel_composite_two_rx.addJointModel(JointModelRY());

  JointModelComposite jmodel_composite_recursive(3);
  jmodel_composite_recursive.addJointModel(JointModelFreeFlyer());
  jmodel_composite_recursive.addJointModel(JointModelPlanar());
  jmodel_composite_recursive.addJointModel(jmodel_composite_two_rx);
  
}


BOOST_AUTO_TEST_CASE ( KinematicModelCompositePlanar)
{
  using namespace Eigen;
  using namespace se3;

  Model model_composite_planar;
  Model model_px_py_rz;

  Inertia body_inertia(Inertia::Random());
  SE3 placement(SE3::Identity());

  model_px_py_rz.addBody(model_px_py_rz.getBodyId("universe"),JointModelPX(), placement ,Inertia::Zero(),
                        "px_joint", "px_body");
  model_px_py_rz.addBody(model_px_py_rz.getBodyId("px_body"),JointModelPY(), placement ,Inertia::Zero(),
                        "py_joint", "py_body");
  model_px_py_rz.addBody(model_px_py_rz.getBodyId("py_body"),JointModelRZ(), placement ,body_inertia,
                        "rz_joint", "rz_body");


  JointModelComposite jmodel_composite_planar(3);
  jmodel_composite_planar.addJointModel(JointModelPX());
  jmodel_composite_planar.addJointModel(JointModelPY());
  jmodel_composite_planar.addJointModel(JointModelRZ());

  model_composite_planar.addBody(model_composite_planar.getBodyId("universe"), jmodel_composite_planar, placement, body_inertia,
                                 "composite_planar_joint", "composite_planar_body");
  // When Model will be cleaned in coming pull request, this will be done in addBody(addJoint)
  boost::get<JointModelComposite>(model_composite_planar.joints[model_composite_planar.getJointId("composite_planar_joint")]).updateComponentsIndexes();


  BOOST_CHECK_MESSAGE(model_composite_planar.nq == model_px_py_rz.nq ,"Model with composite planar joint vs PxPyRz - dimensions nq are not equal");
  BOOST_CHECK_MESSAGE(model_composite_planar.nq == model_px_py_rz.nq ,"Model with composite planar joint vs PxPyRz - dimensions nv are not equal");


  Data data_px_py_pz(model_px_py_rz);
  Data data_composite_planar(model_composite_planar);

  Eigen::VectorXd q(Eigen::VectorXd::Random(model_px_py_rz.nq));
  Eigen::VectorXd q_dot(Eigen::VectorXd::Random(model_px_py_rz.nv));
  Eigen::VectorXd q_ddot(Eigen::VectorXd::Random(model_px_py_rz.nv));
  Eigen::VectorXd q1(Eigen::VectorXd::Random(model_px_py_rz.nq));
  Eigen::VectorXd q2(Eigen::VectorXd::Random(model_px_py_rz.nq));
  Eigen::VectorXd tau(Eigen::VectorXd::Random(model_px_py_rz.nq));
  double u = 0.3;

  // Test that algorithms do not crash
  integrate(model_composite_planar,q,q_dot);
  interpolate(model_composite_planar,q1,q2,u);
  differentiate(model_composite_planar,q1,q2);
  distance(model_composite_planar,q1,q2);
  randomConfiguration(model_composite_planar);

  // aba(model_composite_planar,data_composite_planar, q,q_dot, tau);
  centerOfMass(model_composite_planar, data_composite_planar,q,q_dot,q_ddot,true,false);
  emptyForwardPass(model_composite_planar, data_composite_planar);
  forwardKinematics(model_composite_planar,data_composite_planar, q );
  // forwardKinematics(model_composite_planar,data_composite_planar, q, q_dot);
  // forwardKinematics(model_composite_planar,data_composite_planar, q, q_dot, q_ddot);
  // computeAllTerms(model_px_py_rz,data_px_py_pz,q,q_dot);
  // computeAllTerms(model_composite_planar,data_composite_planar,q,q_dot);

  // Model::Index last_joint_pxpyrz = (Model::Index) model_px_py_rz.nbody-1;
  // Model::Index last_joint_composite = (Model::Index) model_composite_planar.nbody-1;

  // BOOST_CHECK_MESSAGE(data_composite_planar.oMi[last_joint_composite]
  //                         .isApprox(data_px_py_pz.oMi[last_joint_pxpyrz]) , "composite planar joint vs PxPyRz - oMi last joint not equal");

  // BOOST_CHECK_MESSAGE(data_composite_planar.v[last_joint_composite]
  //                         == data_px_py_pz.v[last_joint_pxpyrz] , "composite planar joint vs PxPyRz - v last joint not equal");

  // BOOST_CHECK_MESSAGE(data_composite_planar.a[last_joint_composite]
  //                         == data_px_py_pz.a[last_joint_pxpyrz] , "composite planar joint vs PxPyRz - a last joint not equal");

  // BOOST_CHECK_MESSAGE(data_composite_planar.f[last_joint_composite]
  //                         == data_px_py_pz.f[last_joint_pxpyrz] , "composite planar joint vs PxPyRz - f last joint not equal");

  // BOOST_CHECK_MESSAGE(data_composite_planar.com[last_joint_composite]
  //                         .isApprox(data_px_py_pz.com[last_joint_pxpyrz]) , "composite planar joint vs PxPyRz - com last joint not equal");

  // BOOST_CHECK_MESSAGE(data_composite_planar.vcom[last_joint_composite]
  //                         .isApprox(data_px_py_pz.vcom[last_joint_pxpyrz]) , "composite planar joint vs PxPyRz - vcom last joint not equal");

  // BOOST_CHECK_MESSAGE(data_composite_planar.mass[last_joint_composite]
  //                         == data_px_py_pz.mass[last_joint_pxpyrz] , "composite planar joint vs PxPyRz - mass last joint not equal"); 

  // BOOST_CHECK_MESSAGE(data_composite_planar.kinetic_energy
  //                         == data_px_py_pz.kinetic_energy , "composite planar joint vs PxPyRz - kinetic energy not equal");

  // BOOST_CHECK_MESSAGE(data_composite_planar.potential_energy
  //                         == data_px_py_pz.potential_energy , "composite planar joint vs PxPyRz - potential energy not equal");                          

  // BOOST_CHECK_MESSAGE(data_composite_planar.nle[last_joint_composite]
  //                         .isApprox(data_px_py_pz.nle[last_joint_pxpyrz]) , "composite planar joint vs PxPyRz - nle not equal");

  // BOOST_CHECK_MESSAGE(data_composite_planar.M[last_joint_composite]
  //                         .isApprox(data_px_py_pz.M[last_joint_pxpyrz]) , "composite planar joint vs PxPyRz - Mass Matrix not equal");

  // BOOST_CHECK_MESSAGE(data_composite_planar.J[last_joint_composite]
  //                         .isApprox(data_px_py_pz.J[last_joint_pxpyrz]) , "composite planar joint vs PxPyRz - Jacobian not equal");

  // BOOST_CHECK_MESSAGE(data_composite_planar.Jcom
  //                         .isApprox(data_px_py_pz.Jcom) , "composite planar joint vs PxPyRz - Jacobian com not equal");
  
}

BOOST_AUTO_TEST_SUITE_END ()

