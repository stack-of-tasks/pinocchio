//
// Copyright (c) 2015-2020 CNRS INRIA
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#include "pinocchio/math/fwd.hpp"
#include "pinocchio/multibody/joint/joints.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"

#include <boost/test/unit_test.hpp>
#include <iostream>

using namespace pinocchio;

template<typename D>
void addJointAndBody(Model & model,
                     const JointModelBase<D> & jmodel,
                     const Model::JointIndex parent_id,
                     const SE3 & joint_placement,
                     const std::string & joint_name,
                     const Inertia & Y)
{
  Model::JointIndex idx;
  
  idx = model.addJoint(parent_id,jmodel,joint_placement,joint_name);
  model.appendBodyToJoint(idx,Y);
}

BOOST_AUTO_TEST_SUITE(JointPlanar)
  
BOOST_AUTO_TEST_CASE(spatial)
{
  SE3 M(SE3::Random());
  Motion v(Motion::Random());
  
  MotionPlanar mp(1.,2.,3.);
  Motion mp_dense(mp);
  
  BOOST_CHECK(M.act(mp).isApprox(M.act(mp_dense)));
  BOOST_CHECK(M.actInv(mp).isApprox(M.actInv(mp_dense)));
  
  BOOST_CHECK(v.cross(mp).isApprox(v.cross(mp_dense)));
}

BOOST_AUTO_TEST_CASE(vsFreeFlyer)
{
  using namespace pinocchio;
  typedef SE3::Vector3 Vector3;
  typedef Eigen::Matrix <double, 6, 1> Vector6;
  typedef Eigen::Matrix <double, 4, 1> VectorPl;
  typedef Eigen::Matrix <double, 7, 1> VectorFF;
  typedef SE3::Matrix3 Matrix3;

  Model modelPlanar, modelFreeflyer;

  Inertia inertia(1., Vector3(0.5, 0., 0.0), Matrix3::Identity());
  SE3 pos(1); pos.translation() = SE3::LinearType(1.,0.,0.);

  addJointAndBody(modelPlanar,JointModelPlanar(),0,SE3::Identity(),"planar",inertia);
  addJointAndBody(modelFreeflyer,JointModelFreeFlyer(),0,SE3::Identity(),"free-flyer",inertia);

  Data dataPlanar(modelPlanar);
  Data dataFreeFlyer(modelFreeflyer);

  VectorPl q; q << 1, 1, 0, 1; // Angle is PI /2;
  VectorFF qff; qff << 1, 1, 0, 0, 0, sqrt(2)/2, sqrt(2)/2 ;
  Eigen::VectorXd v = Eigen::VectorXd::Ones(modelPlanar.nv);
  Vector6 vff; vff << 1, 1, 0, 0, 0, 1;
  Eigen::VectorXd tauPlanar = Eigen::VectorXd::Ones(modelPlanar.nv);
  Eigen::VectorXd tauff = Eigen::VectorXd::Ones(modelFreeflyer.nv);
  Eigen::VectorXd aPlanar = Eigen::VectorXd::Ones(modelPlanar.nv);
  Eigen::VectorXd aff(vff);
  
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
  computeJointJacobians(modelPlanar, dataPlanar, q);
  computeJointJacobians(modelFreeflyer, dataFreeFlyer, qff);
  getJointJacobian(modelPlanar, dataPlanar, 1, LOCAL, jacobian_planar);
  getJointJacobian(modelFreeflyer, dataFreeFlyer, 1, LOCAL, jacobian_ff);

  Eigen::Matrix<double, 6, 3> jacobian_expected; jacobian_expected << jacobian_ff.col(0),
                                                                      jacobian_ff.col(1),
                                                                      jacobian_ff.col(5)
                                                                      ;

  BOOST_CHECK(jacobian_planar.isApprox(jacobian_expected));
}

BOOST_AUTO_TEST_SUITE_END()
