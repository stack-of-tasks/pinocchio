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

BOOST_AUTO_TEST_SUITE(JointTranslation)
  
BOOST_AUTO_TEST_CASE(spatial)
{
  typedef TransformTranslationTpl<double,0> TransformTranslation;
  typedef SE3::Vector3 Vector3;
  
  const Vector3 displacement(Vector3::Random());
  SE3 Mplain, Mrand(SE3::Random());
  
  TransformTranslation Mtrans(displacement);
  Mplain = Mtrans;
  BOOST_CHECK(Mplain.translation().isApprox(displacement));
  BOOST_CHECK(Mplain.rotation().isIdentity());
  BOOST_CHECK((Mrand*Mplain).isApprox(Mrand*Mtrans));
  
  SE3 M(SE3::Random());
  Motion v(Motion::Random());
  
  MotionTranslation mp(MotionTranslation::Vector3(1.,2.,3.));
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
  typedef Eigen::Matrix <double, 7, 1> VectorFF;
  typedef SE3::Matrix3 Matrix3;

  Model modelTranslation, modelFreeflyer;

  Inertia inertia(1., Vector3(0.5, 0., 0.0), Matrix3::Identity());
  SE3 pos(1); pos.translation() = SE3::LinearType(1.,0.,0.);

  addJointAndBody(modelTranslation,JointModelTranslation(),0,SE3::Identity(),"translation",inertia);
  addJointAndBody(modelFreeflyer,JointModelFreeFlyer(),0,SE3::Identity(),"free-flyer",inertia);

  Data dataTranslation(modelTranslation);
  Data dataFreeFlyer(modelFreeflyer);

  Eigen::VectorXd q = Eigen::VectorXd::Ones(modelTranslation.nq);
  VectorFF qff; qff << 1, 1, 1, 0, 0, 0, 1 ;
  Eigen::VectorXd v = Eigen::VectorXd::Ones(modelTranslation.nv);
  Vector6 vff; vff << 1, 1, 1, 0, 0, 0;
  Eigen::VectorXd tauTranslation = Eigen::VectorXd::Ones(modelTranslation.nv);
  Eigen::VectorXd tauff(6); tauff << 1, 1, 1, 0, 0, 0;
  Eigen::VectorXd aTranslation = Eigen::VectorXd::Ones(modelTranslation.nv);
  Eigen::VectorXd aff(vff);
  
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
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_translation;jacobian_translation.resize(6,3); jacobian_translation.setZero();
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_ff;jacobian_ff.resize(6,6);jacobian_ff.setZero();
  computeJointJacobians(modelTranslation, dataTranslation, q);
  computeJointJacobians(modelFreeflyer, dataFreeFlyer, qff);
  getJointJacobian(modelTranslation, dataTranslation, 1, LOCAL, jacobian_translation);
  getJointJacobian(modelFreeflyer, dataFreeFlyer, 1, LOCAL, jacobian_ff);

  Eigen::Matrix<double, 6, 3> jacobian_expected; jacobian_expected << jacobian_ff.col(0),
                                                                      jacobian_ff.col(1),
                                                                      jacobian_ff.col(2)
                                                                      ;
  BOOST_CHECK(jacobian_translation.isApprox(jacobian_expected));
}

BOOST_AUTO_TEST_SUITE_END()
