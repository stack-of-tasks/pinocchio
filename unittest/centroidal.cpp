//
// Copyright (c) 2015-2019 CNRS INRIA
//

#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include "pinocchio/parsers/sample-models.hpp"

#include "pinocchio/utils/timer.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

template<typename JointModel>
static void addJointAndBody(pinocchio::Model & model,
                            const pinocchio::JointModelBase<JointModel> & joint,
                            const std::string & parent_name,
                            const std::string & name,
                            const pinocchio::SE3 placement = pinocchio::SE3::Random(),
                            bool setRandomLimits = true)
{
  using namespace pinocchio;
  typedef typename JointModel::ConfigVector_t CV;
  typedef typename JointModel::TangentVector_t TV;
  
  Model::JointIndex idx;
  
  if(setRandomLimits)
    idx = model.addJoint(model.getJointId(parent_name),joint,
                         SE3::Random(),
                         name + "_joint",
                         TV::Random() + TV::Constant(1),
                         TV::Random() + TV::Constant(1),
                         CV::Random() - CV::Constant(1),
                         CV::Random() + CV::Constant(1)
                         );
  else
    idx = model.addJoint(model.getJointId(parent_name),joint,
                         placement, name + "_joint");
  
  model.addJointFrame(idx);
  
  model.appendBodyToJoint(idx,Inertia::Random(),SE3::Identity());
  model.addBodyFrame(name + "_body", idx);
}

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)
  
BOOST_AUTO_TEST_CASE(test_ccrba)
{
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model);
  pinocchio::Data data(model), data_ref(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill( 1.);
  Eigen::VectorXd q = randomConfiguration(model);
  Eigen::VectorXd v = Eigen::VectorXd::Ones(model.nv);
  
  crba(model,data_ref,q);
  data_ref.M.triangularView<Eigen::StrictlyLower>() = data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();
  data_ref.Ycrb[0] = data_ref.liMi[1].act(data_ref.Ycrb[1]);
  
  pinocchio::SE3 cMo (pinocchio::SE3::Matrix3::Identity(), -getComFromCrba(model, data_ref));
  
  ccrba(model, data, q, v);
  BOOST_CHECK(data.com[0].isApprox(-cMo.translation(),1e-12));
  BOOST_CHECK(data.oYcrb[0].matrix().isApprox(data_ref.Ycrb[0].matrix(),1e-12));
  
  pinocchio::Inertia Ig_ref (cMo.act(data.oYcrb[0]));
  BOOST_CHECK(data.Ig.matrix().isApprox(Ig_ref.matrix(),1e-12));
  
  pinocchio::SE3 oM1 (data_ref.liMi[1]);
  pinocchio::SE3 cM1 (cMo * oM1);
  
  pinocchio::Data::Matrix6x Ag_ref (cM1.inverse().toActionMatrix().transpose() * data_ref.M.topRows <6> ());
  BOOST_CHECK(data.Ag.isApprox(Ag_ref,1e-12));
}

BOOST_AUTO_TEST_CASE(test_centroidal_mapping)
{
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model);
  pinocchio::Data data(model), data_ref(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill( 1.);
  Eigen::VectorXd q = randomConfiguration(model);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
  
  computeCentroidalMap(model, data, q);
  ccrba(model,data_ref,q,v);
  
  BOOST_CHECK(data.J.isApprox(data_ref.J));
  BOOST_CHECK(data.Ag.isApprox(data_ref.Ag));
  
  computeJointJacobians(model,data_ref,q);
  
  BOOST_CHECK(data.J.isApprox(data_ref.J));
}
  
BOOST_AUTO_TEST_CASE(test_dccrb)
{
  using namespace pinocchio;
  Model model;
  buildModels::humanoidRandom(model);
  addJointAndBody(model,JointModelSpherical(),"larm6_joint","larm7");
  Data data(model), data_ref(model);
  
  model.lowerPositionLimit.head<7>().fill(-1.);
  model.upperPositionLimit.head<7>().fill(1.);
  
  Eigen::VectorXd q = randomConfiguration(model,model.lowerPositionLimit,model.upperPositionLimit);
  Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
  Eigen::VectorXd a = Eigen::VectorXd::Random(model.nv);
  
  const Eigen::VectorXd g = rnea(model,data_ref,q,0*v,0*a);
  rnea(model,data_ref,q,v,a);
  
  crba(model,data_ref,q);
  data_ref.M.triangularView<Eigen::StrictlyLower>() = data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();
  
  SE3 cMo(SE3::Identity());
  cMo.translation() = -getComFromCrba(model, data_ref);
  
  SE3 oM1 (data_ref.liMi[1]);
  SE3 cM1 (cMo * oM1);
  Data::Matrix6x Ag_ref (cM1.toDualActionMatrix() * data_ref.M.topRows <6> ());
  
  Force hdot_ref(cM1.act(Force(data_ref.tau.head<6>() - g.head<6>())));
  
  ccrba(model,data_ref,q,v);
  dccrba(model,data,q,v);
  BOOST_CHECK(data.Ag.isApprox(Ag_ref));
  BOOST_CHECK(data.Ag.isApprox(data_ref.Ag));
  BOOST_CHECK(data.hg.isApprox(data_ref.hg));
  
  centerOfMass(model,data_ref,q,v,a);
  BOOST_CHECK(data_ref.vcom[0].isApprox(data.hg.linear()/data_ref.M(0,0)));
  BOOST_CHECK(data_ref.vcom[0].isApprox(data.vcom[0]));
  BOOST_CHECK(data_ref.acom[0].isApprox(hdot_ref.linear()/data_ref.M(0,0)));
  
  Force hdot(data.Ag * a + data.dAg * v);
  BOOST_CHECK(hdot.isApprox(hdot_ref));
  
  dccrba(model,data,q,0*v);
  BOOST_CHECK(data.dAg.isZero());
  
  // Check that dYcrb is equal to doYcrb
  {
    // Compute dYcrb
    Data data_ref(model), data_ref_plus(model), data(model);
    
    const double alpha = 1e-8;
    Eigen::VectorXd q_plus(model.nq);
    q_plus = integrate(model,q,alpha*v);
    
    forwardKinematics(model,data_ref,q);
    crba(model,data_ref,q);
    crba(model,data_ref_plus,q_plus);
    forwardKinematics(model,data_ref_plus,q_plus);
    dccrba(model,data,q,v);
    
    for(size_t i = 1; i < (size_t)model.njoints; ++i)
    {
      Inertia::Matrix6 dYcrb = (data_ref_plus.oMi[i].act(data_ref_plus.Ycrb[i]).matrix() -
                                data_ref.oMi[i].act(data_ref.Ycrb[i]).matrix())/alpha;
      BOOST_CHECK(data.doYcrb[i].isApprox(dYcrb,sqrt(alpha)));
    }
  }
  
  {
    Data data(model);
    ccrba(model,data_ref,q,v);
    SE3 oMc_ref(SE3::Identity());
    oMc_ref.translation() = data_ref.com[0];
    const Data::Matrix6x Ag_ref = oMc_ref.toDualActionMatrix() * data_ref.Ag;
    crba(model,data_ref,q);
    const Data::Matrix6x Ag_ref_from_M = data_ref.oMi[1].toDualActionMatrix() * data_ref.M.topRows<6>();
    
    const double alpha = 1e-8;
    Eigen::VectorXd q_plus(model.nq);
    q_plus = integrate(model,q,alpha*v);
    ccrba(model,data_ref,q_plus,v);
    SE3 oMc_ref_plus(SE3::Identity());
    oMc_ref_plus.translation() = data_ref.com[0];
    const Data::Matrix6x Ag_plus_ref = oMc_ref_plus.toDualActionMatrix() * data_ref.Ag;
    crba(model,data_ref,q_plus);
    const Data::Matrix6x Ag_plus_ref_from_M = data_ref.oMi[1].toDualActionMatrix() * data_ref.M.topRows<6>();
    const Data::Matrix6x dAg_ref = (Ag_plus_ref - Ag_ref)/alpha;
    const Data::Matrix6x dAg_ref_from_M = (Ag_plus_ref_from_M - Ag_ref_from_M)/alpha;
    
    dccrba(model, data, q, v);
    SE3 oMc(SE3::Identity());
    oMc.translation() = data.com[0];
    Data::Matrix6x dAg = oMc.toDualActionMatrix() * data.dAg;
    BOOST_CHECK(oMc.isApprox(oMc_ref));
    BOOST_CHECK(dAg.isApprox(dAg_ref,sqrt(alpha)));
    BOOST_CHECK(dAg.isApprox(dAg_ref_from_M,sqrt(alpha)));
  }
  
  // Compute tensor dAg/dq
  {
    std::vector<Data::Matrix6x> dAgdq((size_t)model.nv,Data::Matrix6x::Zero(6,model.nv));
    Data data(model), data_fd(model);
    Eigen::VectorXd v_fd(Eigen::VectorXd::Zero(model.nv));
    ccrba(model,data_fd,q,v);
    SE3 oMc_ref(SE3::Identity());
    oMc_ref.translation() = data_fd.com[0];
    
    Data::Matrix6x Ag0 = oMc_ref.toDualActionMatrix() * data_fd.Ag;
    const Force hg0 = oMc_ref.act(data_fd.hg);
    
    Data::Matrix6x Ag_fd(6,model.nv);
    Force hg_fd;
    const double alpha = 1e-8;
    Eigen::VectorXd q_plus(model.nq);
    Data::Matrix6x dhdq(6,model.nv);
    for(int k = 0; k < model.nv; ++k)
    {
      v_fd[k] = alpha;
      q_plus = integrate(model,q,v_fd);
      ccrba(model,data_fd,q_plus,v);
      SE3 oMc_fd(SE3::Identity());
      oMc_fd.translation() = data_fd.com[0];
      Ag_fd = oMc_fd.toDualActionMatrix() * data_fd.Ag;
      hg_fd = oMc_fd.act(data_fd.hg);
      dAgdq[(size_t)k] = (Ag_fd - Ag0)/alpha;
      dhdq.col(k) = (hg_fd - hg0).toVector()/alpha;
      v_fd[k] = 0.;
    }
    
    Data::Matrix6x dAg_ref(6,model.nv); dAg_ref.setZero();
    for(int k = 0; k < model.nv; ++k)
    {
      dAg_ref += dAgdq[(size_t)k] * v[k];
    }
    
    Data::Matrix6x dAg_ref_bis(6,model.nv); dAg_ref_bis.setZero();
    for(int k = 0; k < model.nv; ++k)
    {
      dAg_ref_bis.col(k) = dAgdq[(size_t)k] * v;
    }
    
    dccrba(model, data, q, v);
    SE3 oMc(SE3::Identity());
    oMc.translation() = data.com[0];
    Data::Matrix6x dAg = oMc.toDualActionMatrix() * data.dAg;
    BOOST_CHECK(dAg.isApprox(dAg_ref,sqrt(alpha)));
    BOOST_CHECK(dhdq.isApprox(dAg_ref_bis,sqrt(alpha)));
    BOOST_CHECK((dAg*v).isApprox(dhdq*v,sqrt(alpha)));
    
  }
}

BOOST_AUTO_TEST_CASE(test_centroidal_mapping_time_derivative)
{
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model);
  pinocchio::Data data(model), data_ref(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill( 1.);
  Eigen::VectorXd q = randomConfiguration(model);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
  
  computeCentroidalMapTimeVariation(model, data, q, v);
  dccrba(model,data_ref,q,v);
  
  BOOST_CHECK(data.J.isApprox(data_ref.J));
  BOOST_CHECK(data.dJ.isApprox(data_ref.dJ));
  BOOST_CHECK(data.Ag.isApprox(data_ref.Ag));
  BOOST_CHECK(data.dAg.isApprox(data_ref.dAg));
  
  computeJointJacobiansTimeVariation(model,data_ref,q,v);
  
  BOOST_CHECK(data.J.isApprox(data_ref.J));
  BOOST_CHECK(data.dJ.isApprox(data_ref.dJ));
}

BOOST_AUTO_TEST_CASE(test_computeCentroidalMomentum_computeCentroidalMomentumTimeVariation)
{
  using namespace pinocchio;
  Model model;
  buildModels::humanoidRandom(model);
  addJointAndBody(model,JointModelSpherical(),"larm6_joint","larm7");
  Data data(model), data_fk1(model), data_fk2(model), data_ref(model);
  
  model.lowerPositionLimit.head<7>().fill(-1.);
  model.upperPositionLimit.head<7>().fill( 1.);
  
  Eigen::VectorXd q = randomConfiguration(model,model.lowerPositionLimit,model.upperPositionLimit);
  Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
  Eigen::VectorXd a = Eigen::VectorXd::Random(model.nv);
  
  ccrba(model,data_ref,q,v);
  forwardKinematics(model,data_ref,q,v);
  centerOfMass(model,data_ref,q,v,false);
  computeCentroidalMomentum(model,data,q,v);
  
  BOOST_CHECK(data.mass[0] == data_ref.mass[0]);
  BOOST_CHECK(data.com[0].isApprox(data_ref.com[0]));
  BOOST_CHECK(data.hg.isApprox(data_ref.hg));
  for(size_t k = 1; k < (size_t)model.njoints; ++k)
  {
    BOOST_CHECK(data.mass[k] == data_ref.mass[k]);
    BOOST_CHECK(data.com[k].isApprox(data_ref.com[k]));
    BOOST_CHECK(data.v[k].isApprox(data_ref.v[k]));
  }
  
  // Check other signature
  forwardKinematics(model,data_fk1,q,v);
  computeCentroidalMomentum(model,data_fk1);
  
  BOOST_CHECK(data_fk1.hg.isApprox(data.hg));
  
  computeCentroidalMomentumTimeVariation(model,data,q,v,a);
  model.gravity.setZero();
  rnea(model,data_ref,q,v,a);
  dccrba(model,data_ref,q,v);
  const Force hgdot(data_ref.Ag * a + data_ref.dAg * v);
  
  BOOST_CHECK(data.mass[0] == data_ref.mass[0]);
  BOOST_CHECK(data.com[0].isApprox(data_ref.com[0]));
  BOOST_CHECK(data.hg.isApprox(data_ref.hg));
  BOOST_CHECK(data.dhg.isApprox(hgdot));
  for(size_t k = 1; k < (size_t)model.njoints; ++k)
  {
    BOOST_CHECK(data.mass[k] == data_ref.mass[k]);
    BOOST_CHECK(data.com[k].isApprox(data_ref.com[k]));
    BOOST_CHECK(data.v[k].isApprox(data_ref.v[k]));
    BOOST_CHECK(data.a[k].isApprox(data_ref.a_gf[k]));
    BOOST_CHECK(data.f[k].isApprox(data_ref.f[k]));
  }
  
  // Check other signature
  forwardKinematics(model,data_fk2,q,v,a);
  computeCentroidalMomentumTimeVariation(model,data_fk2);
  
  BOOST_CHECK(data_fk2.hg.isApprox(data.hg));
  BOOST_CHECK(data_fk2.dhg.isApprox(data.dhg));
  
  // Check against finite differences
  Data data_fd(model);
  const double eps = 1e-8;
  Eigen::VectorXd v_plus = v + eps * a;
  Eigen::VectorXd q_plus = integrate(model,q,eps*v);
  
  const Force hg = computeCentroidalMomentum(model,data_fd,q,v);
  const SE3::Vector3 com = data_fd.com[0];
  const Force hg_plus = computeCentroidalMomentum(model,data_fd,q_plus,v_plus);
  const SE3::Vector3 com_plus = data_fd.com[0];
  
  SE3 transform(SE3::Identity());
  transform.translation() = com_plus - com;
  Force dhg_ref = (transform.act(hg_plus) - hg)/eps;

  BOOST_CHECK(data.dhg.isApprox(dhg_ref,sqrt(eps)));
}

BOOST_AUTO_TEST_SUITE_END()
