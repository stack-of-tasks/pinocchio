//
// Copyright (c) 2018-2019 INRIA
//

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/joint/joint-spherical.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/centroidal-derivatives.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
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
  
  if (setRandomLimits)
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
  
BOOST_AUTO_TEST_CASE(test_centroidal_derivatives)
{
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model);
  const std::string parent_name = model.names.back();
  const std::string joint_name = "ee_spherical_joint";
  addJointAndBody(model, pinocchio::JointModelSpherical(), parent_name , joint_name);
  
  pinocchio::Data data(model), data_ref(model);
  
  model.lowerPositionLimit.head<7>().fill(-1.);
  model.upperPositionLimit.head<7>().fill( 1.);
  
  Eigen::VectorXd q = pinocchio::randomConfiguration(model);
  Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
  Eigen::VectorXd a = Eigen::VectorXd::Random(model.nv);
  
  pinocchio::Data::Matrix6x
    dh_dq(6,model.nv),dhdot_dq(6,model.nv), dhdot_dv(6,model.nv), dhdot_da(6,model.nv);
  pinocchio::computeCentroidalDynamicsDerivatives(model,data,q,v,a,
                                                  dh_dq,dhdot_dq,dhdot_dv,dhdot_da);
  pinocchio::ccrba(model,data_ref,q,v);

  for(size_t k = 0; k < (size_t)model.njoints; ++k)
  {
    BOOST_CHECK(data.oMi[k].isApprox(data_ref.oMi[k]));
    BOOST_CHECK(data.oYcrb[k].isApprox(data_ref.oYcrb[k]));
  }
  BOOST_CHECK(dhdot_da.isApprox(data_ref.Ag));
  
  pinocchio::computeCentroidalMomentumTimeVariation(model,data_ref,q,v,a);
  for(size_t k = 1; k < (size_t)model.njoints; ++k)
  {
    BOOST_CHECK(data.v[k].isApprox(data_ref.v[k]));
    BOOST_CHECK(data.ov[k].isApprox(data.oMi[k].act(data_ref.v[k])));
    BOOST_CHECK(data.oa[k].isApprox(data.oMi[k].act(data_ref.a[k])));
    BOOST_CHECK(data.oh[k].isApprox(data.oMi[k].act(data_ref.h[k])));
  }

  BOOST_CHECK(data.mass[0] == data_ref.mass[0]);
  BOOST_CHECK(data.com[0].isApprox(data_ref.com[0]));
  
  BOOST_CHECK(data.oh[0].isApprox(data_ref.h[0]));
  BOOST_CHECK(data.of[0].isApprox(data_ref.f[0]));

  BOOST_CHECK(data.hg.isApprox(data_ref.hg));
  BOOST_CHECK(data.dhg.isApprox(data_ref.dhg));

  pinocchio::Data data_fd(model);
  
  const double eps = 1e-8;
  const pinocchio::Force dhg = pinocchio::computeCentroidalMomentumTimeVariation(model,data_fd,q,v,a);
  const pinocchio::Force hg = data_fd.hg;
  BOOST_CHECK(data.hg.isApprox(data_ref.hg));
  const pinocchio::Force::Vector3 com = data_fd.com[0];
  
  // Check dhdot_dq and dh_dq with finite differences
  Eigen::VectorXd q_plus(model.nq,1);
  Eigen::VectorXd v_eps(model.nv,1); v_eps.setZero();
  pinocchio::Data::Matrix6x dhdot_dq_fd(6,model.nv);
  pinocchio::Data::Matrix6x dh_dq_fd(6,model.nv);
  
  for(Eigen::DenseIndex k = 0; k < model.nv; ++k)
  {
    v_eps[k] = eps;
    q_plus = pinocchio::integrate(model,q,v_eps);
    
    const pinocchio::Force & dhg_plus
    = pinocchio::computeCentroidalMomentumTimeVariation(model,data_fd,q_plus,v,a);
    const pinocchio::Force hg_plus = data_fd.hg;
    const pinocchio::Force::Vector3 com_plus = data_fd.com[0];
    
    pinocchio::SE3 transform(pinocchio::SE3::Identity());
    transform.translation() = com_plus - com;
    
    dhdot_dq_fd.col(k) = (transform.act(dhg_plus) - dhg).toVector()/eps;
    dh_dq_fd.col(k) = (transform.act(hg_plus) - hg).toVector()/eps;
    v_eps[k] = 0.;
  }
  
  BOOST_CHECK(dhdot_dq.isApprox(dhdot_dq_fd,sqrt(eps)));
  BOOST_CHECK(dh_dq.isApprox(dh_dq_fd,sqrt(eps)));
  // Check dhdot_dv with finite differences
  Eigen::VectorXd v_plus(v);
  pinocchio::Data::Matrix6x dhdot_dv_fd(6,model.nv);
  
  for(Eigen::DenseIndex k = 0; k < model.nv; ++k)
  {
    v_plus[k] += eps;
    
    const pinocchio::Force & dhg_plus
    = pinocchio::computeCentroidalMomentumTimeVariation(model,data_fd,q,v_plus,a);
    dhdot_dv_fd.col(k) = (dhg_plus - dhg).toVector()/eps;
    
    v_plus[k] -= eps;
  }
  
  BOOST_CHECK(dhdot_dv.isApprox(dhdot_dv_fd,sqrt(eps)));
  
  // Check dhdot_da with finite differences
  Eigen::VectorXd a_plus(a);
  pinocchio::Data::Matrix6x dhdot_da_fd(6,model.nv);
  
  for(Eigen::DenseIndex k = 0; k < model.nv; ++k)
  {
    a_plus[k] += eps;
    
    const pinocchio::Force & dhg_plus
    = pinocchio::computeCentroidalMomentumTimeVariation(model,data_fd,q,v,a_plus);
    dhdot_da_fd.col(k) = (dhg_plus - dhg).toVector()/eps;
    
    a_plus[k] -= eps;
  }
  
  BOOST_CHECK(dhdot_da.isApprox(dhdot_da_fd,sqrt(eps)));
  
  pinocchio::computeRNEADerivatives(model,data_ref,q,v,a);
  BOOST_CHECK(data.dAdv.isApprox(data_ref.dAdv));
  BOOST_CHECK(data.dAdq.isApprox(data_ref.dAdq));
  BOOST_CHECK(data.J.isApprox(data_ref.J));
  BOOST_CHECK(data.dJ.isApprox(data_ref.dJ));
  BOOST_CHECK(data.dVdq.isApprox(data_ref.dVdq));
}

BOOST_AUTO_TEST_CASE(test_retrieve_centroidal_derivatives)
{
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model);
  pinocchio::Data data(model), data_ref(model);
  
  model.lowerPositionLimit.head<7>().fill(-1.);
  model.upperPositionLimit.head<7>().fill( 1.);
  
  Eigen::VectorXd q = pinocchio::randomConfiguration(model);
  Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
  Eigen::VectorXd a = Eigen::VectorXd::Random(model.nv);
  
  pinocchio::Data::Matrix6x
    dh_dq(6,model.nv), dhdot_dq(6,model.nv), dhdot_dv(6,model.nv), dhdot_da(6,model.nv);
  pinocchio::Data::Matrix6x
    dh_dq_ref(6,model.nv), dhdot_dq_ref(6,model.nv), dhdot_dv_ref(6,model.nv), dhdot_da_ref(6,model.nv);
  
  pinocchio::computeCentroidalDynamicsDerivatives(model,data_ref,q,v,a,
                                                  dh_dq_ref, dhdot_dq_ref,dhdot_dv_ref,dhdot_da_ref);
  
  pinocchio::computeRNEADerivatives(model,data,q,v,a);
  pinocchio::getCentroidalDynamicsDerivatives(model,data,
                                              dh_dq, dhdot_dq,dhdot_dv,dhdot_da);
  
  BOOST_CHECK(data.J.isApprox(data_ref.J));
  
  for(pinocchio::Model::JointIndex k = 1; k < (pinocchio::Model::JointIndex)model.njoints; ++k)
  {
    BOOST_CHECK(data.oYcrb[k].isApprox(data_ref.oYcrb[k]));
    pinocchio::Force force_ref = data_ref.of[k];
    pinocchio::Force gravity_contribution = data.oYcrb[k] * (-model.gravity);
    pinocchio::Force force = data.of[k] - gravity_contribution;
    BOOST_CHECK(force.isApprox(force_ref));
  }
  
  BOOST_CHECK(data.com[0].isApprox(data_ref.com[0]));
  
  BOOST_CHECK(data.hg.isApprox(data_ref.hg));
  BOOST_CHECK(data.dhg.isApprox(data_ref.dhg));

  BOOST_CHECK(data.Fcrb[0].isApprox(data_ref.dFdq));
  BOOST_CHECK(data.dFdv.isApprox(data_ref.dFdv));
  BOOST_CHECK(data.dFda.isApprox(data_ref.dFda));
  BOOST_CHECK(dh_dq.isApprox(dh_dq_ref));
  BOOST_CHECK(dhdot_dq.isApprox(dhdot_dq_ref));
  BOOST_CHECK(dhdot_dv.isApprox(dhdot_dv_ref));
  BOOST_CHECK(dhdot_da.isApprox(dhdot_da_ref));
  
}

BOOST_AUTO_TEST_SUITE_END()
