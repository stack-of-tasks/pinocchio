//
// Copyright (c) 2018 INRIA
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

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
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
static void addJointAndBody(se3::Model & model,
                            const se3::JointModelBase<JointModel> & joint,
                            const std::string & parent_name,
                            const std::string & name,
                            const se3::SE3 placement = se3::SE3::Random(),
                            bool setRandomLimits = true)
{
  using namespace se3;
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

BOOST_AUTO_TEST_SUITE( BOOST_TEST_MODULE )
  
BOOST_AUTO_TEST_CASE (test_centroidal_derivatives)
{
  se3::Model model;
  se3::buildModels::humanoidSimple(model);
  se3::Data data(model), data_ref(model);
  
  model.lowerPositionLimit.head<7>().fill(-1.);
  model.upperPositionLimit.head<7>().fill( 1.);
  
  Eigen::VectorXd q = se3::randomConfiguration(model);
  Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
  Eigen::VectorXd a = Eigen::VectorXd::Random(model.nv);
  
  se3::Data::Matrix6x dhdot_dq(6,model.nv), dhdot_dv(6,model.nv), dhdot_da(6,model.nv);
  se3::computeCentroidalDynamicsDerivatives(model,data,q,v,a,
                                            dhdot_dq,dhdot_dv,dhdot_da);
  
  se3::ccrba(model,data_ref,q,v);

  for(size_t k = 0; k < (size_t)model.njoints; ++k)
  {
    BOOST_CHECK(data.oMi[k].isApprox(data_ref.oMi[k]));
    BOOST_CHECK(data.oYcrb[k].isApprox(data_ref.oMi[k].act(data_ref.Ycrb[k])));
  }
  BOOST_CHECK(dhdot_da.isApprox(data_ref.Ag));
  
  se3::computeCentroidalDynamics(model,data_ref,q,v,a);
  BOOST_CHECK(data.hg.isApprox(data_ref.hg));
  BOOST_CHECK(data.dhg.isApprox(data_ref.dhg));
  
  se3::Data data_fd(model);
  
  const double eps = 1e-8;
  const se3::Force dhg = se3::computeCentroidalDynamics(model,data_fd,q,v,a);
  const se3::Force::Vector3 com = data_fd.com[0];
  
  // Check dhdot_dq with finite differences
  Eigen::VectorXd q_plus(model.nq,1);
  Eigen::VectorXd v_eps(model.nv,1); v_eps.setZero();
  se3::Data::Matrix6x dhdot_dq_fd(6,model.nv);
  
  for(Eigen::Index k = 0; k < model.nv; ++k)
  {
    v_eps[k] = eps;
    q_plus = se3::integrate(model,q,v_eps);
    
    const se3::Force & dhg_plus
    = se3::computeCentroidalDynamics(model,data_fd,q_plus,v,a);
    const se3::Force::Vector3 com_plus = data_fd.com[0];
    
    se3::SE3 transform(se3::SE3::Identity());
    transform.translation() = com_plus - com;
    
    dhdot_dq_fd.col(k) = (transform.act(dhg_plus) - dhg).toVector()/eps;
    
    v_eps[k] = 0.;
  }
  
  BOOST_CHECK(dhdot_dq.isApprox(dhdot_dq_fd,sqrt(eps)));
  
  // Check dhdot_dv with finite differences
  Eigen::VectorXd v_plus(v);
  se3::Data::Matrix6x dhdot_dv_fd(6,model.nv);
  
  for(Eigen::Index k = 0; k < model.nv; ++k)
  {
    v_plus[k] += eps;
    
    const se3::Force & dhg_plus
    = se3::computeCentroidalDynamics(model,data_fd,q,v_plus,a);
    dhdot_dv_fd.col(k) = (dhg_plus - dhg).toVector()/eps;
    
    v_plus[k] -= eps;
  }
  
  BOOST_CHECK(dhdot_dv.isApprox(dhdot_dv_fd,sqrt(eps)));
  
  // Check dhdot_da with finite differences
  Eigen::VectorXd a_plus(a);
  se3::Data::Matrix6x dhdot_da_fd(6,model.nv);
  
  for(Eigen::Index k = 0; k < model.nv; ++k)
  {
    a_plus[k] += eps;
    
    const se3::Force & dhg_plus
    = se3::computeCentroidalDynamics(model,data_fd,q,v,a_plus);
    dhdot_da_fd.col(k) = (dhg_plus - dhg).toVector()/eps;
    
    a_plus[k] -= eps;
  }
  
  BOOST_CHECK(dhdot_da.isApprox(dhdot_da_fd,sqrt(eps)));
  
}

BOOST_AUTO_TEST_CASE (test_retrieve_centroidal_derivatives)
{
  se3::Model model;
  se3::buildModels::humanoidSimple(model);
  se3::Data data(model), data_ref(model);
  
  model.lowerPositionLimit.head<7>().fill(-1.);
  model.upperPositionLimit.head<7>().fill( 1.);
  
  Eigen::VectorXd q = se3::randomConfiguration(model);
  Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
  Eigen::VectorXd a = Eigen::VectorXd::Random(model.nv);
  
  se3::Data::Matrix6x dhdot_dq(6,model.nv), dhdot_dv(6,model.nv), dhdot_da(6,model.nv);
  se3::Data::Matrix6x dhdot_dq_ref(6,model.nv), dhdot_dv_ref(6,model.nv), dhdot_da_ref(6,model.nv);
  
  se3::computeCentroidalDynamicsDerivatives(model,data_ref,q,v,a,
                                            dhdot_dq_ref,dhdot_dv_ref,dhdot_da_ref);
  
  se3::computeRNEADerivatives(model,data,q,v,a);
  se3::getCentroidalDynamicsDerivatives(model,data,
                                        dhdot_dq,dhdot_dv,dhdot_da);
  
  BOOST_CHECK(data.J.isApprox(data_ref.J));
  
  for(se3::Model::JointIndex k = 1; k < (se3::Model::JointIndex)model.njoints; ++k)
  {
    BOOST_CHECK(data.oYcrb[k].isApprox(data_ref.oYcrb[k]));
    se3::Force force_ref = data_ref.of[k];
    se3::Force gravity_contribution = data.oYcrb[k] * (-model.gravity);
    se3::Force force = data.of[k] - gravity_contribution;
    BOOST_CHECK(force.isApprox(force_ref));
  }
  
  BOOST_CHECK(data.com[0].isApprox(data_ref.com[0]));
  
  BOOST_CHECK(data.hg.isApprox(data_ref.hg));
  BOOST_CHECK(data.dhg.isApprox(data_ref.dhg));
  
  BOOST_CHECK(data.Fcrb[0].isApprox(data_ref.dFdq));
  BOOST_CHECK(data.dFdv.isApprox(data_ref.dFdv));
  BOOST_CHECK(data.dFda.isApprox(data_ref.dFda));
  
  BOOST_CHECK(dhdot_dq.isApprox(dhdot_dq_ref));
  BOOST_CHECK(dhdot_dv.isApprox(dhdot_dv_ref));
  BOOST_CHECK(dhdot_da.isApprox(dhdot_da_ref));
  
}

BOOST_AUTO_TEST_SUITE_END()
