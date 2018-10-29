//
// Copyright (c) 2015-2018 CNRS
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
  
BOOST_AUTO_TEST_CASE (test_ccrba)
{
  se3::Model model;
  se3::buildModels::humanoidRandom(model);
  se3::Data data(model), data_ref(model);
  
  Eigen::VectorXd q = Eigen::VectorXd::Ones(model.nq);
  q.segment <4> (3).normalize();
  Eigen::VectorXd v = Eigen::VectorXd::Ones(model.nv);
  
  crba(model,data_ref,q);
  data_ref.M.triangularView<Eigen::StrictlyLower>() = data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();
  data_ref.Ycrb[0] = data_ref.liMi[1].act(data_ref.Ycrb[1]);
  
  se3::SE3 cMo (se3::SE3::Matrix3::Identity(), -getComFromCrba(model, data_ref));
  
  ccrba(model, data, q, v);
  BOOST_CHECK(data.com[0].isApprox(-cMo.translation(),1e-12));
  BOOST_CHECK(data.Ycrb[0].matrix().isApprox(data_ref.Ycrb[0].matrix(),1e-12));
  
  se3::Inertia Ig_ref (cMo.act(data.Ycrb[0]));
  BOOST_CHECK(data.Ig.matrix().isApprox(Ig_ref.matrix(),1e-12));
  
  se3::SE3 oM1 (data_ref.liMi[1]);
  se3::SE3 cM1 (cMo * oM1);
  
  se3::Data::Matrix6x Ag_ref (cM1.inverse().toActionMatrix().transpose() * data_ref.M.topRows <6> ());
  BOOST_CHECK(data.Ag.isApprox(Ag_ref,1e-12));
}
  
BOOST_AUTO_TEST_CASE (test_dccrb)
{
  using namespace se3;
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
  
  SE3::Vector3 com = data_ref.Ycrb[1].lever();
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
}

BOOST_AUTO_TEST_SUITE_END()
