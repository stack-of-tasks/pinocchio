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

/*
 * Unittest of the RNE algorithm. The code simply test that the algorithm does
 * not cause any serious errors. The numerical values are not cross validated
 * in any way.
 *
 */

#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/utils/timer.hpp"

#include <iostream>

//#define __SSE3__
#include <fenv.h>

#ifdef __SSE3__
#include <pmmintrin.h>
#endif

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE ( test_rnea )
{
  #ifdef __SSE3__
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
  #endif
  using namespace Eigen;
  using namespace se3;

  se3::Model model; buildModels::humanoidRandom(model);
  
  se3::Data data(model);
  data.v[0] = Motion::Zero();
  data.a[0] = -model.gravity;

  VectorXd q = VectorXd::Random(model.nq);
  VectorXd v = VectorXd::Random(model.nv);
  VectorXd a = VectorXd::Random(model.nv);

  #ifdef NDEBUG
    const size_t NBT = 10000;
  #else
    const size_t NBT = 1;
    std::cout << "(the time score in debug mode is not relevant)  " ;
  #endif

  PinocchioTicToc timer(PinocchioTicToc::US); timer.tic();
  SMOOTH(NBT)
    {
      rnea(model,data,q,v,a);
    }
  timer.toc(std::cout,NBT);

}
  
BOOST_AUTO_TEST_CASE ( test_nle_vs_rnea )
{
  using namespace Eigen;
  using namespace se3;
  
  se3::Model model; buildModels::humanoidRandom(model);
  se3::Data data_nle(model);
  se3::Data data_rnea(model);
  
  VectorXd q (VectorXd::Random(model.nq));
  VectorXd v (VectorXd::Random(model.nv));
  
  VectorXd tau_nle (VectorXd::Zero (model.nv));
  VectorXd tau_rnea (VectorXd::Zero (model.nv));
  
  // -------
  q.setZero ();
  v.setZero ();
  
  tau_nle = nonLinearEffects(model,data_nle,q,v);
  tau_rnea = rnea(model,data_rnea,q,v,VectorXd::Zero (model.nv));
  
  BOOST_CHECK (tau_nle.isApprox(tau_rnea, 1e-12));
  
  // -------
  q.setZero ();
  v.setOnes ();
  
  tau_nle = nonLinearEffects(model,data_nle,q,v);
  tau_rnea = rnea(model,data_rnea,q,v,VectorXd::Zero (model.nv));
  
  BOOST_CHECK (tau_nle.isApprox(tau_rnea, 1e-12));
  
  // -------
  q.setOnes ();
  v.setOnes ();
  
  tau_nle = nonLinearEffects(model,data_nle,q,v);
  tau_rnea = rnea(model,data_rnea,q,v,VectorXd::Zero (model.nv));
  
  BOOST_CHECK (tau_nle.isApprox(tau_rnea, 1e-12));
  
  // -------
  q.setRandom ();
  v.setRandom ();
  
  tau_nle = nonLinearEffects(model,data_nle,q,v);
  tau_rnea = rnea(model,data_rnea,q,v,VectorXd::Zero (model.nv));
  
  BOOST_CHECK (tau_nle.isApprox(tau_rnea, 1e-12));
}
  
BOOST_AUTO_TEST_CASE (test_rnea_with_fext)
{
  using namespace Eigen;
  using namespace se3;
  
  Model model;
  buildModels::humanoidRandom(model);
  Data data_rnea_fext(model);
  Data data_rnea(model);
  
  VectorXd q (VectorXd::Random(model.nq));
  q.segment<4>(3).normalize();
  
  VectorXd v (VectorXd::Random(model.nv));
  VectorXd a (VectorXd::Random(model.nv));
  
  container::aligned_vector<Force> fext(model.joints.size(), Force::Zero());
  
  JointIndex rf = model.getJointId("rleg6_joint"); Force Frf = Force::Random();
  fext[rf] = Frf;
  JointIndex lf = model.getJointId("lleg6_joint"); Force Flf = Force::Random();
  fext[lf] = Flf;
  
  rnea(model,data_rnea,q,v,a);
  VectorXd tau_ref(data_rnea.tau);
  Data::Matrix6x Jrf(Data::Matrix6x::Zero(6,model.nv));
  jointJacobian(model,data_rnea,q,rf,Jrf);
  tau_ref -= Jrf.transpose() * Frf.toVector();
  
  Data::Matrix6x Jlf(Data::Matrix6x::Zero(6,model.nv));
  jointJacobian(model,data_rnea,q,lf,Jlf);
  tau_ref -= Jlf.transpose() * Flf.toVector();
  
  rnea(model,data_rnea_fext,q,v,a,fext);
  
  BOOST_CHECK(tau_ref.isApprox(data_rnea_fext.tau));
}
  
BOOST_AUTO_TEST_CASE(test_compute_gravity)
{
  using namespace Eigen;
  using namespace se3;
  
  Model model;
  buildModels::humanoidRandom(model);
  Data data_rnea(model);
  Data data(model);
  
  VectorXd q (VectorXd::Random(model.nq));
  q.segment<4>(3).normalize();
  
  rnea(model,data_rnea,q,VectorXd::Zero(model.nv),VectorXd::Zero(model.nv));
  computeGeneralizedGravity(model,data,q);
  
  BOOST_CHECK(data_rnea.tau.isApprox(data.g));
  
  // Compare with Jcom
  crba(model,data_rnea,q);
  Data::Matrix3x Jcom = getJacobianComFromCrba(model,data_rnea);
  
  VectorXd g_ref(-data_rnea.mass[0]*Jcom.transpose()*Model::gravity981);
  
  BOOST_CHECK(g_ref.isApprox(data.g));
}
  
  BOOST_AUTO_TEST_CASE(test_compute_coriolis)
  {
    using namespace Eigen;
    using namespace se3;
    
    const double prec = Eigen::NumTraits<double>::dummy_precision();
    
    Model model;
    buildModels::humanoidRandom(model);
    Data data_ref(model);
    Data data(model);
    
    VectorXd q (VectorXd::Random(model.nq));
    q.segment<4>(3).normalize();
    
    VectorXd v (VectorXd::Random(model.nv));
    computeCoriolisMatrix(model,data,q,Eigen::VectorXd::Zero(model.nv));
    BOOST_CHECK(data.C.isZero(prec));
    
    
    model.gravity.setZero();
    rnea(model,data_ref,q,v,VectorXd::Zero(model.nv));
    computeJointJacobiansTimeVariation(model,data_ref,q,v);
    computeCoriolisMatrix(model,data,q,v);

    BOOST_CHECK(data.dJ.isApprox(data_ref.dJ));
    BOOST_CHECK(data.J.isApprox(data_ref.J));
    
    VectorXd tau = data.C * v;
    BOOST_CHECK(tau.isApprox(data_ref.tau,prec));
    
    dccrba(model,data_ref,q,v);
    crba(model,data_ref,q);
    
    const Data::Vector3 & com = data_ref.com[0];
    Motion vcom(data_ref.vcom[0],Data::Vector3::Zero());
    SE3 cM1(data.oMi[1]); cM1.translation() -= com;
    
    BOOST_CHECK((cM1.toDualActionMatrix()*data_ref.M.topRows<6>()).isApprox(data_ref.Ag,prec));
    
    Force dh_ref = cM1.act(Force(data_ref.tau.head<6>()));
    Force dh(data_ref.dAg * v);
    BOOST_CHECK(dh.isApprox(dh_ref,prec));
    
    {
      Data data_ref(model), data_ref_plus(model);
      Eigen::MatrixXd dM(data.C + data.C.transpose());

      const double alpha = 1e-8;
      Eigen::VectorXd q_plus(model.nq);
      q_plus = integrate(model,q,alpha*v);
      
      crba(model,data_ref,q);
      data_ref.M.triangularView<Eigen::StrictlyLower>() = data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();
      crba(model,data_ref_plus,q_plus);
      data_ref_plus.M.triangularView<Eigen::StrictlyLower>() = data_ref_plus.M.transpose().triangularView<Eigen::StrictlyLower>();
      
      Eigen::MatrixXd dM_ref = (data_ref_plus.M - data_ref.M)/alpha;
      BOOST_CHECK(dM.isApprox(dM_ref,sqrt(alpha)));
    }
    
//    {
//      //v.setZero();
//      v.setOnes();
//      Eigen::VectorXd v_fd(v);
//      Eigen::MatrixXd drnea_dv(model.nv,model.nv);
//      Data data_fd(model);
//      
//      const VectorXd tau0 = rnea(model,data_fd,q,v,0*v);
//      const double eps = 1e-8;
//      for(int i = 0; i < model.nv; ++i)
//      {
//        v_fd[i] += eps;
//        rnea(model,data_fd,q,v_fd,0*v);
//        drnea_dv.col(i) = (data_fd.tau - tau0)/eps;
//        v_fd[i] -= eps;
//      }
//      BOOST_CHECK(v_fd.isApprox(v));
//      std::cout << "drnea_dv:\n" << drnea_dv.block<6,6>(0,0) << std::endl;
//      std::cout << "C:\n" << data.C.block<6,6>(0,0) << std::endl;;
//    }
    
  }
BOOST_AUTO_TEST_SUITE_END ()
