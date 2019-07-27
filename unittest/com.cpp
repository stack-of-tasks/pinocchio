//
// Copyright (c) 2015-2019 CNRS INRIA
//

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/utils/timer.hpp"
#include "pinocchio/parsers/sample-models.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE ( test_com )
{
  using namespace Eigen;
  using namespace pinocchio;

  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model);
  pinocchio::Data data(model);

  VectorXd q = VectorXd::Ones(model.nq);
  q.middleRows<4> (3).normalize();
  VectorXd v = VectorXd::Ones(model.nv);
  VectorXd a = VectorXd::Ones(model.nv);

  crba(model,data,q);


	/* Test COM against CRBA*/
  Vector3d com = centerOfMass(model,data,q);
  BOOST_CHECK(data.com[0].isApprox(getComFromCrba(model,data), 1e-12));

	/* Test COM against Jcom (both use different way to compute the COM). */
  com = centerOfMass(model,data,q);
  jacobianCenterOfMass(model,data,q);
  BOOST_CHECK(com.isApprox(data.com[0], 1e-12));

	/* Test COM against Jcom (both use different way to compute the COM). */
  centerOfMass(model,data,q,v,a);
  BOOST_CHECK(com.isApprox(data.com[0], 1e-12));

  /* Test vCoM against nle algorithm without gravity field */
  a.setZero();
  model.gravity.setZero();
  centerOfMass(model,data,q,v,a);
  nonLinearEffects(model, data, q, v);

  pinocchio::SE3::Vector3 acom_from_nle (data.nle.head <3> ()/data.mass[0]);
  BOOST_CHECK((data.liMi[1].rotation() * acom_from_nle).isApprox(data.acom[0], 1e-12));

	/* Test Jcom against CRBA  */
  Eigen::MatrixXd Jcom = jacobianCenterOfMass(model,data,q);
  BOOST_CHECK(data.Jcom.isApprox(getJacobianComFromCrba(model,data), 1e-12));

  /* Test CoM velocity againt jacobianCenterOfMass */
  BOOST_CHECK((Jcom * v).isApprox(data.vcom[0], 1e-12));


  centerOfMass(model,data,q,v);
  /* Test CoM velocity againt jacobianCenterOfMass */
  BOOST_CHECK((Jcom * v).isApprox(data.vcom[0], 1e-12));


//  std::cout << "com = [ " << data.com[0].transpose() << " ];" << std::endl;
//  std::cout << "mass = [ " << data.mass[0] << " ];" << std::endl;
//  std::cout << "Jcom = [ " << data.Jcom << " ];" << std::endl;
//  std::cout << "M3 = [ " << data.M.topRows<3>() << " ];" << std::endl;
}

BOOST_AUTO_TEST_CASE ( test_mass )
{
  using namespace Eigen;
  using namespace pinocchio;

  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model);

  double mass = computeTotalMass(model);

  BOOST_CHECK(mass == mass); // checking it is not NaN

  double mass_check = 0.0;
  for(size_t i=1; i<(size_t)(model.njoints);++i)
    mass_check += model.inertias[i].mass();

  BOOST_CHECK_CLOSE(mass, mass_check, 1e-12);

  pinocchio::Data data1(model);

  double mass_data = computeTotalMass(model,data1);

  BOOST_CHECK(mass_data == mass_data); // checking it is not NaN
  BOOST_CHECK_CLOSE(mass, mass_data, 1e-12);
  BOOST_CHECK_CLOSE(data1.mass[0], mass_data, 1e-12);

  pinocchio::Data data2(model);
  VectorXd q = VectorXd::Ones(model.nq);
  q.middleRows<4> (3).normalize();
  centerOfMass(model,data2,q);

  BOOST_CHECK_CLOSE(data2.mass[0], mass, 1e-12);
}

BOOST_AUTO_TEST_CASE ( test_subtree_masses )
{
  using namespace Eigen;
  using namespace pinocchio;

  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model);

  pinocchio::Data data1(model);

  computeSubtreeMasses(model,data1);

  pinocchio::Data data2(model);
  VectorXd q = VectorXd::Ones(model.nq);
  q.middleRows<4> (3).normalize();
  centerOfMass(model,data2,q);

  for(size_t i=0; i<(size_t)(model.njoints);++i)
  {
    BOOST_CHECK_CLOSE(data1.mass[i], data2.mass[i], 1e-12);
  }
}

//BOOST_AUTO_TEST_CASE ( test_timings )
//{
//  using namespace Eigen;
//  using namespace pinocchio;
//
//  pinocchio::Model model;
//  pinocchio::buildModels::humanoidRandom(model);
//  pinocchio::Data data(model);
//
//  long flag = BOOST_BINARY(1111);
//  PinocchioTicToc timer(PinocchioTicToc::US);
//  #ifdef NDEBUG
//    #ifdef _INTENSE_TESTING_
//      const size_t NBT = 1000*1000;
//    #else
//      const size_t NBT = 10;
//    #endif
//  #else
//    const size_t NBT = 1;
//    std::cout << "(the time score in debug mode is not relevant)  " ;
//  #endif
//
//  bool verbose = flag & (flag-1) ; // True is two or more binaries of the flag are 1.
//  if(verbose) std::cout <<"--" << std::endl;
//  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
//
//  if( flag >> 0 & 1 )
//  {
//    timer.tic();
//    SMOOTH(NBT)
//    {
//      centerOfMass(model,data,q);
//    }
//    if(verbose) std::cout << "COM =\t";
//    timer.toc(std::cout,NBT);
//  }
//
//  if( flag >> 1 & 1 )
//  {
//    timer.tic();
//    SMOOTH(NBT)
//    {
//      centerOfMass(model,data,q,false);
//    }
//    if(verbose) std::cout << "Without sub-tree =\t";
//    timer.toc(std::cout,NBT);
//  }
//
//  if( flag >> 2 & 1 )
//  {
//    timer.tic();
//    SMOOTH(NBT)
//    {
//      jacobianCenterOfMass(model,data,q);
//    }
//    if(verbose) std::cout << "Jcom =\t";
//    timer.toc(std::cout,NBT);
//  }
//}

BOOST_AUTO_TEST_CASE(test_subtree_com_jacobian)
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model);
  Data data(model);
  
  model.upperPositionLimit.head<3>().fill(1000);
  model.lowerPositionLimit.head<3>() = -model.upperPositionLimit.head<3>();
  VectorXd q = pinocchio::randomConfiguration(model);
  VectorXd v = VectorXd::Random(model.nv);
  
  Data data_ref(model);
  jacobianCenterOfMass(model,data_ref,q,true);
  
  centerOfMass(model, data, q, v);
  Data::Matrix3x Jcom(3,model.nv); Jcom.setZero();
  jacobianSubtreeCenterOfMass(model, data, 0, Jcom);
  
  BOOST_CHECK(Jcom.isApprox(data_ref.Jcom));

  centerOfMass(model, data_ref, q, v, true);
  computeJointJacobians(model, data_ref, q);
  Data::Matrix3x Jcom_extracted(3,model.nv), Jcom_fd(3,model.nv);
  Data data_extracted(model), data_fd(model);
  const double eps = 1e-8;
  jacobianCenterOfMass(model,data_extracted,q);
  
  // Get subtree jacobian and check that it is consistent with the com velocity
  for(JointIndex joint_id = 1; joint_id < (JointIndex)model.njoints; joint_id++)
  {
    SE3::Vector3 subtreeComVelocityInWorld_ref = data_ref.oMi[joint_id].rotation() * data_ref.vcom[joint_id];
    Jcom.setZero();
    data.J.setZero();
    jacobianSubtreeCenterOfMass(model, data, joint_id, Jcom);
    
    BOOST_CHECK(data.J.middleCols(model.joints[joint_id].idx_v(),data.nvSubtree[joint_id]).isApprox(data_ref.J.middleCols(model.joints[joint_id].idx_v(),data.nvSubtree[joint_id])));
    SE3::Vector3 subtreeComVelocityInWorld = Jcom * v;
    
    Jcom_extracted.setZero();
    getJacobianSubtreeCenterOfMass(model,data_extracted,joint_id,Jcom_extracted);
    
    // Check with finite differences
    Eigen::VectorXd v_plus(model.nv); v_plus.setZero();
    centerOfMass(model,data_fd,q);
    const SE3::Vector3 com = data_fd.oMi[joint_id].act(data_fd.com[joint_id]);
    Jcom_fd.setZero();
    for(Eigen::DenseIndex k = 0; k < model.nv; ++k)
    {
      v_plus[k] = eps;
      Eigen::VectorXd q_plus = integrate(model,q,v_plus);
      centerOfMass(model,data_fd,q_plus);
      const SE3::Vector3 com_plus = data_fd.oMi[joint_id].act(data_fd.com[joint_id]);
      Jcom_fd.col(k) = (com_plus - com)/eps;
      v_plus[k] = 0.;
    }
    
//    Eigen::VectorXd q_plus = integrate(model,q,v*eps);
//    centerOfMass(model,data_fd,q_plus);
//    const SE3::Vector3 com_plus = data_fd.oMi[joint_id].act(data_fd.com[joint_id]);
//    
//    const SE3::Vector3 vcom_subtree_fd = (com_plus - com)/eps;

    BOOST_CHECK(Jcom.isApprox(Jcom_fd,sqrt(eps)));
    BOOST_CHECK(Jcom_extracted.isApprox(Jcom_fd,sqrt(eps)));
    BOOST_CHECK(Jcom_extracted.isApprox(Jcom));
    
    BOOST_CHECK(std::fabs(data.mass[joint_id] - data_ref.mass[joint_id]) <= 1e-12);
    BOOST_CHECK(data.com[joint_id].isApprox(data_ref.oMi[joint_id].act(data_ref.com[joint_id])));
    BOOST_CHECK(subtreeComVelocityInWorld.isApprox(subtreeComVelocityInWorld_ref));
  }
}

BOOST_AUTO_TEST_SUITE_END ()
