//
// Copyright (c) 2015-2018 CNRS
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

BOOST_AUTO_TEST_CASE ( test_subtree_com_jacobian )
{
  using namespace Eigen;
  using namespace pinocchio;

  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model);
  pinocchio::Data data(model);
  VectorXd q = pinocchio::randomConfiguration(model);
  VectorXd v = VectorXd::Random(model.nv);
  computeAllTerms(model, data, q, v);

  // Get subtree jacobian and check that it is consistent with com velocity
  for (JointIndex i = 0; i < model.njoints; i++)
  {
    SE3::Vector3 subtreeComVelocityInWorld = data.oMi[i].rotation() * data.vcom[i];
    Data::Matrix3x Jcom(3, model.nv); Jcom.fill(0);
    jacobianSubtreeCenterOfMass(model, data, i, Jcom);
    BOOST_CHECK((Jcom * v).isApprox(subtreeComVelocityInWorld, 1e-12));
  }
}

BOOST_AUTO_TEST_SUITE_END ()
