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

// #include "pinocchio/multibody/joint/joint-accessor.hpp"
#include "pinocchio/multibody/joint/joint-composite.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/jacobian.hpp"

#include <iostream>
#include <cmath>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>


template <typename T>
void test_joint_methods (T & jmodel)
{
  using namespace se3;

  typename T::JointDataDerived jdata = jmodel.createData();

  JointModelComposite jmodel_composite(jmodel);
  jmodel_composite.setIndexes(jmodel.id(), jmodel.idx_q(), jmodel.idx_v());
  jmodel_composite.updateComponentsIndexes();

  JointDataComposite jdata_composite = jmodel_composite.createData();

  Eigen::VectorXd q1(Eigen::VectorXd::Random (jmodel.nq()));jmodel_composite.normalize(q1);
  Eigen::VectorXd q1_dot(Eigen::VectorXd::Random (jmodel.nv()));
  Eigen::VectorXd q2(Eigen::VectorXd::Random (jmodel.nq()));jmodel_composite.normalize(q2);
  double u = 0.3;
  se3::Inertia::Matrix6 Ia(se3::Inertia::Random().matrix());
  bool update_I = false;

  jmodel.calc(jdata, q1, q1_dot);
  jmodel_composite.calc(jdata_composite, q1, q1_dot);


  std::string error_prefix("Joint Model Composite on " + jmodel.shortname());

  BOOST_CHECK_MESSAGE(jmodel.nq() == jmodel_composite.nq() ,std::string(error_prefix + " - nq "));
  BOOST_CHECK_MESSAGE(jmodel.nv() == jmodel_composite.nv() ,std::string(error_prefix + " - nv "));

  BOOST_CHECK_MESSAGE(jmodel.idx_q() == jmodel_composite.idx_q() ,std::string(error_prefix + " - Idx_q "));
  BOOST_CHECK_MESSAGE(jmodel.idx_v() == jmodel_composite.idx_v() ,std::string(error_prefix + " - Idx_v "));
  BOOST_CHECK_MESSAGE(jmodel.id() == jmodel_composite.id() ,std::string(error_prefix + " - JointId "));

  BOOST_CHECK_MESSAGE(jmodel.integrate(q1,q1_dot).isApprox(jmodel_composite.integrate(q1,q1_dot)) ,std::string(error_prefix + " - integrate "));
  BOOST_CHECK_MESSAGE(jmodel.interpolate(q1,q2,u).isApprox(jmodel_composite.interpolate(q1,q2,u)) ,std::string(error_prefix + " - interpolate "));
  BOOST_CHECK_MESSAGE(jmodel.randomConfiguration( -1 * Eigen::VectorXd::Ones(jmodel.nq()),
                                              Eigen::VectorXd::Ones(jmodel.nq())).size()
                                  == jmodel_composite.randomConfiguration(-1 * Eigen::VectorXd::Ones(jmodel.nq()),
                                                            Eigen::VectorXd::Ones(jmodel.nq())).size()
                      ,std::string(error_prefix + " - RandomConfiguration dimensions "));

  BOOST_CHECK_MESSAGE(jmodel.difference(q1,q2).isApprox(jmodel_composite.difference(q1,q2)) ,std::string(error_prefix + " - difference "));
  BOOST_CHECK_MESSAGE(fabs(jmodel.distance(q1,q2) - jmodel_composite.distance(q1,q2)) <= 1e-6 ,std::string(error_prefix + " - distance "));

  BOOST_CHECK_MESSAGE(((ConstraintXd)jdata.S).matrix().isApprox((jdata_composite.S.matrix())),std::string(error_prefix + " - ConstraintXd "));
  BOOST_CHECK_MESSAGE(jdata.M == jdata_composite.M, std::string(error_prefix + " - Joint transforms ")); // ==  or isApprox ?
  BOOST_CHECK_MESSAGE((Motion)jdata.v == jdata_composite.v, std::string(error_prefix + " - Joint motions "));
  BOOST_CHECK_MESSAGE((Motion)jdata.c == jdata_composite.c, std::string(error_prefix + " - Joint bias "));
  
  jmodel.calc_aba(jdata, Ia, update_I);
  jmodel_composite.calc_aba(jdata_composite, Ia, update_I);

  BOOST_CHECK_MESSAGE((jdata.U).isApprox(jdata_composite.U), std::string(error_prefix + " - Joint U inertia matrix decomposition "));
  BOOST_CHECK_MESSAGE((jdata.Dinv).isApprox(jdata_composite.Dinv), std::string(error_prefix + " - Joint DInv inertia matrix decomposition "));
  BOOST_CHECK_MESSAGE((jdata.UDinv).isApprox(jdata_composite.UDinv), std::string(error_prefix + " - Joint UDInv inertia matrix decomposition "));

 
}

struct TestJointComposite{

  template <typename T>
  void operator()(const T ) const
  {
    T jmodel;
    jmodel.setIndexes(0,0,0);

    test_joint_methods(jmodel);    
  }

  void operator()(const se3::JointModelComposite & ) const
  {
    se3::JointModelComposite jmodel_composite_rx(2);
    jmodel_composite_rx.addJointModel(se3::JointModelRX());
    jmodel_composite_rx.addJointModel(se3::JointModelRY());
    jmodel_composite_rx.setIndexes(1,0,0);
    jmodel_composite_rx.updateComponentsIndexes();

    test_joint_methods(jmodel_composite_rx);

  }

  void operator()(const se3::JointModelRevoluteUnaligned & ) const
  {
    se3::JointModelRevoluteUnaligned jmodel(1.5, 1., 0.);
    jmodel.setIndexes(0,0,0);

    test_joint_methods(jmodel);
  }

  void operator()(const se3::JointModelPrismaticUnaligned & ) const
  {
    se3::JointModelPrismaticUnaligned jmodel(1.5, 1., 0.);
    jmodel.setIndexes(0,0,0);

    test_joint_methods(jmodel);
  }

};


BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

// Test that a composite joint can contain any type of joint
BOOST_AUTO_TEST_CASE ( test_all_joints )
{
  using namespace Eigen;
  using namespace se3;

  boost::mpl::for_each<JointModelVariant::types>(TestJointComposite());

}



BOOST_AUTO_TEST_CASE ( test_recursive_variant)
{
  // functional test. Test if one can create a composite joint containing composite joint

  using namespace Eigen;
  using namespace se3;

  /// Create joint composite with two joints,
  JointModelComposite jmodel_composite_two_rx(2);
  jmodel_composite_two_rx.addJointModel(JointModelRX());
  jmodel_composite_two_rx.addJointModel(JointModelRY());

  /// Create Joint composite with three joints, and add a composite in it, to test the recursive_wrapper
  JointModelComposite jmodel_composite_recursive(3);
  jmodel_composite_recursive.addJointModel(JointModelFreeFlyer());
  jmodel_composite_recursive.addJointModel(JointModelPlanar());
  jmodel_composite_recursive.addJointModel(jmodel_composite_two_rx);
  
}


BOOST_AUTO_TEST_CASE (TestCopyComposite)
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

  Eigen::VectorXd q1(Eigen::VectorXd::Random(3));
  Eigen::VectorXd q1_dot(Eigen::VectorXd::Random(3));

  JointModelComposite model_copy = jmodel_composite_planar;
  JointDataComposite data_copy = model_copy.createData();
  
  BOOST_CHECK_MESSAGE( model_copy.max_joints == jmodel_composite_planar.max_joints, "Test Copy Composite, max_joints are differents");
  BOOST_CHECK_MESSAGE( model_copy.nq() == jmodel_composite_planar.nq(), "Test Copy Composite, nq are differents");
  BOOST_CHECK_MESSAGE( model_copy.nv() == jmodel_composite_planar.nv(), "Test Copy Composite, nv are differents");

  BOOST_CHECK_MESSAGE( model_copy.max_joints == jmodel_composite_planar.max_joints, "Test Copy Composite, max_joints are differents");

  jmodel_composite_planar.calc(jdata_composite_planar,q1, q1_dot);
  model_copy.calc(data_copy,q1, q1_dot);

}


BOOST_AUTO_TEST_CASE ( test_R3xSO3)
{
  std::cout << " Testing R3xSO3 vs jointcomposite<R3 - SO3>" << std::endl;
  using namespace Eigen;
  using namespace se3;

  Model model_composite;
  Model model_zero_mass;
  Model model_ff;



  Inertia body_inertia(Inertia::Random());
  SE3 placement(SE3::Identity());

  model_zero_mass.addJoint(model_zero_mass.getJointId("universe"),JointModelTranslation(), placement, "R3_joint");
  model_zero_mass.addJoint(model_zero_mass.getJointId("R3_joint"), JointModelSpherical(), SE3::Identity(), "SO3_joint");
  model_zero_mass.appendBodyToJoint(model_zero_mass.getJointId("SO3_joint"), body_inertia, SE3::Identity());

  JointModelComposite jmodel_composite(2);
  jmodel_composite.addJointModel(JointModelTranslation());
  jmodel_composite.addJointModel(JointModelSpherical());
  
  model_composite.addJoint(model_composite.getJointId("universe"),jmodel_composite, placement, "composite_R3xSO3_joint");
  model_composite.appendBodyToJoint(model_composite.getJointId("composite_R3xSO3_joint"), body_inertia, SE3::Identity());

  model_ff.addJoint(model_ff.getJointId("universe"),JointModelFreeFlyer(), placement, "ff_joint");
  model_ff.appendBodyToJoint(model_ff.getJointId("ff_joint"), body_inertia, SE3::Identity());

  BOOST_CHECK_MESSAGE(model_composite.nq == model_zero_mass.nq ,"Model with R3 - SO3 vs composite <R3xSO3> - dimensions nq are not equal");
  BOOST_CHECK_MESSAGE(model_composite.nq == model_zero_mass.nq ,"Model with R3 - SO3 vs composite <R3xSO3> - dimensions nv are not equal");


  Data data_zero_mass(model_zero_mass);
  Data data_composite(model_composite);
  Data data_ff(model_ff);

  Eigen::VectorXd q(Eigen::VectorXd::Random(model_zero_mass.nq));normalize(model_zero_mass,q);
  Eigen::VectorXd q_dot(Eigen::VectorXd::Random(model_zero_mass.nv));
  Eigen::VectorXd q_ddot(Eigen::VectorXd::Random(model_zero_mass.nv));
  Eigen::VectorXd q1(Eigen::VectorXd::Random(model_zero_mass.nq));normalize(model_zero_mass,q1);
  Eigen::VectorXd q2(Eigen::VectorXd::Random(model_zero_mass.nq));normalize(model_zero_mass,q2);
  Eigen::VectorXd tau(Eigen::VectorXd::Random(model_zero_mass.nq));
  double u = 0.3;



  aba(model_composite,data_composite, q,q_dot, tau);
  centerOfMass(model_composite, data_composite,q,q_dot,q_ddot,true,false);
  forwardKinematics(model_composite,data_composite, q, q_dot, q_ddot);
  computeAllTerms(model_zero_mass,data_zero_mass,q,q_dot);

  forwardKinematics(model_zero_mass, data_zero_mass, q, q_dot, q_ddot);
  computeAllTerms(model_composite,data_composite,q,q_dot);


  Model::Index index_joint_R3xSO3 = (Model::Index) model_zero_mass.njoints-1;
  Model::Index index_joint_composite = (Model::Index) model_composite.njoints-1;


  BOOST_CHECK_MESSAGE(data_composite.oMi[index_joint_composite]
                          .isApprox(data_zero_mass.oMi[index_joint_R3xSO3]) , "composite<R3xSO3> vs R3-SO3 - oMi last joint not equal");


  BOOST_CHECK_MESSAGE(data_composite.v[index_joint_composite]
                          == data_zero_mass.v[index_joint_R3xSO3] , "composite<R3xSO3> vs R3-SO3 - v last joint not equal");

  // BOOST_CHECK_MESSAGE(data_composite.a[index_joint_composite] //@TODO Uncommente to test once JointComposite maths are ok
  //                         == data_zero_mass.a[index_joint_R3xSO3] , "composite planar joint vs PxPyRz - a last joint not equal");

  // BOOST_CHECK_MESSAGE(data_composite.f[index_joint_composite] //@TODO Uncommente to test once JointComposite maths are ok
  //                         == data_zero_mass.f[index_joint_R3xSO3] , "composite planar joint vs PxPyRz - f last joint not equal");

  BOOST_CHECK_MESSAGE(data_composite.com[index_joint_composite]
                          .isApprox(data_zero_mass.com[index_joint_R3xSO3]) , "composite<R3xSO3> vs R3-SO3 - com last joint not equal");

  BOOST_CHECK_MESSAGE(data_composite.vcom[index_joint_composite]
                          .isApprox(data_zero_mass.vcom[index_joint_R3xSO3]) , "composite<R3xSO3> vs R3-SO3 - vcom last joint not equal");

  BOOST_CHECK_MESSAGE(data_composite.mass[index_joint_composite]
                          == data_zero_mass.mass[index_joint_R3xSO3] , "composite<R3xSO3> vs R3-SO3 - mass last joint not equal"); 

  BOOST_CHECK_MESSAGE(data_composite.kinetic_energy
                          == data_zero_mass.kinetic_energy , "composite<R3xSO3> vs R3-SO3 - kinetic energy not equal");

  BOOST_CHECK_MESSAGE(data_composite.potential_energy
                          == data_zero_mass.potential_energy , "composite<R3xSO3> vs R3-SO3 - potential energy not equal");                          

  // BOOST_CHECK_MESSAGE(data_composite.nle //@TODO Uncommente to test once JointComposite maths are ok
  //                         .isApprox(data_zero_mass.nle) , "composite planar joint vs PxPyRz - nle not equal");

  // BOOST_CHECK_MESSAGE(data_composite.M //@TODO Uncommente to test once JointComposite maths are ok
  //                         .isApprox(data_zero_mass.M) , "composite planar joint vs PxPyRz - Mass Matrix not equal");

  
  BOOST_CHECK_MESSAGE(integrate(model_composite, q,q_dot).isApprox(integrate(model_zero_mass ,q,q_dot)) ,std::string(" composite<R3xSO3> vs R3-SO3 - integrate model error "));
  BOOST_CHECK_MESSAGE(interpolate(model_composite, q1,q2,u).isApprox(interpolate(model_zero_mass ,q1,q2,u)) ,std::string(" composite<R3xSO3> vs R3-SO3 - interpolate model error "));
  BOOST_CHECK_MESSAGE(differentiate(model_composite, q1,q2).isApprox(differentiate(model_zero_mass ,q1,q2)) ,std::string(" composite<R3xSO3> vs R3-SO3 - differentiate model error "));
  BOOST_CHECK_MESSAGE(fabs(distance(model_composite, q1,q2).norm() - distance(model_zero_mass ,q1,q2).norm()) <= 1e-6 ,std::string(" composite<R3xSO3> vs R3-SO3 - distance model error "));

}

BOOST_AUTO_TEST_SUITE_END ()

