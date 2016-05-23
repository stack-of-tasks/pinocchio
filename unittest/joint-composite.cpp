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

#include <iostream>

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE JointCompositeTest
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>


template <typename T>
void test_joint_methods (T & jmodel)
{
  using namespace se3;

  typename T::JointDataDerived jdata = jmodel.createData();

  // JointModelComposite jmodel_composite((T()));
  JointModelComposite jmodel_composite(jmodel);
  jmodel_composite.setIndexes(jmodel.id(), jmodel.idx_q(), jmodel.idx_v());
  jmodel_composite.updateComponentsIndexes();

  JointDataComposite jdata_composite = jmodel_composite.createData();

  Eigen::VectorXd q1(Eigen::VectorXd::Random (jmodel.nq()));
  Eigen::VectorXd q1_dot(Eigen::VectorXd::Random (jmodel.nv()));
  Eigen::VectorXd q2(Eigen::VectorXd::Random (jmodel.nq()));
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
  BOOST_CHECK_MESSAGE(jmodel.distance(q1,q2) == jmodel_composite.distance(q1,q2) ,std::string(error_prefix + " - distance "));

  // pb call-operator car jdata directement le type derivé
  // BOOST_CHECK_MESSAGE((jdata.S().matrix()).isApprox((jdata_composite.S().matrix())),std::string(error_prefix + " - ConstraintXd "));
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

  template <int NQ, int NV>
  void operator()(const se3::JointModelDense<NQ,NV> & ) const
  {
    // Not yet correctly implemented, test has no meaning for the moment
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




BOOST_AUTO_TEST_SUITE ( JointCompositeTest)

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

  std::cout << "\n\n --- Test Copy composite" << std::endl;
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
  std::cout << model_copy << std::endl;
  
  BOOST_CHECK_MESSAGE( model_copy.max_joints == jmodel_composite_planar.max_joints, "Test Copy Composite, max_joints are differents");
  BOOST_CHECK_MESSAGE( model_copy.nq() == jmodel_composite_planar.nq(), "Test Copy Composite, nq are differents");
  BOOST_CHECK_MESSAGE( model_copy.nv() == jmodel_composite_planar.nv(), "Test Copy Composite, nv are differents");

  BOOST_CHECK_MESSAGE( model_copy.max_joints == jmodel_composite_planar.max_joints, "Test Copy Composite, max_joints are differents");

  jmodel_composite_planar.calc(jdata_composite_planar,q1, q1_dot);
  model_copy.calc(data_copy,q1, q1_dot);

}


BOOST_AUTO_TEST_CASE (TestVariantOverComposite)
{

  std::cout << "\n\n --- Test Variant Over composite" << std::endl;
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


  JointModelVariant jmvariant_comp(jmodel_composite_planar);
  JointDataVariant jdvariant_comp(jdata_composite_planar);

  std::cout << " Extract the composite joint from the variant, and visit each joint from the stack" << std::endl;
  JointModelComposite extracted_model = boost::get<JointModelComposite>(jmvariant_comp);
  for (std::size_t i = 0; i < 3; ++i)
  {
    calc_first_order(extracted_model.joints[i], jdata_composite_planar.joints[i], q1, q1_dot);
    std::cout << se3::nq(extracted_model.joints[i]) << std::endl;
  }
  
  std::cout << " Testing visiting a variant over the composite joint" << std::endl;
  std::cout << nv(jmvariant_comp) << std::endl;
  calc_first_order(jmvariant_comp, jdvariant_comp, q1, q1_dot); // here assertion 'false' failed has_fallback_type_
}


// Compare a stack of joint ( PX, PY, RZ) to a planar joint
BOOST_AUTO_TEST_CASE ( KinematicModelCompositePlanar)
{
  std::cout << " Testing Planar Model vs composite planar model" << std::endl;
  using namespace Eigen;
  using namespace se3;

  Model model_composite_planar;
  Model model_planar;

  Inertia body_inertia(Inertia::Random());
  SE3 placement(SE3::Identity());

  model_planar.addJoint(model_planar.getBodyId("universe"),JointModelPlanar(), placement, "planar_joint");
  model_planar.appendBodyToJoint(model_planar.getJointId("planar_joint"), body_inertia, SE3::Identity(), "planar_body");

  JointModelComposite jmodel_composite_planar(3);
  jmodel_composite_planar.addJointModel(JointModelPX());
  jmodel_composite_planar.addJointModel(JointModelPY());
  jmodel_composite_planar.addJointModel(JointModelRZ());
  
  model_composite_planar.addJoint(model_composite_planar.getBodyId("universe"),jmodel_composite_planar, placement, "composite_planar_joint");
  model_composite_planar.appendBodyToJoint(model_composite_planar.getJointId("composite_planar_joint"), body_inertia, SE3::Identity(), "composite_planar_body");
  // When Model will be cleaned in coming pull request, this will be done in addBody(addJoint)
  boost::get<JointModelComposite>(model_composite_planar.joints[model_composite_planar.getJointId("composite_planar_joint")]).updateComponentsIndexes();


  BOOST_CHECK_MESSAGE(model_composite_planar.nq == model_planar.nq ,"Model with planar joint vs composite PxPyRz - dimensions nq are not equal");
  BOOST_CHECK_MESSAGE(model_composite_planar.nq == model_planar.nq ,"Model with planar joint vs composite PxPyRz - dimensions nv are not equal");


  // Data data_planar(model_planar);
  // Data data_composite_planar(model_composite_planar);

  // Eigen::VectorXd q(Eigen::VectorXd::Random(model_planar.nq));
  // Eigen::VectorXd q_dot(Eigen::VectorXd::Random(model_planar.nv));
  // Eigen::VectorXd q_ddot(Eigen::VectorXd::Random(model_planar.nv));
  // Eigen::VectorXd q1(Eigen::VectorXd::Random(model_planar.nq));
  // Eigen::VectorXd q2(Eigen::VectorXd::Random(model_planar.nq));
  // Eigen::VectorXd tau(Eigen::VectorXd::Random(model_planar.nq));
  // double u = 0.3;

  // // Test that algorithms do not crash
  // integrate(model_composite_planar,q,q_dot);
  // interpolate(model_composite_planar,q1,q2,u);
  // differentiate(model_composite_planar,q1,q2);
  // distance(model_composite_planar,q1,q2);
  // randomConfiguration(model_composite_planar);

  // // aba(model_composite_planar,data_composite_planar, q,q_dot, tau);
  // centerOfMass(model_composite_planar, data_composite_planar,q,q_dot,q_ddot,true,false);
  // emptyForwardPass(model_composite_planar, data_composite_planar);
  // forwardKinematics(model_composite_planar,data_composite_planar, q );
  // forwardKinematics(model_composite_planar,data_composite_planar, q, q_dot);
  // forwardKinematics(model_composite_planar,data_composite_planar, q, q_dot, q_ddot);
  // computeAllTerms(model_planar,data_planar,q,q_dot);
  // computeAllTerms(model_composite_planar,data_composite_planar,q,q_dot);

  // Model::Index last_joint_pxpyrz = (Model::Index) model_planar.nbody-1;
  // Model::Index last_joint_composite = (Model::Index) model_composite_planar.nbody-1;

  // BOOST_CHECK_MESSAGE(data_composite_planar.oMi[last_joint_composite]
  //                         .isApprox(data_planar.oMi[last_joint_pxpyrz]) , "composite planar joint vs PxPyRz - oMi last joint not equal");

  // BOOST_CHECK_MESSAGE(data_composite_planar.v[last_joint_composite]
  //                         == data_planar.v[last_joint_pxpyrz] , "composite planar joint vs PxPyRz - v last joint not equal");

  // BOOST_CHECK_MESSAGE(data_composite_planar.a[last_joint_composite]
  //                         == data_planar.a[last_joint_pxpyrz] , "composite planar joint vs PxPyRz - a last joint not equal");

  // BOOST_CHECK_MESSAGE(data_composite_planar.f[last_joint_composite]
  //                         == data_planar.f[last_joint_pxpyrz] , "composite planar joint vs PxPyRz - f last joint not equal");

  // BOOST_CHECK_MESSAGE(data_composite_planar.com[last_joint_composite]
  //                         .isApprox(data_planar.com[last_joint_pxpyrz]) , "composite planar joint vs PxPyRz - com last joint not equal");

  // BOOST_CHECK_MESSAGE(data_composite_planar.vcom[last_joint_composite]
  //                         .isApprox(data_planar.vcom[last_joint_pxpyrz]) , "composite planar joint vs PxPyRz - vcom last joint not equal");

  // BOOST_CHECK_MESSAGE(data_composite_planar.mass[last_joint_composite]
  //                         == data_planar.mass[last_joint_pxpyrz] , "composite planar joint vs PxPyRz - mass last joint not equal"); 

  // BOOST_CHECK_MESSAGE(data_composite_planar.kinetic_energy
  //                         == data_planar.kinetic_energy , "composite planar joint vs PxPyRz - kinetic energy not equal");

  // BOOST_CHECK_MESSAGE(data_composite_planar.potential_energy
  //                         == data_planar.potential_energy , "composite planar joint vs PxPyRz - potential energy not equal");                          

  // BOOST_CHECK_MESSAGE(data_composite_planar.nle[last_joint_composite]
  //                         .isApprox(data_planar.nle[last_joint_pxpyrz]) , "composite planar joint vs PxPyRz - nle not equal");

  // BOOST_CHECK_MESSAGE(data_composite_planar.M[last_joint_composite]
  //                         .isApprox(data_planar.M[last_joint_pxpyrz]) , "composite planar joint vs PxPyRz - Mass Matrix not equal");

  // BOOST_CHECK_MESSAGE(data_composite_planar.J[last_joint_composite]
  //                         .isApprox(data_planar.J[last_joint_pxpyrz]) , "composite planar joint vs PxPyRz - Jacobian not equal");

  // BOOST_CHECK_MESSAGE(data_composite_planar.Jcom
  //                         .isApprox(data_planar.Jcom) , "composite planar joint vs PxPyRz - Jacobian com not equal");
  
}

BOOST_AUTO_TEST_SUITE_END ()

