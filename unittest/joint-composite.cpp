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

#include "pinocchio/multibody/joint/joint-composite.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

using namespace se3;
using namespace Eigen;

template<typename JointModel>
void test_joint_methods(const JointModelBase<JointModel> & jmodel, JointModelComposite & jmodel_composite);

template<typename JointModel>
void test_joint_methods(const JointModelBase<JointModel> & jmodel)
{
  JointModelComposite jmodel_composite(jmodel);
  test_joint_methods(jmodel,jmodel_composite);
}

template<typename JointModel>
void test_joint_methods(const JointModelBase<JointModel> & jmodel, JointModelComposite & jmodel_composite)
{
  typedef typename JointModelBase<JointModel>::JointDataDerived JointData;
  JointData jdata = jmodel.createData();
  
  JointDataComposite jdata_composite = jmodel_composite.createData();
  
  jmodel_composite.setIndexes(jmodel.id(), jmodel.idx_q(), jmodel.idx_v());
  
  typedef typename JointModel::ConfigVector_t ConfigVector_t;
  typedef typename JointModel::TangentVector_t TangentVector_t;
  
  ConfigVector_t ql(ConfigVector_t::Constant(jmodel.nq(),-M_PI));
  ConfigVector_t qu(ConfigVector_t::Constant(jmodel.nq(),M_PI));
  ConfigVector_t q = jmodel.randomConfiguration(ql,qu);
  
  jmodel.calc(jdata,q);
  
  BOOST_CHECK(jmodel.nv() == jmodel_composite.nv());
  BOOST_CHECK(jmodel.nq() == jmodel_composite.nq());
  
  jmodel_composite.calc(jdata_composite,q);
  BOOST_CHECK(jdata_composite.M.isApprox((SE3)jdata.M));
  
  q = jmodel.randomConfiguration(ql,qu);
  TangentVector_t v = TangentVector_t::Random(jmodel.nv());
  jmodel.calc(jdata,q,v);
  jmodel_composite.calc(jdata_composite,q,v);
  
  BOOST_CHECK(jdata_composite.M.isApprox((SE3)jdata.M));
  BOOST_CHECK(jdata_composite.v.isApprox((Motion)jdata.v));
  BOOST_CHECK(jdata_composite.c.isApprox((Motion)jdata.c));
  
  {
    VectorXd q1(jmodel.random());
    jmodel.normalize(q1);
    VectorXd q2(jmodel.random());
    jmodel.normalize(q2);
    VectorXd v(VectorXd::Random(jmodel.nv()));
    
    BOOST_CHECK(jmodel_composite.integrate(q1,v).isApprox(jmodel.integrate(q1,v)));
    BOOST_CHECK(jmodel_composite.difference(q1,q2).isApprox(jmodel.difference(q1,q2)));
    
    const double alpha = 0.2;
    BOOST_CHECK(jmodel_composite.interpolate(q1,q2,alpha).isApprox(jmodel.interpolate(q1,q2,alpha)));
    BOOST_CHECK(std::fabs(jmodel_composite.distance(q1,q2)-jmodel.distance(q1,q2))<= NumTraits<double>::dummy_precision());
  }
  
  Inertia::Matrix6 I(Inertia::Random().matrix());
  jmodel.calc_aba(jdata,I,false);
  jmodel_composite.calc_aba(jdata_composite,I,false);
  
  BOOST_CHECK(jdata.U.isApprox(jdata_composite.U));
  BOOST_CHECK(jdata.Dinv.isApprox(jdata_composite.Dinv));
  BOOST_CHECK(jdata.UDinv.isApprox(jdata_composite.UDinv));
  
  /// TODO: Remove me. This is for testing purposes.
  Eigen::VectorXd qq = q;
  Eigen::VectorXd vv = v;
  Eigen::VectorXd res(jmodel_composite.nq());
  typename se3::IntegrateStep<se3::LieGroupTpl>::ArgsType args(qq, vv, res);
  se3::IntegrateStep<se3::LieGroupTpl>::run(jmodel_composite, args);
}

struct TestJointComposite{

  template <typename JointModel>
  void operator()(const JointModelBase<JointModel> &) const
  {
    JointModel jmodel;
    jmodel.setIndexes(0,0,0);

    test_joint_methods(jmodel);    
  }

  void operator()(const JointModelBase<JointModelComposite> &) const
  {
    JointModelComposite jmodel_composite;
    jmodel_composite.addJoint(se3::JointModelRX());
    jmodel_composite.addJoint(se3::JointModelRY());
    jmodel_composite.setIndexes(0,0,0);

    test_joint_methods(jmodel_composite);

  }

  void operator()(const JointModelBase<JointModelRevoluteUnaligned> &) const
  {
    JointModelRevoluteUnaligned jmodel(1.5, 1., 0.);
    jmodel.setIndexes(0,0,0);

    test_joint_methods(jmodel);
  }

  void operator()(const JointModelBase<JointModelPrismaticUnaligned> &) const
  {
    JointModelPrismaticUnaligned jmodel(1.5, 1., 0.);
    jmodel.setIndexes(0,0,0);

    test_joint_methods(jmodel);
  }

};

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE(test_basic)
{
  boost::mpl::for_each<JointModelVariant::types>(TestJointComposite());
}

BOOST_AUTO_TEST_CASE(test_equivalent)
{
  {
    JointModelSphericalZYX jmodel_spherical;
    jmodel_spherical.setIndexes(0,0,0);
    
    JointModelComposite jmodel_composite((JointModelRZ()));
    jmodel_composite.addJoint(JointModelRY());
    jmodel_composite.addJoint(JointModelRX());
    
    test_joint_methods(jmodel_spherical, jmodel_composite);
  }
  
  {
    JointModelTranslation jmodel_translation;
    jmodel_translation.setIndexes(0,0,0);
    
    JointModelComposite jmodel_composite((JointModelPX()));
    jmodel_composite.addJoint(JointModelPY());
    jmodel_composite.addJoint(JointModelPZ());
    
    test_joint_methods(jmodel_translation, jmodel_composite);
  }
}

BOOST_AUTO_TEST_CASE (test_recursive_variant)
{
  /// Create joint composite with two joints,
  JointModelComposite jmodel_composite_two_rx((JointModelRX()));
  jmodel_composite_two_rx.addJoint(JointModelRY());

  /// Create Joint composite with three joints, and add a composite in it, to test the recursive_wrapper
  JointModelComposite jmodel_composite_recursive((JointModelFreeFlyer()));
  jmodel_composite_recursive.addJoint(JointModelPlanar());
  jmodel_composite_recursive.addJoint(jmodel_composite_two_rx);
}


BOOST_AUTO_TEST_CASE(test_copy)
{
  JointModelComposite jmodel_composite_planar((JointModelPX()));
  jmodel_composite_planar.addJoint(JointModelPY());
  jmodel_composite_planar.addJoint(JointModelRZ());
  jmodel_composite_planar.setIndexes(0,0,0);

  JointDataComposite jdata_composite_planar = jmodel_composite_planar.createData();

  VectorXd q1(VectorXd::Random(3));
  VectorXd q1_dot(VectorXd::Random(3));

  JointModelComposite model_copy = jmodel_composite_planar;
  JointDataComposite data_copy = model_copy.createData();
  
  BOOST_CHECK_MESSAGE( model_copy.nq() == jmodel_composite_planar.nq(), "Test Copy Composite, nq are differents");
  BOOST_CHECK_MESSAGE( model_copy.nv() == jmodel_composite_planar.nv(), "Test Copy Composite, nv are differents");

  jmodel_composite_planar.calc(jdata_composite_planar,q1, q1_dot);
  model_copy.calc(data_copy,q1, q1_dot);

}

BOOST_AUTO_TEST_SUITE_END ()

