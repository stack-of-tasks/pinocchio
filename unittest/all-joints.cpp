//
// Copyright(c) 2015-2021 CNRS INRIA
// Copyright(c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#include "utils/joints-init.hpp"

#include "pinocchio/multibody/joint.hpp"
#include "pinocchio/multibody/liegroup.hpp"

#include <boost/test/unit_test.hpp>
#include <iostream>

using namespace pinocchio;

BOOST_AUTO_TEST_SUITE(joint_model_base_test)

template<typename TestDerived>
struct TestJointModel
{
  template<typename JointModel>
  void operator()(const pinocchio::JointModelBase<JointModel> &) const
  {
    JointModel jmodel = init<JointModel>::run().jmodel;
    return TestDerived::test(jmodel);
  }
};

struct TestJointModelIsEqual : TestJointModel<TestJointModelIsEqual>
{
  template<typename JointModel>
  static void test(const JointModelBase<JointModel> & jmodel)
  {
    JointModel jmodel_copy = jmodel.derived();
    BOOST_CHECK(jmodel_copy == jmodel.derived());

    JointModel jmodel_any;
    BOOST_CHECK(jmodel_any != jmodel.derived());
    BOOST_CHECK(!jmodel_any.isEqual(jmodel.derived()));
  }
};

struct TestJointModelTransform : TestJointModel<TestJointModelTransform>
{
  template<typename JointModel>
  static void test(const JointModelBase<JointModel> & jmodel)
  {
    typedef typename JointModel::JointDataDerived JointData;
    JointData jdata = jmodel.createData();
    Eigen::Matrix<typename JointModel::Scalar, 3, 1> t = jdata.M_accessor().translation();
    PINOCCHIO_UNUSED_VARIABLE(t);
    Eigen::Matrix<typename JointModel::Scalar, 3, 3> R = jdata.M_accessor().rotation();
    PINOCCHIO_UNUSED_VARIABLE(R);
  }
};

BOOST_AUTO_TEST_CASE(isEqual)
{
  typedef JointCollectionDefault::JointModelVariant JointModelVariant;
  boost::mpl::for_each<JointModelVariant::types>(TestJointModelIsEqual());

  JointModelRX joint_revolutex;
  JointModelRY joint_revolutey;

  BOOST_CHECK(joint_revolutex != joint_revolutey);

  JointModel jmodelx(joint_revolutex);
  jmodelx.setIndexes(0, 0, 0);
  TestJointModelIsEqual()(JointModel());

  JointModel jmodel_any;
  BOOST_CHECK(jmodel_any != jmodelx);
}

BOOST_AUTO_TEST_CASE(transform)
{
  typedef JointCollectionDefault::JointModelVariant JointModelVariant;
  boost::mpl::for_each<JointModelVariant::types>(TestJointModelTransform());

  TestJointModelTransform()(JointModel());
}

struct TestJointModelLieGroup : TestJointModel<TestJointModelLieGroup>
{
  template<typename JointModel>
  static void test(const JointModelBase<JointModel> & jmodel)
  {
    auto lgo = jmodel.template lieGroup<LieGroupMap>();
    BOOST_CHECK(lgo.nq() == jmodel.nq());
    BOOST_CHECK(lgo.nv() == jmodel.nv());
  }
};

BOOST_AUTO_TEST_CASE(lie_group)
{
  typedef JointCollectionDefault::JointModelVariant JointModelVariant;
  boost::mpl::for_each<JointModelVariant::types>(TestJointModelLieGroup());
}

struct TestJointModelCast : TestJointModel<TestJointModelCast>
{
  template<typename JointModel>
  static void test(const JointModelBase<JointModel> & jmodel)
  {
    typedef typename JointModel::Scalar Scalar;
    BOOST_CHECK(jmodel == jmodel);
    BOOST_CHECK(jmodel.template cast<Scalar>().isEqual(jmodel));
    BOOST_CHECK(jmodel.template cast<Scalar>() == jmodel);
  }
};

BOOST_AUTO_TEST_CASE(cast)
{
  typedef JointCollectionDefault::JointModelVariant JointModelVariant;
  boost::mpl::for_each<JointModelVariant::types>(TestJointModelCast());

  TestJointModelCast()(JointModel());
}

struct TestJointModelDisp : TestJointModel<TestJointModelDisp>
{
  template<typename JointModel>
  static void test(const JointModelBase<JointModel> & jmodel)
  {
    typedef typename JointModel::JointDataDerived JointData;

    std::cout << "shortname: " << jmodel.shortname() << std::endl;
    std::cout << "classname: " << jmodel.classname() << std::endl;
    std::cout << "disp:\n" << jmodel << std::endl;

    JointData jdata = jmodel.createData();

    std::cout << "shortname: " << jdata.shortname() << std::endl;
    std::cout << "classname: " << jdata.classname() << std::endl;
    std::cout << "disp:\n" << jdata << std::endl;
  }
};

BOOST_AUTO_TEST_CASE(test_disp)
{
  typedef JointCollectionDefault::JointModelVariant JointModelVariant;
  boost::mpl::for_each<JointModelVariant::types>(TestJointModelDisp());

  TestJointModelDisp()(JointModel());
}

BOOST_AUTO_TEST_SUITE_END()
