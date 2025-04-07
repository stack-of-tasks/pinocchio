//
// Copyright(c) 2015-2021 CNRS INRIA
// Copyright(c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#include "pinocchio/math/fwd.hpp"
#include "pinocchio/multibody/joint/joints.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"

#include <boost/test/unit_test.hpp>
#include <iostream>

using namespace pinocchio;

template<typename JointModel_>
struct init;

template<typename JointModel_>
struct init
{
  static JointModel_ run()
  {
    JointModel_ jmodel;
    jmodel.setIndexes(0, 0, 0);
    return jmodel;
  }
};

template<typename Scalar, int Options>
struct init<pinocchio::JointModelRevoluteUnalignedTpl<Scalar, Options>>
{
  typedef pinocchio::JointModelRevoluteUnalignedTpl<Scalar, Options> JointModel;

  static JointModel run()
  {
    typedef typename JointModel::Vector3 Vector3;
    JointModel jmodel(Vector3::Random().normalized());

    jmodel.setIndexes(0, 0, 0);
    return jmodel;
  }
};

template<typename Scalar, int Options>
struct init<pinocchio::JointModelRevoluteUnboundedUnalignedTpl<Scalar, Options>>
{
  typedef pinocchio::JointModelRevoluteUnboundedUnalignedTpl<Scalar, Options> JointModel;

  static JointModel run()
  {
    typedef typename JointModel::Vector3 Vector3;
    JointModel jmodel(Vector3::Random().normalized());

    jmodel.setIndexes(0, 0, 0);
    return jmodel;
  }
};

template<typename Scalar, int Options>
struct init<pinocchio::JointModelPrismaticUnalignedTpl<Scalar, Options>>
{
  typedef pinocchio::JointModelPrismaticUnalignedTpl<Scalar, Options> JointModel;

  static JointModel run()
  {
    typedef typename JointModel::Vector3 Vector3;
    JointModel jmodel(Vector3::Random().normalized());

    jmodel.setIndexes(0, 0, 0);
    return jmodel;
  }
};

template<typename Scalar, int Options, template<typename, int> class JointCollection>
struct init<pinocchio::JointModelTpl<Scalar, Options, JointCollection>>
{
  typedef pinocchio::JointModelTpl<Scalar, Options, JointCollection> JointModel;

  static JointModel run()
  {
    typedef pinocchio::JointModelRevoluteTpl<Scalar, Options, 0> JointModelRX;
    JointModel jmodel((JointModelRX()));

    jmodel.setIndexes(0, 0, 0);
    return jmodel;
  }
};

template<typename Scalar, int Options, template<typename, int> class JointCollection>
struct init<pinocchio::JointModelCompositeTpl<Scalar, Options, JointCollection>>
{
  typedef pinocchio::JointModelCompositeTpl<Scalar, Options, JointCollection> JointModel;

  static JointModel run()
  {
    typedef pinocchio::JointModelRevoluteTpl<Scalar, Options, 0> JointModelRX;
    typedef pinocchio::JointModelRevoluteTpl<Scalar, Options, 1> JointModelRY;
    JointModel jmodel((JointModelRX()));
    jmodel.addJoint(JointModelRY());

    jmodel.setIndexes(0, 0, 0);
    return jmodel;
  }
};

template<typename Scalar, int Options, template<typename, int> class JointCollection>
struct init<pinocchio::JointModelMimicTpl<Scalar, Options, JointCollection>>
{
  typedef pinocchio::JointModelMimicTpl<Scalar, Options, JointCollection> JointModel;

  static JointModel run()
  {

    typedef pinocchio::JointModelRevoluteTpl<Scalar, Options, 0> JointModelRX;
    JointModelRX jmodel_ref = init<JointModelRX>::run();

    JointModel jmodel(jmodel_ref, 1., 0.);
    jmodel.setIndexes(1, 0, 0, 0);
    return jmodel;
  }
};

template<typename Scalar, int Options>
struct init<pinocchio::JointModelUniversalTpl<Scalar, Options>>
{
  typedef pinocchio::JointModelUniversalTpl<Scalar, Options> JointModel;

  static JointModel run()
  {
    JointModel jmodel(XAxis::vector(), YAxis::vector());

    jmodel.setIndexes(0, 0, 0);
    return jmodel;
  }
};

template<typename Scalar, int Options, int axis>
struct init<pinocchio::JointModelHelicalTpl<Scalar, Options, axis>>
{
  typedef pinocchio::JointModelHelicalTpl<Scalar, Options, axis> JointModel;

  static JointModel run()
  {
    JointModel jmodel(static_cast<Scalar>(0.5));

    jmodel.setIndexes(0, 0, 0);
    return jmodel;
  }
};

template<typename Scalar, int Options>
struct init<pinocchio::JointModelHelicalUnalignedTpl<Scalar, Options>>
{
  typedef pinocchio::JointModelHelicalUnalignedTpl<Scalar, Options> JointModel;

  static JointModel run()
  {
    typedef typename JointModel::Vector3 Vector3;
    JointModel jmodel(Vector3::Random().normalized());

    jmodel.setIndexes(0, 0, 0);
    return jmodel;
  }
};

BOOST_AUTO_TEST_SUITE(joint_model_base_test)

template<typename TestDerived>
struct TestJointModel
{
  template<typename JointModel>
  void operator()(const pinocchio::JointModelBase<JointModel> &) const
  {
    JointModel jmodel = init<JointModel>::run();
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

struct TestJointModelCast : TestJointModel<TestJointModelCast>
{
  template<typename JointModel>
  static void test(const JointModelBase<JointModel> & jmodel)
  {
    typedef typename JointModel::Scalar Scalar;
    BOOST_CHECK(jmodel == jmodel);
    BOOST_CHECK(jmodel.template cast<Scalar>().isEqual(jmodel));
    BOOST_CHECK(jmodel.template cast<Scalar>() == jmodel);
    BOOST_CHECK_MESSAGE(
      jmodel.template cast<long double>().template cast<double>() == jmodel,
      std::string("Error when casting " + jmodel.shortname() + " from long double to double."));
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
