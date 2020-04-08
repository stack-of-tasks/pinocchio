//
// Copyright(c) 2015-2020 CNRS INRIA
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

template<typename JointModel_> struct init;

template<typename JointModel_>
struct init
{
  static JointModel_ run()
  {
    JointModel_ jmodel;
    jmodel.setIndexes(0,0,0);
    return jmodel;
  }
};

template<typename Scalar, int Options>
struct init<pinocchio::JointModelRevoluteUnalignedTpl<Scalar,Options> >
{
  typedef pinocchio::JointModelRevoluteUnalignedTpl<Scalar,Options> JointModel;
  
  static JointModel run()
  {
    typedef typename JointModel::Vector3 Vector3;
    JointModel jmodel(Vector3::Random().normalized());
    
    jmodel.setIndexes(0,0,0);
    return jmodel;
  }
};

template<typename Scalar, int Options>
struct init<pinocchio::JointModelRevoluteUnboundedUnalignedTpl<Scalar,Options> >
{
  typedef pinocchio::JointModelRevoluteUnboundedUnalignedTpl<Scalar,Options> JointModel;
  
  static JointModel run()
  {
    typedef typename JointModel::Vector3 Vector3;
    JointModel jmodel(Vector3::Random().normalized());
    
    jmodel.setIndexes(0,0,0);
    return jmodel;
  }
};

template<typename Scalar, int Options>
struct init<pinocchio::JointModelPrismaticUnalignedTpl<Scalar,Options> >
{
  typedef pinocchio::JointModelPrismaticUnalignedTpl<Scalar,Options> JointModel;
  
  static JointModel run()
  {
    typedef typename JointModel::Vector3 Vector3;
    JointModel jmodel(Vector3::Random().normalized());
    
    jmodel.setIndexes(0,0,0);
    return jmodel;
  }
};

template<typename Scalar, int Options, template<typename,int> class JointCollection>
struct init<pinocchio::JointModelTpl<Scalar,Options,JointCollection> >
{
  typedef pinocchio::JointModelTpl<Scalar,Options,JointCollection> JointModel;
  
  static JointModel run()
  {
    typedef pinocchio::JointModelRevoluteTpl<Scalar,Options,0> JointModelRX;
    JointModel jmodel((JointModelRX()));
    
    jmodel.setIndexes(0,0,0);
    return jmodel;
  }
};

template<typename Scalar, int Options, template<typename,int> class JointCollection>
struct init<pinocchio::JointModelCompositeTpl<Scalar,Options,JointCollection> >
{
  typedef pinocchio::JointModelCompositeTpl<Scalar,Options,JointCollection> JointModel;
  
  static JointModel run()
  {
    typedef pinocchio::JointModelRevoluteTpl<Scalar,Options,0> JointModelRX;
    typedef pinocchio::JointModelRevoluteTpl<Scalar,Options,1> JointModelRY;
    JointModel jmodel((JointModelRX()));
    jmodel.addJoint(JointModelRY());
    
    jmodel.setIndexes(0,0,0);
    return jmodel;
  }
};

template<typename JointModel_>
struct init<pinocchio::JointModelMimic<JointModel_> >
{
  typedef pinocchio::JointModelMimic<JointModel_> JointModel;
  
  static JointModel run()
  {
    JointModel_ jmodel_ref = init<JointModel_>::run();
    
    JointModel jmodel(jmodel_ref,1.,0.);
    jmodel.setIndexes(0,0,0);
    
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
  
BOOST_AUTO_TEST_CASE(isEqual)
{
  typedef JointCollectionDefault::JointModelVariant JointModelVariant;
  boost::mpl::for_each<JointModelVariant::types>(TestJointModelIsEqual());
  
  JointModelRX joint_revolutex;
  JointModelRY joint_revolutey;
  
  BOOST_CHECK(joint_revolutex != joint_revolutey);
  
  JointModel jmodelx(joint_revolutex);
  jmodelx.setIndexes(0,0,0);
  TestJointModelIsEqual()(JointModel());
  
  JointModel jmodel_any;
  BOOST_CHECK(jmodel_any != jmodelx);
}

struct TestJointModelCast : TestJointModel<TestJointModelCast>
{
  template<typename JointModel>
  static void test(const JointModelBase<JointModel> & jmodel)
  {
    typedef typename JointModel::Scalar Scalar;
    BOOST_CHECK(jmodel.template cast<Scalar>() == jmodel);
    BOOST_CHECK(jmodel.template cast<long double>().template cast<double>() == jmodel);
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
