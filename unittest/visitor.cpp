//
// Copyright (c) 2015-2019 CNRS INRIA
//

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/visitor.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

namespace bf = boost::fusion;

struct SimpleVisitor
: public pinocchio::fusion::JointUnaryVisitorBase<SimpleVisitor>
{

  typedef bf::vector<const pinocchio::Model &,
                     pinocchio::Data &,
                     pinocchio::JointIndex> ArgsType;
  
  template<typename JointModel>
  static void algo(const pinocchio::JointModelBase<JointModel> & jmodel,
                   pinocchio::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                   const pinocchio::Model & model,
                   pinocchio::Data & data,
                   pinocchio::JointIndex jindex)
  {
    PINOCCHIO_UNUSED_VARIABLE(jdata);
    PINOCCHIO_UNUSED_VARIABLE(model);
    PINOCCHIO_UNUSED_VARIABLE(data);
    PINOCCHIO_UNUSED_VARIABLE(jindex);
    std::cout << "joint name: " << jmodel.shortname() << std::endl;
  }
};

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

template<typename JointModel_> struct init;

template<typename JointModel_>
struct init
{
  static JointModel_ run(const pinocchio::Model &/* model*/)
  {
    JointModel_ jmodel;
    return jmodel;
  }
};

template<typename Scalar, int Options>
struct init<pinocchio::JointModelRevoluteUnalignedTpl<Scalar,Options> >
{
  typedef pinocchio::JointModelRevoluteUnalignedTpl<Scalar,Options> JointModel;
  
  static JointModel run(const pinocchio::Model &/* model*/)
  {
    typedef typename JointModel::Vector3 Vector3;
    JointModel jmodel(Vector3::Random().normalized());
    
    return jmodel;
  }
};

template<typename Scalar, int Options>
struct init<pinocchio::JointModelRevoluteUnboundedUnalignedTpl<Scalar,Options> >
{
  typedef pinocchio::JointModelRevoluteUnboundedUnalignedTpl<Scalar,Options> JointModel;
  
  static JointModel run(const pinocchio::Model &/* model*/)
  {
    typedef typename JointModel::Vector3 Vector3;
    JointModel jmodel(Vector3::Random().normalized());
    
    return jmodel;
  }
};

template<typename Scalar, int Options>
struct init<pinocchio::JointModelPrismaticUnalignedTpl<Scalar,Options> >
{
  typedef pinocchio::JointModelPrismaticUnalignedTpl<Scalar,Options> JointModel;
  
  static JointModel run(const pinocchio::Model &/* model*/)
  {
    typedef typename JointModel::Vector3 Vector3;
    JointModel jmodel(Vector3::Random().normalized());
    
    return jmodel;
  }
};

template<typename Scalar, int Options, template<typename,int> class JointCollection>
struct init<pinocchio::JointModelTpl<Scalar,Options,JointCollection> >
{
  typedef pinocchio::JointModelTpl<Scalar,Options,JointCollection> JointModel;
  
  static JointModel run(const pinocchio::Model &/* model*/)
  {
    typedef pinocchio::JointModelRevoluteTpl<Scalar,Options,0> JointModelRX;
    JointModel jmodel((JointModelRX()));
    
    return jmodel;
  }
};

template<typename Scalar, int Options, template<typename,int> class JointCollection>
struct init<pinocchio::JointModelCompositeTpl<Scalar,Options,JointCollection> >
{
  typedef pinocchio::JointModelCompositeTpl<Scalar,Options,JointCollection> JointModel;
  
  static JointModel run(const pinocchio::Model &/* model*/)
  {
    typedef pinocchio::JointModelRevoluteTpl<Scalar,Options,0> JointModelRX;
    typedef pinocchio::JointModelRevoluteTpl<Scalar,Options,1> JointModelRY;
    typedef pinocchio::JointModelRevoluteTpl<Scalar,Options,2> JointModelRZ;
    
    JointModel jmodel(JointModelRX(),pinocchio::SE3::Random());
    jmodel.addJoint(JointModelRY(),pinocchio::SE3::Random());
    jmodel.addJoint(JointModelRZ(),pinocchio::SE3::Random());
    
    return jmodel;
  }
};

template<typename JointModel_>
struct init<pinocchio::JointModelMimic<JointModel_> >
{
  typedef pinocchio::JointModelMimic<JointModel_> JointModel;
  
  static JointModel run(const pinocchio::Model & model)
  {
    const pinocchio::JointIndex joint_id = model.getJointId(JointModel_::classname());
    std::cout << "joint_id: " << joint_id << std::endl;
    
    JointModel jmodel(boost::get<JointModel_>(model.joints[joint_id]),1.,0.);
    
    return jmodel;
  }
};

struct AppendJointToModel
{
  AppendJointToModel(pinocchio::Model & model) : model(model) {}
  
  template<typename JointModel>
  void operator()(const pinocchio::JointModelBase<JointModel> &) const
  {
    JointModel jmodel = init<JointModel>::run(model);
    model.addJoint(model.joints.size()-1,jmodel,
                   pinocchio::SE3::Random(),jmodel.classname());
  }
  
  pinocchio::Model & model;
};

BOOST_AUTO_TEST_CASE(test_run_over_all_joints)
{
  using namespace pinocchio;

  typedef JointCollectionDefault::JointModelVariant Variant;
  
  Model model;
  boost::mpl::for_each<Variant::types>(AppendJointToModel(model));
  Data data(model);
  
  for( Model::JointIndex i=1;i<(Model::JointIndex)model.njoints;++i )
  {
    SimpleVisitor::run(model.joints[i],data.joints[i],
                       SimpleVisitor::ArgsType(model,data,i));
  }
}

BOOST_AUTO_TEST_SUITE_END()
