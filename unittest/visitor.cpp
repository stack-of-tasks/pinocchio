//
// Copyright (c) 2015-2019 CNRS INRIA
//

#include "utils/joints-init.hpp"

#include "pinocchio/multibody.hpp"
#include "pinocchio/multibody/visitor.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

namespace bf = boost::fusion;

struct SimpleUnaryVisitor1 : public pinocchio::fusion::JointUnaryVisitorBase<SimpleUnaryVisitor1>
{

  typedef bf::vector<const pinocchio::Model &, pinocchio::Data &, pinocchio::JointIndex> ArgsType;

  template<typename JointModel>
  static void algo(
    const pinocchio::JointModelBase<JointModel> & jmodel,
    pinocchio::JointDataBase<typename JointModel::JointDataDerived> & jdata,
    const pinocchio::Model & model,
    pinocchio::Data & data,
    pinocchio::JointIndex jindex)
  {
    PINOCCHIO_UNUSED_VARIABLE(jdata);
    PINOCCHIO_UNUSED_VARIABLE(model);
    PINOCCHIO_UNUSED_VARIABLE(data);
    BOOST_CHECK(jindex == jmodel.id());
    std::cout << "joint name: " << jmodel.shortname() << std::endl;
  }
};

struct SimpleUnaryVisitor2 : public pinocchio::fusion::JointUnaryVisitorBase<SimpleUnaryVisitor2>
{

  typedef bf::vector<const pinocchio::Model &, pinocchio::Data &, pinocchio::JointIndex> ArgsType;

  template<typename JointModel>
  static void algo(
    const pinocchio::JointModelBase<JointModel> & jmodel,
    const pinocchio::Model & model,
    pinocchio::Data & data,
    pinocchio::JointIndex jindex)
  {
    PINOCCHIO_UNUSED_VARIABLE(model);
    PINOCCHIO_UNUSED_VARIABLE(data);
    PINOCCHIO_UNUSED_VARIABLE(jindex);
    BOOST_CHECK(jindex == jmodel.id());
    std::cout << "joint name: " << jmodel.shortname() << std::endl;
  }
};

struct SimpleUnaryVisitor3 : public pinocchio::fusion::JointUnaryVisitorBase<SimpleUnaryVisitor3>
{

  template<typename JointModel>
  static void algo(const pinocchio::JointModelBase<JointModel> & jmodel)
  {
    BOOST_CHECK(!jmodel.shortname().empty());
    std::cout << "joint name: " << jmodel.shortname() << std::endl;
  }
};

struct SimpleUnaryVisitor4 : public pinocchio::fusion::JointUnaryVisitorBase<SimpleUnaryVisitor4>
{

  template<typename JointModel>
  static void algo(
    const pinocchio::JointModelBase<JointModel> & jmodel,
    pinocchio::JointDataBase<typename JointModel::JointDataDerived> & jdata)
  {
    PINOCCHIO_UNUSED_VARIABLE(jdata);
    BOOST_CHECK(!jmodel.shortname().empty());
    std::cout << "joint name: " << jmodel.shortname() << std::endl;
  }
};

struct SimpleBinaryVisitor1 : public pinocchio::fusion::JointBinaryVisitorBase<SimpleBinaryVisitor1>
{

  typedef bf::vector<const pinocchio::Model &, pinocchio::Data &, pinocchio::JointIndex> ArgsType;

  template<typename JointModel1, typename JointModel2>
  static void algo(
    const pinocchio::JointModelBase<JointModel1> & jmodel1,
    const pinocchio::JointModelBase<JointModel2> & jmodel2,
    pinocchio::JointDataBase<typename JointModel1::JointDataDerived> & jdata1,
    pinocchio::JointDataBase<typename JointModel2::JointDataDerived> & jdata2,
    const pinocchio::Model & model,
    pinocchio::Data & data,
    pinocchio::JointIndex jindex)
  {
    PINOCCHIO_UNUSED_VARIABLE(jdata1);
    PINOCCHIO_UNUSED_VARIABLE(jdata2);
    PINOCCHIO_UNUSED_VARIABLE(model);
    PINOCCHIO_UNUSED_VARIABLE(data);
    PINOCCHIO_UNUSED_VARIABLE(jindex);
    BOOST_CHECK(jindex == jmodel1.id());
    BOOST_CHECK(jindex == jmodel2.id());
    std::cout << "joint1 name: " << jmodel1.shortname() << std::endl;
    std::cout << "joint2 name: " << jmodel2.shortname() << std::endl;
  }
};

struct SimpleBinaryVisitor2 : public pinocchio::fusion::JointBinaryVisitorBase<SimpleBinaryVisitor2>
{

  typedef bf::vector<const pinocchio::Model &, pinocchio::Data &, pinocchio::JointIndex> ArgsType;

  template<typename JointModel1, typename JointModel2>
  static void algo(
    const pinocchio::JointModelBase<JointModel1> & jmodel1,
    const pinocchio::JointModelBase<JointModel2> & jmodel2,
    const pinocchio::Model & model,
    pinocchio::Data & data,
    pinocchio::JointIndex jindex)
  {
    PINOCCHIO_UNUSED_VARIABLE(model);
    PINOCCHIO_UNUSED_VARIABLE(data);
    PINOCCHIO_UNUSED_VARIABLE(jindex);
    BOOST_CHECK(jindex == jmodel1.id());
    BOOST_CHECK(jindex == jmodel2.id());
    std::cout << "joint1 name: " << jmodel1.shortname() << std::endl;
    std::cout << "joint2 name: " << jmodel2.shortname() << std::endl;
  }
};

struct SimpleBinaryVisitor3 : public pinocchio::fusion::JointBinaryVisitorBase<SimpleBinaryVisitor3>
{
  template<typename JointModel1, typename JointModel2>
  static void algo(
    const pinocchio::JointModelBase<JointModel1> & jmodel1,
    const pinocchio::JointModelBase<JointModel2> & jmodel2)
  {
    BOOST_CHECK(!jmodel1.shortname().empty());
    BOOST_CHECK(!jmodel2.shortname().empty());
    std::cout << "joint1 name: " << jmodel1.shortname() << std::endl;
    std::cout << "joint2 name: " << jmodel2.shortname() << std::endl;
  }
};

struct SimpleBinaryVisitor4 : public pinocchio::fusion::JointBinaryVisitorBase<SimpleBinaryVisitor4>
{
  template<typename JointModel1, typename JointModel2>
  static void algo(
    const pinocchio::JointModelBase<JointModel1> & jmodel1,
    const pinocchio::JointModelBase<JointModel2> & jmodel2,
    pinocchio::JointDataBase<typename JointModel1::JointDataDerived> & jdata1,
    pinocchio::JointDataBase<typename JointModel2::JointDataDerived> & jdata2)
  {
    PINOCCHIO_UNUSED_VARIABLE(jdata1);
    PINOCCHIO_UNUSED_VARIABLE(jdata2);
    BOOST_CHECK(!jmodel1.shortname().empty());
    BOOST_CHECK(!jmodel2.shortname().empty());
    std::cout << "joint1 name: " << jmodel1.shortname() << std::endl;
    std::cout << "joint2 name: " << jmodel2.shortname() << std::endl;
    std::cout << "jdata1 name: " << jdata1.classname() << std::endl;
    std::cout << "jdata2 name: " << jdata2.classname() << std::endl;
  }
};

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

struct AppendJointToModel
{
  AppendJointToModel(pinocchio::Model & model)
  : model(model)
  {
  }

  template<typename JointModel>
  void operator()(const pinocchio::JointModelBase<JointModel> &) const
  {
    auto jmodel = pinocchio::init<JointModel>::run().jmodel;
    model.addJoint(model.joints.size() - 1, jmodel, pinocchio::SE3::Random(), jmodel.classname());
  }

  pinocchio::Model & model;
};

BOOST_AUTO_TEST_CASE(test_run_over_all_joints_unary_visitor)
{
  using namespace pinocchio;

  typedef JointCollectionDefault::JointModelVariant Variant;

  Model model;
  boost::mpl::for_each<Variant::types>(AppendJointToModel(model));
  Data data(model);

  for (Model::JointIndex i = 1; i < (Model::JointIndex)model.njoints; ++i)
  {
    SimpleUnaryVisitor1::run(
      model.joints[i], data.joints[i], SimpleUnaryVisitor1::ArgsType(model, data, i));
  }

  for (Model::JointIndex i = 1; i < (Model::JointIndex)model.njoints; ++i)
  {
    SimpleUnaryVisitor2::run(model.joints[i], SimpleUnaryVisitor2::ArgsType(model, data, i));
  }

  for (Model::JointIndex i = 1; i < (Model::JointIndex)model.njoints; ++i)
  {
    SimpleUnaryVisitor3::run(model.joints[i]);
  }

  for (Model::JointIndex i = 1; i < (Model::JointIndex)model.njoints; ++i)
  {
    SimpleUnaryVisitor4::run(model.joints[i], data.joints[i]);
  }
}

BOOST_AUTO_TEST_CASE(test_run_over_all_joints_binary_visitor)
{
  using namespace pinocchio;

  typedef JointCollectionDefault::JointModelVariant Variant;

  Model model;
  boost::mpl::for_each<Variant::types>(AppendJointToModel(model));
  Data data(model);

  for (Model::JointIndex i = 1; i < (Model::JointIndex)model.njoints; ++i)
  {
    SimpleBinaryVisitor1::run(
      model.joints[i], model.joints[i], data.joints[i], data.joints[i],
      SimpleBinaryVisitor1::ArgsType(model, data, i));
  }

  for (Model::JointIndex i = 1; i < (Model::JointIndex)model.njoints; ++i)
  {
    SimpleBinaryVisitor2::run(
      model.joints[i], model.joints[i], SimpleBinaryVisitor2::ArgsType(model, data, i));
  }

  for (Model::JointIndex i = 1; i < (Model::JointIndex)model.njoints; ++i)
  {
    SimpleBinaryVisitor3::run(model.joints[i], model.joints[i]);
  }

  for (Model::JointIndex i = 1; i < (Model::JointIndex)model.njoints; ++i)
  {
    SimpleBinaryVisitor4::run(model.joints[i], model.joints[i], data.joints[i], data.joints[i]);
  }
}

BOOST_AUTO_TEST_SUITE_END()
