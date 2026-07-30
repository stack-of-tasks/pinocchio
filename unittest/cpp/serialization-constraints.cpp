//
// Copyright (c) 2026 INRIA
//

#define BOOST_TEST_MODULE serialization_constraints

#include <iostream>

#include "pinocchio/multibody.hpp"
#include "pinocchio/multibody/joint.hpp"
#include "pinocchio/multibody/sample-models.hpp"

#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/check.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/constraints.hpp"

#include "pinocchio/serialization.hpp"
#include "serialization.hpp"

#include <boost/utility/binary.hpp>

#include <boost/test/unit_test.hpp>

template<typename DerivedConstraintModel>
struct JointLimitAndFrictionConstraintModelInitializer
{
  typedef pinocchio::Model Model;
  typedef pinocchio::JointIndex JointIndex;

  static DerivedConstraintModel run(const Model & model)
  {
    const std::string ee_name = "wrist2_joint";
    const JointIndex constraint_id = model.getJointId(ee_name);

    // get joint path to end-effector
    const Model::IndexVector & ee_support = model.supports[constraint_id];
    // get joint ids to put in the joint limit constraint (omit first joint as it is always the
    // universe)
    const Model::IndexVector active_joint_ids(ee_support.begin() + 1, ee_support.end());

    DerivedConstraintModel cmodel(model, active_joint_ids);
    cmodel.name = cmodel.classname();
    cmodel.setCompliance(Eigen::VectorXd::Random(cmodel.residualSize()));

    return cmodel;
  }
};

template<typename DerivedConstraintModel>
struct PointAndFrameConstraintModelInitializer
{
  typedef pinocchio::Model Model;
  typedef pinocchio::JointIndex JointIndex;
  typedef pinocchio::SE3 SE3;

  static DerivedConstraintModel run(const Model & model)
  {
    const std::string joint1_name = "elbow_joint";
    const JointIndex joint1_id = model.getJointId(joint1_name);

    const std::string joint2_name = "wrist2_joint";
    const JointIndex joint2_id = model.getJointId(joint2_name);

    DerivedConstraintModel cmodel(model, joint1_id, SE3::Random(), joint2_id, SE3::Random());
    cmodel.name = cmodel.classname();
    cmodel.setCompliance(Eigen::VectorXd::Random(cmodel.residualSize()));
    cmodel.baumgarte_corrector_parameters().Kd = 1.0;
    cmodel.baumgarte_corrector_parameters().Kp = 3.14;

    return cmodel;
  }
};

template<typename ConstraintModel, class = void>
struct initConstraint;

template<>
struct initConstraint<pinocchio::JointLimitConstraintModel, void>
{
  typedef pinocchio::Model Model;
  typedef pinocchio::JointLimitConstraintModel ConstraintModel;

  static ConstraintModel run(const Model & model)
  {
    // Note: JointLimitConstraintModel's constraint set is automatically constructed
    // uppon construction of the constraint model.
    ConstraintModel cmodel =
      JointLimitAndFrictionConstraintModelInitializer<ConstraintModel>::run(model);
    cmodel.baumgarte_corrector_parameters().Kd = 1.0;
    cmodel.baumgarte_corrector_parameters().Kp = 3.14;

    return cmodel;
  }
};

template<>
struct initConstraint<pinocchio::JointFrictionConstraintModel, void>
{
  typedef pinocchio::Model Model;
  typedef pinocchio::JointFrictionConstraintModel ConstraintModel;

  static ConstraintModel run(const Model & model)
  {
    // Note: The upper/lower bounds of JointFrictionConstraintModel's constraint set
    // need to be set after constructing the constraint model.
    ConstraintModel cmodel =
      JointLimitAndFrictionConstraintModelInitializer<ConstraintModel>::run(model);
    Eigen::VectorXd lb = -Eigen::VectorXd::Random(cmodel.residualSize()).array().abs();
    Eigen::VectorXd ub = Eigen::VectorXd::Random(cmodel.residualSize()).array().abs();
    cmodel.setFrictionLowerLimit(lb);
    cmodel.setFrictionUpperLimit(ub);
    return cmodel;
  }
};

template<>
struct initConstraint<pinocchio::PointAnchorConstraintModel, void>
{
  typedef pinocchio::Model Model;
  typedef pinocchio::PointAnchorConstraintModel ConstraintModel;

  static ConstraintModel run(const Model & model)
  {
    // Note: For point anchor constraints, no need to manually set the constraint set.
    ConstraintModel cmodel = PointAndFrameConstraintModelInitializer<ConstraintModel>::run(model);
    return cmodel;
  }
};

template<>
struct initConstraint<pinocchio::PointContactConstraintModel, void>
{
  typedef pinocchio::Model Model;
  typedef pinocchio::PointContactConstraintModel ConstraintModel;

  static ConstraintModel run(const Model & model)
  {
    // Note: For PointContact constraints, the friction coeff of the coulomb cone needs to be set.
    ConstraintModel cmodel = PointAndFrameConstraintModelInitializer<ConstraintModel>::run(model);
    cmodel.setFriction(0.1234);
    return cmodel;
  }
};

template<>
struct initConstraint<pinocchio::FrameAnchorConstraintModel, void>
{
  typedef pinocchio::Model Model;
  typedef pinocchio::FrameAnchorConstraintModel ConstraintModel;

  static ConstraintModel run(const Model & model)
  {
    // Note: For FrameAnchor constraints, no need to manually set the constraint set.
    ConstraintModel cmodel = PointAndFrameConstraintModelInitializer<ConstraintModel>::run(model);
    return cmodel;
  }
};

struct TestConstraintModel
{
  typedef pinocchio::Model Model;

  void operator()(boost::blank) const
  {
    // do nothing
  }

  template<typename ConstraintModel>
  void operator()(const pinocchio::ConstraintModelBase<ConstraintModel> &) const
  {
    Model model;
    pinocchio::buildModels::manipulator(model);
    ConstraintModel cmodel = initConstraint<ConstraintModel>::run(model);
    std::cout << cmodel << " : Begin testing." << std::endl;
    test(cmodel);
    std::cout << cmodel << " : End testing." << std::endl;
  }

  template<typename ConstraintModel>
  static void test(ConstraintModel & cmodel)
  {
    generic_test(cmodel, TEST_SERIALIZATION_FOLDER "/Constraint", "cmodel");
  }
};

BOOST_AUTO_TEST_CASE(test_constraints_model_serialization)
{
  using namespace pinocchio;
  boost::mpl::for_each<ConstraintModelVariant::types>(TestConstraintModel());
}

struct TestConstraintData
{
  typedef pinocchio::Model Model;
  typedef pinocchio::Data Data;

  void operator()(boost::blank) const
  {
    // do nothing
  }

  template<typename ConstraintData>
  void operator()(const pinocchio::ConstraintDataBase<ConstraintData> &) const
  {
    Model model;
    pinocchio::buildModels::manipulator(model);
    Data data(model);

    // run aba to populate data
    Eigen::VectorXd q = pinocchio::randomConfiguration(model);
    Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
    Eigen::VectorXd tau = Eigen::VectorXd::Random(model.nv);
    pinocchio::aba(model, data, q, v, tau, pinocchio::Convention::WORLD);

    typedef typename ConstraintData::ConstraintModel ConstraintModel;
    ConstraintModel cmodel = initConstraint<ConstraintModel>::run(model);
    ConstraintData cdata(cmodel);
    cmodel.calc(model, data, cdata);
    test(cdata);
  }

  template<typename ConstraintData>
  static void test(ConstraintData & cdata)
  {
    generic_test(cdata, TEST_SERIALIZATION_FOLDER "/Constraint", "cdata");
  }
};

BOOST_AUTO_TEST_CASE(test_constraints_data_serialization)
{
  using namespace pinocchio;
  boost::mpl::for_each<ConstraintDataVariant::types>(TestConstraintData());
}

BOOST_AUTO_TEST_CASE(test_constraint_model_variant)
{
  using namespace pinocchio;

  Model model;
  pinocchio::buildModels::manipulator(model);
  Data data(model);

  // run aba to populate data
  Eigen::VectorXd q = pinocchio::randomConfiguration(model);
  Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
  Eigen::VectorXd tau = Eigen::VectorXd::Random(model.nv);
  aba(model, data, q, v, tau, pinocchio::Convention::WORLD);

  std::vector<ConstraintModel> cmodels;
  std::vector<ConstraintData> cdatas;
  {
    JointLimitConstraintModel cmodel_ = initConstraint<JointLimitConstraintModel>::run(model);
    ConstraintModel cmodel(cmodel_);
    ConstraintData cdata(cmodel.createData());
    cmodel.calc(model, data, cdata);

    cmodels.push_back(cmodel);
    cdatas.push_back(cdata);

    generic_test(cmodel, TEST_SERIALIZATION_FOLDER "/Constraint", "cmodel_variant");
    // generic_test(cdata, TEST_SERIALIZATION_FOLDER "/Constraint", "cdata_variant");
  }
  {
    JointFrictionConstraintModel cmodel_ = initConstraint<JointFrictionConstraintModel>::run(model);
    ConstraintModel cmodel(cmodel_);
    ConstraintData cdata(cmodel.createData());
    cmodel.calc(model, data, cdata);

    cmodels.push_back(cmodel);
    cdatas.push_back(cdata);

    generic_test(cmodel, TEST_SERIALIZATION_FOLDER "/Constraint", "cmodel_variant");
    generic_test(cdata, TEST_SERIALIZATION_FOLDER "/Constraint", "cdata_variant");
  }
  {
    PointContactConstraintModel cmodel_ = initConstraint<PointContactConstraintModel>::run(model);
    ConstraintModel cmodel(cmodel_);
    ConstraintData cdata(cmodel.createData());
    cmodel.calc(model, data, cdata);

    cmodels.push_back(cmodel);
    cdatas.push_back(cdata);

    generic_test(cmodel, TEST_SERIALIZATION_FOLDER "/Constraint", "cmodel_variant");
    generic_test(cdata, TEST_SERIALIZATION_FOLDER "/Constraint", "cdata_variant");
  }
  {
    PointAnchorConstraintModel cmodel_ = initConstraint<PointAnchorConstraintModel>::run(model);
    ConstraintModel cmodel(cmodel_);
    ConstraintData cdata(cmodel.createData());
    cmodel.calc(model, data, cdata);

    cmodels.push_back(cmodel);
    cdatas.push_back(cdata);

    generic_test(cmodel, TEST_SERIALIZATION_FOLDER "/Constraint", "cmodel_variant");
    generic_test(cdata, TEST_SERIALIZATION_FOLDER "/Constraint", "cdata_variant");
  }
  {
    FrameAnchorConstraintModel cmodel_ = initConstraint<FrameAnchorConstraintModel>::run(model);
    ConstraintModel cmodel(cmodel_);
    ConstraintData cdata(cmodel.createData());
    cmodel.calc(model, data, cdata);

    cmodels.push_back(cmodel);
    cdatas.push_back(cdata);

    generic_test(cmodel, TEST_SERIALIZATION_FOLDER "/Constraint", "cmodel_variant");
    generic_test(cdata, TEST_SERIALIZATION_FOLDER "/Constraint", "cdata_variant");
  }

  // test vector of constraints
  for (ConstraintModel & cmodel : cmodels)
  {
    cmodel.setCompliance(Eigen::VectorXd::Random(cmodel.residualSize()));
  }
  generic_test(cmodels, TEST_SERIALIZATION_FOLDER "/Constraint", "cmodel_vector");
}
