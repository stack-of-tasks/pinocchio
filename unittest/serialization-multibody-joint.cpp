//
// Copyright (c) 2026 INRIA
//

#include <iostream>

#include "pinocchio/multibody.hpp"
#include "pinocchio/multibody/joint.hpp"
#include "pinocchio/multibody/sample-models.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"

#include "pinocchio/serialization.hpp"
#include "serialization.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

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

template<typename Scalar, int Options, int axis>
struct init<pinocchio::JointModelHelicalTpl<Scalar, Options, axis>>
{
  typedef pinocchio::JointModelHelicalTpl<Scalar, Options, axis> JointModel;

  static JointModel run()
  {
    JointModel jmodel(static_cast<Scalar>(0.0));

    jmodel.setIndexes(0, 0, 0);
    return jmodel;
  }
};

template<typename Scalar, int Options>
struct init<pinocchio::JointModelEllipsoidTpl<Scalar, Options>>
{
  typedef pinocchio::JointModelEllipsoidTpl<Scalar, Options> JointModel;

  static JointModel run()
  {
    JointModel jmodel(
      static_cast<Scalar>(0.01), static_cast<Scalar>(0.02), static_cast<Scalar>(0.03));

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

template<typename Scalar, int Options>
struct init<pinocchio::JointModelUniversalTpl<Scalar, Options>>
{
  typedef pinocchio::JointModelUniversalTpl<Scalar, Options> JointModel;

  static JointModel run()
  {
    JointModel jmodel(pinocchio::XAxis::vector(), pinocchio::YAxis::vector());

    jmodel.setIndexes(0, 0, 0);
    return jmodel;
  }
};

template<typename Scalar, int Options>
struct init<pinocchio::JointModelSplineTpl<Scalar, Options>>
{
  typedef pinocchio::JointModelSplineTpl<Scalar, Options> JointModel;

  static JointModel run()
  {
    std::vector<pinocchio::SE3> ctrlFrames;
    for (int k = 0; k < 5; k++)
      ctrlFrames.push_back(pinocchio::SE3::Random());

    JointModel jmodel(ctrlFrames, 3);
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

struct TestJointModel
{
  template<typename JointModel>
  void operator()(const pinocchio::JointModelBase<JointModel> &) const
  {
    JointModel jmodel = init<JointModel>::run();
    test(jmodel);
  }

  template<typename JointType>
  static void test(JointType & jmodel)
  {
    generic_test(jmodel, TEST_SERIALIZATION_FOLDER "/Joint", "jmodel");
  }
};

BOOST_AUTO_TEST_CASE(test_multibody_joints_model_serialization)
{
  using namespace pinocchio;
  boost::mpl::for_each<JointModelVariant::types>(TestJointModel());
}

struct TestJointTransform
{
  template<typename JointModel>
  void operator()(const pinocchio::JointModelBase<JointModel> &) const
  {
    typedef typename JointModel::JointDerived JointDerived;
    typedef typename pinocchio::traits<JointDerived>::Transformation_t Transform;
    typedef typename pinocchio::traits<JointDerived>::Constraint_t Constraint;
    typedef typename pinocchio::traits<JointDerived>::JointDataDerived JointData;
    typedef pinocchio::JointDataBase<JointData> JointDataBase;
    JointModel jmodel = init<JointModel>::run();

    JointData jdata = jmodel.createData();
    JointDataBase & jdata_base = static_cast<JointDataBase &>(jdata);

    typedef typename pinocchio::LieGroup<JointModel>::type LieGroupType;
    LieGroupType lg;

    Eigen::VectorXd lb(Eigen::VectorXd::Constant(jmodel.nq(), -1.));
    Eigen::VectorXd ub(Eigen::VectorXd::Constant(jmodel.nq(), 1.));

    Eigen::VectorXd q_random = lg.randomConfiguration(lb, ub);

    jmodel.calc(jdata, q_random);
    Transform & m = jdata_base.M();
    test(m);

    Constraint & S = jdata_base.S();
    test(S);
  }

  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  void operator()(const pinocchio::JointModelCompositeTpl<Scalar, Options, JointCollectionTpl> &)
  {
    // Do nothing
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollection>
  void operator()(const pinocchio::JointModelMimicTpl<Scalar, Options, JointCollection> &)
  {
    // Do nothing
  }

  template<typename Transform>
  static void test(Transform & m)
  {
    generic_test(m, TEST_SERIALIZATION_FOLDER "/JointTransform", "transform");
  }
};

BOOST_AUTO_TEST_CASE(test_multibody_joints_transform_serialization)
{
  using namespace pinocchio;
  boost::mpl::for_each<JointModelVariant::types>(TestJointTransform());
}

struct TestJointMotion
{
  template<typename JointModel>
  void operator()(const pinocchio::JointModelBase<JointModel> &) const
  {
    typedef typename JointModel::JointDerived JointDerived;
    typedef typename pinocchio::traits<JointDerived>::Motion_t Motion;
    typedef typename pinocchio::traits<JointDerived>::Bias_t Bias;
    typedef typename pinocchio::traits<JointDerived>::JointDataDerived JointData;
    typedef pinocchio::JointDataBase<JointData> JointDataBase;
    JointModel jmodel = init<JointModel>::run();

    JointData jdata = jmodel.createData();
    JointDataBase & jdata_base = static_cast<JointDataBase &>(jdata);

    typedef typename pinocchio::LieGroup<JointModel>::type LieGroupType;
    LieGroupType lg;

    Eigen::VectorXd lb(Eigen::VectorXd::Constant(jmodel.nq(), -1.));
    Eigen::VectorXd ub(Eigen::VectorXd::Constant(jmodel.nq(), 1.));

    Eigen::VectorXd q_random = lg.randomConfiguration(lb, ub);
    Eigen::VectorXd v_random = Eigen::VectorXd::Random(jmodel.nv());

    jmodel.calc(jdata, q_random, v_random);
    Motion & m = jdata_base.v();

    test(m);

    Bias & b = jdata_base.c();
    test(b);
  }

  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  void operator()(const pinocchio::JointModelCompositeTpl<Scalar, Options, JointCollectionTpl> &)
  {
    // Do nothing
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollection>
  void operator()(const pinocchio::JointModelMimicTpl<Scalar, Options, JointCollection> &)
  {
    // Do nothing
  }

  template<typename Motion>
  static void test(Motion & m)
  {
    generic_test(m, TEST_SERIALIZATION_FOLDER "/JointMotion", "motion");
  }
};

BOOST_AUTO_TEST_CASE(test_multibody_joints_motion_serialization)
{
  using namespace pinocchio;
  boost::mpl::for_each<JointModelVariant::types>(TestJointMotion());
}

struct TestJointData
{
  template<typename JointModel>
  void operator()(const pinocchio::JointModelBase<JointModel> &) const
  {
    typedef typename JointModel::JointDerived JointDerived;
    typedef typename pinocchio::traits<JointDerived>::JointDataDerived JointData;
    JointModel jmodel = init<JointModel>::run();

    JointData jdata = jmodel.createData();

    typedef typename pinocchio::LieGroup<JointModel>::type LieGroupType;
    LieGroupType lg;

    Eigen::VectorXd lb(Eigen::VectorXd::Constant(jmodel.nq(), -1.));
    Eigen::VectorXd ub(Eigen::VectorXd::Constant(jmodel.nq(), 1.));

    Eigen::VectorXd q_random = lg.randomConfiguration(lb, ub);
    Eigen::VectorXd v_random = Eigen::VectorXd::Random(jmodel.nv());

    jmodel.calc(jdata, q_random, v_random);
    pinocchio::Inertia::Matrix6 I(pinocchio::Inertia::Matrix6::Identity());
    jmodel.calc_aba(jdata, Eigen::VectorXd::Zero(jmodel.nv()), I, false);
    test(jdata);
  }

  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  void operator()(const pinocchio::JointModelCompositeTpl<Scalar, Options, JointCollectionTpl> &)
  {
    typedef pinocchio::JointModelCompositeTpl<Scalar, Options, JointCollectionTpl> JointModel;
    typedef typename JointModel::JointDerived JointDerived;
    typedef typename pinocchio::traits<JointDerived>::JointDataDerived JointData;

    JointModel jmodel_build = init<JointModel>::run();

    pinocchio::Model model;
    model.addJoint(0, jmodel_build, pinocchio::SE3::Random(), "model");
    model.lowerPositionLimit.fill(-1.);
    model.upperPositionLimit.fill(1.);
    model.positionLimitMargin.fill(1.5);

    JointModel & jmodel = boost::get<JointModel>(model.joints[1]);
    Eigen::VectorXd q_random = pinocchio::randomConfiguration(model);
    Eigen::VectorXd v_random = Eigen::VectorXd::Random(model.nv);

    pinocchio::Data data(model);
    JointData & jdata = boost::get<JointData>(data.joints[1]);

    jmodel.calc(jdata, q_random, v_random);
    pinocchio::Inertia::Matrix6 I(pinocchio::Inertia::Matrix6::Identity());
    jmodel.calc_aba(jdata, Eigen::VectorXd::Zero(jmodel.nv()), I, false);

    test(jdata);
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollection>
  void operator()(const pinocchio::JointModelMimicTpl<Scalar, Options, JointCollection> &)
  {
    typedef pinocchio::JointModelMimicTpl<Scalar, Options, JointCollection> JointModel;
    typedef typename JointModel::JointDerived JointDerived;
    typedef typename pinocchio::traits<JointDerived>::JointDataDerived JointData;

    JointModel jmodel = init<JointModel>::run();
    JointData jdata = jmodel.createData();

    Eigen::VectorXd q_random = Eigen::VectorXd::Random(jmodel.jmodel().nq());
    Eigen::VectorXd v_random = Eigen::VectorXd::Random(jmodel.jmodel().nv());
    jmodel.calc(jdata, q_random, v_random);

    test(jdata);
  }

  template<typename JointData>
  static void test(JointData & joint_data)
  {
    generic_test(joint_data, TEST_SERIALIZATION_FOLDER "/JointData", "data");
  }
};

BOOST_AUTO_TEST_CASE(test_multibody_joints_data_serialization)
{
  using namespace pinocchio;
  boost::mpl::for_each<JointModelVariant::types>(TestJointData());
}

BOOST_AUTO_TEST_CASE(test_model_serialization)
{
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model);

  generic_test(model, TEST_SERIALIZATION_FOLDER "/Model", "Model");
}

BOOST_AUTO_TEST_CASE(test_throw_extension)
{
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model);

  const std::string & fake_filename = "this_is_a_fake_filename";

  {
    const std::string complete_filename = fake_filename + ".txt";
    BOOST_REQUIRE_THROW(loadFromText(model, complete_filename), std::invalid_argument);
  }

  saveToText(model, TEST_SERIALIZATION_FOLDER "/model.txt");
  saveToXML(model, TEST_SERIALIZATION_FOLDER "/model.xml", "model");
  saveToBinary(model, TEST_SERIALIZATION_FOLDER "/model.bin");

  {
    const std::string complete_filename = fake_filename + ".txte";

    BOOST_REQUIRE_THROW(loadFromText(model, complete_filename), std::invalid_argument);
  }

  {
    const std::string complete_filename = fake_filename + ".xmle";
    BOOST_REQUIRE_THROW(loadFromXML(model, complete_filename, "model"), std::invalid_argument);
  }

  {
    const std::string complete_filename = fake_filename + ".bine";
    BOOST_REQUIRE_THROW(loadFromBinary(model, complete_filename), std::invalid_argument);
  }
}

BOOST_AUTO_TEST_CASE(test_data_serialization)
{
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model);

  Data data(model);

  generic_test(data, TEST_SERIALIZATION_FOLDER "/Data", "Data");
}

BOOST_AUTO_TEST_SUITE_END()
