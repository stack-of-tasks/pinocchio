//
// Copyright (c) 2019-2023 INRIA
//

#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/geometry.hpp"

#include "pinocchio/serialization/fwd.hpp"
#include "pinocchio/serialization/archive.hpp"

#include "pinocchio/serialization/spatial.hpp"

#include "pinocchio/serialization/frame.hpp"

#include "pinocchio/serialization/joints.hpp"
#include "pinocchio/serialization/model.hpp"
#include "pinocchio/serialization/data.hpp"

#include "pinocchio/serialization/geometry.hpp"

#include "pinocchio/multibody/sample-models.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

template<typename T1, typename T2 = T1>
struct call_equality_op
{
  static bool run(const T1 & v1, const T2 & v2)
  {
    return v1 == v2;
  }
};

template<typename T>
bool run_call_equality_op(const T & v1, const T & v2)
{
  return call_equality_op<T, T>::run(v1, v2);
}

// Bug fix in Eigen::Tensor
#ifdef PINOCCHIO_WITH_EIGEN_TENSOR_MODULE
template<typename Scalar, int NumIndices, int Options, typename IndexType>
struct call_equality_op<pinocchio::Tensor<Scalar, NumIndices, Options, IndexType>>
{
  typedef pinocchio::Tensor<Scalar, NumIndices, Options, IndexType> T;

  static bool run(const T & v1, const T & v2)
  {
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1, Options> VectorXd;
    Eigen::Map<const VectorXd> map1(v1.data(), v1.size(), 1);
    Eigen::Map<const VectorXd> map2(v2.data(), v2.size(), 1);
    return map1 == map2;
  }
};
#endif

template<typename T>
struct empty_contructor_algo
{
  static T * run()
  {
    return new T();
  }
};

template<>
struct empty_contructor_algo<pinocchio::GeometryObject>
{
  static pinocchio::GeometryObject * run()
  {
    return new pinocchio::GeometryObject("", 0, 0, pinocchio::SE3::Identity(), nullptr);
  }
};

template<typename T>
T * empty_contructor()
{
  return empty_contructor_algo<T>::run();
}

template<typename T>
void generic_test(const T & object, const std::string & filename, const std::string & tag_name)
{
  using namespace pinocchio::serialization;

  // Load and save as TXT
  const std::string txt_filename = filename + ".txt";
  saveToText(object, txt_filename);

  {
    T & object_loaded = *empty_contructor<T>();
    loadFromText(object_loaded, txt_filename);

    // Check
    BOOST_CHECK(run_call_equality_op(object_loaded, object));

    delete &object_loaded;
  }

  // Load and save as string stream (TXT format)
  std::stringstream ss_out;
  saveToStringStream(object, ss_out);

  {
    T & object_loaded = *empty_contructor<T>();
    std::istringstream is(ss_out.str());
    loadFromStringStream(object_loaded, is);

    // Check
    BOOST_CHECK(run_call_equality_op(object_loaded, object));

    delete &object_loaded;
  }

  // Load and save as string
  std::string str_out = saveToString(object);

  {
    T & object_loaded = *empty_contructor<T>();
    std::string str_in(str_out);
    loadFromString(object_loaded, str_in);

    // Check
    BOOST_CHECK(run_call_equality_op(object_loaded, object));

    delete &object_loaded;
  }
  // Load and save as XML
  const std::string xml_filename = filename + ".xml";
  saveToXML(object, xml_filename, tag_name);

  {
    T & object_loaded = *empty_contructor<T>();
    loadFromXML(object_loaded, xml_filename, tag_name);
    // Check
    BOOST_CHECK(run_call_equality_op(object_loaded, object));

    delete &object_loaded;
  }

  // Load and save as binary
  const std::string bin_filename = filename + ".bin";
  saveToBinary(object, bin_filename);

  {
    T & object_loaded = *empty_contructor<T>();
    loadFromBinary(object_loaded, bin_filename);

    // Check
    BOOST_CHECK(run_call_equality_op(object_loaded, object));

    delete &object_loaded;
  }

  // Load and save as binary stream
  boost::asio::streambuf buffer;
  saveToBinary(object, buffer);

  {
    T & object_loaded = *empty_contructor<T>();
    loadFromBinary(object_loaded, buffer);

    // Check
    BOOST_CHECK(run_call_equality_op(object_loaded, object));

    delete &object_loaded;
  }

  // Load and save as static binary stream
  pinocchio::serialization::StaticBuffer static_buffer(100000000);
  saveToBinary(object, static_buffer);

  {
    T & object_loaded = *empty_contructor<T>();
    loadFromBinary(object_loaded, static_buffer);

    // Check
    BOOST_CHECK(run_call_equality_op(object_loaded, object));

    delete &object_loaded;
  }
}

BOOST_AUTO_TEST_CASE(test_static_buffer)
{
  using namespace pinocchio::serialization;
  const size_t size = 10000000;
  StaticBuffer static_buffer(size);
  BOOST_CHECK(size == static_buffer.size());

  const size_t new_size = 2 * size;
  static_buffer.resize(new_size);
  BOOST_CHECK(new_size == static_buffer.size());

  BOOST_CHECK(static_buffer.data() != NULL);
  BOOST_CHECK(reinterpret_cast<const StaticBuffer &>(static_buffer).data() != NULL);
  BOOST_CHECK(reinterpret_cast<const StaticBuffer &>(static_buffer).data() == static_buffer.data());
}

BOOST_AUTO_TEST_CASE(test_eigen_serialization)
{
  using namespace pinocchio;

  const Eigen::DenseIndex num_cols = 10;
  const Eigen::DenseIndex num_rows = 20;

  const Eigen::DenseIndex array_size = 3;

  Eigen::MatrixXd Mat = Eigen::MatrixXd::Random(num_rows, num_cols);
  generic_test(Mat, TEST_SERIALIZATION_FOLDER "/eigen_matrix", "matrix");

  Eigen::VectorXd Vec = Eigen::VectorXd::Random(num_rows * num_cols);
  generic_test(Vec, TEST_SERIALIZATION_FOLDER "/eigen_vector", "vector");

  Eigen::array<Eigen::DenseIndex, array_size> array = {1, 2, 3};
  generic_test(array, TEST_SERIALIZATION_FOLDER "/eigen_array", "array");

  const Eigen::DenseIndex tensor_size = 3;
  const Eigen::DenseIndex x_dim = 10, y_dim = 20, z_dim = 30;

  typedef pinocchio::Tensor<double, tensor_size> Tensor3x;
  Tensor3x tensor(x_dim, y_dim, z_dim);

  Eigen::Map<Eigen::VectorXd>(tensor.data(), tensor.size(), 1).setRandom();

  generic_test(tensor, TEST_SERIALIZATION_FOLDER "/eigen_tensor", "tensor");
}

BOOST_AUTO_TEST_CASE(test_spatial_serialization)
{
  using namespace pinocchio;

  SE3 M(SE3::Random());
  generic_test(M, TEST_SERIALIZATION_FOLDER "/SE3", "SE3");

  Motion m(Motion::Random());
  generic_test(m, TEST_SERIALIZATION_FOLDER "/Motion", "Motion");

  Force f(Force::Random());
  generic_test(f, TEST_SERIALIZATION_FOLDER "/Force", "Force");

  Symmetric3 S(Symmetric3::Random());
  generic_test(S, TEST_SERIALIZATION_FOLDER "/Symmetric3", "Symmetric3");

  Inertia I(Inertia::Random());
  generic_test(I, TEST_SERIALIZATION_FOLDER "/Inertia", "Inertia");
}

BOOST_AUTO_TEST_CASE(test_multibody_serialization)
{
  using namespace pinocchio;

  Frame frame("frame", 0, 0, SE3::Random(), SENSOR);
  generic_test(frame, TEST_SERIALIZATION_FOLDER "/Frame", "Frame");
}

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

BOOST_AUTO_TEST_CASE(test_collision_pair)
{
  using namespace pinocchio;

  CollisionPair collision_pair(1, 2);
  generic_test(collision_pair, TEST_SERIALIZATION_FOLDER "/CollisionPair", "CollisionPair");
}

BOOST_AUTO_TEST_CASE(test_model_item)
{
  using namespace pinocchio;

  typedef GeometryObject::Base GeometryObject_ModelItem;
  GeometryObject_ModelItem model_item("pinocchio", 1, 2, SE3::Random());
  generic_test(model_item, TEST_SERIALIZATION_FOLDER "/ModelItem", "ModelItem");
}

BOOST_AUTO_TEST_CASE(test_geometry_object)
{
  using namespace pinocchio;

  {
    GeometryObject geometry_object("nullptr", 1, 2, SE3::Random(), nullptr);
    generic_test(geometry_object, TEST_SERIALIZATION_FOLDER "/GeometryObject", "GeometryObject");
  }

#ifdef PINOCCHIO_WITH_HPP_FCL
  {
    hpp::fcl::Box box(1., 2., 3.);
    generic_test(box, TEST_SERIALIZATION_FOLDER "/Box", "Box");
  }

  #if HPP_FCL_VERSION_AT_LEAST(3, 0, 0)
  {
    typedef GeometryObject::CollisionGeometryPtr CollisionGeometryPtr;
    CollisionGeometryPtr box_ptr = CollisionGeometryPtr(new hpp::fcl::Box(1., 2., 3.));
    GeometryObject geometry_object("box", 1, 2, SE3::Random(), box_ptr);
    generic_test(geometry_object, TEST_SERIALIZATION_FOLDER "/GeometryObject", "GeometryObject");
  }
  #endif // hpp-fcl >= 3.0.0
#endif
}

BOOST_AUTO_TEST_CASE(test_geometry_model_and_data_serialization)
{
  using namespace pinocchio;

  Model model;
  buildModels::humanoid(model);
  Data data(model);

  // Empty structures
  {
    GeometryModel geom_model;
    generic_test(geom_model, TEST_SERIALIZATION_FOLDER "/GeometryModel", "GeometryModel");

    GeometryData geom_data(geom_model);
    generic_test(geom_data, TEST_SERIALIZATION_FOLDER "/GeometryData", "GeometryData");
  }

#ifdef PINOCCHIO_WITH_HPP_FCL
  #if HPP_FCL_VERSION_AT_LEAST(3, 0, 0)
  {
    pinocchio::GeometryModel geom_model;
    pinocchio::buildModels::humanoidGeometries(model, geom_model);
    // Append new objects
    {
      using namespace hpp::fcl;
      BVHModel<OBBRSS> * bvh_ptr = new BVHModel<OBBRSS>();
      //      bvh_ptr->beginModel();
      //      bvh_ptr->addSubModel(p1, t1);
      //      bvh_ptr->endModel();

      GeometryObject obj_bvh(
        "bvh", 0, 0, SE3::Identity(), GeometryObject::CollisionGeometryPtr(bvh_ptr));
      geom_model.addGeometryObject(obj_bvh);

      const double min_altitude = -1.;
      const double x_dim = 1., y_dim = 2.;
      const Eigen::DenseIndex nx = 100, ny = 200;
      const Eigen::MatrixXd heights = Eigen::MatrixXd::Random(ny, nx);

      HeightField<OBBRSS> * hfield_ptr =
        new HeightField<OBBRSS>(x_dim, y_dim, heights, min_altitude);

      GeometryObject obj_hfield(
        "hfield", 0, 0, SE3::Identity(), GeometryObject::CollisionGeometryPtr(hfield_ptr));
      geom_model.addGeometryObject(obj_hfield);
    }
    generic_test(geom_model, TEST_SERIALIZATION_FOLDER "/GeometryModel", "GeometryModel");

    pinocchio::GeometryData geom_data(geom_model);
    const Eigen::VectorXd q = pinocchio::neutral(model);
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateGeometryPlacements(model, data, geom_model, geom_data, q);

    generic_test(geom_data, TEST_SERIALIZATION_FOLDER "/GeometryData", "GeometryData");
  }
  #endif // hpp-fcl >= 3.0.0
#endif   // PINOCCHIO_WITH_HPP_FCL
}

BOOST_AUTO_TEST_SUITE_END()
