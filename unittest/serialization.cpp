//
// Copyright (c) 2019 INRIA
//

#include "pinocchio/serialization/archive.hpp"

#include "pinocchio/serialization/eigen.hpp"
#include "pinocchio/serialization/spatial.hpp"

#include "pinocchio/serialization/frame.hpp"

#include "pinocchio/serialization/joints.hpp"
#include "pinocchio/serialization/model.hpp"

#include "pinocchio/parsers/sample-models.hpp"

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
  return call_equality_op<T,T>::run(v1,v2);
}

// Bug fix in Eigen::Tensor
#ifdef PINOCCHIO_WITH_EIGEN_TENSOR_MODULE
template<typename Scalar, int NumIndices, int Options, typename IndexType>
struct call_equality_op< pinocchio::Tensor<Scalar,NumIndices,Options,IndexType> >
{
  typedef pinocchio::Tensor<Scalar,NumIndices,Options,IndexType> T;
  
  static bool run(const T & v1, const T & v2)
  {
    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1,Options> VectorXd;
    Eigen::Map<const VectorXd> map1(v1.data(),v1.size(),1);
    Eigen::Map<const VectorXd> map2(v2.data(),v2.size(),1);
    return map1 == map2;
  }
};
#endif

template<typename T>
void generic_test(const T & object,
                  const std::string & filename,
                  const std::string & tag_name)
{
  using namespace pinocchio::serialization;
  
  // Load and save as TXT
  const std::string txt_filename = filename + ".txt";
  saveToText(object,txt_filename);
  
  {
    T object_loaded;
    loadFromText(object_loaded,txt_filename);
    
    // Check
    BOOST_CHECK(run_call_equality_op(object_loaded,object));
  }
  
  // Load and save as string stream
  std::stringstream ss_out;
  saveToStringStream(object,ss_out);
  
  {
    T object_loaded;
    std::istringstream is(ss_out.str());
    loadFromStringStream(object_loaded,is);
    
    // Check
    BOOST_CHECK(run_call_equality_op(object_loaded,object));
  }
  
  // Load and save as string
  std::string str_out = saveToString(object);
  
  {
    T object_loaded;
    std::string str_in(str_out);
    loadFromString(object_loaded,str_in);
    
    // Check
    BOOST_CHECK(run_call_equality_op(object_loaded,object));
  }
  
  // Load and save as XML
  const std::string xml_filename = filename + ".xml";
  saveToXML(object,xml_filename,tag_name);
  
  {
    T object_loaded;
    loadFromXML(object_loaded,xml_filename,tag_name);
    
    // Check
    BOOST_CHECK(run_call_equality_op(object_loaded,object));
  }
  
  // Load and save as binary
  const std::string bin_filename = filename + ".bin";
  saveToBinary(object,bin_filename);
  
  {
    T object_loaded;
    loadFromBinary(object_loaded,bin_filename);
    
    // Check
    BOOST_CHECK(run_call_equality_op(object_loaded,object));
  }
}

BOOST_AUTO_TEST_CASE(test_eigen_serialization)
{
  using namespace pinocchio;
  
  const Eigen::DenseIndex num_cols = 10;
  const Eigen::DenseIndex num_rows = 20;
  
  const Eigen::DenseIndex array_size = 3;
  
  Eigen::MatrixXd Mat = Eigen::MatrixXd::Random(num_rows,num_cols);
  generic_test(Mat,TEST_SERIALIZATION_FOLDER"/eigen_matrix","matrix");
  
  Eigen::VectorXd Vec = Eigen::VectorXd::Random(num_rows*num_cols);
  generic_test(Vec,TEST_SERIALIZATION_FOLDER"/eigen_vector","vector");
  
  Eigen::array<Eigen::DenseIndex,array_size> array = { 1, 2, 3 };
  generic_test(array,TEST_SERIALIZATION_FOLDER"/eigen_array","array");
  
  const Eigen::DenseIndex tensor_size = 3;
  const Eigen::DenseIndex x_dim = 10, y_dim = 20, z_dim = 30;
  
  typedef pinocchio::Tensor<double,tensor_size> Tensor3x;
  Tensor3x tensor(x_dim,y_dim,z_dim);
  
  Eigen::Map<Eigen::VectorXd>(tensor.data(),tensor.size(),1).setRandom();
  
  generic_test(tensor,TEST_SERIALIZATION_FOLDER"/eigen_tensor","tensor");
}

BOOST_AUTO_TEST_CASE(test_spatial_serialization)
{
  using namespace pinocchio;
  
  SE3 M(SE3::Random());
  generic_test(M,TEST_SERIALIZATION_FOLDER"/SE3","SE3");
  
  Motion m(Motion::Random());
  generic_test(m,TEST_SERIALIZATION_FOLDER"/Motion","Motion");
  
  Force f(Force::Random());
  generic_test(f,TEST_SERIALIZATION_FOLDER"/Force","Force");
  
  Symmetric3 S(Symmetric3::Random());
  generic_test(S,TEST_SERIALIZATION_FOLDER"/Symmetric3","Symmetric3");
  
  Inertia I(Inertia::Random());
  generic_test(I,TEST_SERIALIZATION_FOLDER"/Inertia","Inertia");
}

BOOST_AUTO_TEST_CASE(test_multibody_serialization)
{
  using namespace pinocchio;
  
  Frame frame("frame",0,0,SE3::Random(),SENSOR);
  generic_test(frame,TEST_SERIALIZATION_FOLDER"/Frame","Frame");
}

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
    
    return jmodel;
  }
};

struct TestJoint
{
  template <typename JointModel>
  void operator()(const pinocchio::JointModelBase<JointModel> &) const
  {
    JointModel jmodel = init<JointModel>::run();
    test(jmodel);
  }
  
  template<typename JointType>
  static void test(JointType & jmodel)
  {
    generic_test(jmodel,TEST_SERIALIZATION_FOLDER"/Joint","jmodel");
  }
  
};

BOOST_AUTO_TEST_CASE(test_multibody_joints_serialization)
{
  using namespace pinocchio;
  boost::mpl::for_each<JointModelVariant::types>(TestJoint());
}

BOOST_AUTO_TEST_CASE(test_model_serialization)
{
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model);
  
  generic_test(model,TEST_SERIALIZATION_FOLDER"/Model","Model");
}

BOOST_AUTO_TEST_CASE(test_throw_extension)
{
  using namespace pinocchio;
  
  Model model;
  buildModels::humanoidRandom(model);
  
  const std::string & fake_filename = "this_is_a_fake_filename";
  
  {
    const std::string complete_filename = fake_filename + ".txt";
    BOOST_REQUIRE_THROW(loadFromText(model,complete_filename),
                        std::invalid_argument);
  }
  
  saveToText(model,TEST_SERIALIZATION_FOLDER"/model.txt");
  saveToXML(model,TEST_SERIALIZATION_FOLDER"/model.xml","model");
  saveToBinary(model,TEST_SERIALIZATION_FOLDER"/model.bin");
  
  {
    const std::string complete_filename = fake_filename + ".txte";
  
    BOOST_REQUIRE_THROW(loadFromText(model,complete_filename),
                        std::invalid_argument);
  }
  
  {
    const std::string complete_filename = fake_filename + ".xmle";
    BOOST_REQUIRE_THROW(loadFromXML(model,complete_filename,"model"),
                        std::invalid_argument);
  }
  
  {
    const std::string complete_filename = fake_filename + ".bine";
    BOOST_REQUIRE_THROW(loadFromBinary(model,complete_filename),
                        std::invalid_argument);
  }
  
}

BOOST_AUTO_TEST_SUITE_END()
