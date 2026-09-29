#include <boost/test/unit_test.hpp>

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
// This is still mandatory and tested in unittest/serialization.cpp
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

template<typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
struct call_equality_op<Eigen::Array<Scalar, Rows, Cols, Options, MaxRows, MaxCols>>
{
  typedef Eigen::Array<Scalar, Rows, Cols, Options, MaxRows, MaxCols> T;

  static bool run(const T & a1, const T & a2)
  {
    return a1.matrix() == a2.matrix();
  }
};

template<typename T>
struct empty_contructor_algo
{
  static T * run()
  {
    return new T();
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
