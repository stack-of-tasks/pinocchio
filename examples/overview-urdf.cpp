#include "pinocchio/parsers/urdf.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/multibody/sample-models.hpp"

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

#include <boost/filesystem.hpp>

#include <iostream>

// PINOCCHIO_MODEL_DIR is defined by the CMake but you can define your own directory here.
#ifndef PINOCCHIO_MODEL_DIR
  #define PINOCCHIO_MODEL_DIR "path_to_the_model_dir"
#endif

#define BOOST_CHECK(check)                                                                         \
  if (!(check))                                                                                    \
    std::cout << BOOST_STRINGIZE(check) << " has failed" << std::endl;

using namespace pinocchio;

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
  pinocchio::serialization::StaticBuffer static_buffer(10000000);
  saveToBinary(object, static_buffer);

  {
    T & object_loaded = *empty_contructor<T>();
    loadFromBinary(object_loaded, static_buffer);

    // Check
    BOOST_CHECK(run_call_equality_op(object_loaded, object));

    delete &object_loaded;
  }
}

int main(int, char **)
{
  namespace fs = boost::filesystem;

  Model model;
  buildModels::humanoid(model);
  Data data(model);

  fs::path model_path = fs::temp_directory_path() / "GeometryModel";
  fs::path data_path = fs::temp_directory_path() / "GeometryData";
  //  boost::serialization::void_cast_register<hpp::fcl::BVHModel<hpp::fcl::OBBRSS>,hpp::fcl::CollisionGeometry>();
  // Empty structures
  {
    GeometryModel geom_model;
    generic_test(geom_model, model_path.string(), "GeometryModel");

    GeometryData geom_data(geom_model);
    generic_test(geom_data, data_path.string(), "GeometryData");
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
    }
    generic_test(geom_model, model_path.string(), "GeometryModel");

    pinocchio::GeometryData geom_data(geom_model);
    const Eigen::VectorXd q = pinocchio::neutral(model);
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateGeometryPlacements(model, data, geom_model, geom_data, q);

    generic_test(geom_data, data_path.string(), "GeometryData");
  }
  #endif // hpp-fcl >= 3.0.0
#endif   // PINOCCHIO_WITH_HPP_FCL
}
