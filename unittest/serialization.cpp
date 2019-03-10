//
// Copyright (c) 2019 INRIA
//

#include "pinocchio/serialization/archive.hpp"
#include "pinocchio/serialization/spatial.hpp"

#include "pinocchio/serialization/frame.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

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
    BOOST_CHECK(object_loaded == object);
  }
  
  // Load and save as XML
  const std::string xml_filename = filename + ".xml";
  saveToXML(object,xml_filename,tag_name);
  
  {
    T object_loaded;
    loadFromXML(object_loaded,xml_filename,tag_name);
    
    // Check
    BOOST_CHECK(object_loaded == object);
  }
  
  // Load and save as binary
  const std::string bin_filename = filename + ".bin";
  saveToBinary(object,xml_filename);
  
  {
    T object_loaded;
    loadFromBinary(object_loaded,xml_filename);
    
    // Check
    BOOST_CHECK(object_loaded == object);
  }
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

BOOST_AUTO_TEST_CASE(test_model_serialization)
{
  using namespace pinocchio;

}

BOOST_AUTO_TEST_SUITE_END()
