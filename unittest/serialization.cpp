//
// Copyright (c) 2019 INRIA
//

#include "pinocchio/serialization/archive.hpp"
#include "pinocchio/serialization/spatial.hpp"

#include "pinocchio/serialization/frame.hpp"

#include "pinocchio/serialization/joints.hpp"
#include "pinocchio/serialization/model.hpp"

#include "pinocchio/parsers/sample-models.hpp"

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
  saveToBinary(object,bin_filename);
  
  {
    T object_loaded;
    loadFromBinary(object_loaded,bin_filename);
    
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

struct TestJoint
{
  template <typename T>
  void operator()(const T) const
  {
    T jmodel;
    jmodel.setIndexes(0,0,0);

    test(jmodel);
  }
  
  void operator()(const pinocchio::JointModelComposite & ) const
  {
    pinocchio::JointModelComposite jmodel((pinocchio::JointModelRX()));
    jmodel.addJoint(pinocchio::JointModelRY());
    jmodel.setIndexes(0,0,0);
    
    test(jmodel);
  }
  
  void operator()(const pinocchio::JointModelRevoluteUnaligned & ) const
  {
    pinocchio::JointModelRevoluteUnaligned jmodel(1.5, 1., 0.);
    jmodel.setIndexes(0,0,0);

    test(jmodel);
  }
  
  void operator()(const pinocchio::JointModelPrismaticUnaligned & ) const
  {
    pinocchio::JointModelPrismaticUnaligned jmodel(1.5, 1., 0.);
    jmodel.setIndexes(0,0,0);

    test(jmodel);
  }
  
  template<typename JointType>
  static void test(JointType & jmodel)
  {
    generic_test(jmodel,TEST_SERIALIZATION_FOLDER"/Joint",jmodel.shortname());
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

BOOST_AUTO_TEST_SUITE_END()
