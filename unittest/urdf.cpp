#include <iostream>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/parser/urdf.hpp"

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE UrdfTest
#include <boost/test/unit_test.hpp>


BOOST_AUTO_TEST_SUITE ( ParsingUrdfFile )

BOOST_AUTO_TEST_CASE ( buildModel )
{
  std::string filename = PINOCCHIO_SOURCE_DIR"/models/simple_humanoid.urdf";

  #ifndef NDEBUG
     std::cout << "Parse filename \"" << filename << "\"" << std::endl;
  #endif
    se3::Model model = se3::urdf::buildModel(filename);
}

BOOST_AUTO_TEST_SUITE_END()
