#include <iostream>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/parser/lua.hpp"

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE UrdfTest
#include <boost/test/unit_test.hpp>


BOOST_AUTO_TEST_SUITE ( ParsingLuaFile )

BOOST_AUTO_TEST_CASE ( buildModel )
{
  std::string filename = PINOCCHIO_SOURCE_DIR"/models/simple_model.lua";

  #ifndef NDEBUG
     std::cout << "Parse filename \"" << filename << "\"" << std::endl;
  #endif
    se3::Model model = se3::lua::buildModel(filename, true, true);
}

BOOST_AUTO_TEST_SUITE_END()
