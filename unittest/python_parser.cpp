#include <iostream>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/parser/python.hpp"

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE PythonTest
#include <boost/test/unit_test.hpp>


BOOST_AUTO_TEST_SUITE ( ParsingPythonFile )

BOOST_AUTO_TEST_CASE ( buildModel )
{
  std::string filename = PINOCCHIO_SOURCE_DIR"/models/simple_model.py";

  #ifndef NDEBUG
   std::cout << "Parse filename \"" << filename << "\"" << std::endl;
  #endif
  se3::Model model = se3::python::buildModel(filename, true);
  #ifndef NDEBUG
   std::cout << "This model has \"" << model.nq << "\" DoF" << std::endl;
  #endif
}

BOOST_AUTO_TEST_SUITE_END()
