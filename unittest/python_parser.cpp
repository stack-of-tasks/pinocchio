//
// Copyright (c) 2016-2020 CNRS INRIA
//

#define BOOST_TEST_MODULE python_parser

#include "pinocchio/multibody.hpp"
#include "pinocchio/parsers/python.hpp"

#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_CASE(buildModel)
{
  std::string filename = PINOCCHIO_MODEL_DIR + std::string("/simple_model.py");

#ifndef NDEBUG
  std::cout << "Parse filename \"" << filename << "\"" << std::endl;
#endif
  pinocchio::Model model = pinocchio::python::buildModel(filename, "model");
#ifndef NDEBUG
  std::cout << "This model has \"" << model.nq << "\" DoF" << std::endl;
#endif

  BOOST_CHECK(model.nq == 9);
  BOOST_CHECK(model.nv == 8);
}
