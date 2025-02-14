#include "pinocchio/visualizers/base-visualizer.hpp"

#include <boost/test/unit_test.hpp>

using namespace pinocchio::visualizers;

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

struct Viz : BaseVisualizer
{
  void loadViewerModel() override
  {
  }
  void displayImpl() override
  {
  }
};

BOOST_AUTO_TEST_SUITE_END()
