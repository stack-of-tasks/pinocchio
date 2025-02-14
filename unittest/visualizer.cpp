#include "pinocchio/visualizers/base-visualizer.hpp"
#include "pinocchio/multibody/sample-models.hpp"

#include <boost/test/unit_test.hpp>

using namespace pinocchio::visualizers;

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

struct TestViz : BaseVisualizer
{
  using BaseVisualizer::BaseVisualizer;
  void loadViewerModel() override
  {
  }
  void displayImpl() override
  {
  }
};

BOOST_AUTO_TEST_CASE(viz_ctor_create_datas)
{
  pinocchio::Model model;
  pinocchio::buildModels::manipulator(model);
  pinocchio::GeometryModel geom_model; // empty
  TestViz viz{model, geom_model};
  BOOST_CHECK(!viz.hasExternalData());
  BOOST_CHECK(!viz.hasCollisionModel());
  // dtor destroys internal datas
}

BOOST_AUTO_TEST_CASE(viz_ctor_borrow_datas)
{
  pinocchio::Model model;
  pinocchio::buildModels::manipulator(model);
  pinocchio::GeometryModel geom_model; // empty
  pinocchio::Data data{model};
  pinocchio::GeometryData geom_data{geom_model};

  {
    TestViz viz{model, geom_model, nullptr, data, geom_data, nullptr};
    BOOST_CHECK(viz.hasExternalData());
    BOOST_CHECK(!viz.hasCollisionModel());
    // viz destroyed, does not destroy datas
  }
  // no double free segfault
}

BOOST_AUTO_TEST_SUITE_END()
