#include "pinocchio/bindings/python/visualizers/visualizer-visitor.hpp"

namespace bp = boost::python;
namespace pin = pinocchio;
using pin::python::VisualizerPythonVisitor;
using pin::visualizers::BaseVisualizer;

using pin::GeometryModel;
using pin::Model;

struct DummyVisualizer : public BaseVisualizer
{
  using BaseVisualizer::BaseVisualizer;
  void loadViewerModel() override
  {
  }
  void displayImpl() override
  {
  }
};

BOOST_PYTHON_MODULE(EXT_MODULE_NAME)
{
  bp::import("pinocchio");

  bp::class_<DummyVisualizer>("DummyVisualizer", bp::no_init)
    .def(bp::init<const Model &, const GeometryModel &>())
    .def(VisualizerPythonVisitor<DummyVisualizer>{});
}
