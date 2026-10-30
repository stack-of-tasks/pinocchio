#ifdef BINDINGS_USE_BOOST
  #include "pinocchio/bindings/python/visualizers/visualizer-visitor.hpp"
#else
  #include "pinocchio/bindings/python-nb/visualizers/visualizer-visitor.hpp"
#endif

namespace pin = pinocchio;
using pin::GeometryModel;
using pin::Model;
using pin::visualizers::BaseVisualizer;

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

#ifdef BINDINGS_USE_BOOST
BOOST_PYTHON_MODULE(EXT_MODULE_NAME)
{
  namespace bp = boost::python;
  using pin::python::VisualizerPythonVisitor;
  bp::import("pinocchio");

  bp::class_<DummyVisualizer>("DummyVisualizer", bp::no_init)
    .def(bp::init<const Model &, const GeometryModel &>())
    .def(VisualizerPythonVisitor<DummyVisualizer>{});
}
#else
NB_MODULE(EXT_MODULE_NAME, m)
{
  namespace nb = nanobind;
  using pin::python_nb::VisualizerPythonVisitor;
  m.import_("pinocchio");

  nb::class_<DummyVisualizer>(m, "DummyVisualizer")
    .def(
      nb::init<const Model &, const GeometryModel &>(), nb::arg("model"), nb::arg("geometry_model"))
    .def(VisualizerPythonVisitor<DummyVisualizer>());
}
#endif
