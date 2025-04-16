//
// Copyright (c) 2024-2025 INRIA
//
#ifndef __pinocchio_python_extra_extras_hpp__
#define __pinocchio_python_extra_extras_hpp__

#include "pinocchio/visualizers/base-visualizer.hpp"
#include "pinocchio/bindings/python/fwd.hpp"

#include <eigenpy/optional.hpp>

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    template<class Visualizer>
    struct VisualizerPythonVisitor : bp::def_visitor<VisualizerPythonVisitor<Visualizer>>
    {
      typedef ::pinocchio::visualizers::BaseVisualizer Base;
      typedef ::pinocchio::visualizers::ConstMatrixRef ConstMatrixRef;
      static_assert(
        std::is_base_of<Base, Visualizer>::value,
        "Visualizer class must be derived from pinocchio::visualizers::BaseVisualizer.");

      static void setCameraPose_proxy(Visualizer & vis, const Base::Matrix4 & pose)
      {
        vis.setCameraPose(pose);
      }

      static void setCameraPose_proxy2(Visualizer & vis, const SE3 & pose)
      {
        vis.setCameraPose(pose);
      }

      static void play_proxy2(Visualizer & vis, const ConstMatrixRef & qs, context::Scalar dt)
      {
        vis.play(qs, dt);
      }

      template<class... PyArgs>
      void visit(bp::class_<PyArgs...> & cl) const
      {
        using ::pinocchio::visualizers::ConstVectorRef;
        eigenpy::OptionalConverter<ConstVectorRef, boost::optional>::registration();

// convenience macro to use a lambda -- passing &Visualizer::<getter-fun> doesn't work
#define DEF_PROP_PROXY(name)                                                                       \
  add_property(                                                                                    \
    #name,                                                                                         \
    bp::make_function(                                                                             \
      +[](Visualizer & v) -> auto & { return v.name(); }, bp::return_internal_reference<>()))

        cl.def("initViewer", &Visualizer::initViewer)
          .def("loadViewerModel", &Visualizer::loadViewerModel)
          .def("rebuildData", &Visualizer::rebuildData)
          .def(
            "display", +[](Visualizer & v, const ConstVectorRef & q) { v.display(q); },
            (bp::arg("self"), bp::arg("q") = boost::none))
          .def("play", play_proxy2, (bp::arg("self"), "qs", "dt"))
          .def("setCameraTarget", &Visualizer::setCameraTarget, (bp::arg("self"), "target"))
          .def("setCameraPosition", &Visualizer::setCameraPosition, (bp::arg("self"), "position"))
          .def("setCameraPose", setCameraPose_proxy, (bp::arg("self"), "pose"))
          .def("setCameraPose", setCameraPose_proxy2, (bp::arg("self"), "pose"))
          .def("setCameraZoom", &Visualizer::setCameraZoom, (bp::arg("self"), "value"))
          .def("clean", &Visualizer::clean, bp::arg("self"))
          .def("hasExternalData", &Visualizer::hasExternalData)
          .DEF_PROP_PROXY(model)
          .DEF_PROP_PROXY(visualModel)
          .DEF_PROP_PROXY(collisionModel)
          .DEF_PROP_PROXY(data)
          .DEF_PROP_PROXY(visualData)
          .DEF_PROP_PROXY(collisionData);
#undef DEF_PROP_PROXY
      }
    };

  } // namespace python
} // namespace pinocchio

#endif // #ifndef __pinocchio_python_extra_extras_hpp__
