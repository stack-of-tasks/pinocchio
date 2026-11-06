// Copyright (c) 2026 INRIA

#pragma once

#include "pinocchio/bindings/python-nb/fwd.hpp"

#include "pinocchio/visualizers/base-visualizer.hpp"

#include <nanobind/stl/optional.h>
#include <nanobind/eigen/dense.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

template<class Visualizer>
struct VisualizerPythonVisitor : nb::def_visitor<VisualizerPythonVisitor<Visualizer>>
{
  typedef ::pinocchio::visualizers::BaseVisualizer Base;
  static_assert(
    std::is_base_of_v<Base, Visualizer>,
    "Visualizer class must be derived from pinocchio::visualizers::BaseVisualizer.");

  static void setCameraPose_proxy(Visualizer & vis, const Base::Matrix4 & pose)
  {
    vis.setCameraPose(pose);
  }

  static void setCameraPose_proxy2(Visualizer & vis, const SE3 & pose)
  {
    vis.setCameraPose(pose);
  }

  static void
  play_proxy(Visualizer & self, const std::vector<visualizers::VectorXs> & qs, context::Scalar dt)
  {
    std::vector<visualizers::ConstVectorRef> qs_;
    qs_.reserve(qs.size());
    for (size_t i = 0; i < qs.size(); i++)
    {
      qs_.emplace_back(qs[i]);
    }
    self.play(qs_, dt);
  }

  static void
  play_proxy2(Visualizer & self, const visualizers::ConstMatrixRef & qs, context::Scalar dt)
  {
    self.play(qs, dt);
  }

  template<class... PyArgs>
  void execute(nb::class_<PyArgs...> & cl) const
  {
    using namespace nb::literals;

// convenience macro to use a lambda -- passing &Visualizer::<getter-fun> doesn't work
#define DEF_PROP_PROXY(name) def_prop_ro(#name, [](Visualizer & v) -> auto & { return v.name(); })

    cl.def("initViewer", &Visualizer::initViewer)
      .def("loadViewerModel", &Visualizer::loadViewerModel)
      .def("rebuildData", &Visualizer::rebuildData)
      .def(
        "display", [](Visualizer & v, const boost::optional<ConstVectorRef> & q) { v.display(q); },
        "q"_a = nb::none())
      .def("play", play_proxy, "qs"_a, "dt"_a)
      .def("play", play_proxy2, "qs"_a, "dt"_a)
      .def("setCameraTarget", &Visualizer::setCameraTarget, "target"_a)
      .def("setCameraPosition", &Visualizer::setCameraPosition, "position"_a)
      .def("setCameraPose", setCameraPose_proxy, "pose"_a)
      .def("setCameraPose", setCameraPose_proxy2, "pose"_a)
      .def("setCameraZoom", &Visualizer::setCameraZoom, "value"_a)
      .def("clean", &Visualizer::clean)
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

PINOCCHIO_PYTHON_NAMESPACE_END
