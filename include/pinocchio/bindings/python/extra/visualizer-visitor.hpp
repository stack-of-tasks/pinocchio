//
// Copyright (c) 2024-2025 INRIA
//
#ifndef __pinocchio_python_extra_extras_hpp__
#define __pinocchio_python_extra_extras_hpp__

#include "pinocchio/extra/base-visualizer.hpp"
#include "pinocchio/bindings/python/fwd.hpp"

#include <eigenpy/optional.hpp>

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    template<class Visualizer>
    struct VisualizerVisitor : bp::def_visitor<VisualizerVisitor<Visualizer>>
    {
      typedef ::pinocchio::visualizers::BaseVisualizer Base;
      typedef ::pinocchio::visualizers::ConstMatrixRef ConstMatrixRef;
      static_assert(std::is_base_of_v<Base, Visualizer>);

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

        cl.def("initViewer", &Visualizer::initViewer)
          .def("loadViewerModel", &Visualizer::loadViewerModel)
          .def("rebuildData", &Visualizer::rebuildData)
          .def("display", &Visualizer::display, (bp::arg("self"), bp::arg("q") = boost::none))
          .def("play", play_proxy2, (bp::arg("self"), "qs", "dt"))
          .def("setCameraTarget", &Visualizer::setCameraTarget, (bp::arg("self"), "target"))
          .def("setCameraPosition", &Visualizer::setCameraPosition, (bp::arg("self"), "position"))
          .def("setCameraPose", setCameraPose_proxy, (bp::arg("self"), "pose"))
          .def("setCameraPose", setCameraPose_proxy2, (bp::arg("self"), "pose"))
          .def("setCameraZoom", &Visualizer::setCameraZoom, (bp::arg("self"), "value"))
          .def("clean", &Visualizer::clean, bp::arg("self"))
          .add_property(
            "model", bp::make_function(&Visualizer::model, bp::return_internal_reference<>()))
          .add_property(
            "visualModel",
            bp::make_function(&Visualizer::visualModel, bp::return_internal_reference<>()))
          .add_property(
            "collisionModel",
            bp::make_function(&Visualizer::collisionModel, bp::return_internal_reference<>()))
          .def_readwrite("data", &Visualizer::data)
          .def_readwrite("visualData", &Visualizer::visualData)
          .def_readwrite("collisionData", &Visualizer::collisionData);
      }
    };

  } // namespace python
} // namespace pinocchio

#endif // #ifndef __pinocchio_python_extra_extras_hpp__
