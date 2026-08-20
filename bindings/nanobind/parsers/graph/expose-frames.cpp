//
// Copyright (c) 2026 INRIA
//

#include "pinocchio/bindings/python-nb/fwd.hpp"
#include "pinocchio/parsers/graph.hpp"

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

using namespace nb::literals;

void exposeFramesGraph(nb::module_ m)
{
  using namespace pinocchio::graph;
  nb::class_<BodyFrame>(
    m, "BodyFrame", "Represents a body frame in the model graph, including its inertia.")
    .def(nb::init<>(), "Default constructor.")
    .def(
      nb::init<const pinocchio::Inertia &>(), "inertia"_a,
      "Constructor initializing with a specific inertia.")
    .def_rw(
      "inertia", &BodyFrame::inertia,
      "Spatial inertia of the body, expressed at its center of mass (CoM).")
    .def_rw("f_type", &BodyFrame::f_type, "Type of the frame (e.g., pinocchio.FrameType.BODY).");

  nb::class_<SensorFrame>(m, "SensorFrame", "Represents a sensor frame in the model graph.")
    .def(nb::init<>(), "Default constructor.")
    .def_rw(
      "f_type", &SensorFrame::f_type, "Type of the frame (should be pinocchio.FrameType.SENSOR).");

  nb::class_<OpFrame>(m, "OpFrame", "Represents an operational (task) frame in the model graph.")
    .def(nb::init<>(), "Default constructor.")
    .def_rw(
      "f_type", &OpFrame::f_type, "Type of the frame (should be pinocchio.FrameType.OP_FRAME).");
}

PINOCCHIO_PYTHON_NAMESPACE_END
