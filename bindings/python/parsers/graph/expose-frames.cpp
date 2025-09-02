//
// Copyright (c) 2025 INRIA
//

#include <boost/python.hpp>

#include "pinocchio/bindings/python/parsers/model-graph.hpp"
#include "pinocchio/parsers/graph/model-graph.hpp"
#include "pinocchio/bindings/python/utils/std-aligned-vector.hpp"

#include <eigenpy/variant.hpp>
namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    void exposeFramesGraph()
    {
      using namespace pinocchio::graph;

      bp::class_<BodyFrame>(
        "BodyFrame", "Represents a body frame in the model graph, including its inertia.",
        bp::init<>(bp::args("self"), "Default constructor."))
        .def(bp::init<const pinocchio::Inertia &>(
          bp::args("self", "inertia"), "Constructor initializing with a specific inertia."))
        .def_readwrite(
          "inertia", &BodyFrame::inertia,
          "Spatial inertia of the body, expressed at its center of mass (CoM).")
        .def_readwrite(
          "f_type", &BodyFrame::f_type, "Type of the frame (e.g., pinocchio.FrameType.BODY).");

      bp::class_<SensorFrame>(
        "SensorFrame", "Represents a sensor frame in the model graph.",
        bp::init<>(bp::args("self"), "Default constructor."))
        .def_readwrite(
          "f_type", &SensorFrame::f_type,
          "Type of the frame (should be pinocchio.FrameType.SENSOR).");

      bp::class_<OpFrame>(
        "OpFrame", "Represents an operational (task) frame in the model graph.",
        bp::init<>(bp::args("self"), "Default constructor."))
        .def_readwrite(
          "f_type", &OpFrame::f_type,
          "Type of the frame (should be pinocchio.FrameType.OP_FRAME).");

      typedef eigenpy::VariantConverter<FrameVariant> Converter;
      Converter::registration();
    }
  } // namespace python
} // namespace pinocchio
