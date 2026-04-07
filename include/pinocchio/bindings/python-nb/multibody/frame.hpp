// Copyright (c) 2026 INRIA

#pragma once

#include "pinocchio/multibody.hpp"

#include "../fwd.hpp"
#include "../utils/comparable.hpp"
#include "../utils/printable.hpp"

#include <nanobind/stl/bind_vector.h>
#include <nanobind/stl/string.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
template<class Frame>
void exposeFrame(nb::module_ m)
{
  using namespace nb::literals;

  nb::class_<Frame>(
    m, "Frame", "A Plucker coordinate frame related to a parent joint inside a kinematic tree.")
    .def(nb::init<>())
    .def(nb::init<const Frame &>(), "other"_a)
    .def(
      nb::init<const std::string &, JointIndex, const SE3 &, FrameType, const Inertia &>(),
      "name"_a, "parent_joint"_a, "placement"_a, "type"_a, "inertia"_a = Inertia::Zero(),
      "Initialize from a name, parent joint index, placement wrt parent joint, type and optional "
      "spatial inertia.")
    .def(
      nb::init<
        const std::string &, JointIndex, FrameIndex, const SE3 &, FrameType, const Inertia &>(),
      "name"_a, "parent_joint"_a, "parent_frame"_a, "placement"_a, "type"_a,
      "inertia"_a = Inertia::Zero(),
      "Initialize from a name, parent joint index, parent frame index, placement wrt parent "
      "joint, type and optional spatial inertia.")
    .def_rw("name", &Frame::name, "Name of the frame.")
    .def_rw("parentJoint", &Frame::parentJoint, "Index of the parent joint.")
    .def_rw("parentFrame", &Frame::parentFrame, "Index of the parent frame.")
    .def_rw(
      "placement", &Frame::placement, "Placement of the frame in the parent joint local frame.")
    .def_rw("type", &Frame::type, "Type of the frame.")
    .def_rw("inertia", &Frame::inertia, "Inertia information attached to the frame.")
    .def(ComparableVisitor<Frame>())
    .def(PrintableVisitor<Frame>());

  nb::bind_vector<std::vector<Frame>, nb::rv_policy::reference_internal>(m, "StdVec_Frame");
};
PINOCCHIO_PYTHON_NAMESPACE_END
