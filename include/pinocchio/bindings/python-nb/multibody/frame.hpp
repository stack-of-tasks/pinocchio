// Copyright (c) 2026 INRIA

#pragma once

#include "pinocchio/multibody.hpp"

#include "../fwd.hpp"
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
      nb::init<const std::string &, JointIndex, const SE3 &, FrameType>(), "name"_a,
      "parent_joint"_a, "placement"_a, "inertia"_a = Inertia::Zero())
    .def(
      nb::init<const std::string &, JointIndex, FrameIndex, const SE3 &, FrameType>(), "name"_a,
      "parent_joint"_a, "parent_frame"_a, "placement"_a, "inertia"_a = Inertia::Zero())
    .def_rw("name", &Frame::name)
    .def_rw("parentJoint", &Frame::parentJoint)
    .def_rw("parentFrame", &Frame::parentFrame)
    //
    .def_rw("placement", &Frame::placement)
    .def_rw("type", &Frame::type)
    .def_rw("inertia", &Frame::inertia, "Inertia information attached to the frame.")
    .def(PrintableVisitor<Frame>());

  nb::bind_vector<std::vector<Frame>>(m, "FrameStdVec");
};
PINOCCHIO_PYTHON_NAMESPACE_END
