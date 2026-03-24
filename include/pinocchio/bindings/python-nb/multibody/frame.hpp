// Copyright (c) 2026 INRIA

#pragma once

#include "pinocchio/multibody.hpp"

#include "../fwd.hpp"

#include <nanobind/stl/bind_vector.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
template<class Frame>
void exposeFrame(nb::module_ m)
{
  nb::class_<Frame>(
    m, "Frame", "A Plucker coordinate frame related to a parent joint inside a kinematic tree.")
    .def(nb::init<>())
    .def(nb::init<const Frame &>(), nb::arg("other"))
    .def_rw("name", &Frame::name)
    .def_rw("parentJoint", &Frame::parentJoint)
    .def_rw("parentFrame", &Frame::parentFrame);

  nb::bind_vector<std::vector<Frame>>(m, "FrameStdVec");
};
PINOCCHIO_PYTHON_NAMESPACE_END
