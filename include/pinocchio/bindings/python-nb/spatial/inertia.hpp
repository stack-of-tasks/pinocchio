#pragma once

#include "../fwd.hpp"

#include "pinocchio/spatial.hpp"

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
template<class Inertia>
void exposeInertia(nb::module_ m)
{
  using namespace nb::literals;
  nb::class_<Inertia>(m, "Inertia").def(nb::init<>()).def(nb::init<const Inertia &>(), "other"_a);
}
PINOCCHIO_PYTHON_NAMESPACE_END
