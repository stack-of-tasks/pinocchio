//
// Copyright (c) 2019-2020 INRIA
//

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/spatial/classic-acceleration.hpp"

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    void exposeClassicAcceleration()
    {
      bp::def(
        "classicAcceleration", &classicAcceleration<context::Motion, context::Motion>,
        bp::args("spatial_velocity", "spatial_acceleration"),
        "Computes the classic acceleration from a given spatial velocity and spatial "
        "acceleration.");

      bp::def(
        "classicAcceleration",
        &classicAcceleration<context::Motion, context::Motion, context::Scalar, context::Options>,
        bp::args("spatial_velocity", "spatial_acceleration", "placement"),
        "Computes the classic acceleration of a frame B, given the spatial velocity and spatial "
        "acceleration of a frame A,\n"
        "and the relative placement A^M_B.");
    }

  } // namespace python
} // namespace pinocchio
