//
// Copyright (c) 2019 INRIA
//

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/spatial/classic-acceleration.hpp"

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;
    
    template<typename Motion1, typename Motion2>
    inline void exposeClassicAcceleration()
    {
      bp::def("classicAcceleration",
              &classicAcceleration<Motion1,Motion2>,
              bp::args("spatial_velocity: Spatial velocity",
                       "spatial_acceleration: Spatial acceleration"),
              "Computes the classic acceleration from a given spatial velocity and spatial acceleration.");
    }
    
  } // namespace python
} // namespace pinocchio
