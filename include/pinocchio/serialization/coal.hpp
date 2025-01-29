//
// Copyright (c) 2015-2024 CNRS INRIA
//

#ifndef __pinocchio_multibody_coal_serialization_hpp__
#define __pinocchio_multibody_coal_serialization_hpp__

#include "pinocchio/multibody/coal.hpp"

namespace boost
{
  namespace serialization
  {

#ifndef PINOCCHIO_WITH_COAL

    template<class Archive>
    void serialize(
      Archive & /*ar*/,
      pinocchio::coal::FakeCollisionGeometry & /*fake_collision_geometry*/,
      const unsigned int /*version*/)
    {
    }

#endif // PINOCCHIO_WITH_COAL

  } // namespace serialization
} // namespace boost

#endif // __pinocchio_multibody_coal_serialization_hpp__
