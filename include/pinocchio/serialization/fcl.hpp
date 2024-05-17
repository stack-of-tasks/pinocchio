//
// Copyright (c) 2015-2024 CNRS INRIA
//

#ifndef __pinocchio_multibody_fcl_serialization_hpp__
#define __pinocchio_multibody_fcl_serialization_hpp__

#include "pinocchio/multibody/fcl.hpp"

namespace boost
{
  namespace serialization
  {

#ifndef PINOCCHIO_WITH_HPP_FCL

    template<class Archive>
    void serialize(
      Archive & /*ar*/,
      pinocchio::fcl::FakeCollisionGeometry & /*fake_collision_geometry*/,
      const unsigned int /*version*/)
    {
    }

#endif // PINOCCHIO_WITH_HPP_FCL

  } // namespace serialization
} // namespace boost

#endif // __pinocchio_multibody_fcl_serialization_hpp__
