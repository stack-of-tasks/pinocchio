//
// Copyright (c) 2015-2023 CNRS INRIA
//

#ifndef __pinocchio_multibody_fcl_hpp__
#define __pinocchio_multibody_fcl_hpp__

#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/multibody/model-item.hpp"
#include "pinocchio/container/aligned-vector.hpp"

#ifdef PINOCCHIO_WITH_HPP_FCL

  #if (WIN32)
    // It appears that std::snprintf is missing for Windows.
    #if !(                                                                                         \
      (defined(_MSC_VER) && _MSC_VER < 1900)                                                       \
      || (defined(__MINGW32__) && !defined(__MINGW64_VERSION_MAJOR)))
      #include <cstdio>
      #include <stdarg.h>
namespace std
{
  inline int _snprintf(char * buffer, std::size_t buf_size, const char * format, ...)
  {
    int res;

    va_list args;
    va_start(args, format);
    res = vsnprintf(buffer, buf_size, format, args);
    va_end(args);

    return res;
  }
} // namespace std
    #endif
  #endif

  #include <hpp/fcl/collision_object.h>
  #include <hpp/fcl/collision.h>
  #include <hpp/fcl/distance.h>
  #include <hpp/fcl/shape/geometric_shapes.h>
  #include "pinocchio/collision/fcl-pinocchio-conversions.hpp"
#endif

#include <map>
#include <vector>
#include <utility>
#include <memory>
#include <limits>
#include <assert.h>

#include <boost/foreach.hpp>

namespace pinocchio
{

#ifndef PINOCCHIO_WITH_HPP_FCL

  namespace fcl
  {

    struct FakeCollisionGeometry
    {
      FakeCollisionGeometry() {};

      bool operator==(const FakeCollisionGeometry &) const
      {
        return true;
      }
    };

    struct AABB
    {
      AABB()
      : min_(0)
      , max_(1) {};

      int min_;
      int max_;
    };

    typedef FakeCollisionGeometry CollisionGeometry;

  } // namespace fcl

#else

  namespace fcl = hpp::fcl;

  inline bool operator==(const fcl::CollisionObject & lhs, const fcl::CollisionObject & rhs)
  {
    return lhs.collisionGeometry() == rhs.collisionGeometry()
           && lhs.getAABB().min_ == rhs.getAABB().min_ && lhs.getAABB().max_ == rhs.getAABB().max_;
  }

#endif // PINOCCHIO_WITH_HPP_FCL

} // namespace pinocchio

#endif // ifndef __pinocchio_multibody_fcl_hpp__
