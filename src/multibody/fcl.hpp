//
// Copyright (c) 2015-2016 CNRS
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.

#ifndef __se3_fcl_hpp__
#define __se3_fcl_hpp__

#include "pinocchio/multibody/fwd.hpp"

#ifdef WITH_HPP_FCL
#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/distance.h>
#include <hpp/fcl/shape/geometric_shapes.h>
#include "pinocchio/spatial/fcl-pinocchio-conversions.hpp"
#endif

#include <iostream>
#include <map>
#include <vector>
#include <utility>
#include <assert.h>

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

namespace se3
{
  struct CollisionPair: public std::pair<GeomIndex, GeomIndex>
  {

    typedef std::pair<GeomIndex, GeomIndex> Base;
   
    ///
    /// \brief Default constructor of a collision pair from two collision object indexes.
    ///        The indexes must be ordered such that co1 < co2. If not, the constructor reverts the indexes.
    ///
    /// \param[in] co1 Index of the first collision object
    /// \param[in] co2 Index of the second collision object
    ///
    CollisionPair(const GeomIndex co1, const GeomIndex co2) : Base(co1,co2)
    {
      assert(co1 != co2 && "The index of collision objects must not be equal.");
    }

    bool operator== (const CollisionPair& rhs) const
    {
      return (first == rhs.first && second == rhs.second)
        || (first == rhs.second && second == rhs.first);
    }
    
    void disp(std::ostream & os) const { os << "collision pair (" << first << "," << second << ")\n"; }
    friend std::ostream & operator << (std::ostream & os, const CollisionPair & X);

  }; // struct CollisionPair
  typedef std::vector<CollisionPair> CollisionPairsVector_t;

#ifndef WITH_HPP_FCL  

  namespace fcl
  {
 
    struct FakeCollisionGeometry
    {
      FakeCollisionGeometry(){};
    };

    struct AABB
    {
      AABB(): min_(0), max_(1){};

      int min_;
      int max_;
    };
    typedef FakeCollisionGeometry CollisionGeometry;

  }

#endif // WITH_HPP_FCL

enum GeometryType
{
  VISUAL,
  COLLISION
};

struct GeometryObject
{

  /// \brief Name of the geometry object
  std::string name;

  /// \brief Index of the parent frame
  ///
  /// Parent frame may be unset (set to -1) as it is mostly used as a documentation of the tree, or in third-party libraries.
  /// The URDF parser of Pinocchio is setting it to the proper value according to the urdf link-joint tree.
  /// In particular, anchor joints of URDF would cause parent frame to be different to joint frame.
  FrameIndex parentFrame;

  /// \brief Index of the parent joint
  JointIndex parentJoint;

  /// \brief The actual cloud of points representing the collision mesh of the object
  boost::shared_ptr<fcl::CollisionGeometry> collision_geometry;

  /// \brief Position of geometry object in parent joint frame
  SE3 placement;

  /// \brief Absolute path to the mesh file
  std::string mesh_path;

  GeometryObject(const std::string & name, const FrameIndex parentF,
                 const JointIndex parentJ, const boost::shared_ptr<fcl::CollisionGeometry> & collision,
                 const SE3 & placement, const std::string & mesh_path)
                : name(name)
                , parentFrame(parentF)
                , parentJoint(parentJ)
                , collision_geometry(collision)
                , placement(placement)
                , mesh_path(mesh_path)
  {}

  GeometryObject & operator=(const GeometryObject & other)
  {
    name = other.name;
    parentFrame = other.parentFrame;
    parentJoint = other.parentJoint;
    collision_geometry = other.collision_geometry;
    placement = other.placement;
    mesh_path = other.mesh_path;
    return *this;
  }

  friend std::ostream & operator<< (std::ostream & os, const GeometryObject & geom_object);
};
  

} // namespace se3

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
#include "pinocchio/multibody/fcl.hxx"


#endif // ifndef __se3_fcl_hpp__
