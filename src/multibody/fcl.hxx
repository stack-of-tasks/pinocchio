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

#ifndef __se3_fcl_hxx__
#define __se3_fcl_hxx__


#include <iostream>


namespace se3
{

  ///
  /// \brief Default constructor of a collision pair from two collision object indexes.
  ///        The indexes must be ordered such that co1 < co2. If not, the constructor reverts the indexes.
  ///
  /// \param[in] co1 Index of the first collision object
  /// \param[in] co2 Index of the second collision object
  ///
  inline CollisionPair::CollisionPair(const GeomIndex co1, const GeomIndex co2) 
    : Base(co1,co2)
  {
    assert(co1 != co2 && "The index of collision objects must not be equal.");
  }

  inline bool CollisionPair::operator== (const CollisionPair& rhs) const
  {
    return (first == rhs.first  && second == rhs.second)
      ||   (first == rhs.second && second == rhs.first );
  }

  inline void CollisionPair::disp(std::ostream & os) const
  { os << "collision pair (" << first << "," << second << ")\n"; }

  inline std::ostream & operator << (std::ostream & os, const CollisionPair & X)
  {
    X.disp(os); return os;
  } 
  

#ifdef WITH_HPP_FCL  

  inline bool operator == (const fcl::CollisionObject & lhs, const fcl::CollisionObject & rhs)
  {
    return lhs.collisionGeometry() == rhs.collisionGeometry()
            && lhs.getAABB().min_ == rhs.getAABB().min_
            && lhs.getAABB().max_ == rhs.getAABB().max_;
  }
  
#endif // WITH_HPP_FCL
  
  inline bool operator==(const GeometryObject & lhs, const GeometryObject & rhs)
  {
    return ( lhs.name           == rhs.name
            && lhs.parentFrame  == rhs.parentFrame
            && lhs.parentJoint  == rhs.parentJoint
            && lhs.fcl          == rhs.fcl
            && lhs.placement    == rhs.placement
            && lhs.meshPath     == rhs.meshPath
            );
  }

  inline std::ostream & operator<< (std::ostream & os, const GeometryObject & geom_object)
  {
    os  << "Name: \t \n" << geom_object.name << "\n"
        << "Parent frame ID: \t \n" << geom_object.parentFrame << "\n"
        << "Parent joint ID: \t \n" << geom_object.parentJoint << "\n"
        << "Position in parent frame: \t \n" << geom_object.placement << "\n"
        << "Absolute path to mesh file: \t \n" << geom_object.meshPath << "\n"
        << std::endl;
    return os;
  }


} // namespace se3


#endif // ifndef __se3_fcl_hxx__
