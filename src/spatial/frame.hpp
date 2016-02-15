//
// Copyright (c) 2016 CNRS
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

#ifndef __se3_frame_hpp__
#define __se3_frame_hpp__

#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/tools/string-generator.hpp"

#include <Eigen/StdVector>
#include <iostream>

namespace se3
{
struct Frame
{
  typedef std::size_t Index;
  
  Frame() : name(random(8)), parent_id(), frame_placement()
  {

  }

  Frame(const std::string & name, Index parent, const SE3 & placement): name(name)
                                                                      , parent_id(parent)
                                                                      , frame_placement(placement)
  {
  }

  bool operator == (const Frame & other) const
  {
    return name == other.name && parent_id == other.parent_id
            && frame_placement == other.frame_placement ;
  }

  std::string name;
  Index parent_id;
  SE3 frame_placement;
};
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(se3::Frame)

}
#endif // ifndef __se3_frame_hpp__
