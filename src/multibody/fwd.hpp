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

#ifndef __se3_multibody_fwd_hpp__
#define __se3_multibody_fwd_hpp__

namespace se3
{
  typedef std::size_t Index;
  typedef Index JointIndex;
  typedef Index GeomIndex;
  typedef Index FrameIndex;
  
  struct Frame;
  struct Model;
  struct Data;
  struct GeometryModel;
  struct GeometryData;
  struct JointModel;
  struct JointData;

  // Forward declaration needed for Model::check
  template<class D> struct AlgorithmCheckerBase;

} // namespace se3

#endif // #ifndef __se3_multibody_fwd_hpp__
