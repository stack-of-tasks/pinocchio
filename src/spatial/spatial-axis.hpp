//
// Copyright (c) 2017 CNRS
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

#ifndef __se3_spatial_axis_hpp__
#define __se3_spatial_axis_hpp__

#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/cartesian-axis.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/force.hpp"
#include <Eigen/Core>

namespace se3
{
  
  template<int _axis>
  struct SpatialAxis
  {
    enum { axis = _axis, dim = 6 };
    typedef CartesianAxis<_axis%3> CartesianAxis;
    
    template<typename S1, int O1, typename S2, int O2>
    inline static void cross(const MotionTpl<S1,O1> & min,
                             MotionTpl<S2,O2> & mout);
    
    template<typename Scalar, int Options>
    static MotionTpl<Scalar,Options> cross(const MotionTpl<Scalar,Options> & min)
    {
      MotionTpl<Scalar,Options> res;
      cross(min,res);
      return res;
    }
    
    template<typename S1, int O1, typename S2, int O2>
    inline static void cross(const ForceTpl<S1,O1> & fin,
                             ForceTpl<S2,O2> & fout);
    
    template<typename Scalar, int Options>
    static ForceTpl<Scalar,Options> cross(const ForceTpl<Scalar,Options> & fin)
    {
      ForceTpl<Scalar,Options> res;
      cross(fin,res);
      return res;
    }
    
    template<typename Scalar, int Options>
    friend MotionTpl<Scalar,Options> & operator<<(MotionTpl<Scalar,Options> & min,
                                                  const SpatialAxis & sa)
    {
      min.setZero();
      min.toVector()[axis] = Scalar(1);
      return min;
    }
    
  }; // struct SpatialAxis
  
  template<int axis>
  template<typename S1, int O1, typename S2, int O2>
  inline void SpatialAxis<axis>::cross(const MotionTpl<S1,O1> & min,
                                       MotionTpl<S2,O2> & mout)
  {
    if(axis<3)
    {
      mout.angular().setZero();
      CartesianAxis::cross(min.angular(),mout.linear());
    }
    else
    {
      CartesianAxis::cross(min.linear(),mout.linear());
      CartesianAxis::cross(min.angular(),mout.angular());
    }
  }
  
  template<int axis>
  template<typename S1, int O1, typename S2, int O2>
  inline void SpatialAxis<axis>::cross(const ForceTpl<S1,O1> & fin,
                                       ForceTpl<S2,O2> & fout)
  {
    if(axis<3)
    {
      fout.linear().setZero();
      CartesianAxis::cross(fin.linear(),fout.angular());
    }
    else
    {
      CartesianAxis::cross(fin.linear(),fout.linear());
      CartesianAxis::cross(fin.angular(),fout.angular());
    }
  }
  
  typedef SpatialAxis<0> AxisVX;
  typedef SpatialAxis<1> AxisVY;
  typedef SpatialAxis<2> AxisVZ;
  
  typedef SpatialAxis<3> AxisWX;
  typedef SpatialAxis<4> AxisWY;
  typedef SpatialAxis<5> AxisWZ;
}

#endif // __se3_spatial_axis_hpp__
