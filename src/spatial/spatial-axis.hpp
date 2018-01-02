//
// Copyright (c) 2017-2018 CNRS
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

namespace se3
{
  
  template<int _axis>
  struct SpatialAxis //: MotionBase< SpatialAxis<_axis> >
  {
    enum { axis = _axis, dim = 6 };
    typedef CartesianAxis<_axis%3> CartesianAxis;
    
    template<typename Derived1, typename Derived2>
    inline static void cross(const MotionDense<Derived1> & min,
                             MotionDense<Derived2> & mout);
    
    template<typename Derived>
    static typename traits<Derived>::MotionPlain cross(const MotionDense<Derived> & min)
    {
      typename MotionDense<Derived>::MotionPlain res;
      cross(min,res);
      return res;
    }
    
    template<typename Derived1, typename Derived2>
    inline static void cross(const ForceDense<Derived1> & fin,
                             ForceDense<Derived2> & fout);
    
    template<typename Derived>
    static typename traits<Derived>::ForcePlain cross(const ForceDense<Derived> & fin)
    {
      typename ForceDense<Derived>::ForcePlain fout;
      cross(fin,fout);
      return fout;
    }
    
    template<typename Derived>
    friend Derived & operator<<(MotionDense<Derived> & min,
                                const SpatialAxis &)
    {
      typedef typename traits<Derived>::Scalar Scalar;
      min.setZero();
      min.toVector()[axis] = Scalar(1);
      return min.derived();
    }
    
  }; // struct SpatialAxis
  
  template<int axis>
  template<typename Derived1, typename Derived2>
  inline void SpatialAxis<axis>::cross(const MotionDense<Derived1> & min,
                                       MotionDense<Derived2> & mout)
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
  template<typename Derived1, typename Derived2>
  inline void SpatialAxis<axis>::cross(const ForceDense<Derived1> & fin,
                                       ForceDense<Derived2> & fout)
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
