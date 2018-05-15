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
  template<int axis> struct SpatialAxis;
  
  namespace internal
  {
    template<int axis, typename MotionDerived>
    struct MotionAlgebraAction<SpatialAxis<axis>, MotionDerived>
    { typedef typename MotionDerived::MotionPlain ReturnType; };
  }
  
  template<int _axis>
  struct SpatialAxis //: MotionBase< SpatialAxis<_axis> >
  {
    enum { axis = _axis, dim = 6 };
    typedef CartesianAxis<_axis%3> CartesianAxis3;
    
    enum { LINEAR = 0, ANGULAR = 3 };
    
    template<typename Derived1, typename Derived2>
    inline static void cross(const MotionDense<Derived1> & min,
                             const MotionDense<Derived2> & mout);
    
    template<typename Derived>
    static typename traits<Derived>::MotionPlain cross(const MotionDense<Derived> & min)
    {
      typename MotionDense<Derived>::MotionPlain res;
      cross(min,res);
      return res;
    }
    
    template<typename Derived1, typename Derived2>
    inline static void cross(const ForceDense<Derived1> & fin,
                             const ForceDense<Derived2> & fout);
    
    template<typename Derived>
    static typename traits<Derived>::ForcePlain cross(const ForceDense<Derived> & fin)
    {
      typename ForceDense<Derived>::ForcePlain fout;
      cross(fin,fout);
      return fout;
    }
    
    template<typename Scalar>
    MotionTpl<Scalar> operator*(const Scalar & s) const
    {
      typedef MotionTpl<Scalar> ReturnType;
      ReturnType res;
      for(Eigen::DenseIndex i = 0; i < dim; ++i)
        res.toVector()[i] = i == axis ? s : Scalar(0);
      
      return res;
    }
    
    template<typename Scalar>
    friend inline MotionTpl<Scalar>
    operator*(const Scalar & s, const SpatialAxis &)
    {
      return SpatialAxis() * s;
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
    
    template<typename MotionDerived>
    typename MotionDerived::MotionPlain
    motionAction(const MotionDense<MotionDerived> & m) const
    {
      typename MotionDerived::MotionPlain res;
      if((LINEAR == 0 && axis<3) || (LINEAR == 3 && axis >= 3))
      {
        res.angular().setZero();
        CartesianAxis3::cross(-m.angular(),res.linear());
      }
      else
      {
        CartesianAxis3::cross(-m.linear(),res.linear());
        CartesianAxis3::cross(-m.angular(),res.angular());
      }
      
      return res;
    }
    
  }; // struct SpatialAxis
  
  template<int axis>
  template<typename Derived1, typename Derived2>
  inline void SpatialAxis<axis>::cross(const MotionDense<Derived1> & min,
                                       const MotionDense<Derived2> & mout)
  {
    Derived2 & mout_ = const_cast<MotionDense<Derived2> &>(mout).derived();
    if((LINEAR == 0 && axis<3) || (LINEAR == 3 && axis >= 3))
    {
      mout_.angular().setZero();
      CartesianAxis3::cross(min.angular(),mout_.linear());
    }
    else
    {
      CartesianAxis3::cross(min.linear(),mout_.linear());
      CartesianAxis3::cross(min.angular(),mout_.angular());
    }
  }
  
  template<int axis>
  template<typename Derived1, typename Derived2>
  inline void SpatialAxis<axis>::cross(const ForceDense<Derived1> & fin,
                                       const ForceDense<Derived2> & fout)
  {
    Derived2 & fout_ = const_cast<ForceDense<Derived2> &>(fout).derived();
    if((LINEAR == 0 && axis<3) || (LINEAR == 3 && axis >= 3))
    {
      fout_.linear().setZero();
      CartesianAxis3::cross(fin.linear(),fout_.angular());
    }
    else
    {
      CartesianAxis3::cross(fin.linear(),fout_.linear());
      CartesianAxis3::cross(fin.angular(),fout_.angular());
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
