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

#ifndef __se3_cartesian_axis_hpp__
#define __se3_cartesian_axis_hpp__

#include <Eigen/Core>

namespace se3
{
  
  template<int _axis>
  struct CartesianAxis
  {
    enum { axis = _axis, dim = 3 };

    template<typename V3_in, typename V3_out>
    inline static void cross(const Eigen::MatrixBase<V3_in> & vin,
                             const Eigen::MatrixBase<V3_out> & vout);
    
    template<typename V3>
    static typename EIGEN_PLAIN_TYPE(V3) cross(const Eigen::MatrixBase<V3> & vin)
    {
      typename EIGEN_PLAIN_TYPE(V3) res;
      cross(vin,res);
      return res;
    }
    
    template<typename Scalar>
    Eigen::Matrix<Scalar,dim,1> operator*(const Scalar & s) const
    {
      typedef Eigen::Matrix<Scalar,dim,1> ReturnType;
      ReturnType res;
      for(Eigen::DenseIndex i = 0; i < dim; ++i)
        res[i] = i == axis ? s : Scalar(0);
      
      return res;
    }
    
    template<typename Scalar>
    friend inline Eigen::Matrix<Scalar,dim,1>
    operator*(const Scalar & s, const CartesianAxis &)
    {
      return CartesianAxis() * s;
    }
    
  }; // struct CartesianAxis
  
  template<>
  template<typename V3_in, typename V3_out>
  inline void CartesianAxis<0>::cross(const Eigen::MatrixBase<V3_in> & vin,
                                         const Eigen::MatrixBase<V3_out> & vout)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(V3_in,3)
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(V3_out,3)
    V3_out & vout_ = const_cast<Eigen::MatrixBase<V3_out> &>(vout).derived();
    vout_[0] = 0.; vout_[1] = -vin[2]; vout_[2] = vin[1];
  }
  
  template<>
  template<typename V3_in, typename V3_out>
  inline void CartesianAxis<1>::cross(const Eigen::MatrixBase<V3_in> & vin,
                                         const Eigen::MatrixBase<V3_out> & vout)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(V3_in,3)
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(V3_out,3)
    V3_out & vout_ = const_cast<Eigen::MatrixBase<V3_out> &>(vout).derived();
    vout_[0] = vin[2]; vout_[1] = 0.; vout_[2] = -vin[0];
  }
  
  template<>
  template<typename V3_in, typename V3_out>
  inline void CartesianAxis<2>::cross(const Eigen::MatrixBase<V3_in> & vin,
                                         const Eigen::MatrixBase<V3_out> & vout)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(V3_in,3)
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(V3_out,3)
    V3_out & vout_ = const_cast<Eigen::MatrixBase<V3_out> &>(vout).derived();
    vout_[0] = -vin[1]; vout_[1] = vin[0]; vout_[2] = 0.;
  }
 
  typedef CartesianAxis<0> AxisX;
  typedef CartesianAxis<1> AxisY;
  typedef CartesianAxis<2> AxisZ;

}

#endif // __se3_cartesian_axis_hpp__
