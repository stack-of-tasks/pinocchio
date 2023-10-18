//
// Copyright (c) 2017-2018 CNRS
//

#ifndef __pinocchio_cartesian_axis_hpp__
#define __pinocchio_cartesian_axis_hpp__

#include "pinocchio/fwd.hpp"

namespace pinocchio
{
  
  template<int _axis>
  struct CartesianAxis
  {
    enum { axis = _axis, dim = 3 };

    template<typename V3_in, typename V3_out>
    inline static void cross(const Eigen::MatrixBase<V3_in> & vin,
                             const Eigen::MatrixBase<V3_out> & vout);
    
    template<typename V3>
    static typename PINOCCHIO_EIGEN_PLAIN_TYPE(V3) cross(const Eigen::MatrixBase<V3> & vin)
    {
      typename PINOCCHIO_EIGEN_PLAIN_TYPE(V3) res;
      cross(vin,res);
      return res;
    }
    
    template<typename Scalar, typename V3_in, typename V3_out>
    inline static void alphaCross(const Scalar & s,
                                  const Eigen::MatrixBase<V3_in> & vin,
                                  const Eigen::MatrixBase<V3_out> & vout);
    
    template<typename Scalar, typename V3>
    static typename PINOCCHIO_EIGEN_PLAIN_TYPE(V3) alphaCross(const Scalar & s,
                                                    const Eigen::MatrixBase<V3> & vin)
    {
      typename PINOCCHIO_EIGEN_PLAIN_TYPE(V3) res;
      alphaCross(s,vin,res);
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
    
    template<typename Vector3Like>
    static void setTo(const Eigen::MatrixBase<Vector3Like> v3)
    {
      Vector3Like & v3_ = PINOCCHIO_EIGEN_CONST_CAST(Vector3Like,v3);
      typedef typename Vector3Like::Scalar Scalar;
      
      for(Eigen::DenseIndex i = 0; i < dim; ++i)
        v3_[i] = i == axis ? Scalar(1) : Scalar(0);
    }
    
  }; // struct CartesianAxis
  
  template<>
  template<typename V3_in, typename V3_out>
  inline void CartesianAxis<0>::cross(const Eigen::MatrixBase<V3_in> & vin,
                                      const Eigen::MatrixBase<V3_out> & vout)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(V3_in,3)
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(V3_out,3)
    V3_out & vout_ = PINOCCHIO_EIGEN_CONST_CAST(V3_out,vout);
    vout_[0] = 0.; vout_[1] = -vin[2]; vout_[2] = vin[1];
  }
  
  template<>
  template<typename V3_in, typename V3_out>
  inline void CartesianAxis<1>::cross(const Eigen::MatrixBase<V3_in> & vin,
                                      const Eigen::MatrixBase<V3_out> & vout)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(V3_in,3)
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(V3_out,3)
    V3_out & vout_ = PINOCCHIO_EIGEN_CONST_CAST(V3_out,vout);
    vout_[0] = vin[2]; vout_[1] = 0.; vout_[2] = -vin[0];
  }
  
  template<>
  template<typename V3_in, typename V3_out>
  inline void CartesianAxis<2>::cross(const Eigen::MatrixBase<V3_in> & vin,
                                      const Eigen::MatrixBase<V3_out> & vout)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(V3_in,3)
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(V3_out,3)
    V3_out & vout_ = PINOCCHIO_EIGEN_CONST_CAST(V3_out,vout);
    vout_[0] = -vin[1]; vout_[1] = vin[0]; vout_[2] = 0.;
  }
  
  template<>
  template<typename Scalar, typename V3_in, typename V3_out>
  inline void CartesianAxis<0>::alphaCross(const Scalar & s,
                                           const Eigen::MatrixBase<V3_in> & vin,
                                           const Eigen::MatrixBase<V3_out> & vout)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(V3_in,3)
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(V3_out,3)
    V3_out & vout_ = PINOCCHIO_EIGEN_CONST_CAST(V3_out,vout);
    vout_[0] = 0.; vout_[1] = -s*vin[2]; vout_[2] = s*vin[1];
  }
  
  template<>
  template<typename Scalar, typename V3_in, typename V3_out>
  inline void CartesianAxis<1>::alphaCross(const Scalar & s,
                                           const Eigen::MatrixBase<V3_in> & vin,
                                           const Eigen::MatrixBase<V3_out> & vout)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(V3_in,3)
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(V3_out,3)
    V3_out & vout_ = PINOCCHIO_EIGEN_CONST_CAST(V3_out,vout);
    vout_[0] = s*vin[2]; vout_[1] = 0.; vout_[2] = -s*vin[0];
  }
  
  template<>
  template<typename Scalar, typename V3_in, typename V3_out>
  inline void CartesianAxis<2>::alphaCross(const Scalar & s,
                                           const Eigen::MatrixBase<V3_in> & vin,
                                           const Eigen::MatrixBase<V3_out> & vout)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(V3_in,3)
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(V3_out,3)
    V3_out & vout_ = PINOCCHIO_EIGEN_CONST_CAST(V3_out,vout);
    vout_[0] = -s*vin[1]; vout_[1] = s*vin[0]; vout_[2] = 0.;
  }
 
  typedef CartesianAxis<0> AxisX;
  typedef CartesianAxis<1> AxisY;
  typedef CartesianAxis<2> AxisZ;

}

#endif // __pinocchio_cartesian_axis_hpp__
