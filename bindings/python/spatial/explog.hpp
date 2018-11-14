//
// Copyright (c) 2015-2018 CNRS INRIA
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_python_explog_hpp__
#define __pinocchio_python_explog_hpp__

#include "pinocchio/spatial/explog.hpp"

namespace pinocchio
{
  namespace python
  {
    
    template<typename Vector3Like>
    Eigen::Matrix<typename Vector3Like::Scalar,3,3,EIGEN_PLAIN_TYPE(Vector3Like)::Options>
    Jexp3_proxy(const Vector3Like & v)
    {
      typedef Eigen::Matrix<typename Vector3Like::Scalar,3,3,EIGEN_PLAIN_TYPE(Vector3Like)::Options> ReturnType;
      ReturnType res; Jexp3(v,res);
      return res;
    }
    
    template<typename Matrix3Like>
    typename EIGEN_PLAIN_TYPE(Matrix3Like)
    Jlog3_proxy(const Matrix3Like & M)
    {
      typedef typename EIGEN_PLAIN_TYPE(Matrix3Like) ReturnType;
      ReturnType res; Jlog3(M,res);
      return res;
    }
    
    template<typename Scalar, int Options>
    SE3Tpl<Scalar,Options> exp6_proxy(const MotionTpl<Scalar,Options> & v)
    {
      return exp6(v);
    }
    
    template<typename Scalar, int Options>
    typename SE3Tpl<Scalar,Options>::Matrix6 Jlog6_proxy(const SE3Tpl<Scalar,Options> & M)
    {
      typedef typename SE3Tpl<Scalar,Options>::Matrix6 ReturnType;
      ReturnType res; Jlog6(M,res);
      return res;
    }
    
    template<typename Scalar, int Options>
    typename MotionTpl<Scalar,Options>::Matrix6
    Jexp6_proxy(const MotionTpl<Scalar,Options> & v)
    {
      typedef typename MotionTpl<Scalar,Options>::Matrix6 ReturnType;
      ReturnType res; Jexp6(v,res);
      return res;
    }
    
    Eigen::Vector3d log3_proxy(const Eigen::Matrix3d & R)
    {
      return log3(R);
    }
    
  } // namespace python
} //namespace pinocchio

#endif // ifndef __pinocchio_python_explog_hpp__
