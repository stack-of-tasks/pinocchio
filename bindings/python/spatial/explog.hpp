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
    Eigen::Matrix<typename Vector3Like::Scalar,3,3,PINOCCHIO_EIGEN_PLAIN_TYPE(Vector3Like)::Options>
    exp3_proxy(const Vector3Like & v)
    {
      return exp3(v);
    }
    
    template<typename Vector3Like>
    Eigen::Matrix<typename Vector3Like::Scalar,3,3,PINOCCHIO_EIGEN_PLAIN_TYPE(Vector3Like)::Options>
    Jexp3_proxy(const Vector3Like & v)
    {
      typedef Eigen::Matrix<typename Vector3Like::Scalar,3,3,PINOCCHIO_EIGEN_PLAIN_TYPE(Vector3Like)::Options> ReturnType;
      ReturnType res; Jexp3(v,res);
      return res;
    }
    
    template<typename Matrix3Like>
    typename PINOCCHIO_EIGEN_PLAIN_TYPE(Matrix3Like)
    Jlog3_proxy(const Matrix3Like & M)
    {
      typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(Matrix3Like) ReturnType;
      ReturnType res; Jlog3(M,res);
      return res;
    }
    
    template<typename Scalar, int Options>
    SE3Tpl<Scalar,Options> exp6_proxy(const MotionTpl<Scalar,Options> & v)
    {
      return exp6(v);
    }
    
    template<typename Vector6Like>
    SE3Tpl<typename Vector6Like::Scalar,PINOCCHIO_EIGEN_PLAIN_TYPE(Vector6Like)::Options>
    exp6_proxy(const Vector6Like & vec6)
    {
      return exp6(vec6);
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
    
    template<typename Vector6Like>
    Eigen::Matrix<typename Vector6Like::Scalar,6,6,PINOCCHIO_EIGEN_PLAIN_TYPE(Vector6Like)::Options>
    Jexp6_proxy(const Vector6Like & vec6)
    {
      typedef MotionRef<const Vector6Like> Motion;
      Motion v(vec6);
      typedef typename Motion::Matrix6 ReturnType;
      ReturnType res; Jexp6(v,res);
      return res;
    }
    
    template<typename Matrix3Like>
    Eigen::Matrix<typename Matrix3Like::Scalar,3,1,PINOCCHIO_EIGEN_PLAIN_TYPE(Matrix3Like)::Options>
    log3_proxy(const Matrix3Like & R)
    {
      return log3(R);
    }
    
    template<typename Matrix4Like>
    MotionTpl<typename Matrix4Like::Scalar,PINOCCHIO_EIGEN_PLAIN_TYPE(Matrix4Like)::Options>
    log6_proxy(const Matrix4Like & homegenous_matrix)
    {
      return log6(homegenous_matrix);
    }
    
  } // namespace python
} //namespace pinocchio

#endif // ifndef __pinocchio_python_explog_hpp__
