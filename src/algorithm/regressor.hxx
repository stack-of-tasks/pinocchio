//
// Copyright (c) 2018 CNRS
//

#ifndef __pinocchio_regressor_hxx__
#define __pinocchio_regressor_hxx__

#include "pinocchio/algorithm/check.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/spatial/skew.hpp"
#include "pinocchio/spatial/symmetric3.hpp"

namespace pinocchio
{
  
  namespace regressor
  {
    
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
    inline typename DataTpl<Scalar,Options,JointCollectionTpl>::Matrix3x &
    computeStaticRegressor(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                           DataTpl<Scalar,Options,JointCollectionTpl> & data,
                           const Eigen::MatrixBase<ConfigVectorType> & q)
    {
      assert(model.check(data) && "data is not consistent with model.");
      assert(q.size() == model.nq);
      
      typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
      typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
      typedef typename Model::JointIndex JointIndex;
      typedef typename Data::SE3 SE3;

      typedef typename Data::Matrix3x Matrix3x;
      typedef typename SizeDepType<4>::ColsReturn<Matrix3x>::Type ColsBlock;
      
      forwardKinematics(model,data,q.derived());
      
      // Computes the total mass of the system
      Scalar mass = Scalar(0);
      for(JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
        mass += model.inertias[(JointIndex)i].mass();
      
      const Scalar mass_inv = Scalar(1)/mass;
      for(JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
      {
        const SE3 & oMi = data.oMi[i];
        ColsBlock sr_cols = data.staticRegressor.template middleCols<4>((Eigen::DenseIndex)(i-1)*4);
        sr_cols.col(0) = oMi.translation();
        sr_cols.template rightCols<3>() = oMi.rotation();
        sr_cols *= mass_inv;
      }
      
      return data.staticRegressor;
    }
  }


  namespace details {
    // auxiliary function for bodyRegressor: bigL(omega)*I.toDynamicParameters().tail<6>() = I.inertia() * omega
    template<typename Vector3Like>
    inline Eigen::Matrix<typename Vector3Like::Scalar,3,6,PINOCCHIO_EIGEN_PLAIN_TYPE(Vector3Like)::Options>
    bigL(const Eigen::MatrixBase<Vector3Like> & omega)
    {
      typedef typename Vector3Like::Scalar Scalar;
      enum { Options = PINOCCHIO_EIGEN_PLAIN_TYPE(Vector3Like)::Options };
      typedef Eigen::Matrix<Scalar,3,6,Options> ReturnType;

      ReturnType L;
      L <<  omega[0],  omega[1], Scalar(0),  omega[2], Scalar(0), Scalar(0),
           Scalar(0),  omega[0],  omega[1], Scalar(0),  omega[2], Scalar(0),
           Scalar(0), Scalar(0), Scalar(0),  omega[0],  omega[1],  omega[2];
      return L;
    }
  }

  template<typename MotionVelocity, typename MotionAcceleration>
  inline Eigen::Matrix<typename MotionVelocity::Scalar,6,10,PINOCCHIO_EIGEN_PLAIN_TYPE(typename MotionVelocity::Vector3)::Options>
  bodyRegressor(const MotionDense<MotionVelocity> & v, const MotionDense<MotionAcceleration> & a)
  {
    typedef typename MotionVelocity::Scalar Scalar;
    enum { Options = PINOCCHIO_EIGEN_PLAIN_TYPE(typename MotionVelocity::Vector3)::Options };

    typedef Symmetric3Tpl<Scalar,Options> Symmetric3;
    typedef typename Symmetric3::SkewSquare SkewSquare;
    typedef Eigen::Matrix<Scalar,6,10,Options> ReturnType;
    using ::pinocchio::details::bigL;

    Eigen::Matrix<Scalar, 3, 1, Options> acc = a.linear() + v.angular().cross(v.linear());

    ReturnType res;

    res.template block<3,1>(MotionVelocity::LINEAR,0) = acc;
    res.template block<3,3>(MotionVelocity::LINEAR,1) = Symmetric3(SkewSquare(v.angular())).matrix();
    addSkew(a.angular(), res.template block<3,3>(MotionVelocity::LINEAR,1));

    res.template block<3,6>(MotionVelocity::LINEAR,4).setZero();

    res.template block<3,1>(MotionVelocity::ANGULAR,0).setZero();
    skew(-acc, res.template block<3,3>(MotionVelocity::ANGULAR,1));
    res.template block<3,6>(MotionVelocity::ANGULAR,4) = bigL(a.angular()) + cross(v.angular(), bigL(v.angular()));

    return res;
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline Eigen::Matrix<Scalar,6,10,Options>
  jointBodyRegressor(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                     DataTpl<Scalar,Options,JointCollectionTpl> & data,
                     JointIndex jointId)
  {
    assert(model.check(data) && "data is not consistent with model.");

    PINOCCHIO_UNUSED_VARIABLE(model);

    return bodyRegressor(data.v[jointId], data.a_gf[jointId]);
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline Eigen::Matrix<Scalar,6,10,Options>
  frameBodyRegressor(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                     DataTpl<Scalar,Options,JointCollectionTpl> & data,
                     FrameIndex frameId)
  {
    assert(model.check(data) && "data is not consistent with model.");

    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef typename Model::Frame Frame;
    typedef typename Model::JointIndex JointIndex;
    typedef typename Model::SE3 SE3;
    typedef typename Model::Motion Motion;

    const Frame & frame = model.frames[frameId];
    const JointIndex & parent = frame.parent;
    const SE3 & placement = frame.placement;

    const Motion v  = placement.actInv(data.v[parent]);
    const Motion a  = placement.actInv(data.a_gf[parent]);

    return bodyRegressor(v, a);
  }

} // namespace pinocchio

#endif // ifndef __pinocchio_regressor_hxx__
