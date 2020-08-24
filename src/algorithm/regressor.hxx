//
// Copyright (c) 2018-2019 CNRS INRIA
//

#ifndef __pinocchio_regressor_hxx__
#define __pinocchio_regressor_hxx__

#include "pinocchio/algorithm/check.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/spatial/skew.hpp"
#include "pinocchio/spatial/symmetric3.hpp"

namespace pinocchio
{
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
  inline typename DataTpl<Scalar,Options,JointCollectionTpl>::Matrix3x &
  computeStaticRegressor(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                         DataTpl<Scalar,Options,JointCollectionTpl> & data,
                         const Eigen::MatrixBase<ConfigVectorType> & q)
  {
    assert(model.check(data) && "data is not consistent with model.");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(q.size(), model.nq);
    
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

  namespace details {
    // auxiliary function for bodyRegressor: bigL(omega)*I.toDynamicParameters().tail<6>() = I.inertia() * omega
/*
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
*/

    // auxiliary function for bodyRegressor: res += bigL(omega)
    template<typename Vector3Like, typename OutputType>
    inline void
    addBigL(const Eigen::MatrixBase<Vector3Like> & omega, const Eigen::MatrixBase<OutputType> & out)
    {
      OutputType & res = PINOCCHIO_EIGEN_CONST_CAST(OutputType, out);
      res(0,0)+=omega[0]; res(0,1)+=omega[1]; res(0,3)+=omega[2];
      res(1,1)+=omega[0]; res(1,2)+=omega[1]; res(1,4)+=omega[2];
      res(2,3)+=omega[0]; res(2,4)+=omega[1]; res(2,5)+=omega[2];
    }

    // auxiliary function for bodyRegressor: res = cross(omega,bigL(omega))
    template<typename Vector3Like, typename OutputType>
    inline void
    crossBigL(const Eigen::MatrixBase<Vector3Like> & v, const Eigen::MatrixBase<OutputType> & out)
    {
      typedef typename Vector3Like::Scalar Scalar;
      OutputType & res = PINOCCHIO_EIGEN_CONST_CAST(OutputType, out);

      res <<  Scalar(0),          -v[2]*v[0], -v[2]*v[1],           v[1]*v[0], v[1]*v[1]-v[2]*v[2],  v[2]*v[1],
              v[2]*v[0],           v[2]*v[1],  Scalar(0), v[2]*v[2]-v[0]*v[0],          -v[1]*v[0], -v[2]*v[0],
             -v[1]*v[0], v[0]*v[0]-v[1]*v[1],  v[1]*v[0],          -v[2]*v[1],           v[2]*v[0],  Scalar(0);
    }
  }

  template<typename MotionVelocity, typename MotionAcceleration, typename OutputType>
  inline void
  bodyRegressor(const MotionDense<MotionVelocity> & v,
                const MotionDense<MotionAcceleration> & a,
                const Eigen::MatrixBase<OutputType> & regressor)
  {
    PINOCCHIO_ASSERT_MATRIX_SPECIFIC_SIZE(OutputType, regressor, 6, 10);

    typedef typename MotionVelocity::Scalar Scalar;
    enum { Options = PINOCCHIO_EIGEN_PLAIN_TYPE(typename MotionVelocity::Vector3)::Options };

    typedef Symmetric3Tpl<Scalar,Options> Symmetric3;
    typedef typename Symmetric3::SkewSquare SkewSquare;
    using ::pinocchio::details::crossBigL;
    using ::pinocchio::details::addBigL;

    OutputType & res = PINOCCHIO_EIGEN_CONST_CAST(OutputType, regressor);

    res.template block<3,1>(MotionVelocity::LINEAR,0) = a.linear() + v.angular().cross(v.linear());
    const Eigen::Block<OutputType,3,1> & acc = res.template block<3,1>(MotionVelocity::LINEAR,0);
    res.template block<3,3>(MotionVelocity::LINEAR,1) = Symmetric3(SkewSquare(v.angular())).matrix();
    addSkew(a.angular(), res.template block<3,3>(MotionVelocity::LINEAR,1));

    res.template block<3,6>(MotionVelocity::LINEAR,4).setZero();

    res.template block<3,1>(MotionVelocity::ANGULAR,0).setZero();
    skew(-acc, res.template block<3,3>(MotionVelocity::ANGULAR,1));
    // res.template block<3,6>(MotionVelocity::ANGULAR,4) = bigL(a.angular()) + cross(v.angular(), bigL(v.angular()));
    crossBigL(v.angular(), res.template block<3,6>(MotionVelocity::ANGULAR,4));
    addBigL(a.angular(), res.template block<3,6>(MotionVelocity::ANGULAR,4));
  }

  template<typename MotionVelocity, typename MotionAcceleration>
  inline Eigen::Matrix<typename MotionVelocity::Scalar,6,10,PINOCCHIO_EIGEN_PLAIN_TYPE(typename MotionVelocity::Vector3)::Options>
  bodyRegressor(const MotionDense<MotionVelocity> & v, const MotionDense<MotionAcceleration> & a)
  {
    typedef typename MotionVelocity::Scalar Scalar;
    enum { Options = PINOCCHIO_EIGEN_PLAIN_TYPE(typename MotionVelocity::Vector3)::Options };
    typedef Eigen::Matrix<Scalar,6,10,Options> ReturnType;

    ReturnType res;
    bodyRegressor(v,a,res);
    return res;
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline typename DataTpl<Scalar,Options,JointCollectionTpl>::BodyRegressorType &
  jointBodyRegressor(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                     DataTpl<Scalar,Options,JointCollectionTpl> & data,
                     JointIndex jointId)
  {
    assert(model.check(data) && "data is not consistent with model.");

    PINOCCHIO_UNUSED_VARIABLE(model);

    bodyRegressor(data.v[jointId], data.a_gf[jointId], data.bodyRegressor);
    return data.bodyRegressor;
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline typename DataTpl<Scalar,Options,JointCollectionTpl>::BodyRegressorType &
  frameBodyRegressor(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                     DataTpl<Scalar,Options,JointCollectionTpl> & data,
                     FrameIndex frameId)
  {
    assert(model.check(data) && "data is not consistent with model.");

    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef typename Model::Frame Frame;
    typedef typename Model::JointIndex JointIndex;
    typedef typename Model::SE3 SE3;

    const Frame & frame = model.frames[frameId];
    const JointIndex & parent = frame.parent;
    const SE3 & placement = frame.placement;

    bodyRegressor(placement.actInv(data.v[parent]), placement.actInv(data.a_gf[parent]), data.bodyRegressor);
    return data.bodyRegressor;
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2>
  struct JointTorqueRegressorForwardStep
  : public fusion::JointUnaryVisitorBase< JointTorqueRegressorForwardStep<Scalar,Options,JointCollectionTpl,ConfigVectorType,TangentVectorType1,TangentVectorType2> >
  {
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;

    typedef boost::fusion::vector<const Model &,
                                  Data &,
                                  const ConfigVectorType &,
                                  const TangentVectorType1 &,
                                  const TangentVectorType2 &
                                  > ArgsType;

    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
                     JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data,
                     const Eigen::MatrixBase<ConfigVectorType> & q,
                     const Eigen::MatrixBase<TangentVectorType1> & v,
                     const Eigen::MatrixBase<TangentVectorType2> & a)
    {
      typedef typename Model::JointIndex JointIndex;

      const JointIndex i = jmodel.id();
      const JointIndex parent = model.parents[i];

      jmodel.calc(jdata.derived(),q.derived(),v.derived());

      data.liMi[i] = model.jointPlacements[i]*jdata.M();

      data.v[i] = jdata.v();
      if(parent>0)
        data.v[i] += data.liMi[i].actInv(data.v[parent]);

      data.a_gf[i] = jdata.c() + (data.v[i] ^ jdata.v());
      data.a_gf[i] += jdata.S() * jmodel.jointVelocitySelector(a);
      data.a_gf[i] += data.liMi[i].actInv(data.a_gf[parent]);
    }

  };

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  struct JointTorqueRegressorBackwardStep
  : public fusion::JointUnaryVisitorBase< JointTorqueRegressorBackwardStep<Scalar,Options,JointCollectionTpl> >
  {
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    typedef typename DataTpl<Scalar,Options,JointCollectionTpl>::BodyRegressorType BodyRegressorType;
    
    typedef boost::fusion::vector<const Model &,
                                  Data &,
                                  const JointIndex &
                                  > ArgsType;
    
    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
                     JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data,
                     const JointIndex & col_idx)
    {
      typedef typename Model::JointIndex JointIndex;
      
      const JointIndex i = jmodel.id();
      const JointIndex parent = model.parents[i];
      
      data.jointTorqueRegressor.block(jmodel.idx_v(),10*(Eigen::DenseIndex(col_idx)-1),
                                      jmodel.nv(),10) = jdata.S().transpose()*data.bodyRegressor;
      
      if(parent>0)
        forceSet::se3Action(data.liMi[i],data.bodyRegressor,data.bodyRegressor);
    }
  };

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2>
  inline typename DataTpl<Scalar,Options,JointCollectionTpl>::MatrixXs &
  computeJointTorqueRegressor(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                              DataTpl<Scalar,Options,JointCollectionTpl> & data,
                              const Eigen::MatrixBase<ConfigVectorType> & q,
                              const Eigen::MatrixBase<TangentVectorType1> & v,
                              const Eigen::MatrixBase<TangentVectorType2> & a)
  {
    assert(model.check(data) && "data is not consistent with model.");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(q.size(), model.nq);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(v.size(), model.nv);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(a.size(), model.nv);

    data.v[0].setZero();
    data.a_gf[0] = -model.gravity;
    data.jointTorqueRegressor.setZero();

    typedef JointTorqueRegressorForwardStep<Scalar,Options,JointCollectionTpl,ConfigVectorType,TangentVectorType1,TangentVectorType2> Pass1;
    typename Pass1::ArgsType arg1(model,data,q.derived(),v.derived(),a.derived());
    for(JointIndex i=1; i<(JointIndex)model.njoints; ++i)
    {
      Pass1::run(model.joints[i],data.joints[i],
                 arg1);
    }

    typedef JointTorqueRegressorBackwardStep<Scalar,Options,JointCollectionTpl> Pass2;
    for(JointIndex i=(JointIndex)model.njoints-1; i>0; --i)
    {
      jointBodyRegressor(model,data,i);

      typename Pass2::ArgsType arg2(model,data,i);
      for(JointIndex j=i; j>0; j = model.parents[j])
      {
        Pass2::run(model.joints[j],data.joints[j],
                   arg2);
      }
    }

    return data.jointTorqueRegressor;
  }

} // namespace pinocchio

#endif // ifndef __pinocchio_regressor_hxx__
