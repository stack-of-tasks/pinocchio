//
// Copyright (c) 2017-2018 CNRS
//

#ifndef __pinocchio_kinematics_derivatives_hxx__
#define __pinocchio_kinematics_derivatives_hxx__

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/algorithm/check.hpp"

namespace pinocchio
{
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2>
  struct ForwardKinematicsDerivativesForwardStep
  : public fusion::JointVisitorBase< ForwardKinematicsDerivativesForwardStep<Scalar,Options,JointCollectionTpl,ConfigVectorType,TangentVectorType1,TangentVectorType2> >
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
      typedef typename Data::SE3 SE3;
      typedef typename Data::Motion Motion;
      
      const JointIndex & i = jmodel.id();
      const JointIndex & parent = model.parents[i];
      SE3 & oMi = data.oMi[i];
      Motion & vi = data.v[i];
      Motion & ai = data.a[i];
      Motion & ov = data.ov[i];
      Motion & oa = data.oa[i];
      
      jmodel.calc(jdata.derived(),q.derived(),v.derived());
      
      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      
      if(parent>0)
        oMi = data.oMi[parent]*data.liMi[i];
      else
        oMi = data.liMi[i];
      
      vi = jdata.v();
      if(parent>0)
        vi += data.liMi[i].actInv(data.v[parent]);
      
      ai = jdata.S() * jmodel.jointVelocitySelector(a) + jdata.c() + (vi ^ jdata.v());
      if(parent>0)
        ai += data.liMi[i].actInv(data.a[parent]);

      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<typename Data::Matrix6x>::Type ColsBlock;
      ColsBlock dJcols = jmodel.jointCols(data.dJ);
      ColsBlock Jcols = jmodel.jointCols(data.J);
      
      Jcols = oMi.act(jdata.S());
      ov = oMi.act(vi); // Spatial velocity of joint i expressed in the global frame o
      motionSet::motionAction(ov,Jcols,dJcols);
      oa = oMi.act(ai); // Spatial acceleration of joint i expressed in the global frame o
    }
    
  };
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2>
  inline void computeForwardKinematicsDerivatives(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                                  DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                                  const Eigen::MatrixBase<ConfigVectorType> & q,
                                                  const Eigen::MatrixBase<TangentVectorType1> & v,
                                                  const Eigen::MatrixBase<TangentVectorType2> & a)
  {
    assert(q.size() == model.nq && "The configuration vector is not of right size");
    assert(v.size() == model.nv && "The velocity vector is not of right size");
    assert(a.size() == model.nv && "The acceleration vector is not of right size");
    assert(model.check(data) && "data is not consistent with model.");
    
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef typename Model::JointIndex JointIndex;
    
    data.v[0].setZero();
    data.a[0].setZero();
    
    typedef ForwardKinematicsDerivativesForwardStep<Scalar,Options,JointCollectionTpl,ConfigVectorType,TangentVectorType1,TangentVectorType2> Pass1;
    for(JointIndex i=1; i<(JointIndex) model.njoints; ++i)
    {
      Pass1::run(model.joints[i],data.joints[i],
                 typename Pass1::ArgsType(model,data,q.derived(),v.derived(),a.derived()));
    }
  }
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename Matrix6xOut1, typename Matrix6xOut2>
  struct JointVelocityDerivativesBackwardStep
  : public fusion::JointVisitorBase< JointVelocityDerivativesBackwardStep<Scalar,Options,JointCollectionTpl,Matrix6xOut1,Matrix6xOut2> >
  {
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    
    typedef boost::fusion::vector<const Model &,
                                  Data &,
                                  const typename Model::JointIndex &,
                                  const ReferenceFrame &,
                                  Matrix6xOut1 &,
                                  Matrix6xOut2 &
                                  > ArgsType;
    
    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
                     const Model & model,
                     Data & data,
                     const typename Model::JointIndex & jointId,
                     const ReferenceFrame & rf,
                     const Eigen::MatrixBase<Matrix6xOut1> & v_partial_dq,
                     const Eigen::MatrixBase<Matrix6xOut2> & v_partial_dv)
    {
      typedef typename Model::JointIndex JointIndex;
      typedef typename Data::SE3 SE3;
      typedef typename Data::Motion Motion;
      
      const JointIndex & i = jmodel.id();
      const JointIndex & parent = model.parents[i];
      Motion & vtmp = data.ov[0]; // Temporary variable
      
      const SE3 & oMlast = data.oMi[jointId];
      const Motion & vlast = data.ov[jointId];

      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<typename Data::Matrix6x>::Type ColsBlock;
      ColsBlock Jcols = jmodel.jointCols(data.J);
      
      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Matrix6xOut1>::Type ColsBlockOut1;
      Matrix6xOut1 & v_partial_dq_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut1,v_partial_dq);
      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Matrix6xOut2>::Type ColsBlockOut2;
      Matrix6xOut2 & v_partial_dv_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut2,v_partial_dv);
      
      // dvec/dv
      ColsBlockOut2 v_partial_dv_cols = jmodel.jointCols(v_partial_dv_);
      if(rf == WORLD)
        v_partial_dv_cols = Jcols;
      else
        motionSet::se3ActionInverse(oMlast,Jcols,v_partial_dv_cols);

      // dvec/dq
      ColsBlockOut1 v_partial_dq_cols = jmodel.jointCols(v_partial_dq_);
      if(rf == WORLD)
      {
        if(parent > 0)
          vtmp = data.ov[parent] - vlast;
        else
          vtmp = -vlast;
        motionSet::motionAction(vtmp,Jcols,v_partial_dq_cols);
      }
      else
      {
        if(parent > 0)
        {
          vtmp = oMlast.actInv(data.ov[parent]);
          motionSet::motionAction(vtmp,v_partial_dv_cols,v_partial_dq_cols);
        }
      }
      
      
    }
    
  };
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename Matrix6xOut1, typename Matrix6xOut2>
  inline void getJointVelocityDerivatives(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                          DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                          const Model::JointIndex jointId,
                                          const ReferenceFrame rf,
                                          const Eigen::MatrixBase<Matrix6xOut1> & v_partial_dq,
                                          const Eigen::MatrixBase<Matrix6xOut2> & v_partial_dv)
  {
    EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Matrix6xOut1,Data::Matrix6x);
    EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Matrix6xOut2,Data::Matrix6x);
    
    assert(v_partial_dq.cols() ==  model.nv);
    assert(v_partial_dv.cols() ==  model.nv);
    assert(model.check(data) && "data is not consistent with model.");
    
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef typename Model::JointIndex JointIndex;
    
    typedef JointVelocityDerivativesBackwardStep<Scalar,Options,JointCollectionTpl,Matrix6xOut1,Matrix6xOut2> Pass1;
    for(JointIndex i = jointId; i > 0; i = model.parents[i])
    {
      Pass1::run(model.joints[i],
                 typename Pass1::ArgsType(model,data,
                                          jointId,rf,
                                          PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut1,v_partial_dq),
                                          PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut2,v_partial_dv)));
    }
  
    // Set back ov[0] to a zero value
    data.ov[0].setZero();
  }
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename Matrix6xOut1, typename Matrix6xOut2, typename Matrix6xOut3, typename Matrix6xOut4>
  struct JointAccelerationDerivativesBackwardStep
  : public fusion::JointVisitorBase< JointAccelerationDerivativesBackwardStep<Scalar,Options,JointCollectionTpl,Matrix6xOut1,Matrix6xOut2,Matrix6xOut3,Matrix6xOut4> >
  {
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    
    typedef boost::fusion::vector<const Model &,
                                  Data &,
                                  const typename Model::JointIndex &,
                                  const ReferenceFrame &,
                                  Matrix6xOut1 &,
                                  Matrix6xOut2 &,
                                  Matrix6xOut3 &,
                                  Matrix6xOut4 &
                                  > ArgsType;
    
    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
                     const Model & model,
                     Data & data,
                     const typename Model::JointIndex & jointId,
                     const ReferenceFrame & rf,
                     const Eigen::MatrixBase<Matrix6xOut1> & v_partial_dq,
                     const Eigen::MatrixBase<Matrix6xOut2> & a_partial_dq,
                     const Eigen::MatrixBase<Matrix6xOut3> & a_partial_dv,
                     const Eigen::MatrixBase<Matrix6xOut4> & a_partial_da)
    {
      typedef typename Model::JointIndex JointIndex;
      typedef typename Data::SE3 SE3;
      typedef typename Data::Motion Motion;
      
      const JointIndex & i = jmodel.id();
      const JointIndex & parent = model.parents[i];
      Motion & vtmp = data.ov[0]; // Temporary variable
      Motion & atmp = data.oa[0]; // Temporary variable
      
      const SE3 & oMlast = data.oMi[jointId];
      const Motion & vlast = data.ov[jointId];
      const Motion & alast = data.oa[jointId];

      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<typename Data::Matrix6x>::Type ColsBlock;
      ColsBlock dJcols = jmodel.jointCols(data.dJ);
      ColsBlock Jcols = jmodel.jointCols(data.J);
      
      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Matrix6xOut1>::Type ColsBlockOut1;
      Matrix6xOut1 & v_partial_dq_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut1,v_partial_dq);
      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Matrix6xOut2>::Type ColsBlockOut2;
      Matrix6xOut2 & a_partial_dq_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut2,a_partial_dq);
      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Matrix6xOut3>::Type ColsBlockOut3;
      Matrix6xOut3 & a_partial_dv_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut3,a_partial_dv);
      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Matrix6xOut4>::Type ColsBlockOut4;
      Matrix6xOut4 & a_partial_da_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut4,a_partial_da);
      
      ColsBlockOut1 v_partial_dq_cols = jmodel.jointCols(v_partial_dq_);
      ColsBlockOut2 a_partial_dq_cols = jmodel.jointCols(a_partial_dq_);
      ColsBlockOut3 a_partial_dv_cols = jmodel.jointCols(a_partial_dv_);
      ColsBlockOut4 a_partial_da_cols = jmodel.jointCols(a_partial_da_);
      
      // dacc/da
      if(rf == WORLD)
        a_partial_da_cols = Jcols;
      else
        motionSet::se3ActionInverse(oMlast,Jcols,a_partial_da_cols);
      
      // dacc/dv
      if(rf == WORLD)
      {
        if(parent > 0)
          vtmp = data.ov[parent] - vlast;
        else
          vtmp = -vlast;
        
        /// also computes dvec/dq
        motionSet::motionAction(vtmp,Jcols,v_partial_dq_cols);
        
        a_partial_dv_cols = v_partial_dq_cols + dJcols;
      }
      else
      {
       /// also computes dvec/dq
        if(parent > 0)
        {
          vtmp = oMlast.actInv(data.ov[parent]);
          motionSet::motionAction(vtmp,a_partial_da_cols,v_partial_dq_cols);
        }
        
        if(parent > 0)
          vtmp -= data.v[jointId];
        else
          vtmp = -data.v[jointId];
        
        motionSet::motionAction(vtmp,a_partial_da_cols,a_partial_dv_cols);
        motionSet::se3ActionInverse<ADDTO>(oMlast,dJcols,a_partial_dv_cols);
      }
      
      // dacc/dq
      if(rf == WORLD)
      {
        if(parent > 0)
          atmp = data.oa[parent] - alast;
        else
          atmp = -alast;
        motionSet::motionAction(atmp,Jcols,a_partial_dq_cols);
        
        if(parent >0)
          motionSet::motionAction<ADDTO>(vtmp,dJcols,a_partial_dq_cols);
      }
      else
      {
        if(parent > 0)
        {
          atmp = oMlast.actInv(data.oa[parent]);
          motionSet::motionAction(atmp,a_partial_da_cols,a_partial_dq_cols);
        }
        
        motionSet::motionAction<ADDTO>(vtmp,v_partial_dq_cols,a_partial_dq_cols);
      }

      
    }
    
  };
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename Matrix6xOut1, typename Matrix6xOut2, typename Matrix6xOut3, typename Matrix6xOut4>
  inline void getJointAccelerationDerivatives(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                              DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                              const Model::JointIndex jointId,
                                              const ReferenceFrame rf,
                                              const Eigen::MatrixBase<Matrix6xOut1> & v_partial_dq,
                                              const Eigen::MatrixBase<Matrix6xOut2> & a_partial_dq,
                                              const Eigen::MatrixBase<Matrix6xOut3> & a_partial_dv,
                                              const Eigen::MatrixBase<Matrix6xOut4> & a_partial_da)
  {
    EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Matrix6xOut1,Data::Matrix6x);
    EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Matrix6xOut2,Data::Matrix6x);
    EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Matrix6xOut3,Data::Matrix6x);
    EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Matrix6xOut4,Data::Matrix6x);
    
    assert(v_partial_dq.cols() ==  model.nv);
    assert(a_partial_dq.cols() ==  model.nv);
    assert(a_partial_dv.cols() ==  model.nv);
    assert(a_partial_da.cols() ==  model.nv);
    assert(model.check(data) && "data is not consistent with model.");
    
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef typename Model::JointIndex JointIndex;
    
    typedef JointAccelerationDerivativesBackwardStep<Scalar,Options,JointCollectionTpl,Matrix6xOut1,Matrix6xOut2,Matrix6xOut3,Matrix6xOut4> Pass1;
    for(JointIndex i = jointId; i > 0; i = model.parents[i])
    {
      Pass1::run(model.joints[i],
                 typename Pass1::ArgsType(model,data,
                                          jointId,
                                          rf,
                                          PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut1,v_partial_dq),
                                          PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut2,a_partial_dq),
                                          PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut3,a_partial_dv),
                                          PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut4,a_partial_da)));
      
    }
    
    // Set Zero to temporary spatial variables
    data.ov[0].setZero();
    data.oa[0].setZero();
  }

} // namespace pinocchio

#endif // ifndef __pinocchio_kinematics_derivatives_hxx__

