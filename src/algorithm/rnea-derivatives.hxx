//
// Copyright (c) 2017-2018 CNRS, INRIA
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

#ifndef __se3_rnea_derivatives_hxx__
#define __se3_rnea_derivatives_hxx__

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/algorithm/check.hpp"

namespace se3
{
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
  struct ComputeGeneralizedGravityDerivativeForwardStep
  : public fusion::JointVisitorBase< ComputeGeneralizedGravityDerivativeForwardStep<Scalar,Options,JointCollectionTpl,ConfigVectorType> >
  {
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    
    typedef boost::fusion::vector<const Model &,
                                  Data &,
                                  const ConfigVectorType &
                                  > ArgsType;
    
    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
                     JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data,
                     const Eigen::MatrixBase<ConfigVectorType> & q)
    {
      typedef typename Model::JointIndex JointIndex;
      typedef typename Data::Motion Motion;
      
      const JointIndex & i = jmodel.id();
      const JointIndex & parent = model.parents[i];
      const Motion & oa = data.a_gf[0];
      
      jmodel.calc(jdata.derived(),q.derived());
      
      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      
      if(parent > 0)
        data.oMi[i] = data.oMi[parent] * data.liMi[i];
      else
        data.oMi[i] = data.liMi[i];
      
      data.oYcrb[i] = data.oMi[i].act(model.inertias[i]);

      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<typename Data::Matrix6x>::Type ColsBlock;
      ColsBlock J_cols = jmodel.jointCols(data.J);
      ColsBlock dAdq_cols = jmodel.jointCols(data.dAdq);
      J_cols = data.oMi[i].act(jdata.S());
      motionSet::motionAction(oa,J_cols,dAdq_cols);
    }
    
  };
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ReturnMatrixType>
  struct ComputeGeneralizedGravityDerivativeBackwardStep
  : public fusion::JointVisitorBase< ComputeGeneralizedGravityDerivativeBackwardStep<Scalar,Options,JointCollectionTpl,ReturnMatrixType> >
  {
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    
    typedef boost::fusion::vector<const Model &,
                                  Data &,
                                  ReturnMatrixType &
                                  > ArgsType;
    
    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
                     const Model & model,
                     Data & data,
                     const Eigen::MatrixBase<ReturnMatrixType> & gravity_partial_dq)
    {
      typedef typename Model::JointIndex JointIndex;
      typedef typename Data::Motion Motion;
      
      const JointIndex & i = jmodel.id();
      const JointIndex & parent = model.parents[i];
      
      typename Data::RowMatrix6 & M6tmpR = data.M6tmpR;
      const Motion & oa = data.a_gf[0];

      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<typename Data::Matrix6x>::Type ColsBlock;

      ColsBlock J_cols = jmodel.jointCols(data.J);
      ColsBlock dAdq_cols = jmodel.jointCols(data.dAdq);
      ColsBlock dFdq_cols = jmodel.jointCols(data.dFdq);
      
      motionSet::inertiaAction(data.oYcrb[i],dAdq_cols,dFdq_cols);
      
      ReturnMatrixType & gravity_partial_dq_ = EIGEN_CONST_CAST(ReturnMatrixType,gravity_partial_dq);
      gravity_partial_dq_.block(jmodel.idx_v(),jmodel.idx_v(),jmodel.nv(),data.nvSubtree[i]).noalias()
      = J_cols.transpose()*data.dFdq.middleCols(jmodel.idx_v(),data.nvSubtree[i]);
      
      data.of[i] = data.oYcrb[i] * oa;
      motionSet::act<ADDTO>(J_cols,data.of[i],dFdq_cols);
      
      lhsInertiaMult(data.oYcrb[i],J_cols.transpose(),M6tmpR.topRows(jmodel.nv()));
      for(int j = data.parents_fromRow[(typename Model::Index)jmodel.idx_v()];j >= 0; j = data.parents_fromRow[(typename Model::Index)j])
        gravity_partial_dq_.middleRows(jmodel.idx_v(),jmodel.nv()).col(j).noalias() = M6tmpR.topRows(jmodel.nv()) * data.dAdq.col(j);
      
      jmodel.jointVelocitySelector(data.g).noalias() = J_cols.transpose()*data.of[i].toVector();
      if(parent>0)
      {
        data.oYcrb[parent] += data.oYcrb[i];
      }
    }
    
    template<typename Min, typename Mout>
    static void lhsInertiaMult(const typename Data::Inertia & Y,
                               const Eigen::MatrixBase<Min> & J,
                               const Eigen::MatrixBase<Mout> & F)
    {
      Mout & F_ = EIGEN_CONST_CAST(Mout,F);
      motionSet::inertiaAction(Y,J.derived().transpose(),F_.transpose());
    }
  };
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename ReturnMatrixType>
  inline void
  computeGeneralizedGravityDerivatives(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                       DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                       const Eigen::MatrixBase<ConfigVectorType> & q,
                                       const Eigen::MatrixBase<ReturnMatrixType> & gravity_partial_dq)
  {
    assert(q.size() == model.nq && "The configuration vector is not of right size");
    assert(gravity_partial_dq.cols() == model.nv);
    assert(gravity_partial_dq.rows() == model.nv);
    assert(model.check(data) && "data is not consistent with model.");
    
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef typename Model::JointIndex JointIndex;
    
    data.a_gf[0] = -model.gravity;
    
    typedef ComputeGeneralizedGravityDerivativeForwardStep<Scalar,Options,JointCollectionTpl,ConfigVectorType> Pass1;
    for(JointIndex i=1; i<(JointIndex) model.njoints; ++i)
    {
      Pass1::run(model.joints[i],data.joints[i],
                 typename Pass1::ArgsType(model,data,q.derived()));
    }
    
    typedef ComputeGeneralizedGravityDerivativeBackwardStep<Scalar,Options,JointCollectionTpl,ReturnMatrixType> Pass2;
    ReturnMatrixType & gravity_partial_dq_ = EIGEN_CONST_CAST(ReturnMatrixType,gravity_partial_dq);
    for(JointIndex i=(JointIndex)(model.njoints-1); i>0; --i)
    {
      Pass2::run(model.joints[i],
                 typename Pass2::ArgsType(model,data,gravity_partial_dq_));
    }
  }
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2>
  struct ComputeRNEADerivativesForwardStep
  : public fusion::JointVisitorBase< ComputeRNEADerivativesForwardStep<Scalar,Options,JointCollectionTpl,ConfigVectorType,TangentVectorType1,TangentVectorType2> >
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
      typedef typename Data::Motion Motion;

      const JointIndex & i = jmodel.id();
      const JointIndex & parent = model.parents[i];
      Motion & ov = data.ov[i];
      Motion & oa = data.oa[i];
      
      jmodel.calc(jdata.derived(),q.derived(),v.derived());
      
      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      
      data.v[i] = jdata.v();
      
      if(parent > 0)
      {
        data.oMi[i] = data.oMi[parent] * data.liMi[i];
        data.v[i] += data.liMi[i].actInv(data.v[parent]);
      }
      else
        data.oMi[i] = data.liMi[i];
      
      data.a[i] = jdata.S() * jmodel.jointVelocitySelector(a) + jdata.c() + (data.v[i] ^ jdata.v());
      if(parent > 0)
      {
        data.a[i] += data.liMi[i].actInv(data.a[parent]);
      }
      
      data.oYcrb[i] = data.oMi[i].act(model.inertias[i]);
      ov = data.oMi[i].act(data.v[i]);
      oa = data.oMi[i].act(data.a[i]) + data.oa[0]; // add gravity contribution
      
      data.oh[i] = data.oYcrb[i] * ov;
      data.of[i] = data.oYcrb[i] * oa + ov.cross(data.oh[i]);
      
      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<typename Data::Matrix6x>::Type ColsBlock;
      ColsBlock J_cols = jmodel.jointCols(data.J);
      ColsBlock dJ_cols = jmodel.jointCols(data.dJ);
      ColsBlock dVdq_cols = jmodel.jointCols(data.dVdq);
      ColsBlock dAdq_cols = jmodel.jointCols(data.dAdq);
      ColsBlock dAdv_cols = jmodel.jointCols(data.dAdv);

      J_cols = data.oMi[i].act(jdata.S());
      motionSet::motionAction(ov,J_cols,dJ_cols);
      motionSet::motionAction(data.oa[parent],J_cols,dAdq_cols);

      if(parent > 0)
      {
        motionSet::motionAction<ADDTO>(data.ov[parent],dJ_cols,dAdq_cols);
        dVdq_cols = dJ_cols;
        dAdv_cols.noalias() = (Scalar)2*dJ_cols;
      }
      else
      {
        dVdq_cols.setZero();
        dAdv_cols = dJ_cols;
      }

      // computes variation of inertias
      data.doYcrb[i] = data.oYcrb[i].variation(ov);
      
      addForceCrossMatrix(data.oh[i],data.doYcrb[i]);
    }
    
    template<typename ForceDerived, typename M6>
    static void addForceCrossMatrix(const ForceDense<ForceDerived> & f,
                                    const Eigen::MatrixBase<M6> & mout)
    {
      M6 & mout_ = EIGEN_CONST_CAST(M6,mout);
      addSkew(-f.linear(),mout_.template block<3,3>(ForceDerived::LINEAR,ForceDerived::ANGULAR));
      addSkew(-f.linear(),mout_.template block<3,3>(ForceDerived::ANGULAR,ForceDerived::LINEAR));
      addSkew(-f.angular(),mout_.template block<3,3>(ForceDerived::ANGULAR,ForceDerived::ANGULAR));
    }
    
  };
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename MatrixType1, typename MatrixType2, typename MatrixType3>
  struct ComputeRNEADerivativesBackwardStep
  : public fusion::JointVisitorBase<ComputeRNEADerivativesBackwardStep<Scalar,Options,JointCollectionTpl,MatrixType1,MatrixType2,MatrixType3> >
  {
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    
    typedef boost::fusion::vector<const Model &,
                                  Data &,
                                  const MatrixType1 &,
                                  const MatrixType2 &,
                                  const MatrixType3 &
                                  > ArgsType;
    
    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
                     const Model & model,
                     Data & data,
                     const Eigen::MatrixBase<MatrixType1> & rnea_partial_dq,
                     const Eigen::MatrixBase<MatrixType2> & rnea_partial_dv,
                     const Eigen::MatrixBase<MatrixType3> & rnea_partial_da)
    {
      typedef typename Model::JointIndex JointIndex;
      
      const JointIndex & i = jmodel.id();
      const JointIndex & parent = model.parents[i];
      typename Data::RowMatrix6 & M6tmpR = data.M6tmpR;
      typename Data::RowMatrix6 & M6tmpR2 = data.M6tmpR2;

      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<typename Data::Matrix6x>::Type ColsBlock;
      
      ColsBlock J_cols = jmodel.jointCols(data.J);
      ColsBlock dVdq_cols = jmodel.jointCols(data.dVdq);
      ColsBlock dAdq_cols = jmodel.jointCols(data.dAdq);
      ColsBlock dAdv_cols = jmodel.jointCols(data.dAdv);
      ColsBlock dFdq_cols = jmodel.jointCols(data.dFdq);
      ColsBlock dFdv_cols = jmodel.jointCols(data.dFdv);
      ColsBlock dFda_cols = jmodel.jointCols(data.dFda);
      
      MatrixType1 & rnea_partial_dq_ = EIGEN_CONST_CAST(MatrixType1,rnea_partial_dq);
      MatrixType2 & rnea_partial_dv_ = EIGEN_CONST_CAST(MatrixType2,rnea_partial_dv);
      MatrixType3 & rnea_partial_da_ = EIGEN_CONST_CAST(MatrixType3,rnea_partial_da);
      
      // tau
      jmodel.jointVelocitySelector(data.tau).noalias() = J_cols.transpose()*data.of[i].toVector();
      
      // dtau/da similar to data.M
      motionSet::inertiaAction(data.oYcrb[i],J_cols,dFda_cols);
      rnea_partial_da_.block(jmodel.idx_v(),jmodel.idx_v(),jmodel.nv(),data.nvSubtree[i]).noalias()
      = J_cols.transpose()*data.dFda.middleCols(jmodel.idx_v(),data.nvSubtree[i]);
      
      // dtau/dv
      dFdv_cols.noalias() = data.doYcrb[i] * J_cols;
      motionSet::inertiaAction<ADDTO>(data.oYcrb[i],dAdv_cols,dFdv_cols);

      rnea_partial_dv_.block(jmodel.idx_v(),jmodel.idx_v(),jmodel.nv(),data.nvSubtree[i]).noalias()
      = J_cols.transpose()*data.dFdv.middleCols(jmodel.idx_v(),data.nvSubtree[i]);
      
      // dtau/dq
      if(parent>0)
      {
        dFdq_cols.noalias() = data.doYcrb[i] * dVdq_cols;
        motionSet::inertiaAction<ADDTO>(data.oYcrb[i],dAdq_cols,dFdq_cols);
      }
      else
        motionSet::inertiaAction(data.oYcrb[i],dAdq_cols,dFdq_cols);

      rnea_partial_dq_.block(jmodel.idx_v(),jmodel.idx_v(),jmodel.nv(),data.nvSubtree[i]).noalias()
      = J_cols.transpose()*data.dFdq.middleCols(jmodel.idx_v(),data.nvSubtree[i]);
      
      motionSet::act<ADDTO>(J_cols,data.of[i],dFdq_cols);
      
      if(parent > 0)
      {
        lhsInertiaMult(data.oYcrb[i],J_cols.transpose(),M6tmpR.topRows(jmodel.nv()));
        M6tmpR2.topRows(jmodel.nv()).noalias() = J_cols.transpose() * data.doYcrb[i];
        for(int j = data.parents_fromRow[(typename Model::Index)jmodel.idx_v()];j >= 0; j = data.parents_fromRow[(typename Model::Index)j])
        {
          rnea_partial_dq_.middleRows(jmodel.idx_v(),jmodel.nv()).col(j).noalias()
          = M6tmpR.topRows(jmodel.nv()) * data.dAdq.col(j)
          + M6tmpR2.topRows(jmodel.nv()) * data.dVdq.col(j);
        }
        for(int j = data.parents_fromRow[(typename Model::Index)jmodel.idx_v()];j >= 0; j = data.parents_fromRow[(typename Model::Index)j])
        {
          rnea_partial_dv_.middleRows(jmodel.idx_v(),jmodel.nv()).col(j).noalias()
          = M6tmpR.topRows(jmodel.nv()) * data.dAdv.col(j)
          + M6tmpR2.topRows(jmodel.nv()) * data.J.col(j);
        }
      }
      
      if(parent>0)
      {
        data.oYcrb[parent] += data.oYcrb[i];
        data.doYcrb[parent] += data.doYcrb[i];
        data.of[parent] += data.of[i];
      }
      
      // Restore the status of dAdq_cols (remove gravity)
      assert(model.gravity.angular().isZero() && "The gravity must be a pure force vector, no angular part");
      for(Eigen::Index k =0; k < jmodel.nv(); ++k)
      {
        MotionRef<typename ColsBlock::ColXpr> min(J_cols.col(k));
        MotionRef<typename ColsBlock::ColXpr> mout(dAdq_cols.col(k));
        mout.linear() += model.gravity.linear().cross(min.angular());
      }
    }
    
    template<typename Min, typename Mout>
    static void lhsInertiaMult(const typename Data::Inertia & Y,
                               const Eigen::MatrixBase<Min> & J,
                               const Eigen::MatrixBase<Mout> & F)
    {
      Mout & F_ = EIGEN_CONST_CAST(Mout,F);
      motionSet::inertiaAction(Y,J.derived().transpose(),F_.transpose());
    }
  };
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2,
  typename MatrixType1, typename MatrixType2, typename MatrixType3>
  inline void
  computeRNEADerivatives(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                         DataTpl<Scalar,Options,JointCollectionTpl> & data,
                         const Eigen::MatrixBase<ConfigVectorType> & q,
                         const Eigen::MatrixBase<TangentVectorType1> & v,
                         const Eigen::MatrixBase<TangentVectorType2> & a,
                         const Eigen::MatrixBase<MatrixType1> & rnea_partial_dq,
                         const Eigen::MatrixBase<MatrixType2> & rnea_partial_dv,
                         const Eigen::MatrixBase<MatrixType3> & rnea_partial_da)
  {
    assert(q.size() == model.nq && "The joint configuration vector is not of right size");
    assert(v.size() == model.nv && "The joint velocity vector is not of right size");
    assert(a.size() == model.nv && "The joint acceleration vector is not of right size");
    assert(rnea_partial_dq.cols() == model.nv);
    assert(rnea_partial_dq.rows() == model.nv);
    assert(rnea_partial_dv.cols() == model.nv);
    assert(rnea_partial_dv.rows() == model.nv);
    assert(rnea_partial_da.cols() == model.nv);
    assert(rnea_partial_da.rows() == model.nv);
    assert(model.check(data) && "data is not consistent with model.");
    
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef typename Model::JointIndex JointIndex;
    
    data.oa[0] = -model.gravity;
    
    typedef ComputeRNEADerivativesForwardStep<Scalar,Options,JointCollectionTpl,ConfigVectorType,TangentVectorType1,TangentVectorType2> Pass1;
    for(JointIndex i=1; i<(JointIndex) model.njoints; ++i)
    {
      Pass1::run(model.joints[i],data.joints[i],
                 typename Pass1::ArgsType(model,data,q.derived(),v.derived(),a.derived()));
    }
    
    typedef ComputeRNEADerivativesBackwardStep<Scalar,Options,JointCollectionTpl,MatrixType1,MatrixType2,MatrixType3> Pass2;
    for(JointIndex i=(JointIndex)(model.njoints-1); i>0; --i)
    {
      Pass2::run(model.joints[i],
                 typename Pass2::ArgsType(model,data,
                                          EIGEN_CONST_CAST(MatrixType1,rnea_partial_dq),
                                          EIGEN_CONST_CAST(MatrixType2,rnea_partial_dv),
                                          EIGEN_CONST_CAST(MatrixType3,rnea_partial_da)));
    }
  }
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2,
  typename MatrixType1, typename MatrixType2, typename MatrixType3>
  inline void
  computeRNEADerivatives(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                         DataTpl<Scalar,Options,JointCollectionTpl> & data,
                         const Eigen::MatrixBase<ConfigVectorType> & q,
                         const Eigen::MatrixBase<TangentVectorType1> & v,
                         const Eigen::MatrixBase<TangentVectorType2> & a,
                         const container::aligned_vector< ForceTpl<Scalar,Options> > fext,
                         const Eigen::MatrixBase<MatrixType1> & rnea_partial_dq,
                         const Eigen::MatrixBase<MatrixType2> & rnea_partial_dv,
                         const Eigen::MatrixBase<MatrixType3> & rnea_partial_da)
  {
    assert(q.size() == model.nq && "The joint configuration vector is not of right size");
    assert(v.size() == model.nv && "The joint velocity vector is not of right size");
    assert(a.size() == model.nv && "The joint acceleration vector is not of right size");
    assert(fext.size() == (size_t)model.njoints && "The size of the external forces is not of right size");
    assert(rnea_partial_dq.cols() == model.nv);
    assert(rnea_partial_dq.rows() == model.nv);
    assert(rnea_partial_dv.cols() == model.nv);
    assert(rnea_partial_dv.rows() == model.nv);
    assert(rnea_partial_da.cols() == model.nv);
    assert(rnea_partial_da.rows() == model.nv);
    assert(model.check(data) && "data is not consistent with model.");
    
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef typename Model::JointIndex JointIndex;
    
    data.oa[0] = -model.gravity;
    
    typedef ComputeRNEADerivativesForwardStep<Scalar,Options,JointCollectionTpl,ConfigVectorType,TangentVectorType1,TangentVectorType2> Pass1;
    for(JointIndex i=1; i<(JointIndex) model.njoints; ++i)
    {
      Pass1::run(model.joints[i],data.joints[i],
                 typename Pass1::ArgsType(model,data,q.derived(),v.derived(),a.derived()));
      data.of[i] -= data.oMi[i].act(fext[i]);
    }
    
    typedef ComputeRNEADerivativesBackwardStep<Scalar,Options,JointCollectionTpl,MatrixType1,MatrixType2,MatrixType3> Pass2;
    for(JointIndex i=(JointIndex)(model.njoints-1); i>0; --i)
    {
      Pass2::run(model.joints[i],
                 typename Pass2::ArgsType(model,data,
                                          EIGEN_CONST_CAST(MatrixType1,rnea_partial_dq),
                                          EIGEN_CONST_CAST(MatrixType2,rnea_partial_dv),
                                          EIGEN_CONST_CAST(MatrixType3,rnea_partial_da)));
    }
  }
  

} // namespace se3

#endif // ifndef __se3_rnea_derivatives_hxx__

