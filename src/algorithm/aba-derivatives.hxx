//
// Copyright (c) 2018 CNRS
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

#ifndef __se3_aba_derivatives_hxx__
#define __se3_aba_derivatives_hxx__

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/algorithm/check.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hxx"

namespace se3
{
  struct computeABADerivativesForwardStep1 : public fusion::JointVisitor<computeABADerivativesForwardStep1>
  {
    typedef boost::fusion::vector<
    const se3::Model &,
    se3::Data &,
    const Eigen::VectorXd &,
    const Eigen::VectorXd &
    > ArgsType;
    
    JOINT_VISITOR_INIT(computeABADerivativesForwardStep1);
    
    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const se3::Model & model,
                     se3::Data & data,
                     const Eigen::VectorXd & q,
                     const Eigen::VectorXd & v)
    {
      const Model::JointIndex & i = jmodel.id();
      const Model::JointIndex & parent = model.parents[i];
      Motion & ov = data.ov[i];

      jmodel.calc(jdata.derived(),q,v);
      
      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      data.v[i] = jdata.v();

      if(parent > 0)
      {
        data.oMi[i] = data.oMi[parent] * data.liMi[i];
        data.v[i] += data.liMi[i].actInv(data.v[parent]);
      }
      else
        data.oMi[i] = data.liMi[i];
      
      ov = data.oMi[i].act(data.v[i]);
      data.a[i] = jdata.c() + (data.v[i] ^ jdata.v());
      data.Yaba[i] = model.inertias[i].matrix();
      data.oYcrb[i] = data.oMi[i].act(model.inertias[i]);
      
      data.oh[i] = data.oYcrb[i] * ov;
      data.of[i] = ov.cross(data.oh[i]);
      data.f[i] = data.oMi[i].actInv(data.of[i]);

      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Data::Matrix6x>::Type ColsBlock;
      ColsBlock J_cols = jmodel.jointCols(data.J);
      J_cols = data.oMi[i].act(jdata.S());

    }
    
  };
  
  struct computeABADerivativesBackwardStep1 : public fusion::JointVisitor<computeABADerivativesBackwardStep1>
  {
    typedef boost::fusion::vector<const Model &,
    Data &,
    Data::RowMatrixXd &
    >  ArgsType;
    
    JOINT_VISITOR_INIT(computeABADerivativesBackwardStep1);
    
    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data,
                     Data::RowMatrixXd & Minv)
    {
      const Model::JointIndex & i = jmodel.id();
      const Model::JointIndex & parent = model.parents[i];

      Inertia::Matrix6 & Ia = data.Yaba[i];
      jmodel.calc_aba(jdata.derived(), Ia, parent > 0);
      
      Data::Matrix6x & Fcrb = data.Fcrb[0];
      Data::Matrix6x & FcrbTmp = data.Fcrb.back();

      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Data::Matrix6x>::Type ColsBlock;

      ColsBlock U_cols = jmodel.jointCols(data.IS);
      forceSet::se3Action(data.oMi[i],jdata.U(),U_cols); // expressed in the world frame
      
      Minv.block(jmodel.idx_v(),jmodel.idx_v(),jmodel.nv(),jmodel.nv()) = jdata.Dinv();
      const int nv_children = data.nvSubtree[i] - jmodel.nv();
      if(nv_children > 0)
      {
        ColsBlock J_cols = jmodel.jointCols(data.J);
        ColsBlock SDinv_cols = jmodel.jointCols(data.SDinv);
        SDinv_cols.noalias() = J_cols * jdata.Dinv();
        
        Minv.block(jmodel.idx_v(),jmodel.idx_v()+jmodel.nv(),jmodel.nv(),nv_children).noalias()
        = -SDinv_cols.transpose() * Fcrb.middleCols(jmodel.idx_v()+jmodel.nv(),nv_children);
        
        if(parent > 0)
        {
          FcrbTmp.leftCols(data.nvSubtree[i]).noalias()
          = U_cols * Minv.block(jmodel.idx_v(),jmodel.idx_v(),jmodel.nv(),data.nvSubtree[i]);
          Fcrb.middleCols(jmodel.idx_v(),data.nvSubtree[i]) += FcrbTmp.leftCols(data.nvSubtree[i]);
        }
      }
      else // This a leaf of the kinematic tree
      {
        Fcrb.middleCols(jmodel.idx_v(),data.nvSubtree[i]).noalias()
        = U_cols * Minv.block(jmodel.idx_v(),jmodel.idx_v(),jmodel.nv(),data.nvSubtree[i]);
      }
      
      jmodel.jointVelocitySelector(data.u) -= jdata.S().transpose()*data.f[i];

      if (parent > 0)
      {
        Force & pa = data.f[i];
        pa.toVector() += Ia * data.a[i].toVector() + jdata.UDinv() * jmodel.jointVelocitySelector(data.u);
        data.Yaba[parent] += AbaBackwardStep::SE3actOn(data.liMi[i], Ia);
        data.f[parent] += data.liMi[i].act(pa);
      }

    }
    
  };
  
  struct computeABADerivativesForwardStep2 : public fusion::JointVisitor<computeABADerivativesForwardStep2>
  {
    typedef boost::fusion::vector<const se3::Model &,
    se3::Data &,
    Data::RowMatrixXd &
    > ArgsType;
    
    JOINT_VISITOR_INIT(computeABADerivativesForwardStep2);
    
    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const se3::Model & model,
                     se3::Data & data,
                     Data::RowMatrixXd & Minv)
    {
      const Model::JointIndex & i = jmodel.id();
      const Model::Index & parent = model.parents[i];
      Motion & ov = data.ov[i];
      Motion & oa = data.oa[i];
      Force & of = data.of[i];
      
      data.a[i] += data.liMi[i].actInv(data.a[parent]);
      jmodel.jointVelocitySelector(data.ddq).noalias() =
      jdata.Dinv() * jmodel.jointVelocitySelector(data.u) - jdata.UDinv().transpose() * data.a[i].toVector();
      
      data.a[i] += jdata.S() * jmodel.jointVelocitySelector(data.ddq);
      oa = data.oMi[i].act(data.a[i]);
      of = data.oYcrb[i] * oa + ov.cross(data.oh[i]);

      Data::Matrix6x & FcrbTmp = data.Fcrb.back();
      
      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Data::Matrix6x>::Type ColsBlock;
      ColsBlock UDinv_cols = jmodel.jointCols(data.UDinv);
      forceSet::se3Action(data.oMi[i],jdata.UDinv(),UDinv_cols); // expressed in the world frame

      if(parent > 0)
      {
        FcrbTmp.topRows(jmodel.nv()).rightCols(model.nv - jmodel.idx_v()).noalias()
        = UDinv_cols.transpose() * data.Fcrb[parent].rightCols(model.nv - jmodel.idx_v());
        Minv.middleRows(jmodel.idx_v(),jmodel.nv()).rightCols(model.nv - jmodel.idx_v())
        -= FcrbTmp.topRows(jmodel.nv()).rightCols(model.nv - jmodel.idx_v());
      }
      
      ColsBlock J_cols = jmodel.jointCols(data.J);
      data.Fcrb[i].rightCols(model.nv - jmodel.idx_v()).noalias() = J_cols * Minv.middleRows(jmodel.idx_v(),jmodel.nv()).rightCols(model.nv - jmodel.idx_v());
      if(parent > 0)
        data.Fcrb[i].rightCols(model.nv - jmodel.idx_v()) += data.Fcrb[parent].rightCols(model.nv - jmodel.idx_v());
      
      ColsBlock dJ_cols = jmodel.jointCols(data.dJ);
      ColsBlock dVdq_cols = jmodel.jointCols(data.dVdq);
      ColsBlock dAdq_cols = jmodel.jointCols(data.dAdq);
      ColsBlock dAdv_cols = jmodel.jointCols(data.dAdv);
      
      motionSet::motionAction(ov,J_cols,dJ_cols);
      motionSet::motionAction(data.oa[parent],J_cols,dAdq_cols);
      dAdv_cols = dJ_cols;
      if(parent > 0)
      {
        motionSet::motionAction(data.ov[parent],J_cols,dVdq_cols);
        motionSet::motionAction<ADDTO>(data.ov[parent],dVdq_cols,dAdq_cols);
        dAdv_cols += dVdq_cols;
      }
      else
        dVdq_cols.setZero();
      
      // computes variation of inertias
      data.doYcrb[i] = data.oYcrb[i].variation(ov);
      computeRNEADerivativesForwardStep::addForceCrossMatrix(data.oh[i],data.doYcrb[i]);
    }
    
  };
  
  struct computeABADerivativesBackwardStep2 : public fusion::JointModelVisitor<computeABADerivativesBackwardStep2>
  {
    typedef boost::fusion::vector<const Model &,
    Data &
    >  ArgsType;
    
    JOINT_MODEL_VISITOR_INIT(computeABADerivativesBackwardStep2);
    
    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
                     const Model & model,
                     Data & data)
    {
      const Model::JointIndex & i = jmodel.id();
      const Model::JointIndex & parent = model.parents[i];
      Data::RowMatrix6 & M6tmpR = data.M6tmpR;
      
      Eigen::MatrixXd & rnea_partial_dq = data.dtau_dq;
      Eigen::MatrixXd & rnea_partial_dv = data.dtau_dv;

      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Data::Matrix6x>::Type ColsBlock;
      
      ColsBlock J_cols = jmodel.jointCols(data.J);
      ColsBlock dVdq_cols = jmodel.jointCols(data.dVdq);
      ColsBlock dAdq_cols = jmodel.jointCols(data.dAdq);
      ColsBlock dAdv_cols = jmodel.jointCols(data.dAdv);
      ColsBlock dFdq_cols = jmodel.jointCols(data.dFdq);
      ColsBlock dFdv_cols = jmodel.jointCols(data.dFdv);

      // dtau/dv
      motionSet::inertiaAction(data.oYcrb[i],dAdv_cols,dFdv_cols);
      dFdv_cols += data.doYcrb[i] * J_cols;
      
      rnea_partial_dv.block(jmodel.idx_v(),jmodel.idx_v(),jmodel.nv(),data.nvSubtree[i]).noalias()
      = J_cols.transpose()*data.dFdv.middleCols(jmodel.idx_v(),data.nvSubtree[i]);
      
      // dtau/dq
      motionSet::inertiaAction(data.oYcrb[i],dAdq_cols,dFdq_cols);
      if(parent>0)
        dFdq_cols += data.doYcrb[i] * dVdq_cols;
      
      rnea_partial_dq.block(jmodel.idx_v(),jmodel.idx_v(),jmodel.nv(),data.nvSubtree[i]).noalias()
      = J_cols.transpose()*data.dFdq.middleCols(jmodel.idx_v(),data.nvSubtree[i]);
      
      motionSet::act<ADDTO>(J_cols,data.of[i],dFdq_cols);
      
      if(parent > 0)
      {
        lhsInertiaMult(data.oYcrb[i],J_cols.transpose(),M6tmpR.topRows(jmodel.nv()));
        for(int j = data.parents_fromRow[(Model::Index)jmodel.idx_v()];j >= 0; j = data.parents_fromRow[(Model::Index)j])
          rnea_partial_dq.middleRows(jmodel.idx_v(),jmodel.nv()).col(j).noalias() = M6tmpR.topRows(jmodel.nv()) * data.dAdq.col(j);
        for(int j = data.parents_fromRow[(Model::Index)jmodel.idx_v()];j >= 0; j = data.parents_fromRow[(Model::Index)j])
          rnea_partial_dv.middleRows(jmodel.idx_v(),jmodel.nv()).col(j).noalias() = M6tmpR.topRows(jmodel.nv()) * data.dAdv.col(j);
        
        M6tmpR.topRows(jmodel.nv()).noalias() = J_cols.transpose() * data.doYcrb[i];
        for(int j = data.parents_fromRow[(Model::Index)jmodel.idx_v()];j >= 0; j = data.parents_fromRow[(Model::Index)j])
          rnea_partial_dq.middleRows(jmodel.idx_v(),jmodel.nv()).col(j) += M6tmpR.topRows(jmodel.nv()) * data.dVdq.col(j);
        for(int j = data.parents_fromRow[(Model::Index)jmodel.idx_v()];j >= 0; j = data.parents_fromRow[(Model::Index)j])
          rnea_partial_dv.middleRows(jmodel.idx_v(),jmodel.nv()).col(j) += M6tmpR.topRows(jmodel.nv()) * data.J.col(j);
      }
      
      if(parent>0)
      {
        data.oYcrb[parent] += data.oYcrb[i];
        data.doYcrb[parent] += data.doYcrb[i];
        data.of[parent] += data.of[i];
      }
    }
    
    template<typename Min, typename Mout>
    static void lhsInertiaMult(const Inertia & Y,
                               const Eigen::MatrixBase<Min> & J,
                               const Eigen::MatrixBase<Mout> & F)
    {
      Mout & F_ = const_cast<Mout &>(F.derived());
      motionSet::inertiaAction(Y,J.derived().transpose(),F_.transpose());
    }
  };
  
  inline void computeABADerivatives(const Model & model,
                                    Data & data,
                                    const Eigen::VectorXd & q,
                                    const Eigen::VectorXd & v,
                                    const Eigen::VectorXd & tau,
                                    Eigen::MatrixXd & aba_partial_dq,
                                    Eigen::MatrixXd & aba_partial_dv,
                                    Data::RowMatrixXd & aba_partial_dtau)
  {
    assert(q.size() == model.nq && "The joint configuration vector is not of right size");
    assert(v.size() == model.nv && "The joint velocity vector is not of right size");
    assert(tau.size() == model.nv && "The joint acceleration vector is not of right size");
    assert(aba_partial_dq.cols() == model.nv);
    assert(aba_partial_dq.rows() == model.nv);
    assert(aba_partial_dv.cols() == model.nv);
    assert(aba_partial_dv.rows() == model.nv);
    assert(aba_partial_dtau.cols() == model.nv);
    assert(aba_partial_dtau.rows() == model.nv);
    assert(model.check(data) && "data is not consistent with model.");
    
    data.a[0] = -model.gravity;
    data.oa[0] = -model.gravity;
    data.u = tau;
    
    Data::RowMatrixXd & Minv = aba_partial_dtau;
    
    /// First, compute Minv and a, the joint acceleration vector
    for(Model::JointIndex i=1; i<(Model::JointIndex) model.njoints; ++i)
    {
      computeABADerivativesForwardStep1::run(model.joints[i],data.joints[i],
                                            computeABADerivativesForwardStep1::ArgsType(model,data,q,v));
    }
    
    data.Fcrb[0].setZero();
    for(size_t i=(size_t) (model.njoints-1);i>0;--i)
    {
      computeABADerivativesBackwardStep1::run(model.joints[i],data.joints[i],
                                              computeABADerivativesBackwardStep1::ArgsType(model,data,Minv));
    }
    
    for(Model::JointIndex i=1; i<(Model::JointIndex) model.njoints; ++i)
    {
      computeABADerivativesForwardStep2::run(model.joints[i],data.joints[i],
                                             computeABADerivativesForwardStep2::ArgsType(model,data,Minv));
    }
    
    for(size_t i=(size_t) (model.njoints-1);i>0;--i)
    {
      computeABADerivativesBackwardStep2::run(model.joints[i],
                                              computeABADerivativesBackwardStep2::ArgsType(model,data));
    }
    
    Minv.triangularView<Eigen::StrictlyLower>()
    = Minv.transpose().triangularView<Eigen::StrictlyLower>();
    
    aba_partial_dq.noalias() = -Minv*data.dtau_dq;
    aba_partial_dv.noalias() = -Minv*data.dtau_dv;
    
  }
  
  
} // namespace se3

#endif // ifndef __se3_aba_derivatives_hxx__

