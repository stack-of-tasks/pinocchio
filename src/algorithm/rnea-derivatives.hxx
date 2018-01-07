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

#ifndef __se3_rnea_derivatives_hxx__
#define __se3_rnea_derivatives_hxx__

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/algorithm/check.hpp"

namespace se3
{
  
  struct computeGeneralizedGravityDerivativeForwardStep : public fusion::JointVisitor<computeGeneralizedGravityDerivativeForwardStep>
  {
    typedef boost::fusion::vector< const se3::Model &,
    se3::Data &,
    const Eigen::VectorXd &
    > ArgsType;
    
    JOINT_VISITOR_INIT(computeGeneralizedGravityDerivativeForwardStep);
    
    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const se3::Model & model,
                     se3::Data & data,
                     const Eigen::VectorXd & q)
    {
      const Model::JointIndex & i = jmodel.id();
      const Model::JointIndex & parent = model.parents[i];
      Motion & oa = data.a_gf[0];
      
      jmodel.calc(jdata.derived(),q);
      
      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      
      if(parent > 0)
        data.oMi[i] = data.oMi[parent] * data.liMi[i];
      else
        data.oMi[i] = data.liMi[i];
      
      data.oYo[i] = data.oMi[i].act(model.inertias[i]);
      data.f[i].setZero();
      
      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Data::Matrix6x>::Type ColsBlock;
      ColsBlock J_cols = jmodel.jointCols(data.J);
      ColsBlock axS_cols = jmodel.jointCols(data.axS);
      J_cols = data.oMi[i].act(jdata.S());
      motionSet::motionAction(oa,J_cols,axS_cols);

    }
    
  };
  
  struct computeGeneralizedGravityDerivativeBackwardStep : public fusion::JointVisitor<computeGeneralizedGravityDerivativeBackwardStep>
  {
    typedef boost::fusion::vector<const Model &,
    Data &,
    Eigen::MatrixXd &
    >  ArgsType;
    
    JOINT_VISITOR_INIT(computeGeneralizedGravityDerivativeBackwardStep);
    
    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
                     JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data,
                     Eigen::MatrixXd & gravity_partial_dq)
    {
      const Model::JointIndex & i = jmodel.id();
      const Model::JointIndex & parent = model.parents[i];
      Data::Matrix6R & M6tmpR = data.M6tmpR;
      Motion & oa = data.a_gf[0];

      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Data::Matrix6x>::Type ColsBlock;

      ColsBlock J_cols = jmodel.jointCols(data.J);
      ColsBlock axS_cols = jmodel.jointCols(data.axS);
      ColsBlock dFdq_cols = jmodel.jointCols(data.dFdq);
      
      motionSet::inertiaAction(data.oYo[i],axS_cols,dFdq_cols);
      
      gravity_partial_dq.block(jmodel.idx_v(),jmodel.idx_v(),jmodel.nv(),data.nvSubtree[i]).noalias()
      = J_cols.transpose()*data.dFdq.middleCols(jmodel.idx_v(),data.nvSubtree[i]);
      
      data.f[i] = data.oYo[i] * oa;
      motionSet::act<ADDTO>(J_cols,data.f[i],dFdq_cols);
      
      lhsInertiaMult(data.oYo[i],J_cols.transpose(),M6tmpR.topRows(jmodel.nv()));
      for(int j = data.parents_fromRow[(Model::Index)jmodel.idx_v()];j >= 0; j = data.parents_fromRow[(Model::Index)j])
        gravity_partial_dq.middleRows(jmodel.idx_v(),jmodel.nv()).col(j).noalias() = M6tmpR.topRows(jmodel.nv()) * data.axS.col(j);
      
      jmodel.jointVelocitySelector(data.g).noalias() = J_cols.transpose()*data.f[i].toVector();
      if(parent>0)
      {
        data.oYo[parent] += data.oYo[i];
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
  
  inline void
  computeGeneralizedGravityDerivatives(const Model & model, Data & data,
                                       const Eigen::VectorXd & q,
                                       Eigen::MatrixXd & gravity_partial_dq)
  {
    assert(q.size() == model.nq && "The configuration vector is not of right size");
    assert(gravity_partial_dq.cols() == model.nv);
    assert(gravity_partial_dq.rows() == model.nv);
    assert(model.check(data) && "data is not consistent with model.");
    
    data.a_gf[0] = -model.gravity;
    
    for(Model::JointIndex i=1; i<(Model::JointIndex) model.njoints; ++i)
    {
      computeGeneralizedGravityDerivativeForwardStep::run(model.joints[i],data.joints[i],
                                                          computeGeneralizedGravityDerivativeForwardStep::ArgsType(model,data,q));
    }
    
    for(size_t i=(size_t) (model.njoints-1);i>0;--i)
    {
      computeGeneralizedGravityDerivativeBackwardStep::run(model.joints[i],data.joints[i],
                                                           computeGeneralizedGravityDerivativeBackwardStep::ArgsType(model,data,gravity_partial_dq));
    }
  }
  

} // namespace se3

#endif // ifndef __se3_rnea_derivatives_hxx__

