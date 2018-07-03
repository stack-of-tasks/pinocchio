//
// Copyright (c) 2015-2018 CNRS
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

#ifndef __se3_centroidal_hxx__
#define __se3_centroidal_hxx__

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/spatial/act-on-set.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/check.hpp"

/// @cond DEV

namespace se3
{
  
  struct CcrbaForwardStep : public fusion::JointVisitor<CcrbaForwardStep>
  {
    typedef boost::fusion::vector< const se3::Model &,
    se3::Data &,
    const Eigen::VectorXd &
    > ArgsType;
    
    JOINT_VISITOR_INIT(CcrbaForwardStep);
    
    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const se3::Model & model,
                     se3::Data & data,
                     const Eigen::VectorXd & q)
    {
      const Model::JointIndex & i = (Model::JointIndex) jmodel.id();
      const Model::Index & parent = model.parents[i];
      
      jmodel.calc(jdata.derived(),q);
      
      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      data.Ycrb[i] = model.inertias[i];
      
      if (parent>0) data.oMi[i] = data.oMi[parent]*data.liMi[i];
      else data.oMi[i] = data.liMi[i];
    }
    
  }; // struct CcrbaForwardStep
  
  struct CcrbaBackwardStep : public fusion::JointVisitor<CcrbaBackwardStep>
  {
    typedef boost::fusion::vector< const se3::Model &,
    se3::Data &
    > ArgsType;
    
    JOINT_VISITOR_INIT(CcrbaBackwardStep);
    
    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const se3::Model & model,
                     se3::Data & data)
    {
      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Data::Matrix6x>::Type ColsBlock;
      
      const Model::JointIndex & i = (Model::JointIndex) jmodel.id();
      const Model::Index & parent = model.parents[i];
      
      data.Ycrb[parent] += data.liMi[i].act(data.Ycrb[i]);
      
      jdata.U() = data.Ycrb[i] * jdata.S();
      
      ColsBlock jF
      = data.Ag.middleCols <JointModel::NV> (jmodel.idx_v());
      //        = data.Ag.middleCols(jmodel.idx_v(), jmodel.nv());
      
      forceSet::se3Action(data.oMi[i],jdata.U(),jF);
    }
    
  }; // struct CcrbaBackwardStep
  
  inline const Data::Matrix6x &
  ccrba(const Model & model, Data & data,
        const Eigen::VectorXd & q,
        const Eigen::VectorXd & v)
  {
    assert(model.check(data) && "data is not consistent with model.");
    typedef Eigen::Block<Data::Matrix6x,3,-1> Block3x;
    
    forwardKinematics(model, data, q);
    data.Ycrb[0].setZero();
    for(Model::Index i=1;i<(Model::Index)(model.njoints);++i )
      data.Ycrb[i] = model.inertias[i];
    
    
    for(Model::Index i=(Model::Index)(model.njoints-1);i>0;--i)
    {
      CcrbaBackwardStep::run(model.joints[i],data.joints[i],
                             CcrbaBackwardStep::ArgsType(model,data));
    }
    
    // Express the centroidal map around the center of mass
    data.com[0] = data.Ycrb[0].lever();
    
    const Block3x Ag_lin = data.Ag.middleRows<3>(Force::LINEAR);
    Block3x Ag_ang = data.Ag.middleRows<3>(Force::ANGULAR);
    for (long i = 0; i<model.nv; ++i)
      Ag_ang.col(i) += Ag_lin.col(i).cross(data.com[0]);
    
    data.hg.toVector().noalias() = data.Ag*v;
    
    data.Ig.mass() = data.Ycrb[0].mass();
    data.Ig.lever().setZero();
    data.Ig.inertia() = data.Ycrb[0].inertia();
    
    return data.Ag;
  }
  
  struct DCcrbaForwardStep : public fusion::JointVisitor<DCcrbaForwardStep>
  {
    typedef boost::fusion::vector< const se3::Model &,
    se3::Data &,
    const Eigen::VectorXd &,
    const Eigen::VectorXd &
    > ArgsType;
    
    JOINT_VISITOR_INIT(DCcrbaForwardStep);
    
    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const se3::Model & model,
                     se3::Data & data,
                     const Eigen::VectorXd & q,
                     const Eigen::VectorXd & v)
    {
      const Model::JointIndex & i = (Model::JointIndex) jmodel.id();
      const Model::Index & parent = model.parents[i];
      
      jmodel.calc(jdata.derived(),q,v);
      
      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      data.Ycrb[i] = model.inertias[i];
      
      data.v[i] = jdata.v();
      
      if (parent>0)
      {
        data.oMi[i] = data.oMi[parent]*data.liMi[i];
        data.v[i] += data.liMi[i].actInv(data.v[parent]);
      }
      else data.oMi[i] = data.liMi[i];
    }
    
  }; // struct DCcrbaForwardStep
  
  struct DCcrbaBackwardStep : public fusion::JointVisitor<DCcrbaBackwardStep>
  {
    typedef boost::fusion::vector< const se3::Model &,
    se3::Data &
    > ArgsType;
    
    JOINT_VISITOR_INIT(DCcrbaBackwardStep);
    
    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const se3::Model & model,
                     se3::Data & data)
    {
      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Data::Matrix6x>::Type ColsBlock;
      
      const Model::JointIndex & i = (Model::JointIndex) jmodel.id();
      const Model::Index & parent = model.parents[i];
      const Inertia & Y = data.oYcrb[i];
      const Inertia::Matrix6 & doYcrb = data.doYcrb[i];
      
      ColsBlock J_cols = jmodel.jointCols(data.J);
      J_cols = data.oMi[i].act(jdata.S());
      
      ColsBlock dJ_cols = jmodel.jointCols(data.dJ);
      motionSet::motionAction(data.ov[i],J_cols,dJ_cols);
      
      data.oYcrb[parent] += Y;
      if(parent > 0)
        data.doYcrb[parent] += doYcrb;
      
      // Calc Ag
      ColsBlock Ag_cols = jmodel.jointCols(data.Ag);
      motionSet::inertiaAction(Y,J_cols,Ag_cols);
      
      // Calc dAg = Ivx + vxI
      ColsBlock dAg_cols = jmodel.jointCols(data.dAg);
      motionSet::inertiaAction(Y,dJ_cols,dAg_cols);
      dAg_cols += doYcrb * J_cols;
    }
    
  }; // struct DCcrbaBackwardStep
  
  inline const Data::Matrix6x &
  dccrba(const Model & model, Data & data,
         const Eigen::VectorXd & q,
         const Eigen::VectorXd & v)
  {
    assert(model.check(data) && "data is not consistent with model.");
    typedef Eigen::Block <Data::Matrix6x,3,-1> Block3x;
    
    forwardKinematics(model,data,q,v);
    data.oYcrb[0].setZero();
    for(Model::Index i=1;i<(Model::Index)(model.njoints);++i)
    {
      data.oYcrb[i] = data.oMi[i].act(model.inertias[i]);
      data.ov[i] = data.oMi[i].act(data.v[i]); // v_i expressed in the world frame
      data.doYcrb[i] = data.oYcrb[i].variation(data.ov[i]);
    }
    
    for(Model::Index i=(Model::Index)(model.njoints-1);i>0;--i)
    {
      DCcrbaBackwardStep::run(model.joints[i],data.joints[i],
                              DCcrbaBackwardStep::ArgsType(model,data));
    }
    
    // Express the centroidal map around the center of mass
    data.com[0] = data.oYcrb[0].lever();
    
    const Block3x Ag_lin = data.Ag.middleRows<3> (Force::LINEAR);
    Block3x Ag_ang = data.Ag.middleRows<3>  (Force::ANGULAR);
    for (long i = 0; i<model.nv; ++i)
      Ag_ang.col(i) += Ag_lin.col(i).cross(data.com[0]);
    
    data.hg.toVector().noalias() = data.Ag*v;
    data.vcom[0].noalias() = data.hg.linear()/data.oYcrb[0].mass();
    
    const Block3x dAg_lin = data.dAg.middleRows<3>(Force::LINEAR);
    Block3x dAg_ang = data.dAg.middleRows<3>(Force::ANGULAR);
    for (long i = 0; i<model.nv; ++i)
      dAg_ang.col(i) += dAg_lin.col(i).cross(data.com[0]);
    
    data.Ig.mass() = data.oYcrb[0].mass();
    data.Ig.lever().setZero();
    data.Ig.inertia() = data.oYcrb[0].inertia();
    
    return data.dAg;
  }
  
  // --- CHECKER ---------------------------------------------------------------
  // --- CHECKER ---------------------------------------------------------------
  // --- CHECKER ---------------------------------------------------------------
  
  
} // namespace se3

/// @endcond

#endif // ifndef __se3_centroidal_hxx__

