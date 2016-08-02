//
// Copyright (c) 2015-2016 CNRS
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

#ifndef __se3_crba_hxx__
#define __se3_crba_hxx__

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/spatial/act-on-set.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

/// @cond DEV

namespace se3 
{
  struct CrbaForwardStep : public fusion::JointVisitor<CrbaForwardStep>
  {
    typedef boost::fusion::vector<const se3::Model&,
                                  se3::Data &,
                                  const Eigen::VectorXd &
                                  > ArgsType;

    JOINT_VISITOR_INIT(CrbaForwardStep);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
		     se3::JointDataBase<typename JointModel::JointDataDerived> & jdata,
		     const se3::Model & model,
		     se3::Data & data,
		     const Eigen::VectorXd & q)
    {
      const Model::JointIndex & i = (Model::JointIndex) jmodel.id();
      jmodel.calc(jdata.derived(),q);
      
      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      data.Ycrb[i] = model.inertias[i];
    }

  };

  struct CrbaBackwardStep : public fusion::JointVisitor<CrbaBackwardStep>
  {
    typedef boost::fusion::vector<const Model&,
				  Data&>  ArgsType;
    
    JOINT_VISITOR_INIT(CrbaBackwardStep);

    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
                     JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data)
    {
      /*
       * F[1:6,i] = Y*S
       * M[i,SUBTREE] = S'*F[1:6,SUBTREE]
       * if li>0 
       *   Yli += liXi Yi
       *   F[1:6,SUBTREE] = liXi F[1:6,SUBTREE]
       */
      typedef Data::Matrix6x::ColsBlockXpr Block;
      const Model::JointIndex & i = (Model::JointIndex) jmodel.id();

      /* F[1:6,i] = Y*S */
      //data.Fcrb[i].block<6,JointModel::NV>(0,jmodel.idx_v()) = data.Ycrb[i] * jdata.S();
      jmodel.jointCols(data.Fcrb[i]) = data.Ycrb[i] * jdata.S();

      /* M[i,SUBTREE] = S'*F[1:6,SUBTREE] */
      data.M.block(jmodel.idx_v(),jmodel.idx_v(),jmodel.nv(),data.nvSubtree[i]) 
      = jdata.S().transpose()*data.Fcrb[i].middleCols(jmodel.idx_v(),data.nvSubtree[i]);

      const Model::JointIndex & parent   = model.parents[i];
      if(parent>0)
      {
        /*   Yli += liXi Yi */
        data.Ycrb[parent] += data.liMi[i].act(data.Ycrb[i]);
        
        /*   F[1:6,SUBTREE] = liXi F[1:6,SUBTREE] */
        Block jF
        = data.Fcrb[parent].middleCols(jmodel.idx_v(),data.nvSubtree[i]);
        Block iF
        = data.Fcrb[i].middleCols(jmodel.idx_v(),data.nvSubtree[i]);
        forceSet::se3Action(data.liMi[i], iF, jF);
      }
    }
  };

  inline const Eigen::MatrixXd&
  crba(const Model & model, Data& data,
       const Eigen::VectorXd & q)
  {
    for( Model::JointIndex i=1;i<(Model::JointIndex)(model.njoint);++i )
    {
      CrbaForwardStep::run(model.joints[i],data.joints[i],
                           CrbaForwardStep::ArgsType(model,data,q));
    }
    
    for( Model::JointIndex i=(Model::JointIndex)(model.njoint-1);i>0;--i )
    {
      CrbaBackwardStep::run(model.joints[i],data.joints[i],
                            CrbaBackwardStep::ArgsType(model,data));
    }

    return data.M;
  }
  
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
      typedef typename Data::Matrix6x::NColsBlockXpr<JointModel::NV>::Type ColsBlock;
      
      const Model::JointIndex & i = (Model::JointIndex) jmodel.id();
      const Model::Index & parent = model.parents[i];
      
      data.Ycrb[parent] += data.liMi[i].act(data.Ycrb[i]);
      
      jdata.U() = data.Ycrb[i] * jdata.S();
      ColsBlock jF
      = data.Ag.middleCols <JointModel::NV> (jmodel.idx_v());
      forceSet::se3Action(data.oMi[i],jdata.U(),jF);
    }
    
  }; // struct CcrbaBackwardStep
  
  inline const Data::Matrix6x &
  ccrba(const Model & model, Data & data,
        const Eigen::VectorXd & q,
        const Eigen::VectorXd & v)
  {
    typedef Eigen::Block <Data::Matrix6x,3,-1> Block3x;
    
    forwardKinematics(model, data, q);
    data.Ycrb[0].setZero();
    for(Model::Index i=1;i<(Model::Index)(model.njoint);++i )
      data.Ycrb[i] = model.inertias[i];
    
    
    for(Model::Index i=(Model::Index)(model.njoint-1);i>0;--i)
    {
      CcrbaBackwardStep::run(model.joints[i],data.joints[i],
                             CcrbaBackwardStep::ArgsType(model,data));
    }
    data.com[0] = data.Ycrb[0].lever();
    
    const Block3x Ag_lin = data.Ag.middleRows<3> (Force::LINEAR);
    Block3x Ag_ang = data.Ag.middleRows<3>  (Force::ANGULAR);
    for (long i = 0; i<model.nv; ++i)
      Ag_ang.col(i) += Ag_lin.col(i).cross(data.com[0]);
    
    data.hg = data.Ag*v;
    
    data.Ig.mass() = data.Ycrb[0].mass();
    data.Ig.lever().setZero();
    data.Ig.inertia() = data.Ycrb[0].inertia();
    
    return data.Ag;
  }
} // namespace se3

/// @endcond

#endif // ifndef __se3_crba_hxx__
