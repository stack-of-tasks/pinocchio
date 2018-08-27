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

#ifndef __se3_jacobian_hxx__
#define __se3_jacobian_hxx__

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/algorithm/check.hpp"

/// @cond DEV

namespace se3
{
  struct JointJacobiansForwardStep : public fusion::JointVisitor<JointJacobiansForwardStep>
  {
    typedef boost::fusion::vector <const se3::Model &,
                                   se3::Data &,
                                   const Eigen::VectorXd &
                                   > ArgsType;
    
    JOINT_VISITOR_INIT(JointJacobiansForwardStep);
    
    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const se3::Model & model,
                     se3::Data & data,
                     const Eigen::VectorXd & q)
    {
      const Model::JointIndex & i = (Model::JointIndex) jmodel.id();
      const Model::JointIndex & parent = model.parents[i];
      
      jmodel.calc(jdata.derived(),q);
      
      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      if(parent>0) data.oMi[i] = data.oMi[parent]*data.liMi[i];
      else         data.oMi[i] = data.liMi[i];
      
      jmodel.jointCols(data.J) = data.oMi[i].act(jdata.S());
    }
  
  };
  
  inline const Data::Matrix6x &
  computeJointJacobians(const Model & model, Data & data,
                        const Eigen::VectorXd & q)
  {
    assert(model.check(data) && "data is not consistent with model.");
    
    for( Model::JointIndex i=1; i< (Model::JointIndex) model.njoints;++i )
    {
      JointJacobiansForwardStep::run(model.joints[i],data.joints[i],
                                JointJacobiansForwardStep::ArgsType(model,data,q));
    }
  
    return data.J;
  }
  
  struct JointJacobiansForwardStep2 : public fusion::JointVisitor<JointJacobiansForwardStep2>
  {
    typedef boost::fusion::vector<se3::Data &> ArgsType;
    
    JOINT_VISITOR_INIT(JointJacobiansForwardStep2);
    
    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     se3::Data & data)
    {
      const Model::JointIndex & i = (Model::JointIndex) jmodel.id();

      jmodel.jointCols(data.J) = data.oMi[i].act(jdata.S());
    }
    
  };
  
  inline const Data::Matrix6x &
  computeJointJacobians(const Model & model, Data & data)
  {
    assert(model.check(data) && "data is not consistent with model.");
    
    for( Model::JointIndex i=1; i< (Model::JointIndex) model.njoints;++i )
    {
      JointJacobiansForwardStep2::run(model.joints[i],data.joints[i],
                                 JointJacobiansForwardStep2::ArgsType(data));
    }
    
    return data.J;
  }
  
  /* Return the jacobian of the output frame attached to joint <jointId> in the
   world frame or in the local frame depending on the template argument. The
   function computeJacobians should have been called first. */
  template<ReferenceFrame rf>
  void getJointJacobian(const Model & model,
                   const Data & data,
                   const Model::JointIndex jointId,
                   Data::Matrix6x & J)
  {
    assert( J.rows() == data.J.rows() );
    assert( J.cols() == data.J.cols() );
    assert(model.check(data) && "data is not consistent with model.");
    
    const SE3 & oMjoint = data.oMi[jointId];
    int colRef = nv(model.joints[jointId])+idx_v(model.joints[jointId])-1;
    for(int j=colRef;j>=0;j=data.parents_fromRow[(Model::Index)j])
    {
      if(rf == WORLD)   J.col(j) = data.J.col(j);
      else              J.col(j) = oMjoint.actInv(Motion(data.J.col(j))).toVector();
    }
  }
  
  
  struct JointJacobianForwardStep : public fusion::JointVisitor<JointJacobianForwardStep>
  {
    typedef boost::fusion::vector<const se3::Model &,
                                  se3::Data &,
                                  const Eigen::VectorXd &,
                                  se3::Data::Matrix6x &
                                  > ArgsType;
    
    JOINT_VISITOR_INIT(JointJacobianForwardStep);
    
    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const se3::Model & model,
                     se3::Data & data,
                     const Eigen::VectorXd & q,
                     se3::Data::Matrix6x & J)
    {
      const Model::JointIndex & i = (Model::JointIndex) jmodel.id();
      const Model::JointIndex & parent = model.parents[i];
      
      jmodel.calc(jdata.derived(),q);
      
      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      data.iMf[parent] = data.liMi[i]*data.iMf[i];
      
      jmodel.jointCols(J) = data.iMf[i].inverse().act(jdata.S());
    }
  
  };
  
  inline void jointJacobian(const Model & model, Data & data,
                            const Eigen::VectorXd & q,
                            const Model::JointIndex jointId,
                            Data::Matrix6x & J)
  {
    assert(model.check(data) && "data is not consistent with model.");
    
    data.iMf[jointId].setIdentity();
    for( Model::JointIndex i=jointId;i>0;i=model.parents[i] )
    {
      JointJacobianForwardStep::run(model.joints[i],data.joints[i],
                               JointJacobianForwardStep::ArgsType(model,data,q,J));
    }
  }
  
  struct JointJacobiansTimeVariationForwardStep : public fusion::JointVisitor<JointJacobiansTimeVariationForwardStep>
  {
    typedef boost::fusion::vector <const se3::Model &,
    se3::Data &,
    const Eigen::VectorXd &,
    const Eigen::VectorXd &
    > ArgsType;
    
    JOINT_VISITOR_INIT(JointJacobiansTimeVariationForwardStep);
    
    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const se3::Model & model,
                     se3::Data & data,
                     const Eigen::VectorXd & q,
                     const Eigen::VectorXd & v)
    {
      const Model::JointIndex & i = (Model::JointIndex) jmodel.id();
      const Model::JointIndex & parent = model.parents[i];
      
      SE3 & oMi = data.oMi[i];
      Motion & vJ = data.v[i];
      
      jmodel.calc(jdata.derived(),q,v);
      
      vJ = jdata.v();
      
      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      if(parent>0)
      {
        oMi = data.oMi[parent]*data.liMi[i];
        vJ += data.liMi[i].actInv(data.v[parent]);
      }
      else
      {
        oMi = data.liMi[i];
      }
      
      jmodel.jointCols(data.J) = oMi.act(jdata.S());
      
      // Spatial velocity of joint i expressed in the global frame o
      data.ov[i] = oMi.act(vJ);
      
      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Data::Matrix6x>::Type ColsBlock;
      ColsBlock dJcols = jmodel.jointCols(data.dJ);
      ColsBlock Jcols = jmodel.jointCols(data.J);
      
      motionSet::motionAction(data.ov[i],Jcols,dJcols);
    }
    
  };
  
  inline const Data::Matrix6x &
  computeJointJacobiansTimeVariation(const Model & model,
                                     Data & data,
                                     const Eigen::VectorXd & q,
                                     const Eigen::VectorXd & v)
  {
    assert(model.check(data) && "data is not consistent with model.");
    
    for(Model::JointIndex i=1; i< (Model::JointIndex) model.njoints;++i)
    {
      JointJacobiansTimeVariationForwardStep::run(model.joints[i],data.joints[i],
                                             JointJacobiansTimeVariationForwardStep::ArgsType(model,data,q,v));
    }
    
    return data.dJ;
  }
  
  template<ReferenceFrame rf>
  void getJointJacobianTimeVariation(const Model & model,
                                     const Data & data,
                                     const Model::JointIndex jointId,
                                     Data::Matrix6x & dJ)
  {
    assert( dJ.rows() == data.dJ.rows() );
    assert( dJ.cols() == data.dJ.cols() );
    assert(model.check(data) && "data is not consistent with model.");
    
    const SE3 & oMjoint = data.oMi[jointId];
    int colRef = nv(model.joints[jointId])+idx_v(model.joints[jointId])-1;
    for(int j=colRef;j>=0;j=data.parents_fromRow[(Model::Index)j])
    {
      if(rf == WORLD)   dJ.col(j) = data.dJ.col(j);
      else              dJ.col(j) = oMjoint.actInv(Motion(data.dJ.col(j))).toVector();
    }
  }
  
  
} // namespace se3

/// @endcond

#endif // ifndef __se3_jacobian_hxx__
