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

#ifndef __se3_kinematics_derivatives_hxx__
#define __se3_kinematics_derivatives_hxx__

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/algorithm/check.hpp"

namespace se3
{
  
  struct ForwardKinematicsDerivativesForwardStep : public fusion::JointVisitor<ForwardKinematicsDerivativesForwardStep>
  {
    typedef boost::fusion::vector<const se3::Model &,
    se3::Data &,
    const Eigen::VectorXd &,
    const Eigen::VectorXd &,
    const Eigen::VectorXd &
    > ArgsType;
    
    JOINT_VISITOR_INIT(ForwardKinematicsDerivativesForwardStep);
    
    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const se3::Model & model,
                     se3::Data & data,
                     const Eigen::VectorXd & q,
                     const Eigen::VectorXd & v,
                     const Eigen::VectorXd & a)
    {
      const Model::JointIndex & i = jmodel.id();
      const Model::JointIndex & parent = model.parents[i];
      SE3 & oMi = data.oMi[i];
      Motion & vi = data.v[i];
      Motion & ai = data.a[i];
      Motion & ov = data.ov[i];
      Motion & oa = data.oa[i];
      
      jmodel.calc(jdata.derived(),q,v);
      
      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      vi = jdata.v();

      if(parent>0)
      {
        oMi = data.oMi[parent]*data.liMi[i];
        vi += data.liMi[i].actInv(data.v[parent]);
      }
      else
        oMi = data.liMi[i];
      
      ai = jdata.S() * jmodel.jointVelocitySelector(a) + jdata.c() + (vi ^ jdata.v());
      if(parent>0)
        ai += data.liMi[i].actInv(data.a[parent]);

      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Data::Matrix6x>::Type ColsBlock;
      ColsBlock dJcols = jmodel.jointCols(data.dJ);
      ColsBlock Jcols = jmodel.jointCols(data.J);
      
      Jcols = oMi.act(jdata.S());
      ov = oMi.act(vi); // Spatial velocity of joint i expressed in the global frame o
      motionSet::motionAction(ov,Jcols,dJcols);
      oa = oMi.act(ai); // Spatial acceleration of joint i expressed in the global frame o
    }
    
  };
  
  inline void
  computeForwardKinematicsDerivatives(const Model & model, Data & data,
                                      const Eigen::VectorXd & q,
                                      const Eigen::VectorXd & v,
                                      const Eigen::VectorXd & a)
  {
    assert(q.size() == model.nq && "The configuration vector is not of right size");
    assert(v.size() == model.nv && "The velocity vector is not of right size");
    assert(a.size() == model.nv && "The acceleration vector is not of right size");
    assert(model.check(data) && "data is not consistent with model.");
    
    data.v[0].setZero();
    data.a[0].setZero();
    
    for(Model::JointIndex i=1; i<(Model::JointIndex) model.njoints; ++i)
    {
      ForwardKinematicsDerivativesForwardStep::run(model.joints[i],data.joints[i],
                                                   ForwardKinematicsDerivativesForwardStep::ArgsType(model,data,q,v,a));
    }
  }
  
  template<ReferenceFrame rf>
  struct JointVelocityDerivativesBackwardStep : public fusion::JointModelVisitor< JointVelocityDerivativesBackwardStep<rf> >
  {
    typedef boost::fusion::vector<const se3::Model &,
    se3::Data &,
    const SE3 &,
    const Motion &,
    Data::Matrix6x &,
    Data::Matrix6x &
    > ArgsType;
    
    JOINT_MODEL_VISITOR_INIT(JointVelocityDerivativesBackwardStep);
    
    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     const se3::Model & model,
                     se3::Data & data,
                     const SE3 & oMlast,
                     const Motion & vlast,
                     Data::Matrix6x & v_partial_dq,
                     Data::Matrix6x & v_partial_dv)
    {
      const Model::JointIndex & i = jmodel.id();
      const Model::JointIndex & parent = model.parents[i];
      Motion & vtmp = data.ov[0]; // Temporary variable

      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Data::Matrix6x>::Type ColsBlock;
      ColsBlock Jcols = jmodel.jointCols(data.J);
      
      // dvec/dv
      ColsBlock v_partial_dv_cols = jmodel.jointCols(v_partial_dv);
      if(rf == WORLD)
        v_partial_dv_cols = Jcols;
      else
        motionSet::se3ActionInverse(oMlast,Jcols,v_partial_dv_cols);

      // dvec/dq
      ColsBlock v_partial_dq_cols = jmodel.jointCols(v_partial_dq);
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
  
  template<ReferenceFrame rf>
  inline void getJointVelocityDerivatives(const Model & model,
                                          Data & data,
                                          const Model::JointIndex jointId,
                                          Data::Matrix6x & v_partial_dq,
                                          Data::Matrix6x & v_partial_dv)
  {
    assert( v_partial_dq.cols() ==  model.nv );
    assert( v_partial_dv.cols() ==  model.nv );
    assert(model.check(data) && "data is not consistent with model.");
    
    const SE3 & oMlast = data.oMi[jointId];
    const Motion & vlast = data.ov[jointId];
    
    for(Model::JointIndex i = jointId; i > 0; i = model.parents[i])
    {
      JointVelocityDerivativesBackwardStep<rf>::run(model.joints[i],
                                                    typename JointVelocityDerivativesBackwardStep<rf>::ArgsType(model,data,
                                                                                                                oMlast,vlast,
                                                                                                                v_partial_dq,
                                                                                                                v_partial_dv));
    }
  
    // Set back ov[0] to a zero value
    data.ov[0].setZero();
  }
  
  template<ReferenceFrame rf>
  struct JointAccelerationDerivativesBackwardStep
  : public fusion::JointModelVisitor< JointAccelerationDerivativesBackwardStep<rf> >
  {
    typedef boost::fusion::vector<const se3::Model &,
    se3::Data &,
    const Model::JointIndex,
    Data::Matrix6x &,
    Data::Matrix6x &,
    Data::Matrix6x &,
    Data::Matrix6x &
    > ArgsType;
    
    JOINT_MODEL_VISITOR_INIT(JointAccelerationDerivativesBackwardStep);
    
    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     const se3::Model & model,
                     se3::Data & data,
                     const Model::JointIndex jointId,
                     Data::Matrix6x & v_partial_dq,
                     Data::Matrix6x & a_partial_dq,
                     Data::Matrix6x & a_partial_dv,
                     Data::Matrix6x & a_partial_da)
    {
      const Model::JointIndex & i = jmodel.id();
      const Model::JointIndex & parent = model.parents[i];
      Motion & vtmp = data.ov[0]; // Temporary variable
      Motion & atmp = data.oa[0]; // Temporary variable
      
      const SE3 & oMlast = data.oMi[jointId];
      const Motion & vlast = data.ov[jointId];
      const Motion & alast = data.oa[jointId];

      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Data::Matrix6x>::Type ColsBlock;
      ColsBlock dJcols = jmodel.jointCols(data.dJ);
      ColsBlock Jcols = jmodel.jointCols(data.J);
      
      ColsBlock v_partial_dq_cols = jmodel.jointCols(v_partial_dq);
      ColsBlock a_partial_dq_cols = jmodel.jointCols(a_partial_dq);
      ColsBlock a_partial_dv_cols = jmodel.jointCols(a_partial_dv);
      ColsBlock a_partial_da_cols = jmodel.jointCols(a_partial_da);
      
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
  
  template<ReferenceFrame rf>
  inline void getJointAccelerationDerivatives(const Model & model,
                                              Data & data,
                                              const Model::JointIndex jointId,
                                              Data::Matrix6x & v_partial_dq,
                                              Data::Matrix6x & a_partial_dq,
                                              Data::Matrix6x & a_partial_dv,
                                              Data::Matrix6x & a_partial_da)
  {
    assert( v_partial_dq.cols() ==  model.nv );
    assert( a_partial_dq.cols() ==  model.nv );
    assert( a_partial_dv.cols() ==  model.nv );
    assert( a_partial_da.cols() ==  model.nv );
    assert(model.check(data) && "data is not consistent with model.");
    
    for(Model::JointIndex i = jointId; i > 0; i = model.parents[i])
    {
      JointAccelerationDerivativesBackwardStep<rf>::run(model.joints[i],
                                                        typename JointAccelerationDerivativesBackwardStep<rf>::ArgsType(model,data,
                                                                                                                        jointId,
                                                                                                                        v_partial_dq,
                                                                                                                        a_partial_dq,
                                                                                                                        a_partial_dv,
                                                                                                                        a_partial_da));
      
    }
    
    // Set Zero to temporary spatial variables
    data.ov[0].setZero();
    data.oa[0].setZero();
  }

} // namespace se3

#endif // ifndef __se3_kinematics_derivatives_hxx__

