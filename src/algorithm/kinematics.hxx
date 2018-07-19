//
// Copyright (c) 2016-2018 CNRS
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

#ifndef __se3_kinematics_hxx__
#define __se3_kinematics_hxx__

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/algorithm/check.hpp"

namespace se3 
{
  
  template<typename JointCollection>
  struct EmptyForwardStep
  : fusion::JointVisitorBase< EmptyForwardStep<JointCollection> >
  {
    typedef ModelTpl<JointCollection> Model;
    typedef DataTpl<JointCollection> Data;
    
    typedef boost::fusion::vector<const Model &,
                                  Data &
                                  > ArgsType;
    
    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> &,
                     JointDataBase<typename JointModel::JointDataDerived> &,
                     const Model &,
                     Data &)
    { // do nothing
    }
    
  };
 
  template<typename JointCollection>
  inline void emptyForwardPass(const ModelTpl<JointCollection> & model,
                               DataTpl<JointCollection> & data)
  {
    assert(model.check(data) && "data is not consistent with model.");
    
    typedef typename ModelTpl<JointCollection>::JointIndex JointIndex;
    typedef EmptyForwardStep<JointCollection> Algo;
    
    for(JointIndex i=1; i < (JointIndex)model.njoints; ++i)
    {
      Algo::run(model.joints[i],
                data.joints[i],
                typename Algo::ArgsType(model,data)
                );
    }
  }
  
  template<typename JointCollection>
  inline void updateGlobalPlacements(const ModelTpl<JointCollection> & model,
                                     DataTpl<JointCollection> & data)
  {
    assert(model.check(data) && "data is not consistent with model.");
    
    typedef typename ModelTpl<JointCollection>::JointIndex JointIndex;
    
    for(JointIndex i=1; i <(JointIndex) model.njoints; ++i)
    {
      const JointIndex & parent = model.parents[i];
      
      if (parent>0)
        data.oMi[i] = data.oMi[parent] * data.liMi[i];
      else
        data.oMi[i] = data.liMi[i];
    }
  }
  
  
  template<typename JointCollection, typename ConfigVectorType>
  struct ForwardKinematicZeroStep
  : fusion::JointVisitorBase< ForwardKinematicZeroStep<JointCollection,ConfigVectorType> >
  {
    typedef ModelTpl<JointCollection> Model;
    typedef DataTpl<JointCollection> Data;
    
    typedef boost::fusion::vector<const Model &,
                                  Data &,
                                  const ConfigVectorType &> ArgsType;
    
    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
                     JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data,
                     const Eigen::MatrixBase<ConfigVectorType> & q)
    {
      typedef typename Model::JointIndex JointIndex;
      
      const JointIndex & i = jmodel.id();
      const JointIndex & parent = model.parents[i];

      jmodel.calc(jdata.derived(),q);

      data.liMi[i] = model.jointPlacements[i] * jdata.M();

      if (parent>0)
        data.oMi[i] = data.oMi[parent] * data.liMi[i];
      else
        data.oMi[i] = data.liMi[i];
    }
  };

  template<typename JointCollection, typename ConfigVectorType>
  inline void forwardKinematics(const ModelTpl<JointCollection> & model,
                                DataTpl<JointCollection> & data,
                                const Eigen::MatrixBase<ConfigVectorType> & q)
  {
    assert(q.size() == model.nq && "The configuration vector is not of right size");
    assert(model.check(data) && "data is not consistent with model.");
    
    typedef typename ModelTpl<JointCollection>::JointIndex JointIndex;
    
    typedef ForwardKinematicZeroStep<JointCollection,ConfigVectorType> Algo;
    for(JointIndex i=1; i < (JointIndex)model.njoints; ++i)
    {
      Algo::run(model.joints[i], data.joints[i],
                typename Algo::ArgsType(model,data,q.derived()));
    }
  }

  template<typename JointCollection, typename ConfigVectorType, typename TangentVectorType>
  struct ForwardKinematicFirstStep
  : fusion::JointVisitorBase< ForwardKinematicFirstStep<JointCollection,ConfigVectorType,TangentVectorType> >
  {
    typedef ModelTpl<JointCollection> Model;
    typedef DataTpl<JointCollection> Data;
    
    typedef boost::fusion::vector<const Model &,
                                  Data &,
                                  const ConfigVectorType &,
                                  const TangentVectorType &
                                  > ArgsType;

    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
                     JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data,
                     const Eigen::MatrixBase<ConfigVectorType> & q,
                     const Eigen::MatrixBase<TangentVectorType> & v)
    {
      typedef typename ModelTpl<JointCollection>::JointIndex JointIndex;
      
      const JointIndex & i = jmodel.id();
      const JointIndex & parent = model.parents[i];
      
      jmodel.calc(jdata.derived(),q,v);
      
      data.v[i] = jdata.v();
      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      
      if(parent>0)
      {
        data.oMi[i] = data.oMi[parent]*data.liMi[i];
        data.v[i] += data.liMi[i].actInv(data.v[parent]);
      }
      else
        data.oMi[i] = data.liMi[i];
    }

  };

  
  template<typename JointCollection, typename ConfigVectorType, typename TangentVectorType>
  inline void forwardKinematics(const ModelTpl<JointCollection> & model,
                                DataTpl<JointCollection> & data,
                                const Eigen::MatrixBase<ConfigVectorType> & q,
                                const Eigen::MatrixBase<TangentVectorType> & v)
  {
    assert(q.size() == model.nq && "The configuration vector is not of right size");
    assert(v.size() == model.nv && "The velocity vector is not of right size");
    assert(model.check(data) && "data is not consistent with model.");
    
    typedef typename ModelTpl<JointCollection>::JointIndex JointIndex;
    
    data.v[0].setZero();

    typedef ForwardKinematicFirstStep<JointCollection,ConfigVectorType,TangentVectorType> Algo;
    for(JointIndex i=1; i<(JointIndex) model.njoints; ++i)
    {
      Algo::run(model.joints[i],data.joints[i],
                typename Algo::ArgsType(model,data,q.derived(),v.derived()));
    }
  }
  
  template<typename JointCollection, typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2>
  struct ForwardKinematicSecondStep :
  fusion::JointVisitorBase< ForwardKinematicSecondStep<JointCollection,ConfigVectorType,TangentVectorType1,TangentVectorType2> >
  {
    typedef ModelTpl<JointCollection> Model;
    typedef DataTpl<JointCollection> Data;
    
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
      
      const JointIndex & i = jmodel.id();
      const JointIndex & parent = model.parents[i];
      
      jmodel.calc(jdata.derived(),q,v);

      data.v[i] = jdata.v();
      data.liMi[i] = model.jointPlacements[i] * jdata.M();
      
      if(parent>0)
      {
        data.oMi[i] = data.oMi[parent] * data.liMi[i];
        data.v[i] += data.liMi[i].actInv(data.v[parent]);
      }
      else
        data.oMi[i] = data.liMi[i];
      
      data.a[i]  = jdata.S() * jmodel.jointVelocitySelector(a) + jdata.c() + (data.v[i] ^ jdata.v()) ;
      data.a[i] += data.liMi[i].actInv(data.a[parent]);
    }
  };
  
  template<typename JointCollection, typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2>
  inline void forwardKinematics(const ModelTpl<JointCollection> & model,
                                DataTpl<JointCollection> & data,
                                const Eigen::MatrixBase<ConfigVectorType> & q,
                                const Eigen::MatrixBase<TangentVectorType1> & v,
                                const Eigen::MatrixBase<TangentVectorType2> & a)
  {
    assert(q.size() == model.nq && "The configuration vector is not of right size");
    assert(v.size() == model.nv && "The velocity vector is not of right size");
    assert(a.size() == model.nv && "The acceleration vector is not of right size");
    assert(model.check(data) && "data is not consistent with model.");
    
    typedef typename ModelTpl<JointCollection>::JointIndex JointIndex;
    
    data.v[0].setZero();
    data.a[0].setZero();
    
    typedef ForwardKinematicSecondStep<JointCollection,ConfigVectorType,TangentVectorType1,TangentVectorType2> Algo;
    for(JointIndex i=1; i < (JointIndex)model.njoints; ++i)
    {
      Algo::run(model.joints[i],data.joints[i],
                typename Algo::ArgsType(model,data,q.derived(),v.derived(),a.derived()));
    }
  }
} // namespace se3

#endif // ifndef __se3_kinematics_hxx__
