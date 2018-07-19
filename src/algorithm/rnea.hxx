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

#ifndef __se3_rnea_hxx__
#define __se3_rnea_hxx__

/// @cond DEV

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/algorithm/check.hpp"

namespace se3
{
  template<typename JointCollection, typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2>
  struct RneaForwardStep
  : public fusion::JointVisitorBase< RneaForwardStep<JointCollection,ConfigVectorType,TangentVectorType1,TangentVectorType2> >
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
      
      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      
      data.v[i] = jdata.v();
      if(parent>0) data.v[i] += data.liMi[i].actInv(data.v[parent]);

      data.a_gf[i] = jdata.S()*jmodel.jointVelocitySelector(a) + jdata.c() + (data.v[i] ^ jdata.v()) ;
      data.a_gf[i] += data.liMi[i].actInv(data.a_gf[parent]);
      
      data.f[i] = model.inertias[i]*data.a_gf[i] + model.inertias[i].vxiv(data.v[i]); // -f_ext
    }

  };

  template<typename JointCollection>
  struct RneaBackwardStep : public fusion::JointVisitorBase< RneaBackwardStep<JointCollection> >
  {
    typedef ModelTpl<JointCollection> Model;
    typedef DataTpl<JointCollection> Data;
    
    typedef boost::fusion::vector<const Model &,
                                  Data &
                                  > ArgsType;
    
    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
                     JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data)
    {
      typedef typename Model::JointIndex JointIndex;
      
      const JointIndex & i = jmodel.id();
      const JointIndex & parent  = model.parents[i];
      
      jmodel.jointVelocitySelector(data.tau)  = jdata.S().transpose()*data.f[i];
      if(parent>0) data.f[parent] += data.liMi[i].act(data.f[i]);
    }
  };

  template<typename JointCollection, typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2>
  inline const typename DataTpl<JointCollection>::TangentVectorType &
  rnea(const ModelTpl<JointCollection> & model,
       DataTpl<JointCollection> & data,
       const Eigen::MatrixBase<ConfigVectorType> & q,
       const Eigen::MatrixBase<TangentVectorType1> & v,
       const Eigen::MatrixBase<TangentVectorType2> & a)
  {
    assert(model.check(data) && "data is not consistent with model.");
    assert(q.size() == model.nq && "The configuration vector is not of right size");
    assert(v.size() == model.nv && "The velocity vector is not of right size");
    assert(a.size() == model.nv && "The acceleration vector is not of right size");
    
    typedef typename Model::JointIndex JointIndex;
    
    data.v[0].setZero();
    data.a_gf[0] = -model.gravity;

    typedef RneaForwardStep<JointCollection,ConfigVectorType,TangentVectorType1,TangentVectorType2> Pass1;
    for(JointIndex i=1; i<(JointIndex)model.njoints; ++i)
    {
      Pass1::run(model.joints[i],data.joints[i],
                 typename Pass1::ArgsType(model,data,q.derived(),v.derived(),a.derived()));
    }
    
    typedef RneaBackwardStep<JointCollection> Pass2;
    for(JointIndex i=(JointIndex)model.njoints-1; i>0; --i)
    {
      Pass2::run(model.joints[i],data.joints[i],
                 typename Pass2::ArgsType(model,data));
    }

    return data.tau;
  }
  
  template<typename JointCollection, typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2, typename ForceDerived>
  inline const typename DataTpl<JointCollection>::TangentVectorType &
  rnea(const ModelTpl<JointCollection> & model,
       DataTpl<JointCollection> & data,
       const Eigen::MatrixBase<ConfigVectorType> & q,
       const Eigen::MatrixBase<TangentVectorType1> & v,
       const Eigen::MatrixBase<TangentVectorType2> & a,
       const container::aligned_vector<ForceDerived> & fext)
  {
    assert(fext.size() == model.joints.size());
    assert(model.check(data) && "data is not consistent with model.");
    assert(q.size() == model.nq && "The configuration vector is not of right size");
    assert(v.size() == model.nv && "The velocity vector is not of right size");
    assert(a.size() == model.nv && "The acceleration vector is not of right size");
    
    typedef typename ModelTpl<JointCollection>::JointIndex JointIndex;
    
    data.v[0].setZero();
    data.a_gf[0] = -model.gravity;
    
    typedef RneaForwardStep<JointCollection,ConfigVectorType,TangentVectorType1,TangentVectorType2> Pass1;
    for(JointIndex i=1; i<(JointIndex)model.njoints; ++i)
    {
      Pass1::run(model.joints[i],data.joints[i],
                 typename Pass1::ArgsType(model,data,q.derived(),v.derived(),a.derived()));
      data.f[i] -= fext[i];
    }
    
    typedef RneaBackwardStep<JointCollection> Pass2;
    for(JointIndex i=(JointIndex)model.njoints-1; i>0; --i)
    {
      Pass2::run(model.joints[i],data.joints[i],
                 typename Pass2::ArgsType(model,data));
    }

    
    return data.tau;
  }
 
  template<typename JointCollection, typename ConfigVectorType, typename TangentVectorType>
  struct NLEForwardStep
  : public fusion::JointVisitorBase< NLEForwardStep<JointCollection,ConfigVectorType,TangentVectorType> >
  {
    typedef ModelTpl<JointCollection> Model;
    typedef DataTpl<JointCollection> Data;
    
    typedef boost::fusion::vector<const Model &,
                                  Data &,
                                  const ConfigVectorType &,
                                  const TangentVectorType &
                                  > ArgsType;
    
    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data,
                     const Eigen::MatrixBase<ConfigVectorType> & q,
                     const Eigen::MatrixBase<TangentVectorType> & v)
    {
      typedef typename Model::JointIndex JointIndex;
      
      const JointIndex & i = jmodel.id();
      const JointIndex & parent = model.parents[i];
      
      jmodel.calc(jdata.derived(),q,v);
      
      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      
      data.v[i] = jdata.v();
      if(parent>0) data.v[i] += data.liMi[i].actInv(data.v[parent]);
      
      data.a_gf[i]  = jdata.c() + (data.v[i] ^ jdata.v());
      data.a_gf[i] += data.liMi[i].actInv(data.a_gf[parent]);
      
      data.f[i] = model.inertias[i]*data.a_gf[i] + model.inertias[i].vxiv(data.v[i]); // -f_ext
    }
    
  };
  
  template<typename JointCollection>
  struct NLEBackwardStep
  : public fusion::JointVisitorBase< NLEBackwardStep<JointCollection> >
  {
    typedef ModelTpl<JointCollection> Model;
    typedef DataTpl<JointCollection> Data;
    
    typedef boost::fusion::vector<const Model &,
                                  Data &
                                  >  ArgsType;
    
    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
                     JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data)
    {
      typedef typename Model::JointIndex JointIndex;
      
      const JointIndex & i = jmodel.id();
      const JointIndex & parent  = model.parents[i];
      
      jmodel.jointVelocitySelector(data.nle) = jdata.S().transpose()*data.f[i];
      if(parent>0) data.f[parent] += data.liMi[i].act(data.f[i]);
    }
  };
  
  template<typename JointCollection, typename ConfigVectorType, typename TangentVectorType>
  inline const typename DataTpl<JointCollection>::TangentVectorType &
  nonLinearEffects(const ModelTpl<JointCollection> & model,
                   DataTpl<JointCollection> & data,
                   const Eigen::MatrixBase<ConfigVectorType> & q,
                   const Eigen::MatrixBase<TangentVectorType> & v)
  {
    assert(model.check(data) && "data is not consistent with model.");
    assert(q.size() == model.nq && "The configuration vector is not of right size");
    assert(v.size() == model.nv && "The velocity vector is not of right size");
    
    typedef typename ModelTpl<JointCollection>::JointIndex JointIndex;
    
    data.v[0].setZero ();
    data.a_gf[0] = -model.gravity;
    
    typedef NLEForwardStep<JointCollection,ConfigVectorType,TangentVectorType> Pass1;
    for(JointIndex i=1; i<(JointIndex)model.njoints; ++i)
    {
      Pass1::run(model.joints[i],data.joints[i],
                 typename Pass1::ArgsType(model,data,q.derived(),v.derived()));
    }
    
    typedef NLEBackwardStep<JointCollection> Pass2;
    for(JointIndex i=(JointIndex)(model.njoints-1); i>0; --i)
    {
      Pass2::run(model.joints[i],data.joints[i],
                 typename Pass2::ArgsType(model,data));
    }
    
    return data.nle;
  }
  
  template<typename JointCollection, typename ConfigVectorType>
  struct ComputeGeneralizedGravityForwardStep
  : public fusion::JointVisitorBase< ComputeGeneralizedGravityForwardStep<JointCollection,ConfigVectorType> >
  {
    typedef ModelTpl<JointCollection> Model;
    typedef DataTpl<JointCollection> Data;
    
    typedef boost::fusion::vector<const Model &,
                                  Data &,
                                  const ConfigVectorType &
                                  > ArgsType;
    
    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data,
                     const Eigen::MatrixBase<ConfigVectorType> & q)
    {
      typedef typename Model::JointIndex JointIndex;
      
      const JointIndex & i = jmodel.id();
      const JointIndex & parent = model.parents[i];
      
      jmodel.calc(jdata.derived(),q);
      
      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      
      data.a_gf[i] = data.liMi[i].actInv(data.a_gf[(size_t) parent]);
      data.f[i] = model.inertias[i]*data.a_gf[i];
    }
    
  };
  
  template<typename JointCollection>
  struct ComputeGeneralizedGravityBackwardStep
  : public fusion::JointVisitorBase< ComputeGeneralizedGravityBackwardStep<JointCollection> >
  {
    typedef boost::fusion::vector<const Model &,
                                  Data &
                                  > ArgsType;
    
    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
                     JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data)
    {
      typedef typename Model::JointIndex JointIndex;
      
      const JointIndex & i = jmodel.id();
      const JointIndex & parent = model.parents[i];
      
      jmodel.jointVelocitySelector(data.g) = jdata.S().transpose()*data.f[i];
      if(parent>0) data.f[(size_t) parent] += data.liMi[i].act(data.f[i]);
    }
  };
  
  template<typename JointCollection, typename ConfigVectorType>
  inline const typename DataTpl<JointCollection>::TangentVectorType &
  computeGeneralizedGravity(const ModelTpl<JointCollection> & model,
                            DataTpl<JointCollection> & data,
                            const Eigen::MatrixBase<ConfigVectorType> & q)
  {
    assert(model.check(data) && "data is not consistent with model.");
    assert(q.size() == model.nq && "The configuration vector is not of right size");
    
    typedef typename ModelTpl<JointCollection>::JointIndex JointIndex;
    
    data.a_gf[0] = -model.gravity;
    
    typedef ComputeGeneralizedGravityForwardStep<JointCollection,ConfigVectorType> Pass1;
    for(JointIndex i=1; i<(JointIndex)model.njoints; ++i)
    {
      Pass1::run(model.joints[i],data.joints[i],
                 typename Pass1::ArgsType(model,data,q.derived()));
    }
    
    typedef ComputeGeneralizedGravityBackwardStep<JointCollection> Pass2;
    for(JointIndex i=(JointIndex)(model.njoints-1); i>0; --i)
    {
      Pass2::run(model.joints[i],data.joints[i],
                 typename Pass2::ArgsType(model,data));
    }
    
    return data.g;
  }
  
  template<typename JointCollection, typename ConfigVectorType, typename TangentVectorType>
  struct CoriolisMatrixForwardStep
  : public fusion::JointVisitorBase< CoriolisMatrixForwardStep<JointCollection,ConfigVectorType,TangentVectorType> >
  {
    typedef ModelTpl<JointCollection> Model;
    typedef DataTpl<JointCollection> Data;
    
    typedef boost::fusion::vector<const Model &,
                                  Data &,
                                  const ConfigVectorType &,
                                  const TangentVectorType &
                                  > ArgsType;

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data,
                     const Eigen::MatrixBase<ConfigVectorType> & q,
                     const Eigen::MatrixBase<TangentVectorType> & v)
    {
      typedef typename Model::JointIndex JointIndex;
      
      const JointIndex & i = jmodel.id();
      const JointIndex & parent = model.parents[i];

      jmodel.calc(jdata.derived(),q,v);

      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      if(parent>0) data.oMi[i] = data.oMi[parent] * data.liMi[i];
      else data.oMi[i] = data.liMi[i];
      
      // express quantities in the world frame
      data.oYcrb[i] = data.oMi[i].act(model.inertias[i]);
      
      data.v[i] = jdata.v();
      if(parent>0) data.v[i] += data.liMi[i].actInv(data.v[parent]);
      data.ov[i] = data.oMi[i].act(data.v[i]);
      
      // computes S expressed at the world frame
      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<typename Data::Matrix6x>::Type ColsBlock;
      ColsBlock Jcols = jmodel.jointCols(data.J);
      Jcols = data.oMi[i].act(jdata.S()); // collection of S expressed at the world frame

      // computes vxS expressed at the world frame
      ColsBlock dJcols = jmodel.jointCols(data.dJ);
      motionSet::motionAction(data.ov[i],Jcols,dJcols);

      // computes vxI
      typedef typename Data::Inertia Inertia;
      Inertia::vxi(data.ov[i],data.oYcrb[i],data.vxI[i]);
    }

  };

  template<typename JointCollection>
  struct CoriolisMatrixBackwardStep
  : public fusion::JointVisitorBase< CoriolisMatrixBackwardStep<JointCollection> >
  {
    typedef ModelTpl<JointCollection> Model;
    typedef DataTpl<JointCollection> Data;
    
    typedef boost::fusion::vector<const Model &,
                                  Data &
                                  > ArgsType;

    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
                     const Model & model,
                     Data & data)
    {
      typedef typename Model::JointIndex JointIndex;
      
      const JointIndex & i = jmodel.id();
      const JointIndex & parent = model.parents[i];
      typename Data::RowMatrix6 & M6tmpR = data.M6tmpR;

      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<typename Data::Matrix6x>::Type ColsBlock;
      ColsBlock dJcols = jmodel.jointCols(data.dJ);
      ColsBlock Jcols = jmodel.jointCols(data.J);
      
      motionSet::inertiaAction(data.oYcrb[i],dJcols,jmodel.jointCols(data.dFdv));
      jmodel.jointCols(data.dFdv) += data.vxI[i] * Jcols;

      data.C.block(jmodel.idx_v(),jmodel.idx_v(),jmodel.nv(),data.nvSubtree[i]).noalias()
      = Jcols.transpose()*data.dFdv.middleCols(jmodel.idx_v(),data.nvSubtree[i]);
      
      lhsInertiaMult(data.oYcrb[i],Jcols.transpose(),M6tmpR.topRows(jmodel.nv()));
      for(int j = data.parents_fromRow[(JointIndex)jmodel.idx_v()];j >= 0; j = data.parents_fromRow[(JointIndex)j])
        data.C.middleRows(jmodel.idx_v(),jmodel.nv()).col(j).noalias() = M6tmpR.topRows(jmodel.nv()) * data.dJ.col(j);

      M6tmpR.topRows(jmodel.nv()).noalias() = Jcols.transpose() * data.vxI[i];
      for(int j = data.parents_fromRow[(JointIndex)jmodel.idx_v()];j >= 0; j = data.parents_fromRow[(JointIndex)j])
        data.C.middleRows(jmodel.idx_v(),jmodel.nv()).col(j) += M6tmpR.topRows(jmodel.nv()) * data.J.col(j);

      if(parent>0)
      {
        data.oYcrb[parent] += data.oYcrb[i];
        data.vxI[parent] += data.vxI[i];
      }
      
    }
    
    template<typename Min, typename Mout>
    static void lhsInertiaMult(const typename Data::Inertia & Y,
                               const Eigen::MatrixBase<Min> & J,
                               const Eigen::MatrixBase<Mout> & F)
    {
      Mout & F_ = const_cast<Mout &>(F.derived());
      motionSet::inertiaAction(Y,J.derived().transpose(),F_.transpose());
    }
  };
  
  template<typename JointCollection, typename ConfigVectorType, typename TangentVectorType>
  inline const typename DataTpl<JointCollection>::MatrixXs &
  computeCoriolisMatrix(const ModelTpl<JointCollection> & model,
                        DataTpl<JointCollection> & data,
                        const Eigen::MatrixBase<ConfigVectorType> & q,
                        const Eigen::MatrixBase<TangentVectorType> & v)
  {
    assert(model.check(data) && "data is not consistent with model.");
    assert(q.size() == model.nq);
    assert(v.size() == model.nv);
    
    typedef typename ModelTpl<JointCollection>::JointIndex JointIndex;
    
    typedef CoriolisMatrixForwardStep<JointCollection,ConfigVectorType,TangentVectorType> Pass1;
    for(JointIndex i=1; i<(JointIndex)model.njoints; ++i)
    {
      Pass1::run(model.joints[i],data.joints[i],
                 typename Pass1::ArgsType(model,data,q.derived(),v.derived()));
    }
    
    typedef CoriolisMatrixBackwardStep<JointCollection> Pass2;
    for(JointIndex i=(JointIndex)(model.njoints-1); i>0; --i)
    {
      Pass2::run(model.joints[i],
                 typename Pass2::ArgsType(model,data));
    }
    
    return data.C;
  }
  
} // namespace se3

/// @endcond

#endif // ifndef __se3_rnea_hxx__
