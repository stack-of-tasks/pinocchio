//
// Copyright (c) 2015-2017 CNRS
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
  struct RneaForwardStep : public fusion::JointVisitor<RneaForwardStep>
  {
    typedef boost::fusion::vector<const se3::Model &,
			    se3::Data &,
			    const Eigen::VectorXd &,
			    const Eigen::VectorXd &,
			    const Eigen::VectorXd &
			    > ArgsType;

    JOINT_VISITOR_INIT(RneaForwardStep);

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
      
      jmodel.calc(jdata.derived(),q,v);
      
      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      
      data.v[i] = jdata.v();
      if(parent>0) data.v[i] += data.liMi[i].actInv(data.v[parent]);

      data.a_gf[i] = jdata.S()*jmodel.jointVelocitySelector(a) + jdata.c() + (data.v[i] ^ jdata.v()) ;
      data.a_gf[i] += data.liMi[i].actInv(data.a_gf[parent]);
      
      data.f[i] = model.inertias[i]*data.a_gf[i] + model.inertias[i].vxiv(data.v[i]); // -f_ext
    }

  };

  struct RneaBackwardStep : public fusion::JointVisitor<RneaBackwardStep>
  {
    typedef boost::fusion::vector<const Model &,
                                  Data &
                                  > ArgsType;
    
    JOINT_VISITOR_INIT(RneaBackwardStep);

    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
                     JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data)
    {
      const Model::JointIndex & i = jmodel.id();
      const Model::JointIndex & parent  = model.parents[i];
      
      jmodel.jointVelocitySelector(data.tau)  = jdata.S().transpose()*data.f[i];
      if(parent>0) data.f[parent] += data.liMi[i].act(data.f[i]);
    }
  };

  inline const Eigen::VectorXd&
  rnea(const Model & model, Data& data,
       const Eigen::VectorXd & q,
       const Eigen::VectorXd & v,
       const Eigen::VectorXd & a)
  {
    assert(model.check(data) && "data is not consistent with model.");
    
    data.v[0].setZero();
    data.a_gf[0] = -model.gravity;

    for( Model::JointIndex i=1;i<(Model::JointIndex)model.njoints;++i )
    {
      RneaForwardStep::run(model.joints[i],data.joints[i],
                           RneaForwardStep::ArgsType(model,data,q,v,a));
    }
    
    for( Model::JointIndex i=(Model::JointIndex)model.njoints-1;i>0;--i )
    {
      RneaBackwardStep::run(model.joints[i],data.joints[i],
                            RneaBackwardStep::ArgsType(model,data));
    }

    return data.tau;
  }
  
  inline const Eigen::VectorXd &
  rnea(const Model & model, Data & data,
       const Eigen::VectorXd & q,
       const Eigen::VectorXd & v,
       const Eigen::VectorXd & a,
       const container::aligned_vector<Force> & fext)
  {
    assert(fext.size() == model.joints.size());
    assert(model.check(data) && "data is not consistent with model.");
    
    data.v[0].setZero();
    data.a_gf[0] = -model.gravity;
    
    for( Model::JointIndex i=1;i<(Model::JointIndex)model.njoints;++i )
    {
      RneaForwardStep::run(model.joints[i],data.joints[i],
                           RneaForwardStep::ArgsType(model,data,q,v,a));
      data.f[i] -= fext[i];
    }
    
    for( Model::JointIndex i=(Model::JointIndex)model.njoints-1;i>0;--i )
    {
      RneaBackwardStep::run(model.joints[i],data.joints[i],
                            RneaBackwardStep::ArgsType(model,data));
    }
    
    return data.tau;
  }
  
  struct NLEForwardStep : public fusion::JointVisitor<NLEForwardStep>
  {
    typedef boost::fusion::vector< const se3::Model &,
    se3::Data &,
    const Eigen::VectorXd &,
    const Eigen::VectorXd &
    > ArgsType;
    
    JOINT_VISITOR_INIT(NLEForwardStep);
    
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
      
      jmodel.calc(jdata.derived(),q,v);
      
      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      
      data.v[i] = jdata.v();
      if(parent>0) data.v[i] += data.liMi[i].actInv(data.v[parent]);
      
      data.a_gf[i]  = jdata.c() + (data.v[i] ^ jdata.v());
      data.a_gf[i] += data.liMi[i].actInv(data.a_gf[parent]);
      
      data.f[i] = model.inertias[i]*data.a_gf[i] + model.inertias[i].vxiv(data.v[i]); // -f_ext
    }
    
  };
  
  struct NLEBackwardStep : public fusion::JointVisitor<NLEBackwardStep>
  {
    typedef boost::fusion::vector<const Model &,
                                  Data &
                                  >  ArgsType;
    
    JOINT_VISITOR_INIT(NLEBackwardStep);
    
    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
                     JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data)
    {
      const Model::JointIndex & i = jmodel.id();
      const Model::JointIndex & parent  = model.parents[i];
      
      jmodel.jointVelocitySelector(data.nle)  = jdata.S().transpose()*data.f[i];
      if(parent>0) data.f[parent] += data.liMi[i].act(data.f[i]);
    }
  };
  
  inline const Eigen::VectorXd &
  nonLinearEffects(const Model & model, Data & data,
                   const Eigen::VectorXd & q,
                   const Eigen::VectorXd & v)
  {
    assert(model.check(data) && "data is not consistent with model.");
    
    data.v[0].setZero ();
    data.a_gf[0] = -model.gravity;
    
    for( size_t i=1;i<(size_t) model.njoints;++i )
    {
      NLEForwardStep::run(model.joints[i],data.joints[i],
                          NLEForwardStep::ArgsType(model,data,q,v));
    }
    
    for( size_t i=(size_t) (model.njoints-1);i>0;--i )
    {
      NLEBackwardStep::run(model.joints[i],data.joints[i],
                           NLEBackwardStep::ArgsType(model,data));
    }
    
    return data.nle;
  }
  
  struct computeGeneralizedGravityForwardStep : public fusion::JointVisitor<computeGeneralizedGravityForwardStep>
  {
    typedef boost::fusion::vector< const se3::Model &,
    se3::Data &,
    const Eigen::VectorXd &
    > ArgsType;
    
    JOINT_VISITOR_INIT(computeGeneralizedGravityForwardStep);
    
    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const se3::Model & model,
                     se3::Data & data,
                     const Eigen::VectorXd & q)
    {
      const Model::JointIndex & i = jmodel.id();
      const Model::JointIndex & parent = model.parents[i];
      
      jmodel.calc(jdata.derived(),q);
      
      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      
      data.a_gf[i] = data.liMi[i].actInv(data.a_gf[(size_t) parent]);
      data.f[i] = model.inertias[i]*data.a_gf[i];
    }
    
  };
  
  struct computeGeneralizedGravityBackwardStep : public fusion::JointVisitor<computeGeneralizedGravityBackwardStep>
  {
    typedef boost::fusion::vector<const Model &,
    Data &
    >  ArgsType;
    
    JOINT_VISITOR_INIT(computeGeneralizedGravityBackwardStep);
    
    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
                     JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data)
    {
      const Model::JointIndex & i = jmodel.id();
      const Model::JointIndex & parent = model.parents[i];
      
      jmodel.jointVelocitySelector(data.g) = jdata.S().transpose()*data.f[i];
      if(parent>0) data.f[(size_t) parent] += data.liMi[i].act(data.f[i]);
    }
  };
  
  inline const Eigen::VectorXd &
  computeGeneralizedGravity(const Model & model, Data & data,
                            const Eigen::VectorXd & q)
  {
    assert(model.check(data) && "data is not consistent with model.");
    
    data.a_gf[0] = -model.gravity;
    
    for(size_t i=1;i<(size_t) model.njoints;++i)
    {
      computeGeneralizedGravityForwardStep::run(model.joints[i],data.joints[i],
                                     computeGeneralizedGravityForwardStep::ArgsType(model,data,q));
    }
    
    for(size_t i=(size_t) (model.njoints-1);i>0;--i)
    {
      computeGeneralizedGravityBackwardStep::run(model.joints[i],data.joints[i],
                                      computeGeneralizedGravityBackwardStep::ArgsType(model,data));
    }
    
    return data.g;
  }
  
  struct CoriolisMatrixForwardStep : public fusion::JointVisitor<CoriolisMatrixForwardStep>
  {
    typedef boost::fusion::vector<const se3::Model &,
    se3::Data &,
    const Eigen::VectorXd &,
    const Eigen::VectorXd &
    > ArgsType;

    JOINT_VISITOR_INIT(CoriolisMatrixForwardStep);

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
      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Data::Matrix6x>::Type ColsBlock;
      ColsBlock Jcols = jmodel.jointCols(data.J);
      Jcols = data.oMi[i].act(jdata.S()); // collection of S expressed at the world frame

      // computes vxS expressed at the world frame
      ColsBlock dJcols = jmodel.jointCols(data.dJ);
      motionSet::motionAction(data.ov[i],Jcols,dJcols);

      // computes vxI
      Inertia::vxi(data.ov[i],data.oYcrb[i],data.vxI[i]);
    }

  };

  struct CoriolisMatrixBackwardStep : public fusion::JointModelVisitor<CoriolisMatrixBackwardStep>
  {
    typedef boost::fusion::vector<const Model &,
    Data &
    > ArgsType;

    JOINT_MODEL_VISITOR_INIT(CoriolisMatrixBackwardStep);

    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
                     const Model & model,
                     Data & data)
    {
      const Model::JointIndex & i = jmodel.id();
      const Model::JointIndex & parent = model.parents[i];
      Data::RowMatrix6 & M6tmpR = data.M6tmpR;

      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Data::Matrix6x>::Type ColsBlock;
      ColsBlock dJcols = jmodel.jointCols(data.dJ);
      ColsBlock Jcols = jmodel.jointCols(data.J);
      
      motionSet::inertiaAction(data.oYcrb[i],dJcols,jmodel.jointCols(data.dFdv));
      jmodel.jointCols(data.dFdv) += data.vxI[i] * Jcols;

      data.C.block(jmodel.idx_v(),jmodel.idx_v(),jmodel.nv(),data.nvSubtree[i]).noalias()
      = Jcols.transpose()*data.dFdv.middleCols(jmodel.idx_v(),data.nvSubtree[i]);
      
      lhsInertiaMult(data.oYcrb[i],Jcols.transpose(),M6tmpR.topRows(jmodel.nv()));
      for(int j = data.parents_fromRow[(Model::Index)jmodel.idx_v()];j >= 0; j = data.parents_fromRow[(Model::Index)j])
        data.C.middleRows(jmodel.idx_v(),jmodel.nv()).col(j).noalias() = M6tmpR.topRows(jmodel.nv()) * data.dJ.col(j);

      M6tmpR.topRows(jmodel.nv()).noalias() = Jcols.transpose() * data.vxI[i];
      for(int j = data.parents_fromRow[(Model::Index)jmodel.idx_v()];j >= 0; j = data.parents_fromRow[(Model::Index)j])
        data.C.middleRows(jmodel.idx_v(),jmodel.nv()).col(j) += M6tmpR.topRows(jmodel.nv()) * data.J.col(j);

      if(parent>0)
      {
        data.oYcrb[parent] += data.oYcrb[i];
        data.vxI[parent] += data.vxI[i];
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
  
  inline const Eigen::MatrixXd &
  computeCoriolisMatrix(const Model & model, Data & data,
                        const Eigen::VectorXd & q,
                        const Eigen::VectorXd & v)
  {
    assert(model.check(data) && "data is not consistent with model.");
    assert(q.size() == model.nq);
    assert(v.size() == model.nv);
    
    for(size_t i=1;i<(size_t) model.njoints;++i)
    {
      CoriolisMatrixForwardStep::run(model.joints[i],data.joints[i],
                                     CoriolisMatrixForwardStep::ArgsType(model,data,q,v));
    }
    
    for(size_t i=(size_t) (model.njoints-1);i>0;--i)
    {
      CoriolisMatrixBackwardStep::run(model.joints[i],  
                                      CoriolisMatrixBackwardStep::ArgsType(model,data));
    }
    
    return data.C;
  }
  
} // namespace se3

/// @endcond

#endif // ifndef __se3_rnea_hxx__
