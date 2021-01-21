//
// Copyright (c) 2015-2021 CNRS INRIA
//

#ifndef __pinocchio_compute_all_terms_hxx__
#define __pinocchio_compute_all_terms_hxx__

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/spatial/act-on-set.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/energy.hpp"
#include "pinocchio/algorithm/check.hpp"

namespace pinocchio
{
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType>
  struct CATForwardStep
  : public fusion::JointUnaryVisitorBase< CATForwardStep<Scalar,Options,JointCollectionTpl,ConfigVectorType,TangentVectorType> >
  {
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    
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
      typedef typename Model::JointIndex JointIndex;

      const JointIndex i = jmodel.id();
      const JointIndex parent = model.parents[i];
      
      jmodel.calc(jdata.derived(),q.derived(),v.derived());
      
      // CRBA
      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      data.Ycrb[i] = model.inertias[i];

      // Jacobian + NLE
      data.v[i] = jdata.v();

      if(parent>0)
      {
        data.oMi[i] = data.oMi[parent]*data.liMi[i];
        data.v[i] += data.liMi[i].actInv(data.v[parent]);
      }
      else
        data.oMi[i] = data.liMi[i];

      jmodel.jointCols(data.J) = data.oMi[i].act(jdata.S());

      data.a_gf[i] = data.a[i] = jdata.c() + (data.v[i] ^ jdata.v());
      if (parent > 0)
        data.a[i] += data.liMi[i].actInv(data.a[parent]);
      
      data.a_gf[i] += data.liMi[i].actInv(data.a_gf[parent]);

      data.h[i] = model.inertias[i]*data.v[i];
      data.f[i] = model.inertias[i]*data.a_gf[i] + data.v[i].cross(data.h[i]); // -f_ext
      
    }

  };

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  struct CATBackwardStep
  : public fusion::JointUnaryVisitorBase <CATBackwardStep<Scalar,Options,JointCollectionTpl> >
  {
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    
    typedef boost::fusion::vector<const Model &,
                                  Data &
                                  > ArgsType;

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
      typedef typename Model::JointIndex JointIndex;
      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<typename Data::Matrix6x>::Type ColsBlock;
      
      const JointIndex i = jmodel.id();
      const JointIndex parent = model.parents[i];

      /* F[1:6,i] = Y*S */
      jdata.U() = data.Ycrb[i] * jdata.S();
      
      ColsBlock J_cols = data.J.template middleCols<JointModel::NV>(jmodel.idx_v());
      ColsBlock Ag_cols = data.Ag.template middleCols<JointModel::NV>(jmodel.idx_v());
      forceSet::se3Action(data.oMi[i],jdata.U(),Ag_cols);

      /* M[i,SUBTREE] = S'*F[1:6,SUBTREE] */
      data.M.block(jmodel.idx_v(),jmodel.idx_v(),jmodel.nv(),data.nvSubtree[i])
      = J_cols.transpose()*data.Ag.middleCols(jmodel.idx_v(),data.nvSubtree[i]);

      jmodel.jointVelocitySelector(data.nle) = jdata.S().transpose()*data.f[i];
    
      data.Ycrb[parent] += data.liMi[i].act(data.Ycrb[i]);
      data.h[parent] += data.liMi[i].act(data.h[i]);
      data.f[parent] += data.liMi[i].act(data.f[i]);
      
      // CoM
      data.mass[i] = data.Ycrb[i].mass();
      data.com[i] = data.Ycrb[i].lever();
      data.vcom[i] = data.h[i].linear() / data.mass[i];
    }
  };
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType>
  inline void computeAllTerms(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                              DataTpl<Scalar,Options,JointCollectionTpl> & data,
                              const Eigen::MatrixBase<ConfigVectorType> & q,
                              const Eigen::MatrixBase<TangentVectorType> & v)
  {
    assert(model.check(data) && "data is not consistent with model.");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(q.size(), model.nq, "The configuration vector is not of right size");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(v.size(), model.nv, "The velocity vector is not of right size");

    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    
    data.v[0].setZero();
    data.a[0].setZero();
    data.h[0].setZero();
    data.a_gf[0] = -model.gravity;
    data.Ycrb[0].setZero();

    typedef CATForwardStep<Scalar,Options,JointCollectionTpl,ConfigVectorType,TangentVectorType> Pass1;
    for(JointIndex i=1;i<(JointIndex) model.njoints;++i)
    {
      Pass1::run(model.joints[i],data.joints[i],
                 typename Pass1::ArgsType(model,data,q.derived(),v.derived()));
    }

    typedef CATBackwardStep<Scalar,Options,JointCollectionTpl> Pass2;
    for(JointIndex i=(JointIndex)(model.njoints-1);i>0;--i)
    {
      Pass2::run(model.joints[i],data.joints[i],
                 typename Pass2::ArgsType(model,data));
    }
    
    // CoM
    data.mass[0] = data.Ycrb[0].mass();
    data.com[0] = data.Ycrb[0].lever();
    data.vcom[0] = data.h[0].linear() / data.mass[0];
    
    // JCoM
    typedef Eigen::Block<typename Data::Matrix6x,3,-1> Block3x;
    const Block3x Ag_lin = data.Ag.template middleRows<3>(Force::LINEAR);
    Block3x Ag_ang = data.Ag.template middleRows<3>(Force::ANGULAR);
    for(long i = 0; i<model.nv; ++i)
      Ag_ang.col(i) += Ag_lin.col(i).cross(data.com[0]);
    
    data.hg = data.h[0];
    data.hg.angular() += data.hg.linear().cross(data.com[0]);
    data.Jcom = data.Ag.template topRows<3>()/data.mass[0];
    
    data.Ig.mass() = data.Ycrb[0].mass();
    data.Ig.lever().setZero();
    data.Ig.inertia() = data.Ycrb[0].inertia();
    
    // Energy
    computeKineticEnergy(model, data);
    computePotentialEnergy(model, data);

  }

} // namespace pinocchio

/// \endinternal

#endif // ifndef __pinocchio_compute_all_terms_hxx__
