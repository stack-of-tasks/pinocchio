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

#ifndef __se3_aba_hxx__
#define __se3_aba_hxx__

#include "pinocchio/spatial/act-on-set.hpp"
#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/algorithm/check.hpp"

/// @cond DEV

namespace se3
{
  template<typename JointCollection, typename ConfigVectorType, typename TangentVectorType>
  struct AbaForwardStep1
  : public fusion::JointVisitorBase< AbaForwardStep1<JointCollection,ConfigVectorType,TangentVectorType> >
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
      jmodel.calc(jdata.derived(),q,v);
      
      const JointIndex & parent = model.parents[i];
      data.liMi[i] = model.jointPlacements[i] * jdata.M();
      
      data.v[i] = jdata.v();
      if (parent>0)
        data.v[i] += data.liMi[i].actInv(data.v[parent]);

      data.a[i] = jdata.c() + (data.v[i] ^ jdata.v());
      
      data.Yaba[i] = model.inertias[i].matrix();
      data.f[i] = model.inertias[i].vxiv(data.v[i]); // -f_ext
    }
    
  };
  
  template<typename JointCollection>
  struct AbaBackwardStep
  : public fusion::JointVisitorBase< AbaBackwardStep<JointCollection> >
  {
    typedef ModelTpl<JointCollection> Model;
    typedef DataTpl<JointCollection> Data;
    
    typedef boost::fusion::vector<const Model &,
                                  Data &> ArgsType;
    
    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
                     JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data)
    {
      typedef typename Model::JointIndex JointIndex;
      typedef typename Data::Inertia Inertia;
      typedef typename Data::Force Force;
      
      const JointIndex & i = jmodel.id();
      const JointIndex & parent  = model.parents[i];
      typename Inertia::Matrix6 & Ia = data.Yaba[i];
      
      jmodel.jointVelocitySelector(data.u) -= jdata.S().transpose()*data.f[i];
      jmodel.calc_aba(jdata.derived(), Ia, parent > 0);

      if (parent > 0)
      {
        Force & pa = data.f[i];
        pa.toVector() += Ia * data.a[i].toVector() + jdata.UDinv() * jmodel.jointVelocitySelector(data.u);
        data.Yaba[parent] += SE3actOn(data.liMi[i], Ia);
        data.f[parent] += data.liMi[i].act(pa);
      }
    }
    
    template<typename Scalar, int Options, typename Matrix6Type>
    inline static typename EIGEN_PLAIN_TYPE(Matrix6Type)
    SE3actOn(const SE3Tpl<Scalar,Options> & M,
             const Eigen::MatrixBase<Matrix6Type> & I)
    {
      typedef SE3Tpl<Scalar,Options> SE3;
      typedef typename SE3::Matrix3 Matrix3;
      typedef typename SE3::Vector3 Vector3;
      
      typedef const Eigen::Block<Matrix6Type,3,3> constBlock3;
      
      typedef typename EIGEN_PLAIN_TYPE(Matrix6Type) ReturnType;
      typedef Eigen::Block<ReturnType,3,3> Block3;
      
      Matrix6Type & I_ = const_cast<Matrix6Type &>(I.derived());
      const constBlock3 & Ai = I_.template block<3,3>(Inertia::LINEAR, Inertia::LINEAR);
      const constBlock3 & Bi = I_.template block<3,3>(Inertia::LINEAR, Inertia::ANGULAR);
      const constBlock3 & Di = I_.template block<3,3>(Inertia::ANGULAR, Inertia::ANGULAR);
      
      const Matrix3 & R = M.rotation();
      const Vector3 & t = M.translation();
      
      ReturnType res;
      Block3 Ao = res.template block<3,3>(Inertia::LINEAR, Inertia::LINEAR);
      Block3 Bo = res.template block<3,3>(Inertia::LINEAR, Inertia::ANGULAR);
      Block3 Co = res.template block<3,3>(Inertia::ANGULAR, Inertia::LINEAR);
      Block3 Do = res.template block<3,3>(Inertia::ANGULAR, Inertia::ANGULAR);
      
      Do.noalias() = R*Ai; // tmp variable
      Ao.noalias() = Do*R.transpose();
      
      Do.noalias() = R*Bi; // tmp variable
      Bo.noalias() = Do*R.transpose();
      
      Co.noalias() = R*Di; // tmp variable
      Do.noalias() = Co*R.transpose();

      Do.row(0) += t.cross(Bo.col(0));
      Do.row(1) += t.cross(Bo.col(1));
      Do.row(2) += t.cross(Bo.col(2));
      
      Co.col(0) = t.cross(Ao.col(0));
      Co.col(1) = t.cross(Ao.col(1));
      Co.col(2) = t.cross(Ao.col(2));
      Co += Bo.transpose();
      
      Bo = Co.transpose();
      Do.col(0) += t.cross(Bo.col(0));
      Do.col(1) += t.cross(Bo.col(1));
      Do.col(2) += t.cross(Bo.col(2));
      
      return res;
    }
  };
  
  template<typename JointCollection>
  struct AbaForwardStep2
  : public fusion::JointVisitorBase< AbaForwardStep2<JointCollection> >
  {
    typedef ModelTpl<JointCollection> Model;
    typedef DataTpl<JointCollection> Data;
    
    typedef boost::fusion::vector<const Model &,
                                  Data &> ArgsType;
    
    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data)
    {
      typedef typename Model::JointIndex JointIndex;
      
      const JointIndex & i = jmodel.id();
      const JointIndex & parent = model.parents[i];
      
      data.a[i] += data.liMi[i].actInv(data.a[parent]);
      jmodel.jointVelocitySelector(data.ddq).noalias() =
      jdata.Dinv() * jmodel.jointVelocitySelector(data.u) - jdata.UDinv().transpose() * data.a[i].toVector();
      
      data.a[i] += jdata.S() * jmodel.jointVelocitySelector(data.ddq);
    }
    
  };
  
  template<typename JointCollection, typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2>
  inline const typename DataTpl<JointCollection>::TangentVectorType &
  aba(const ModelTpl<JointCollection> & model,
      DataTpl<JointCollection> & data,
      const Eigen::MatrixBase<ConfigVectorType> & q,
      const Eigen::MatrixBase<TangentVectorType1> & v,
      const Eigen::MatrixBase<TangentVectorType2> & tau)
  {
    assert(model.check(data) && "data is not consistent with model.");
    assert(q.size() == model.nq && "The joint configuration vector is not of right size");
    assert(v.size() == model.nv && "The joint velocity vector is not of right size");
    assert(tau.size() == model.nv && "The joint acceleration vector is not of right size");
    
    typedef typename ModelTpl<JointCollection>::JointIndex JointIndex;
    
    data.v[0].setZero();
    data.a[0] = -model.gravity;
    data.u = tau;
    
    typedef AbaForwardStep1<JointCollection,ConfigVectorType,TangentVectorType1> Pass1;
    for(JointIndex i=1; i<(JointIndex)model.njoints; ++i)
    {
      Pass1::run(model.joints[i],data.joints[i],
                 typename Pass1::ArgsType(model,data,q.derived(),v.derived()));
    }
    
    typedef AbaBackwardStep<JointCollection> Pass2;
    for(JointIndex i=(JointIndex)model.njoints-1;i>0; --i)
    {
      Pass2::run(model.joints[i],data.joints[i],
                 typename Pass2::ArgsType(model,data));
    }
    
    typedef AbaForwardStep2<JointCollection> Pass3;
    for(JointIndex i=1; i<(JointIndex)model.njoints; ++i)
    {
      Pass3::run(model.joints[i],data.joints[i],
                 typename Pass3::ArgsType(model,data));
    }
    
    return data.ddq;
  }

  template<typename JointCollection, typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2, typename ForceDerived>
  inline const typename DataTpl<JointCollection>::TangentVectorType &
  aba(const ModelTpl<JointCollection> & model,
      DataTpl<JointCollection> & data,
      const Eigen::MatrixBase<ConfigVectorType> & q,
      const Eigen::MatrixBase<TangentVectorType1> & v,
      const Eigen::MatrixBase<TangentVectorType2> & tau,
      const container::aligned_vector<ForceDerived> & fext)

  {
    assert(model.check(data) && "data is not consistent with model.");
    assert(q.size() == model.nq && "The joint configuration vector is not of right size");
    assert(v.size() == model.nv && "The joint velocity vector is not of right size");
    assert(tau.size() == model.nv && "The joint acceleration vector is not of right size");
    
    typedef typename ModelTpl<JointCollection>::JointIndex JointIndex;
    
    data.v[0].setZero();
    data.a[0] = -model.gravity;
    data.u = tau;
    
    typedef AbaForwardStep1<JointCollection,ConfigVectorType,TangentVectorType1> Pass1;
    for(JointIndex i=1;i<(JointIndex)model.njoints;++i)
    {
      Pass1::run(model.joints[i],data.joints[i],
                 typename Pass1::ArgsType(model,data,q.derived(),v.derived()));
      data.f[i] -= fext[i];
    }
    
    typedef AbaBackwardStep<JointCollection> Pass2;
    for(JointIndex i=(JointIndex)model.njoints-1;i>0; --i)
    {
      Pass2::run(model.joints[i],data.joints[i],
                 typename Pass2::ArgsType(model,data));
    }
    
    typedef AbaForwardStep2<JointCollection> Pass3;
    for(JointIndex i=1; i<(JointIndex)model.njoints; ++i)
    {
      Pass3::run(model.joints[i],data.joints[i],
                 typename Pass3::ArgsType(model,data));
    }
    
    return data.ddq;
  }
  
  template<typename JointCollection, typename ConfigVectorType>
  struct ComputeMinverseForwardStep1
  : public fusion::JointVisitorBase< ComputeMinverseForwardStep1<JointCollection,ConfigVectorType> >
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
      jmodel.calc(jdata.derived(),q);
      
      const JointIndex & parent = model.parents[i];
      data.liMi[i] = model.jointPlacements[i] * jdata.M();
      
      if (parent>0)
        data.oMi[i] = data.oMi[parent] * data.liMi[i];
      else
        data.oMi[i] = data.liMi[i];
      
      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<typename Data::Matrix6x>::Type ColsBlock;
      ColsBlock J_cols = jmodel.jointCols(data.J);
      J_cols = data.oMi[i].act(jdata.S());
      
      data.Yaba[i] = model.inertias[i].matrix();
    }
    
  };
  
  template<typename JointCollection>
  struct ComputeMinverseBackwardStep
  : public fusion::JointVisitorBase< ComputeMinverseBackwardStep<JointCollection> >
  {
    typedef ModelTpl<JointCollection> Model;
    typedef DataTpl<JointCollection> Data;
    
    typedef boost::fusion::vector<const Model &,
                                  Data &> ArgsType;
    
    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
                     JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data)
    {
      typedef typename Model::JointIndex JointIndex;
      typedef typename Data::Inertia Inertia;
      
      const JointIndex & i = jmodel.id();
      const JointIndex & parent  = model.parents[i];
      
      typename Inertia::Matrix6 & Ia = data.Yaba[i];
      typename Data::RowMatrixXd & Minv = data.Minv;
      typename Data::Matrix6x & Fcrb = data.Fcrb[0];
      typename Data::Matrix6x & FcrbTmp = data.Fcrb.back();

      jmodel.calc_aba(jdata.derived(), Ia, parent > 0);
      
      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<typename Data::Matrix6x>::Type ColsBlock;

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
      else
      {
        Fcrb.middleCols(jmodel.idx_v(),data.nvSubtree[i]).noalias()
        = U_cols * Minv.block(jmodel.idx_v(),jmodel.idx_v(),jmodel.nv(),data.nvSubtree[i]);
      }
      
      if(parent > 0)
        data.Yaba[parent] += AbaBackwardStep<JointCollection>::SE3actOn(data.liMi[i], Ia);
    }
  };
  
  template<typename JointCollection>
  struct ComputeMinverseForwardStep2
  : public fusion::JointVisitorBase< ComputeMinverseForwardStep2<JointCollection> >
  {
    typedef ModelTpl<JointCollection> Model;
    typedef DataTpl<JointCollection> Data;
    
    typedef boost::fusion::vector<const Model &,
                                  Data &> ArgsType;
    
    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data)
    {
      typedef typename Model::JointIndex JointIndex;
      
      const JointIndex & i = jmodel.id();
      const JointIndex & parent = model.parents[i];
      typename Data::RowMatrixXd & Minv = data.Minv;
      typename Data::Matrix6x & FcrbTmp = data.Fcrb.back();
      
      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<typename Data::Matrix6x>::Type ColsBlock;
      ColsBlock UDinv_cols = jmodel.jointCols(data.UDinv);
      forceSet::se3Action(data.oMi[i],jdata.UDinv(),UDinv_cols); // expressed in the world frame
      ColsBlock J_cols = jmodel.jointCols(data.J);

      if(parent > 0)
      {
        FcrbTmp.topRows(jmodel.nv()).rightCols(model.nv - jmodel.idx_v()).noalias()
        = UDinv_cols.transpose() * data.Fcrb[parent].rightCols(model.nv - jmodel.idx_v());
        Minv.middleRows(jmodel.idx_v(),jmodel.nv()).rightCols(model.nv - jmodel.idx_v())
        -= FcrbTmp.topRows(jmodel.nv()).rightCols(model.nv - jmodel.idx_v());
      }
      
      data.Fcrb[i].rightCols(model.nv - jmodel.idx_v()).noalias() = J_cols * Minv.middleRows(jmodel.idx_v(),jmodel.nv()).rightCols(model.nv - jmodel.idx_v());
      if(parent > 0)
        data.Fcrb[i].rightCols(model.nv - jmodel.idx_v()) += data.Fcrb[parent].rightCols(model.nv - jmodel.idx_v());
    }
    
  };

  template<typename JointCollection, typename ConfigVectorType>
  inline const typename DataTpl<JointCollection>::RowMatrixXd &
  computeMinverse(const ModelTpl<JointCollection> & model,
                  DataTpl<JointCollection> & data,
                  const Eigen::MatrixBase<ConfigVectorType> & q)
  {
    assert(model.check(data) && "data is not consistent with model.");
    assert(q.size() == model.nq && "The joint configuration vector is not of right size");
    
    typedef typename ModelTpl<JointCollection>::JointIndex JointIndex;
    
    typedef ComputeMinverseForwardStep1<JointCollection,ConfigVectorType> Pass1;
    for(JointIndex i=1; i<(JointIndex)model.njoints; ++i)
    {
      Pass1::run(model.joints[i],data.joints[i],
                 typename Pass1::ArgsType(model,data,q.derived()));
    }
    
    data.Fcrb[0].setZero();
    typedef ComputeMinverseBackwardStep<JointCollection> Pass2;
    for(JointIndex i=(JointIndex)model.njoints-1; i>0; --i)
    {
      Pass2::run(model.joints[i],data.joints[i],
                 typename Pass2::ArgsType(model,data));
    }

    typedef ComputeMinverseForwardStep2<JointCollection> Pass3;
    for(JointIndex i=1; i<(JointIndex)model.njoints; ++i)
    {
      Pass3::run(model.joints[i],data.joints[i],
                 typename Pass3::ArgsType(model,data));
    }
    
    return data.Minv;
  }


  // --- CHECKER ---------------------------------------------------------------
  // --- CHECKER ---------------------------------------------------------------
  // --- CHECKER ---------------------------------------------------------------

  // Check whether all masses are nonzero and diagonal of inertia is nonzero
  // The second test is overconstraining.
  template<typename JointCollection>
  inline bool ABAChecker::checkModel_impl(const ModelTpl<JointCollection> & model) const
  {
    typedef ModelTpl<JointCollection> Model;
    typedef typename Model::JointIndex JointIndex;
    
    for(JointIndex j=1;j<(JointIndex)model.njoints;j++)
      if(    (model.inertias[j].mass   ()           < 1e-5) 
          || (model.inertias[j].inertia().data()[0] < 1e-5)
          || (model.inertias[j].inertia().data()[3] < 1e-5)
          || (model.inertias[j].inertia().data()[5] < 1e-5) )
        return false;
    return true;
  }

} // namespace se3

/// @endcond

#endif // ifndef __se3_aba_hxx__
