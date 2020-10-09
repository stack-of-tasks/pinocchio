//
// Copyright (c) 2015-2019 CNRS INRIA
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_multibody_data_hxx__
#define __pinocchio_multibody_data_hxx__

#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/utils/string-generator.hpp"
#include "pinocchio/multibody/liegroup/liegroup-algo.hpp"

#include <boost/bind.hpp>
#include <boost/utility.hpp>

/// @cond DEV

namespace pinocchio
{
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline DataTpl<Scalar,Options,JointCollectionTpl>::
  DataTpl(const Model & model)
  : joints(0)
  , a((std::size_t)model.njoints,Motion::Zero())
  , oa((std::size_t)model.njoints,Motion::Zero())
  , a_gf((std::size_t)model.njoints,Motion::Zero())
  , oa_gf((std::size_t)model.njoints,Motion::Zero())
  , v((std::size_t)model.njoints,Motion::Zero())
  , ov((std::size_t)model.njoints,Motion::Zero())
  , f((std::size_t)model.njoints,Force::Zero())
  , of((std::size_t)model.njoints,Force::Zero())
  , h((std::size_t)model.njoints,Force::Zero())
  , oh((std::size_t)model.njoints,Force::Zero())
  , oMi((std::size_t)model.njoints,SE3::Identity())
  , liMi((std::size_t)model.njoints,SE3::Identity())
  , tau(VectorXs::Zero(model.nv))
  , nle(VectorXs::Zero(model.nv))
  , g(VectorXs::Zero(model.nv))
  , oMf((std::size_t)model.nframes,SE3::Identity())
  , Ycrb((std::size_t)model.njoints,Inertia::Zero())
  , dYcrb((std::size_t)model.njoints,Inertia::Zero())
  , M(MatrixXs::Zero(model.nv,model.nv))
  , Minv(MatrixXs::Zero(model.nv,model.nv))
  , C(MatrixXs::Zero(model.nv,model.nv))
  , dHdq(Matrix6x::Zero(6,model.nv))
  , dFdq(Matrix6x::Zero(6,model.nv))
  , dFdv(Matrix6x::Zero(6,model.nv))
  , dFda(Matrix6x::Zero(6,model.nv))
  , SDinv(Matrix6x::Zero(6,model.nv))
  , UDinv(Matrix6x::Zero(6,model.nv))
  , IS(MatrixXs::Zero(6,model.nv))
  , vxI((std::size_t)model.njoints,Inertia::Matrix6::Zero())
  , Ivx((std::size_t)model.njoints,Inertia::Matrix6::Zero())
  , oinertias((std::size_t)model.njoints,Inertia::Zero())
  , oYcrb((std::size_t)model.njoints,Inertia::Zero())
  , doYcrb((std::size_t)model.njoints,Inertia::Matrix6::Zero())
  , ddq(VectorXs::Zero(model.nv))
  , Yaba((std::size_t)model.njoints,Inertia::Matrix6::Zero())
  , u(VectorXs::Zero(model.nv))
  , Ag(Matrix6x::Zero(6,model.nv))
  , dAg(Matrix6x::Zero(6,model.nv))
  , hg(Force::Zero())
  , dhg(Force::Zero())
  , Ig(Inertia::Zero())
  , Fcrb((std::size_t)model.njoints,Matrix6x::Zero(6,model.nv))
  , lastChild((std::size_t)model.njoints,-1)
  , nvSubtree((std::size_t)model.njoints,-1)
  , start_idx_v_fromRow((std::size_t)model.nv,-1)
  , end_idx_v_fromRow((std::size_t)model.nv,-1)
  , U(MatrixXs::Identity(model.nv,model.nv))
  , D(VectorXs::Zero(model.nv))
  , Dinv(VectorXs::Zero(model.nv))
  , tmp(VectorXs::Zero(model.nv))
  , parents_fromRow((std::size_t)model.nv,-1)
  , supports_fromRow((std::size_t)model.nv)
  , nvSubtree_fromRow((std::size_t)model.nv,-1)
  , J(Matrix6x::Zero(6,model.nv))
  , dJ(Matrix6x::Zero(6,model.nv))
  , dVdq(Matrix6x::Zero(6,model.nv))
  , dAdq(Matrix6x::Zero(6,model.nv))
  , dAdv(Matrix6x::Zero(6,model.nv))
  , dtau_dq(MatrixXs::Zero(model.nv,model.nv))
  , dtau_dv(MatrixXs::Zero(model.nv,model.nv))
  , ddq_dq(MatrixXs::Zero(model.nv,model.nv))
  , ddq_dv(MatrixXs::Zero(model.nv,model.nv))
  , iMf((std::size_t)model.njoints,SE3::Identity())
  , com((std::size_t)model.njoints,Vector3::Zero())
  , vcom((std::size_t)model.njoints,Vector3::Zero())
  , acom((std::size_t)model.njoints,Vector3::Zero())
  , mass((std::size_t)model.njoints,(Scalar)(-1))
  , Jcom(Matrix3x::Zero(3,model.nv))
  , kinetic_energy((Scalar)-1)
  , potential_energy((Scalar)-1)
  , JMinvJt()
  , llt_JMinvJt()
  , lambda_c()
  , sDUiJt(MatrixXs::Zero(model.nv,model.nv))
  , torque_residual(VectorXs::Zero(model.nv))
  , dq_after(VectorXs::Zero(model.nv))
  , impulse_c()
  , staticRegressor(Matrix3x::Zero(3,4*(model.njoints-1)))
  , bodyRegressor(BodyRegressorType::Zero())
  , jointTorqueRegressor(MatrixXs::Zero(model.nv,10*(model.njoints-1)))
#if EIGEN_VERSION_AT_LEAST(3,2,90) && !EIGEN_VERSION_AT_LEAST(3,2,93)
  , kinematic_hessians(6,std::max(1,model.nv),std::max(1,model.nv)) // the minimum size should be 1 for compatibility reasons
#else
  , kinematic_hessians(6,model.nv,model.nv)
#endif
  {
    typedef typename Model::JointIndex JointIndex;
    
    /* Create data structure associated to the joints */
    for(JointIndex i=0;i<(JointIndex)(model.njoints);++i)
      joints.push_back(CreateJointData<Scalar,Options,JointCollectionTpl>::run(model.joints[i]));

    /* Init for CRBA */
    M.setZero(); Minv.setZero();
    for(JointIndex i=0;i<(JointIndex)(model.njoints);++i)
    { Fcrb[i].resize(6,model.nv); }
    
    computeLastChild(model);

    /* Init for Coriolis */

    /* Init for Cholesky */
    computeParents_fromRow(model);
    computeSupports_fromRow(model);
    
    /* Init universe states relatively to itself */
    a_gf[0] = -model.gravity;
    
    kinematic_hessians.setZero();
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline void DataTpl<Scalar,Options,JointCollectionTpl>::
  computeLastChild(const Model & model)
  {
    typedef typename Model::Index Index;
    
    std::fill(lastChild.begin(),lastChild.end(),-1);
    for( int i=model.njoints-1;i>=0;--i )
    {
      if(lastChild[(Index)i] == -1) lastChild[(Index)i] = i;
      const Index & parent = model.parents[(Index)i];
      lastChild[parent] = std::max<int>(lastChild[(Index)i],lastChild[parent]);
      
      nvSubtree[(Index)i]
      = idx_v(model.joints[(Index)lastChild[(Index)i]]) + nv(model.joints[(Index)lastChild[(Index)i]])
      - idx_v(model.joints[(Index)i]);
    }
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline void DataTpl<Scalar,Options,JointCollectionTpl>
  ::computeParents_fromRow(const Model & model)
  {
    typedef typename Model::Index Index;
    
    for(Index joint=1;joint<(Index)(model.njoints);joint++)
    {
      const Index & parent = model.parents[joint];
      const int nvj    = nv   (model.joints[joint]);
      const int idx_vj = idx_v(model.joints[joint]);
      
      assert(idx_vj >= 0 && idx_vj < model.nv);
      
      if(parent>0) parents_fromRow[(size_t)idx_vj] = idx_v(model.joints[parent])+nv(model.joints[parent])-1;
      else         parents_fromRow[(size_t)idx_vj] = -1;
      nvSubtree_fromRow[(size_t)idx_vj] = nvSubtree[joint];
      
      start_idx_v_fromRow[(size_t)idx_vj] = idx_vj;
      end_idx_v_fromRow[(size_t)idx_vj] = idx_vj+nvj-1;
      for(int row=1;row<nvj;++row)
      {
        parents_fromRow[(size_t)(idx_vj+row)] = idx_vj+row-1;
        nvSubtree_fromRow[(size_t)(idx_vj+row)] = nvSubtree[joint]-row;
        start_idx_v_fromRow[(size_t)(idx_vj+row)] = start_idx_v_fromRow[(size_t)idx_vj];
        end_idx_v_fromRow[(size_t)(idx_vj+row)] = end_idx_v_fromRow[(size_t)idx_vj];
      }
    }
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline void DataTpl<Scalar,Options,JointCollectionTpl>
  ::computeSupports_fromRow(const Model & model)
  {
    typedef typename Model::JointIndex JointIndex;
    
    for(JointIndex joint_id = 1;
        joint_id < (JointIndex)(model.njoints);
        joint_id++)
    {
      const int nvj    = nv   (model.joints[joint_id]);
      const int idx_vj = idx_v(model.joints[joint_id]);
      
      assert(idx_vj >= 0 && idx_vj < model.nv);
      
      const int parent_fromRow = parents_fromRow[(size_t)idx_vj];
      
      if(parent_fromRow >= 0)
        supports_fromRow[(size_t)idx_vj] = supports_fromRow[(size_t)parent_fromRow];
      
      supports_fromRow[(size_t)idx_vj].push_back(idx_vj);
      
      for(int row = 1; row < nvj; ++row)
      {
        supports_fromRow[(size_t)(idx_vj+row)] = supports_fromRow[(size_t)(idx_vj+row-1)];
        supports_fromRow[(size_t)(idx_vj+row)].push_back(idx_vj+row);
      }
    }
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  bool operator==(const DataTpl<Scalar,Options,JointCollectionTpl> & data1,
                  const DataTpl<Scalar,Options,JointCollectionTpl> & data2)
  {
    bool value =
       data1.joints == data2.joints
    && data1.a == data2.a
    && data1.oa == data2.oa
    && data1.a_gf == data2.a_gf
    && data1.oa_gf == data2.oa_gf
    && data1.v == data2.v
    && data1.ov == data2.ov
    && data1.f == data2.f
    && data1.of == data2.of
    && data1.h == data2.h
    && data1.oh == data2.oh
    && data1.oMi == data2.oMi
    && data1.liMi == data2.liMi
    && data1.tau == data2.tau
    && data1.nle == data2.nle
    && data1.g == data2.g
    && data1.oMf == data2.oMf
    && data1.Ycrb == data2.Ycrb
    && data1.dYcrb == data2.dYcrb
    && data1.M == data2.M
    && data1.Minv == data2.Minv
    && data1.C == data2.C
    && data1.dHdq == data2.dHdq
    && data1.dFdq == data2.dFdq
    && data1.dFdv == data2.dFdv
    && data1.dFda == data2.dFda
    && data1.SDinv == data2.SDinv
    && data1.UDinv == data2.UDinv
    && data1.IS == data2.IS
    && data1.vxI == data2.vxI
    && data1.Ivx == data2.Ivx
    && data1.oinertias == data2.oinertias
    && data1.oYcrb == data2.oYcrb
    && data1.doYcrb == data2.doYcrb
    && data1.ddq == data2.ddq
    && data1.Yaba == data2.Yaba
    && data1.u == data2.u
    && data1.Ag == data2.Ag
    && data1.dAg == data2.dAg
    && data1.hg == data2.hg
    && data1.dhg == data2.dhg
    && data1.Ig == data2.Ig
    && data1.Fcrb == data2.Fcrb
    && data1.lastChild == data2.lastChild
    && data1.nvSubtree == data2.nvSubtree
    && data1.start_idx_v_fromRow == data2.start_idx_v_fromRow
    && data1.end_idx_v_fromRow == data2.end_idx_v_fromRow
    && data1.U == data2.U
    && data1.D == data2.D
    && data1.Dinv == data2.Dinv
    && data1.parents_fromRow == data2.parents_fromRow
    && data1.supports_fromRow == data2.supports_fromRow
    && data1.nvSubtree_fromRow == data2.nvSubtree_fromRow
    && data1.J == data2.J
    && data1.dJ == data2.dJ
    && data1.dVdq == data2.dVdq
    && data1.dAdq == data2.dAdq
    && data1.dAdv == data2.dAdv
    && data1.dtau_dq == data2.dtau_dq
    && data1.dtau_dv == data2.dtau_dv
    && data1.ddq_dq == data2.ddq_dq
    && data1.ddq_dv == data2.ddq_dv
    && data1.iMf == data2.iMf
    && data1.com == data2.com
    && data1.vcom == data2.vcom
    && data1.acom == data2.acom
    && data1.mass == data2.mass
    && data1.Jcom == data2.Jcom
    && data1.kinetic_energy == data2.kinetic_energy
    && data1.potential_energy == data2.potential_energy
    && data1.JMinvJt == data2.JMinvJt
    && data1.lambda_c == data2.lambda_c
    && data1.torque_residual == data2.torque_residual
    && data1.dq_after == data2.dq_after
    && data1.impulse_c == data2.impulse_c
    && data1.staticRegressor == data2.staticRegressor
    && data1.bodyRegressor == data2.bodyRegressor
    && data1.jointTorqueRegressor == data2.jointTorqueRegressor
    ;
    
    // operator== for Eigen::Tensor provides an Expression which might be not evaluated as a boolean
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    typedef Eigen::Map<const typename Data::VectorXs> MapVectorXs;
    value &=
       MapVectorXs(data1.kinematic_hessians.data(),data1.kinematic_hessians.size())
    == MapVectorXs(data2.kinematic_hessians.data(),data2.kinematic_hessians.size());

    return value;
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  bool operator!=(const DataTpl<Scalar,Options,JointCollectionTpl> & data1,
                  const DataTpl<Scalar,Options,JointCollectionTpl> & data2)
  {
    return !(data1 == data2);
  }

} // namespace pinocchio

/// @endcond

#endif // ifndef __pinocchio_multibody_data_hxx__
