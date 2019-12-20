//
// Copyright (c) 2015-2019 CNRS INRIA
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_data_hxx__
#define __pinocchio_data_hxx__

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
  , oYcrb((std::size_t)model.njoints,Inertia::Zero())
  , doYcrb((std::size_t)model.njoints,Inertia::Matrix6::Zero())
  , ddq(VectorXs::Zero(model.nv))
  , Yaba((std::size_t)model.njoints,Inertia::Matrix6::Zero())
  , u(VectorXs::Zero(model.nv))
  , Ag(Matrix6x::Zero(6,model.nv))
  , dAg(Matrix6x::Zero(6,model.nv))
  , Fcrb((std::size_t)model.njoints,Matrix6x::Zero(6,model.nv))
  , lastChild((std::size_t)model.njoints)
  , nvSubtree((std::size_t)model.njoints)
  , start_idx_v_fromRow((std::size_t)model.nv)
  , end_idx_v_fromRow((std::size_t)model.nv)
  , U(MatrixXs::Identity(model.nv,model.nv))
  , D(VectorXs::Zero(model.nv))
  , Dinv(VectorXs::Zero(model.nv))
  , tmp(VectorXs::Zero(model.nv))
  , parents_fromRow((std::size_t)model.nv)
  , supports_fromRow((std::size_t)model.nv)
  , nvSubtree_fromRow((std::size_t)model.nv)
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
  , JMinvJt()
  , llt_JMinvJt()
  , lambda_c()
  , sDUiJt(MatrixXs::Zero(model.nv,model.nv))
  , torque_residual(VectorXs::Zero(model.nv))
  , dq_after(VectorXs::Zero(model.nv))
  , impulse_c()
  , staticRegressor(Matrix3x::Zero(3,4*(model.njoints-1)))
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
      lastChild[parent] = std::max(lastChild[(Index)i],lastChild[parent]);
      
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
    return
      data1.joints == data2.joints
    ;
  }

} // namespace pinocchio

/// @endcond

#endif // ifndef __pinocchio_data_hxx__
