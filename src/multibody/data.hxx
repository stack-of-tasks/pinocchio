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
  , a((std::size_t)model.njoints)
  , oa((std::size_t)model.njoints)
  , a_gf((std::size_t)model.njoints)
  , oa_gf((std::size_t)model.njoints)
  , v((std::size_t)model.njoints)
  , ov((std::size_t)model.njoints)
  , f((std::size_t)model.njoints)
  , of((std::size_t)model.njoints)
  , h((std::size_t)model.njoints)
  , oh((std::size_t)model.njoints)
  , oMi((std::size_t)model.njoints)
  , liMi((std::size_t)model.njoints)
  , tau(model.nv)
  , nle(model.nv)
  , g(model.nv)
  , oMf((std::size_t)model.nframes)
  , Ycrb((std::size_t)model.njoints)
  , dYcrb((std::size_t)model.njoints)
  , M(model.nv,model.nv)
  , Minv(model.nv,model.nv)
  , C(model.nv,model.nv)
  , dHdq(6,model.nv)
  , dFdq(6,model.nv)
  , dFdv(6,model.nv)
  , dFda(6,model.nv)
  , SDinv(6,model.nv)
  , UDinv(6,model.nv)
  , IS(6,model.nv)
  , vxI((std::size_t)model.njoints)
  , Ivx((std::size_t)model.njoints)
  , oYcrb((std::size_t)model.njoints)
  , doYcrb((std::size_t)model.njoints)
  , ddq(model.nv)
  , Yaba((std::size_t)model.njoints)
  , u(model.nv)
  , Ag(6,model.nv)
  , dAg(6,model.nv)
  , Fcrb((std::size_t)model.njoints)
  , lastChild((std::size_t)model.njoints)
  , nvSubtree((std::size_t)model.njoints)
  , U(model.nv,model.nv)
  , D(model.nv)
  , Dinv(model.nv)
  , tmp(model.nv)
  , parents_fromRow((std::size_t)model.nv)
  , nvSubtree_fromRow((std::size_t)model.nv)
  , J(6,model.nv)
  , dJ(6,model.nv)
  , dVdq(6,model.nv)
  , dAdq(6,model.nv)
  , dAdv(6,model.nv)
  , dtau_dq(MatrixXs::Zero(model.nv,model.nv))
  , dtau_dv(MatrixXs::Zero(model.nv,model.nv))
  , ddq_dq(MatrixXs::Zero(model.nv,model.nv))
  , ddq_dv(MatrixXs::Zero(model.nv,model.nv))
  , iMf((std::size_t)model.njoints)
  , com((std::size_t)model.njoints)
  , vcom((std::size_t)model.njoints)
  , acom((std::size_t)model.njoints)
  , mass((std::size_t)model.njoints)
  , Jcom(3,model.nv)
  , JMinvJt()
  , llt_JMinvJt()
  , lambda_c()
  , sDUiJt(model.nv,model.nv)
  , torque_residual(model.nv)
  , dq_after(model.nv)
  , impulse_c()
  , staticRegressor(3,4*(model.njoints-1))
  {
    typedef typename Model::JointIndex JointIndex;
    
    /* Create data strcture associated to the joints */
    for(JointIndex i=0;i<(JointIndex)(model.njoints);++i)
      joints.push_back(CreateJointData<Scalar,Options,JointCollectionTpl>::run(model.joints[i]));

    /* Init for CRBA */
    M.setZero(); Minv.setZero();
    for(JointIndex i=0;i<(JointIndex)(model.njoints);++i)
    { Fcrb[i].resize(6,model.nv); }
    
    computeLastChild(model);
    
    /* Init for Coriolis */
    C.setZero();

    /* Init for Cholesky */
    U.setIdentity();
    computeParents_fromRow(model);

    /* Init Jacobian */
    J.setZero();
    Ag.setZero();
    
    /* Init universe states relatively to itself */
    
    a[0].setZero();
    oa[0].setZero();
    v[0].setZero();
    ov[0].setZero();
    a_gf[0] = -model.gravity;
    f[0].setZero();
    h[0].setZero();
    oMi[0].setIdentity();
    liMi[0].setIdentity();
    oMf[0].setIdentity();
    
    Yaba[0].setZero();
    Ycrb[0].setZero();
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
      
      if(parent>0) parents_fromRow[(Index)idx_vj] = idx_v(model.joints[parent])+nv(model.joints[parent])-1;
      else         parents_fromRow[(Index)idx_vj] = -1;
      nvSubtree_fromRow[(Index)idx_vj] = nvSubtree[joint];
      
      for(int row=1;row<nvj;++row)
      {
        parents_fromRow[(Index)(idx_vj+row)] = idx_vj+row-1;
        nvSubtree_fromRow[(Index)(idx_vj+row)] = nvSubtree[joint]-row;
      }
    }
  }

} // namespace pinocchio

/// @endcond

#endif // ifndef __pinocchio_data_hxx__
