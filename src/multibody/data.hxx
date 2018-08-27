//
// Copyright (c) 2015-2018 CNRS
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
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

#ifndef __se3_data_hxx__
#define __se3_data_hxx__

#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/utils/string-generator.hpp"
#include "pinocchio/multibody/liegroup/liegroup-algo.hpp"

#include <boost/bind.hpp>
#include <boost/utility.hpp>

/// @cond DEV

namespace se3
{
  
  inline Data::Data(const Model & model)
  : joints(0)
  , a((std::size_t)model.njoints)
  , oa((std::size_t)model.njoints)
  , a_gf((std::size_t)model.njoints)
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
  , dtau_dq(Eigen::MatrixXd::Zero(model.nv,model.nv))
  , dtau_dv(Eigen::MatrixXd::Zero(model.nv,model.nv))
  , ddq_dq(Eigen::MatrixXd::Zero(model.nv,model.nv))
  , ddq_dv(Eigen::MatrixXd::Zero(model.nv,model.nv))
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
    /* Create data strcture associated to the joints */
    for(Model::Index i=0;i<(Model::JointIndex)(model.njoints);++i) 
      joints.push_back(CreateJointData::run(model.joints[i]));

    /* Init for CRBA */
    M.fill(0); Minv.setZero();
    for(Model::Index i=0;i<(Model::Index)(model.njoints);++i ) { Fcrb[i].resize(6,model.nv); }
    computeLastChild(model);
    
    /* Init for Coriolis */
    C.fill(0.);

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
  }

  inline void Data::computeLastChild(const Model & model)
  {
    typedef Model::Index Index;
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

  inline void Data::computeParents_fromRow(const Model & model)
  {
    for( Model::Index joint=1;joint<(Model::Index)(model.njoints);joint++)
    {
      const Model::Index & parent = model.parents[joint];
      const int nvj    = nv   (model.joints[joint]);
      const int idx_vj = idx_v(model.joints[joint]);
      
      if(parent>0) parents_fromRow[(Model::Index)idx_vj] = idx_v(model.joints[parent])+nv(model.joints[parent])-1;
      else         parents_fromRow[(Model::Index)idx_vj] = -1;
      nvSubtree_fromRow[(Model::Index)idx_vj] = nvSubtree[joint];
      
      for(int row=1;row<nvj;++row)
      {
        parents_fromRow[(Model::Index)(idx_vj+row)] = idx_vj+row-1;
        nvSubtree_fromRow[(Model::Index)(idx_vj+row)] = nvSubtree[joint]-row;
      }
    }
  }

} // namespace se3

/// @endcond

#endif // ifndef __se3_data_hxx__
