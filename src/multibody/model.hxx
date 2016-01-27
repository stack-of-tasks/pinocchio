//
// Copyright (c) 2015-2016 CNRS
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

#ifndef __se3_model_hxx__
#define __se3_model_hxx__

#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/force.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/multibody/joint/joint-variant.hpp"
#include <iostream>

namespace se3
{
  inline std::ostream& operator<< (std::ostream & os, const Model & model)
  {
    os << "Nb bodies = " << model.nbody << " (nq="<< model.nq<<",nv="<<model.nv<<")" << std::endl;
    for(Model::Index i=0;i<(Model::Index)(model.nbody);++i)
      {
	os << "  Joint "<<model.names[i] << ": parent=" << model.parents[i] 
	   << ( model.hasVisual[i] ? " (has visual) " : "(doesnt have visual)" ) << std::endl;
      }

    return os;
  }

  inline std::string random (const int len)
  {
    std::string res;
    static const char alphanum[] =
        "0123456789"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz";

    for (int i=0; i<len;++i)
      res += alphanum[((size_t)std::rand() % (sizeof(alphanum) - 1))];
    return res;
  }

  template<typename D>
  Model::Index Model::addBody (Index parent, const JointModelBase<D> & j, const SE3 & placement,
                               const Inertia & Y, const std::string & jointName,
                               const std::string & bodyName, bool visual)
  {
    assert( (nbody==(int)joints.size())&&(nbody==(int)inertias.size())
      &&(nbody==(int)parents.size())&&(nbody==(int)jointPlacements.size()) );
    assert( (j.nq()>=0)&&(j.nv()>=0) );

    Model::Index idx = (Model::Index) (nbody ++);

    joints         .push_back(j.derived()); 
    boost::get<D>(joints.back()).setIndexes((int)idx,nq,nv);

    inertias       .push_back(Y);
    parents        .push_back(parent);
    jointPlacements.push_back(placement);
    names          .push_back( (jointName!="")?jointName:random(8) );
    hasVisual      .push_back(visual);
    bodyNames      .push_back( (bodyName!="")?bodyName:random(8));
    nq += j.nq();
    nv += j.nv();
    return idx;
  }

  template<typename D>
  Model::Index Model::addBody (Index parent, const JointModelBase<D> & j, const SE3 & placement,
                               const Inertia & Y,
                               const Eigen::VectorXd & effort, const Eigen::VectorXd & velocity,
                               const Eigen::VectorXd & lowPos, const Eigen::VectorXd & upPos,
                               const std::string & jointName,
                               const std::string & bodyName, bool visual)
  {
    assert( (nbody==(int)joints.size())&&(nbody==(int)inertias.size())
	    &&(nbody==(int)parents.size())&&(nbody==(int)jointPlacements.size()) );
    assert( (j.nq()>=0)&&(j.nv()>=0) );

    Model::Index idx = (Model::Index) (nbody ++);

    joints         .push_back(j.derived()); 
    boost::get<D>(joints.back()).setIndexes((int)idx,nq,nv);

    boost::get<D>(joints.back()).setMaxEffortLimit(effort);
    boost::get<D>(joints.back()).setMaxVelocityLimit(velocity);

    boost::get<D>(joints.back()).setLowerPositionLimit(lowPos);
    boost::get<D>(joints.back()).setUpperPositionLimit(upPos);

    inertias       .push_back(Y);
    parents        .push_back(parent);
    jointPlacements.push_back(placement);
    names          .push_back( (jointName!="")?jointName:random(8) );
    hasVisual      .push_back(visual);
    bodyNames      .push_back( (bodyName!="")?bodyName:random(8));
    nq += j.nq();
    nv += j.nv();
    return idx;
  }

  inline Model::Index Model::addFixedBody (Index lastMovingParent,
                                           const SE3 & placementFromLastMoving,
                                           const std::string & bodyName,
                                           bool visual)
  {

    Model::Index idx = (Model::Index) (nFixBody++);
    fix_lastMovingParent.push_back(lastMovingParent);
    fix_lmpMi      .push_back(placementFromLastMoving);
    fix_hasVisual  .push_back(visual);
    fix_bodyNames  .push_back( (bodyName!="")?bodyName:random(8));
    return idx;
  }

  inline void Model::mergeFixedBody (Index parent, const SE3 & placement, const Inertia & Y)
  {
    const Inertia & iYf = Y.se3Action(placement); //TODO
    inertias[parent] += iYf;
  }

  inline Model::Index Model::getBodyId (const std::string & name) const
  {
    std::vector<std::string>::iterator::difference_type
      res = std::find(bodyNames.begin(),bodyNames.end(),name) - bodyNames.begin();
    assert( (res<INT_MAX) && "Id superior to int range. Should never happen.");
    assert( (res>=0)&&(res<nbody) && "The body name you asked does not exist" );
    return Model::Index(res);
  }
  
  inline bool Model::existBodyName (const std::string & name) const
  {
    return (bodyNames.end() != std::find(bodyNames.begin(),bodyNames.end(),name));
  }

  inline const std::string& Model::getBodyName (Model::Index index) const
  {
    assert( index < (Model::Index)nbody );
    return bodyNames[index];
  }

  inline Model::Index Model::getJointId (const std::string & name) const
  {
    typedef std::vector<std::string>::iterator::difference_type it_diff_t;
    it_diff_t res = std::find(names.begin(),names.end(),name) - names.begin();
    assert( (res<INT_MAX) && "Id superior to int range. Should never happen.");
    assert( (res>=0)&&(res<(it_diff_t) joints.size()) && "The joint name you asked does not exist" );
    return Model::Index(res);
  }
  
  inline bool Model::existJointName (const std::string & name) const
  {
    return (names.end() != std::find(names.begin(),names.end(),name));
  }

  inline const std::string& Model::getJointName (Model::Index index) const
  {
    assert( index < (Model::Index)joints.size() );
    return names[index];
  }

  inline Data::Data (const Model & ref)
    :model(ref)
    ,joints(0)
    ,a((std::size_t)ref.nbody)
    ,v((std::size_t)ref.nbody)
    ,f((std::size_t)ref.nbody)
    ,oMi((std::size_t)ref.nbody)
    ,liMi((std::size_t)ref.nbody)
    ,tau(ref.nv)
    ,nle(ref.nv)
    ,Ycrb((std::size_t)ref.nbody)
    ,M(ref.nv,ref.nv)
    ,Fcrb((std::size_t)ref.nbody)
    ,lastChild((std::size_t)ref.nbody)
    ,nvSubtree((std::size_t)ref.nbody)
    ,U(ref.nv,ref.nv)
    ,D(ref.nv)
    ,tmp(ref.nv)
    ,parents_fromRow((std::size_t)ref.nv)
    ,nvSubtree_fromRow((std::size_t)ref.nv)
    ,J(6,ref.nv)
    ,iMf((std::size_t)ref.nbody)
    ,com((std::size_t)ref.nbody)
    ,vcom((std::size_t)ref.nbody)
    ,acom((std::size_t)ref.nbody)
    ,mass((std::size_t)ref.nbody)
    ,Jcom(3,ref.nv)
    ,effortLimit(ref.nq)
    ,velocityLimit(ref.nv)
    ,lowerPositionLimit(ref.nq)
    ,upperPositionLimit(ref.nq)
  {
    /* Create data strcture associated to the joints */
    for(Model::Index i=0;i<(Model::Index)(model.nbody);++i) 
      joints.push_back(CreateJointData::run(model.joints[i]));

    /* Init for CRBA */
    M.fill(0);
    for(Model::Index i=0;i<(Model::Index)(ref.nbody);++i ) { Fcrb[i].resize(6,model.nv); }
    computeLastChild(ref);

    /* Init for Cholesky */
    U = Eigen::MatrixXd::Identity(ref.nv,ref.nv);
    computeParents_fromRow(ref);

    /* Init Jacobian */
    J.fill(0);
    
    /* Init universe states relatively to itself */
    
    a[0].setZero();
    v[0].setZero();
    f[0].setZero();
    oMi[0].setIdentity();
    liMi[0].setIdentity();
  }

  inline void Data::computeLastChild (const Model & model)
  {
    typedef Model::Index Index;
    std::fill(lastChild.begin(),lastChild.end(),-1);
    for( int i=model.nbody-1;i>=0;--i )
    {
      if(lastChild[(Index)i] == -1) lastChild[(Index)i] = i;
      const Index & parent = model.parents[(Index)i];
      lastChild[parent] = std::max(lastChild[(Index)i],lastChild[parent]);
      
      nvSubtree[(Index)i]
      = idx_v(model.joints[(Index)lastChild[(Index)i]]) + nv(model.joints[(Index)lastChild[(Index)i]])
      - idx_v(model.joints[(Index)i]);
    }
  }

  inline void Data::computeParents_fromRow (const Model & model)
  {
    for( Model::Index joint=1;joint<(Model::Index)(model.nbody);joint++)
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

#endif // ifndef __se3_model_hxx__
