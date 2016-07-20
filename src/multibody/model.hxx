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
#include "pinocchio/multibody/joint/joint-basic-visitors.hpp"
#include <iostream>

#include <boost/bind.hpp>

/// @cond DEV

namespace se3
{
  inline std::ostream& operator<< (std::ostream & os, const Model & model)
  {
    os << "Nb bodies = " << model.nbody << " (nq="<< model.nq<<",nv="<<model.nv<<")" << std::endl;
    for(Model::Index i=0;i<(Model::Index)(model.nbody);++i)
      {
	os << "  Joint "<<model.names[i] << ": parent=" << model.parents[i]  << std::endl;
      }

    return os;
  }
  


  template<typename D>
  Model::JointIndex Model::addJointAndBody(JointIndex parent, const JointModelBase<D> & j, const SE3 & jointPlacement,
                                           const Inertia & Y, const std::string & jointName,
                                           const std::string & bodyName)
  {
    JointIndex idx = addJoint(parent,j,jointPlacement,jointName);
    appendBodyToJoint(idx, SE3::Identity(), Y, bodyName);
    return idx;
  }

  template<typename D>
  Model::JointIndex Model::addJointAndBody(JointIndex parent, const JointModelBase<D> & j, const SE3 & jointPlacement,
                                           const Inertia & Y,
                                           const Eigen::VectorXd & effort, const Eigen::VectorXd & velocity,
                                           const Eigen::VectorXd & lowPos, const Eigen::VectorXd & upPos,
                                           const std::string & jointName,
                                           const std::string & bodyName)
  {
    JointIndex idx = addJoint(parent,j,jointPlacement,
                              effort, velocity, lowPos, upPos,
                              jointName);
    appendBodyToJoint(idx, SE3::Identity(), Y, bodyName);
    return idx;
  }


  template<typename D>
  Model::JointIndex Model::addJoint(JointIndex parent, const JointModelBase<D> & j, const SE3 & jointPlacement,
                                    const std::string & jointName)
  {
    assert( (njoint==(int)joints.size())&&(njoint==(int)inertias.size())
      &&(njoint==(int)parents.size())&&(njoint==(int)jointPlacements.size()) );
    assert( (j.nq()>=0)&&(j.nv()>=0) );

    Model::JointIndex idx = (Model::JointIndex) (njoint ++);

    joints         .push_back(JointModel(j.derived())); 
    boost::get<D>(joints.back()).setIndexes(idx,nq,nv);

    inertias       .push_back(Inertia::Zero());
    parents        .push_back(parent);
    jointPlacements.push_back(jointPlacement);
    names          .push_back( (jointName!="")?jointName:random(8) );
    nq += j.nq();
    nv += j.nv();

    neutralConfigurations.conservativeResize(nq);neutralConfigurations.bottomRows<D::NQ>() = j.neutralConfiguration();
    effortLimit.conservativeResize(nv);effortLimit.bottomRows<D::NV>().fill(std::numeric_limits<double>::infinity());
    velocityLimit.conservativeResize(nv);velocityLimit.bottomRows<D::NV>().fill(std::numeric_limits<double>::infinity());
    lowerPositionLimit.conservativeResize(nq);lowerPositionLimit.bottomRows<D::NQ>().fill(-std::numeric_limits<double>::infinity());
    upperPositionLimit.conservativeResize(nq);upperPositionLimit.bottomRows<D::NQ>().fill(std::numeric_limits<double>::infinity());
    return idx;
  }

  template<typename D>
  Model::JointIndex Model::addJoint(JointIndex parent,const JointModelBase<D> & j,const SE3 & jointPlacement,
                     const Eigen::VectorXd & effort, const Eigen::VectorXd & velocity,
                     const Eigen::VectorXd & lowPos, const Eigen::VectorXd & upPos,
                     const std::string & jointName)
  {
    assert( (njoint==(int)joints.size())&&(njoint==(int)inertias.size())
      &&(njoint==(int)parents.size())&&(njoint==(int)jointPlacements.size()) );
    assert( (j.nq()>=0)&&(j.nv()>=0) );

    assert( effort.size() == j.nv() && velocity.size() == j.nv()
           && lowPos.size() == j.nq() && upPos.size() == j.nq() );

    Model::JointIndex idx = (Model::JointIndex) (njoint ++);

    joints         .push_back(JointModel(j.derived())); 
    boost::get<D>(joints.back()).setIndexes(idx,nq,nv);

    inertias       .push_back(Inertia::Zero());
    parents        .push_back(parent);
    jointPlacements.push_back(jointPlacement);
    names          .push_back( (jointName!="")?jointName:random(8) );
    nq += j.nq();
    nv += j.nv();

    
    effortLimit.conservativeResize(nv);effortLimit.bottomRows<D::NV>() = effort;
    velocityLimit.conservativeResize(nv);velocityLimit.bottomRows<D::NV>() = velocity;
    lowerPositionLimit.conservativeResize(nq);lowerPositionLimit.bottomRows<D::NQ>() = lowPos;
    upperPositionLimit.conservativeResize(nq);upperPositionLimit.bottomRows<D::NQ>() = upPos;

    // Ensure that limits are not inf
    Eigen::VectorXd neutralConf((lowerPositionLimit.bottomRows<D::NQ>() + upperPositionLimit.bottomRows<D::NQ>())/2 );
    
    neutralConfigurations.conservativeResize(nq);
    if ( std::isfinite(neutralConf.norm()) )
    {
      neutralConfigurations.bottomRows<D::NQ>() = neutralConf;
    }
    else
    {
      assert( false && "One of the position limit is inf or NaN");
      neutralConfigurations.bottomRows<D::NQ>() = j.neutralConfiguration();
    }

    addFrame((jointName!="")?jointName:random(8), idx, SE3::Identity(), JOINT);

    return idx;
  }


  inline void Model::appendBodyToJoint(const JointIndex parent, const SE3 & bodyPlacement, const Inertia & Y,
                                       const std::string & bodyName)
  {
    const Inertia & iYf = Y.se3Action(bodyPlacement);
    inertias[parent] += iYf;

    addFrame((bodyName!="")?bodyName:random(8), parent, bodyPlacement, BODY);

    nbody ++;
  }
  
  inline Model::JointIndex Model::getBodyId (const std::string & name) const
  {
    return getFrameId(name);
  }
  
  inline bool Model::existBodyName (const std::string & name) const
  {
    return existFrame(name);
  }

  inline const std::string& Model::getBodyName (const Model::JointIndex index) const
  {
    assert( index < (Model::Index)nbody );
    return getFrameName(index);
  }

  inline Model::JointIndex Model::getJointId (const std::string & name) const
  {
    typedef std::vector<std::string>::iterator::difference_type it_diff_t;
    it_diff_t res = std::find(names.begin(),names.end(),name) - names.begin();
    assert( (res<INT_MAX) && "Id superior to int range. Should never happen.");
    assert( (res>=0)&&(res<(it_diff_t) joints.size()) && "The joint name you asked does not exist" );
    return Model::JointIndex(res);
  }
  
  inline bool Model::existJointName (const std::string & name) const
  {
    return (names.end() != std::find(names.begin(),names.end(),name));
  }

  inline const std::string& Model::getJointName (const JointIndex index) const
  {
    assert( index < (Model::JointIndex)joints.size() );
    return names[index];
  }

  inline Model::FrameIndex Model::getFrameId ( const std::string & name ) const
  {
    std::vector<Frame>::const_iterator it = std::find_if( frames.begin()
                                                        , frames.end()
                                                        , boost::bind(&Frame::name, _1) == name
                                                        );
    return Model::FrameIndex(it - frames.begin());
  }

  inline bool Model::existFrame ( const std::string & name ) const
  {
    return std::find_if( frames.begin(), frames.end(), boost::bind(&Frame::name, _1) == name) != frames.end();
  }

  inline const std::string & Model::getFrameName ( const FrameIndex index ) const
  {
    return frames[index].name;
  }

  inline Model::JointIndex Model::getFrameParent( const std::string & name ) const
  {
    assert(existFrame(name) && "The Frame you requested does not exist");
    std::vector<Frame>::const_iterator it = std::find_if( frames.begin()
                                                        , frames.end()
                                                        , boost::bind(&Frame::name, _1) == name
                                                        );
    
    std::vector<Frame>::iterator::difference_type it_diff = it - frames.begin();
    return getFrameParent(Model::JointIndex(it_diff));
  }

  inline Model::JointIndex Model::getFrameParent( const FrameIndex index ) const
  {
    return frames[index].parent;
  }

  inline FrameType Model::getFrameType( const std::string & name ) const
  {
    assert(existFrame(name) && "The Frame you requested does not exist");
    std::vector<Frame>::const_iterator it = std::find_if( frames.begin()
                                                        , frames.end()
                                                        , boost::bind(&Frame::name, _1) == name
                                                        );
    
    std::vector<Frame>::iterator::difference_type it_diff = it - frames.begin();
    return getFrameType(Model::JointIndex(it_diff));
  }

  inline FrameType Model::getFrameType( const FrameIndex index ) const
  {
    return frames[index].type;
  }

  inline const SE3 & Model::getFramePlacement( const std::string & name) const
  {
    assert(existFrame(name) && "The Frame you requested does not exist");
    std::vector<Frame>::const_iterator it = std::find_if( frames.begin()
                                                        , frames.end()
                                                        , boost::bind(&Frame::name, _1) == name
                                                        );
    
    std::vector<Frame>::iterator::difference_type it_diff = it - frames.begin();
    return getFramePlacement(Model::Index(it_diff));
  }

  inline const SE3 & Model::getFramePlacement( const FrameIndex index ) const
  {
    return frames[index].placement;
  }

  inline bool Model::addFrame ( const Frame & frame )
  {
    if( !existFrame(frame.name) )
    {
      frames.push_back(frame);
      nFrames++;
      return true;
    }
    else
    {
      return false;
    }
  }

  inline bool Model::addFrame ( const std::string & name, JointIndex index, const SE3 & placement, const FrameType type)
  {
    if( !existFrame(name) )
      return addFrame(Frame(name, index, placement, type));
    else
      return false;
  }


  inline Data::Data (const Model & ref)
    :model(ref)
    ,joints(0)
    ,a((std::size_t)ref.nbody)
    ,a_gf((std::size_t)ref.nbody)
    ,v((std::size_t)ref.nbody)
    ,f((std::size_t)ref.nbody)
    ,oMi((std::size_t)ref.nbody)
    ,liMi((std::size_t)ref.nbody)
    ,tau(ref.nv)
    ,nle(ref.nv)
    ,oMf((std::size_t)ref.nFrames)
    ,Ycrb((std::size_t)ref.nbody)
    ,M(ref.nv,ref.nv)
    ,ddq(ref.nv)
    ,Yaba((std::size_t)ref.nbody)
    ,u(ref.nv)
    ,Ag(6, ref.nv)
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
    ,JMinvJt()
    ,llt_JMinvJt()
    ,lambda_c()
    ,sDUiJt(ref.nv,ref.nv)
    ,torque_residual(ref.nv)
    ,dq_after(model.nv)
    ,impulse_c()
  {
    /* Create data strcture associated to the joints */
    for(Model::Index i=0;i<(Model::JointIndex)(model.nbody);++i) 
      joints.push_back(CreateJointData::run(model.joints[i]));

    /* Init for CRBA */
    M.fill(0);
    for(Model::Index i=0;i<(Model::Index)(ref.nbody);++i ) { Fcrb[i].resize(6,model.nv); }
    computeLastChild(ref);

    /* Init for Cholesky */
    U.setIdentity();
    computeParents_fromRow(ref);

    /* Init Jacobian */
    J.fill(0);
    
    /* Init universe states relatively to itself */
    
    a[0].setZero();
    v[0].setZero();
    a_gf[0] = -model.gravity;
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

/// @endcond

#endif // ifndef __se3_model_hxx__
