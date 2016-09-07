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
#include "pinocchio/tools/string-generator.hpp"

#include <boost/bind.hpp>

/// @cond DEV

namespace se3
{
  inline std::ostream& operator<< (std::ostream & os, const Model & model)
  {
    os << "Nb joints = " << model.njoint << " (nq="<< model.nq<<",nv="<<model.nv<<")" << std::endl;
    for(Model::Index i=0;i<(Model::Index)(model.njoint);++i)
    {
      os << "  Joint "<< model.names[i] << ": parent=" << model.parents[i]  << std::endl;
    }
    
    return os;
  }
  
  template<typename JointModelDerived>
  Model::JointIndex Model::addJoint(const Model::JointIndex parent,
                                    const JointModelBase<JointModelDerived> & joint_model,
                                    const SE3 & joint_placement,
                                    const std::string & joint_name,
                                    const Eigen::VectorXd & max_effort,
                                    const Eigen::VectorXd & max_velocity,
                                    const Eigen::VectorXd & min_config,
                                    const Eigen::VectorXd & max_config
                                    )
  {
    typedef JointModelDerived D;
    assert( (njoint==(int)joints.size())&&(njoint==(int)inertias.size())
           &&(njoint==(int)parents.size())&&(njoint==(int)jointPlacements.size()) );
    assert((joint_model.nq()>=0) && (joint_model.nv()>=0));
    
    assert(max_effort.size() == joint_model.nv()
           && max_velocity.size() == joint_model.nv()
           && min_config.size() == joint_model.nq()
           && max_config.size() == joint_model.nq());
    
    Model::JointIndex idx = (Model::JointIndex) (njoint++);
    
    joints         .push_back(JointModel(joint_model.derived()));
    boost::get<JointModelDerived>(joints.back()).setIndexes(idx,nq,nv);
    
    inertias       .push_back(Inertia::Zero());
    parents        .push_back(parent);
    jointPlacements.push_back(joint_placement);
    names          .push_back((joint_name!="")?joint_name:randomStringGenerator(8));
    nq += joint_model.nq();
    nv += joint_model.nv();
    
    effortLimit.conservativeResize(nv);effortLimit.bottomRows<D::NV>() = max_effort;
    velocityLimit.conservativeResize(nv);velocityLimit.bottomRows<D::NV>() = max_velocity;
    lowerPositionLimit.conservativeResize(nq);lowerPositionLimit.bottomRows<D::NQ>() = min_config;
    upperPositionLimit.conservativeResize(nq);upperPositionLimit.bottomRows<D::NQ>() = max_config;
    
    neutralConfiguration.conservativeResize(nq);
    neutralConfiguration.tail(joint_model.nq()) = joint_model.neutralConfiguration();
    
    // Init and add joint index to its parent subtrees.
    subtrees.push_back(IndexVector(1));
    subtrees[idx][0] = idx;
    addJointIndexToParentSubtrees(idx);
    return idx;
  }

  inline int Model::addJointFrame (const JointIndex& jidx,
                                         int         fidx)
  {
    if (fidx < 0) {
      fidx = getFrameId(names[parents[jidx]]);
    }
    if (fidx >= frames.size())
      throw std::invalid_argument ("Frame not found");
    // Add a the joint frame attached to itself to the frame vector - redundant information but useful.
    return addFrame(Frame(names[jidx],jidx,fidx,SE3::Identity(),JOINT));
  }

  inline void Model::appendBodyToJoint(const Model::JointIndex joint_index,
                                       const Inertia & Y,
                                       const SE3 & body_placement)
  {
    const Inertia & iYf = Y.se3Action(body_placement);
    inertias[joint_index] += iYf;
    nbody++;
  }

  inline int Model::addBodyFrame (const std::string & body_name,
                                  const JointIndex  & parentJoint,
                                  const SE3         & body_placement,
                                        int           previousFrame)
  {
    if (previousFrame < 0) {
      previousFrame = getFrameId(names[parentJoint]);
    }
    assert(previousFrame < frames.size() && "Frame index out of bound");
    return addFrame(Frame(body_name, parentJoint, previousFrame, body_placement, BODY));
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

  inline Model::JointIndex Model::getFrameParent( const FrameIndex index ) const
  {
    return frames[index].parent;
  }

  inline FrameType Model::getFrameType( const FrameIndex index ) const
  {
    return frames[index].type;
  }

  inline const SE3 & Model::getFramePlacement( const FrameIndex index ) const
  {
    return frames[index].placement;
  }

  inline int Model::addFrame ( const Frame & frame )
  {
    if( !existFrame(frame.name) )
    {
      frames.push_back(frame);
      nFrames++;
      return nFrames - 1;
    }
    else
    {
      return -1;
    }
  }
  
  inline void Model::addJointIndexToParentSubtrees(const JointIndex joint_id)
  {
    for(JointIndex parent = parents[joint_id]; parent>0; parent = parents[parent])
      subtrees[parent].push_back(joint_id);
  }


  inline Data::Data (const Model & model)
    :joints(0)
    ,a((std::size_t)model.njoint)
    ,a_gf((std::size_t)model.njoint)
    ,v((std::size_t)model.njoint)
    ,f((std::size_t)model.njoint)
    ,oMi((std::size_t)model.njoint)
    ,liMi((std::size_t)model.njoint)
    ,tau(model.nv)
    ,nle(model.nv)
    ,oMf((std::size_t)model.nFrames)
    ,Ycrb((std::size_t)model.njoint)
    ,M(model.nv,model.nv)
    ,ddq(model.nv)
    ,Yaba((std::size_t)model.njoint)
    ,u(model.nv)
    ,Ag(6,model.nv)
    ,Fcrb((std::size_t)model.njoint)
    ,lastChild((std::size_t)model.njoint)
    ,nvSubtree((std::size_t)model.njoint)
    ,U(model.nv,model.nv)
    ,D(model.nv)
    ,tmp(model.nv)
    ,parents_fromRow((std::size_t)model.nv)
    ,nvSubtree_fromRow((std::size_t)model.nv)
    ,J(6,model.nv)
    ,iMf((std::size_t)model.njoint)
    ,com((std::size_t)model.njoint)
    ,vcom((std::size_t)model.njoint)
    ,acom((std::size_t)model.njoint)
    ,mass((std::size_t)model.njoint)
    ,Jcom(3,model.nv)
    ,JMinvJt()
    ,llt_JMinvJt()
    ,lambda_c()
    ,sDUiJt(model.nv,model.nv)
    ,torque_residual(model.nv)
    ,dq_after(model.nv)
    ,impulse_c()
  {
    /* Create data strcture associated to the joints */
    for(Model::Index i=0;i<(Model::JointIndex)(model.njoint);++i) 
      joints.push_back(CreateJointData::run(model.joints[i]));

    /* Init for CRBA */
    M.fill(0);
    for(Model::Index i=0;i<(Model::Index)(model.njoint);++i ) { Fcrb[i].resize(6,model.nv); }
    computeLastChild(model);

    /* Init for Cholesky */
    U.setIdentity();
    computeParents_fromRow(model);

    /* Init Jacobian */
    J.setZero();
    Ag.setZero();
    
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
    for( int i=model.njoint-1;i>=0;--i )
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
    for( Model::Index joint=1;joint<(Model::Index)(model.njoint);joint++)
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
