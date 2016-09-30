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
#include <boost/utility.hpp>

/// @cond DEV

namespace se3
{
  namespace details
  {
    struct FilterFrame {
      const std::string& name;
      const FrameType & typeMask;
      FilterFrame (const std::string& name, const FrameType& typeMask)
        : name (name), typeMask (typeMask) {}
      bool operator() (const Frame& frame) const
      { return (typeMask & frame.type) && (name == frame.name); }
    };
  }

  inline std::ostream& operator<< (std::ostream & os, const Model & model)
  {
    os << "Nb joints = " << model.njoints << " (nq="<< model.nq<<",nv="<<model.nv<<")" << std::endl;
    for(Model::Index i=0;i<(Model::Index)(model.njoints);++i)
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
    assert( (njoints==(int)joints.size())&&(njoints==(int)inertias.size())
           &&(njoints==(int)parents.size())&&(njoints==(int)jointPlacements.size()) );
    assert((joint_model.nq()>=0) && (joint_model.nv()>=0));

    assert(max_effort.size() == joint_model.nv()
           && max_velocity.size() == joint_model.nv()
           && min_config.size() == joint_model.nq()
           && max_config.size() == joint_model.nq());
    
    Model::JointIndex idx = (Model::JointIndex) (njoints++);
    
    joints         .push_back(JointModel(joint_model.derived()));
    boost::get<JointModelDerived>(joints.back()).setIndexes(idx,nq,nv);
    
    inertias       .push_back(Inertia::Zero());
    parents        .push_back(parent);
    jointPlacements.push_back(joint_placement);
    names          .push_back(joint_name);
    nq += joint_model.nq();
    nv += joint_model.nv();

    // Optimal efficiency here would be using the static-dim bottomRows, while specifying the dimension in argument in the case where D::NV is Eigen::Dynamic.
    // However, this option is not compiling in Travis (why?).
    // As efficiency of Model::addJoint is not critical, the dynamic bottomRows is used here.
    effortLimit.conservativeResize(nv);effortLimit.bottomRows(joint_model.nv()) = max_effort;
    velocityLimit.conservativeResize(nv);velocityLimit.bottomRows(joint_model.nv()) = max_velocity;
    lowerPositionLimit.conservativeResize(nq);lowerPositionLimit.bottomRows(joint_model.nq()) = min_config;
    upperPositionLimit.conservativeResize(nq);upperPositionLimit.bottomRows(joint_model.nq()) = max_config;
    
    neutralConfiguration.conservativeResize(nq);
    neutralConfiguration.tail(joint_model.nq()) = joint_model.neutralConfiguration();
    
    // Init and add joint index to its parent subtrees.
    subtrees.push_back(IndexVector(1));
    subtrees[idx][0] = idx;
    addJointIndexToParentSubtrees(idx);
    return idx;
  }

  template<typename JointModelDerived>
  Model::JointIndex Model::addJoint(const Model::JointIndex parent,
                                    const JointModelBase<JointModelDerived> & joint_model,
                                    const SE3 & joint_placement,
                                    const std::string & joint_name
                                    )
  {
    Eigen::VectorXd max_effort, max_velocity, min_config, max_config;

    max_effort = Eigen::VectorXd::Constant(joint_model.nv(), std::numeric_limits<double>::max());
    max_velocity = Eigen::VectorXd::Constant(joint_model.nv(), std::numeric_limits<double>::max());
    min_config = Eigen::VectorXd::Constant(joint_model.nq(), std::numeric_limits<double>::max());
    max_config = Eigen::VectorXd::Constant(joint_model.nq(), std::numeric_limits<double>::max());

    return addJoint(parent, joint_model, joint_placement, joint_name, max_effort, max_velocity, min_config, max_config);
  }
  
  inline int Model::addJointFrame (const JointIndex& jidx,
                                         int         fidx)
  {
    if (fidx < 0) {
      // FIXED_JOINT is required because the parent can be the universe and its
      // type is FIXED_JOINT
      fidx = (int)getFrameId(names[parents[jidx]], (FrameType)(JOINT | FIXED_JOINT));
    }
    assert((size_t)fidx < frames.size() && "Frame index out of bound");
    // Add a the joint frame attached to itself to the frame vector - redundant information but useful.
    return addFrame(Frame(names[jidx],jidx,(FrameIndex)fidx,SE3::Identity(),JOINT));
  }


  inline void Model::appendBodyToJoint(const Model::JointIndex joint_index,
                                       const Inertia & Y,
                                       const SE3 & body_placement)
  {
    const Inertia & iYf = Y.se3Action(body_placement);
    inertias[joint_index] += iYf;
    nbodies++;
  }

  inline int Model::addBodyFrame (const std::string & body_name,
                                  const JointIndex  & parentJoint,
                                  const SE3         & body_placement,
                                        int           previousFrame)
  {
    if (previousFrame < 0) {
      // FIXED_JOINT is required because the parent can be the universe and its
      // type is FIXED_JOINT
      previousFrame = (int)getFrameId(names[parentJoint], (FrameType)(JOINT | FIXED_JOINT));
    }
    assert((size_t)previousFrame < frames.size() && "Frame index out of bound");
    return addFrame(Frame(body_name, parentJoint, (FrameIndex)previousFrame, body_placement, BODY));
  }
  
  inline Model::JointIndex Model::getBodyId (const std::string & name) const
  {
    return getFrameId(name, BODY);
  }
  
  inline bool Model::existBodyName (const std::string & name) const
  {
    return existFrame(name, BODY);
  }


  inline Model::JointIndex Model::getJointId (const std::string & name) const
  {
    typedef std::vector<std::string>::iterator::difference_type it_diff_t;
    it_diff_t res = std::find(names.begin(),names.end(),name) - names.begin();
    assert( (res<INT_MAX) && "Id superior to int range. Should never happen.");
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

  inline Model::FrameIndex Model::getFrameId ( const std::string & name, const FrameType & type ) const
  {
    container::aligned_vector<Frame>::const_iterator it = std::find_if( frames.begin()
                                                        , frames.end()
                                                        , details::FilterFrame (name, type)
                                                        );
    assert (it != frames.end() && "Frame not found");
    assert ((std::find_if( boost::next(it), frames.end(), details::FilterFrame (name, type)) == frames.end())
        && "Several frames match the filter");
    return Model::FrameIndex(it - frames.begin());
  }

  inline bool Model::existFrame ( const std::string & name, const FrameType & type) const
  {
    return std::find_if( frames.begin(), frames.end(),
        details::FilterFrame (name, type)) != frames.end();
  }


  inline int Model::addFrame ( const Frame & frame )
  {
    if( !existFrame(frame.name, frame.type) )
    {
      frames.push_back(frame);
      nframes++;
      return nframes - 1;
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
    ,a((std::size_t)model.njoints)
    ,a_gf((std::size_t)model.njoints)
    ,v((std::size_t)model.njoints)
    ,f((std::size_t)model.njoints)
    ,oMi((std::size_t)model.njoints)
    ,liMi((std::size_t)model.njoints)
    ,tau(model.nv)
    ,nle(model.nv)
    ,oMf((std::size_t)model.nframes)
    ,Ycrb((std::size_t)model.njoints)
    ,M(model.nv,model.nv)
    ,ddq(model.nv)
    ,Yaba((std::size_t)model.njoints)
    ,u(model.nv)
    ,Ag(6,model.nv)
    ,Fcrb((std::size_t)model.njoints)
    ,lastChild((std::size_t)model.njoints)
    ,nvSubtree((std::size_t)model.njoints)
    ,U(model.nv,model.nv)
    ,D(model.nv)
    ,tmp(model.nv)
    ,parents_fromRow((std::size_t)model.nv)
    ,nvSubtree_fromRow((std::size_t)model.nv)
    ,J(6,model.nv)
    ,iMf((std::size_t)model.njoints)
    ,com((std::size_t)model.njoints)
    ,vcom((std::size_t)model.njoints)
    ,acom((std::size_t)model.njoints)
    ,mass((std::size_t)model.njoints)
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
    for(Model::Index i=0;i<(Model::JointIndex)(model.njoints);++i) 
      joints.push_back(CreateJointData::run(model.joints[i]));

    /* Init for CRBA */
    M.fill(0);
    for(Model::Index i=0;i<(Model::Index)(model.njoints);++i ) { Fcrb[i].resize(6,model.nv); }
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

  inline void Data::computeParents_fromRow (const Model & model)
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

#endif // ifndef __se3_model_hxx__
