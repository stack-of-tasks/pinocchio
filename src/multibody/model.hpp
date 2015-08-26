//
// Copyright (c) 2015 CNRS
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

#ifndef __se3_model_hpp__
#define __se3_model_hpp__


#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/force.hpp"
#include "pinocchio/multibody/joint.hpp"
#include <iostream>

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(se3::SE3)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(se3::Inertia)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(se3::Force)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(se3::Motion)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix<double,6,Eigen::Dynamic>)

namespace se3
{
  class Model;
  class Data;

  class Model
  {
  public:
    typedef std::size_t Index;

    int nq;                               // Dimension of the configuration representation
    int nv;                               // Dimension of the velocity vector space
    int nbody;                            // Number of bodies (= number of joints + 1)
    int nFixBody;                         // Number of fixed-bodies (= number of fixed-joints)

    std::vector<Inertia> inertias;        // Spatial inertias of the body <i> in the supporting joint frame <i>
    std::vector<SE3> jointPlacements;     // Placement (SE3) of the input of joint <i> in parent joint output <li>
    JointModelVector joints;              // Model of joint <i>
    std::vector<Index> parents;           // Joint parent of joint <i>, denoted <li> (li==parents[i])
    std::vector<std::string> names;       // Name of joint <i>
    std::vector<std::string> bodyNames;   // Name of the body attached to the output of joint <i>
    std::vector<bool> hasVisual;          // True iff body <i> has a visual mesh.

    std::vector<SE3> fix_lmpMi;           // Fixed-body relative placement (wrt last moving parent)
    std::vector<Model::Index> fix_lastMovingParent; // Fixed-body index of the last moving parent
    std::vector<bool> fix_hasVisual;      // True iff fixed-body <i> has a visual mesh.
    std::vector<std::string> fix_bodyNames;// Name of fixed-joint <i>

    Motion gravity;                       // Spatial gravity
    static const Eigen::Vector3d gravity981; // Default 3D gravity (=(0,0,9.81))

    Model()
      : nq(0)
      , nv(0)
      , nbody(1)
      , nFixBody(0)
      , inertias(1)
      , jointPlacements(1)
      , joints(1)
      , parents(1)
      , names(1)
      , bodyNames(1)
      , hasVisual(1)
      , gravity( gravity981,Eigen::Vector3d::Zero() )
    {
      names[0]     = "universe";     // Should be "universe joint (trivial)"
      bodyNames[0] = "universe";
    }
    ~Model() {} // std::cout << "Destroy model" << std::endl; }
    template<typename D>
    Index addBody(  Index parent,const JointModelBase<D> & j,const SE3 & placement,
		                const Inertia & Y,
                    const std::string & jointName = "", const std::string & bodyName = "",
                    bool visual = false);
    template<typename D>
    Index addBody(  Index parent,const JointModelBase<D> & j,const SE3 & placement,
                    const Inertia & Y,
                    const Eigen::VectorXd & effort, const Eigen::VectorXd & velocity,
                    const Eigen::VectorXd & lowPos, const Eigen::VectorXd & upPos,
                    const std::string & jointName = "", const std::string & bodyName = "",
                    bool visual = false);
    Index addFixedBody( Index fix_lastMovingParent,
                        const SE3 & placementFromLastMoving,
                        const std::string &jointName = "",
                        bool visual=false);
    void mergeFixedBody(Index parent, const SE3 & placement, const Inertia & Y);
    Index getBodyId( const std::string & name ) const;
    bool existBodyName( const std::string & name ) const;
    const std::string& getBodyName( Index index ) const;
  };

  class Data
  {
  public:
    typedef Eigen::Matrix<double,6,Eigen::Dynamic> Matrix6x;
    typedef Eigen::Matrix<double,3,Eigen::Dynamic> Matrix3x;
    
  public:
    const Model& model;
    JointDataVector joints;
    std::vector<Motion> a;                // Body acceleration
    std::vector<Motion> v;                // Body velocity
    std::vector<Force> f;                 // Body force
    std::vector<SE3> oMi;                 // Body absolute placement (wrt world)
    std::vector<SE3> liMi;                // Body relative placement (wrt parent)
    Eigen::VectorXd tau;                  // Joint forces
    Eigen::VectorXd nle;                  // Non linear effects

    std::vector<Inertia> Ycrb;            // Inertia of the sub-tree composit rigid body
    Eigen::MatrixXd M;                    // Joint Inertia

    std::vector<Matrix6x> Fcrb;           // Spatial forces set, used in CRBA

    std::vector<int> lastChild;  // Index of the last child (for CRBA)
    std::vector<int> nvSubtree;           // Dimension of the subtree motion space (for CRBA)

    Eigen::MatrixXd U;                    // Joint Inertia square root (upper triangle)
    Eigen::VectorXd D;                    // Diagonal of UDUT inertia decomposition
    Eigen::VectorXd tmp;                  // Temporary of size NV used in Cholesky
    std::vector<int> parents_fromRow;     // First previous non-zero row in M (used in Cholesky)
    std::vector<int> nvSubtree_fromRow;   // 
    
    Eigen::MatrixXd J;                    // Jacobian of joint placement
    std::vector<SE3> iMf;                 // Body placement wrt to algorithm end effector.

    std::vector<Eigen::Vector3d> com;     // Subtree com position.
    std::vector<Eigen::Vector3d> vcom;    // Subtree com velocity.
    std::vector<Eigen::Vector3d> acom;    // Subtree com acceleration.
    std::vector<double> mass;             // Subtree total mass.
    Eigen::Matrix<double,3,Eigen::Dynamic> Jcom; // Jacobian of center of mass.

    Eigen::VectorXd effortLimit;          // Joint max effort
    Eigen::VectorXd velocityLimit;        // Joint max velocity

    Eigen::VectorXd lowerPositionLimit;   // limit for joint lower position
    Eigen::VectorXd upperPositionLimit;   // limit for joint upper position

    Data( const Model& ref );

  private:
    void computeLastChild(const Model& model);
    void computeParents_fromRow(const Model& model);

  };

  const Eigen::Vector3d Model::gravity981 (0,0,-9.81);

} // namespace se3



/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
namespace se3
{
  std::ostream& operator<< ( std::ostream & os, const Model& model )
  {
    os << "Nb bodies = " << model.nbody << " (nq="<< model.nq<<",nv="<<model.nv<<")" << std::endl;
    for(Model::Index i=0;i<(Model::Index)(model.nbody);++i)
      {
	os << "  Joint "<<model.names[i] << ": parent=" << model.parents[i] 
	   << ( model.hasVisual[i] ? " (has visual) " : "(doesnt have visual)" ) << std::endl;
      }

    return os;
  }

  std::string random(const int len)
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
  Model::Index Model::addBody( Index parent,const JointModelBase<D> & j,const SE3 & placement,
             const Inertia & Y,const std::string & jointName, 
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
  Model::Index Model::addBody( Index parent,const JointModelBase<D> & j,const SE3 & placement,
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

  Model::Index Model::addFixedBody( Index lastMovingParent,
                                    const SE3 & placementFromLastMoving,
                                    const std::string & bodyName,
                                    bool visual )
  {

    Model::Index idx = (Model::Index) (nFixBody++);
    fix_lastMovingParent.push_back(lastMovingParent);
    fix_lmpMi      .push_back(placementFromLastMoving);
    fix_hasVisual  .push_back(visual);
    fix_bodyNames  .push_back( (bodyName!="")?bodyName:random(8));
    return idx;
  }

  void Model::mergeFixedBody(Index parent, const SE3 & placement, const Inertia & Y)
  {
    const Inertia & iYf = Y.se3Action(placement); //TODO
    inertias[parent] += iYf;
  }

  Model::Index Model::getBodyId( const std::string & name ) const
  {
    std::vector<std::string>::iterator::difference_type
      res = std::find(names.begin(),names.end(),name) - names.begin();
    assert( (res<INT_MAX) && "Id superior to int range. Should never happen.");
    assert( (res>=0)&&(res<nbody)&&"The body name you asked do not exist" );
    return Model::Index(res);
  }
  bool Model::existBodyName( const std::string & name ) const
  {
    std::vector<std::string>::iterator::difference_type
      res = std::find(names.begin(),names.end(),name) - names.begin();
    return (res>=0)&&(res<nbody);
  }
  
  const std::string& Model::getBodyName( Model::Index index ) const
  {
    assert( (index>=0)&&(index < (Model::Index)nbody) );
    return names[index];
  }  

  Data::Data( const Model& ref )
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
    for(Model::Index i=0;i<(Model::Index)(model.nbody);++i) 
      joints.push_back(CreateJointData::run(model.joints[i]));

    /* Init for CRBA */
    M.fill(NAN);    
    for(Model::Index i=0;i<(Model::Index)(ref.nbody);++i ) { Fcrb[i].resize(6,model.nv); Fcrb[i].fill(NAN); }
    computeLastChild(ref);

    /* Init for Cholesky */
    U = Eigen::MatrixXd::Identity(ref.nv,ref.nv);
    computeParents_fromRow(ref);

    /* Init Jacobian */
    J.fill(0);
  }

  void Data::computeLastChild(const Model& model)
  {
    typedef Model::Index Index;
    std::fill(lastChild.begin(),lastChild.end(),-1);
    for( int i=model.nbody-1;i>=0;--i )
      {
	if(lastChild[(Model::Index)i] == -1) lastChild[(Model::Index)i] = i;
	const Index & parent = model.parents[(Model::Index)i];
	lastChild[parent] = std::max(lastChild[(Model::Index)i],lastChild[parent]);

	nvSubtree[(Model::Index)i] 
	  = idx_v(model.joints[(Model::Index)lastChild[(Model::Index)i]]) + nv(model.joints[(Model::Index)lastChild[(Model::Index)i]])
	  - idx_v(model.joints[(Model::Index)i]);
      }
  }

  void Data::computeParents_fromRow( const Model& model )
  {
    for( Model::Index joint=1;joint<(Model::Index)(model.nbody);joint++)
      {
	const Model::Index & parent = model.parents[joint];
	const int nvj    = nv   (model.joints[joint]);
	const int idx_vj = idx_v(model.joints[joint]);

	if(parent>0) parents_fromRow[(Model::Index)idx_vj] = idx_v(model.joints[parent])+nv(model.joints[parent])-1;
	else         parents_fromRow[(Model::Index)idx_vj] = -1;
	nvSubtree_fromRow[(Model::Index)idx_vj] = nvSubtree[joint];

    for( int row=1;row<nvj;++row)
	  {
	    parents_fromRow[(Model::Index)(idx_vj+row)] = idx_vj+row-1;
	    nvSubtree_fromRow[(Model::Index)(idx_vj+row)] = nvSubtree[joint]-row;
	  }
      }
  }

} // namespace se3

#endif // ifndef __se3_model_hpp__
