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

#ifndef __se3_model_hpp__
#define __se3_model_hpp__


#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/force.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/spatial/frame.hpp"
#include "pinocchio/multibody/joint/joint-variant.hpp"
#include <iostream>

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(se3::SE3)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(se3::Inertia)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(se3::Force)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(se3::Motion)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix<double,6,Eigen::Dynamic>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(se3::SE3::Vector3)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(se3::Frame)

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
    int nOperationalFrames;                     // Number of extra frames

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

    std::vector<Frame> operational_frames;

    Motion gravity;                       // Spatial gravity
    static const Eigen::Vector3d gravity981; // Default 3D gravity (=(0,0,-9.81))

    Model()
      : nq(0)
      , nv(0)
      , nbody(1)
      , nFixBody(0)
      , nOperationalFrames(0)
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
    Index getJointId( const std::string & name ) const;
    bool existJointName( const std::string & name ) const;
    const std::string& getJointName( Index index ) const;

    Index getFrameId ( const std::string & name ) const;
    bool existFrame ( const std::string & name ) const;
    const std::string & getFrameName ( Index index ) const;
    const Index& getFrameParent( const std::string & name ) const;
    const Index& getFrameParent( Index index ) const;
    const SE3 & getJointToFrameTransform( const std::string & name ) const;
    const SE3 & getJointToFrameTransform( Index index ) const;

    bool addFrame ( const Frame & frame );
    bool addFrame ( const std::string & name, Index index, const SE3 & placement );

  };

  class Data
  {
  public:
    typedef Eigen::Matrix<double,6,Eigen::Dynamic> Matrix6x;
    typedef Eigen::Matrix<double,3,Eigen::Dynamic> Matrix3x;
    typedef SE3::Vector3 Vector3;
    
  public:
    const Model& model;
    JointDataVector joints;
    std::vector<Motion> a;                // Body acceleration
    std::vector<Motion> a_gf;             // Body acceleration with gravity
    std::vector<Motion> v;                // Body velocity
    std::vector<Force> f;                 // Body force
    std::vector<SE3> oMi;                 // Body absolute placement (wrt world)
    std::vector<SE3> liMi;                // Body relative placement (wrt parent)
    Eigen::VectorXd tau;                  // Joint forces
    Eigen::VectorXd nle;                  // Non linear effects

    std::vector<SE3> oMof;                // Absolute position of extra frames

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
    
    Matrix6x J;                    // Jacobian of joint placement
    std::vector<SE3> iMf;                 // Body placement wrt to algorithm end effector.

    std::vector<Eigen::Vector3d> com;     // Subtree com position.
    std::vector<Eigen::Vector3d> vcom;    // Subtree com velocity.
    std::vector<Eigen::Vector3d> acom;    // Subtree com acceleration.
    std::vector<double> mass;             // Subtree total mass.
    Matrix3x Jcom; // Jacobian of center of mass.

    Eigen::VectorXd effortLimit;          // Joint max effort
    Eigen::VectorXd velocityLimit;        // Joint max velocity

    Eigen::VectorXd lowerPositionLimit;   // limit for joint lower position
    Eigen::VectorXd upperPositionLimit;   // limit for joint upper position
    
    double kinetic_energy; // kinetic energy of the model
    double potential_energy; // potential energy of the model

    Data( const Model& ref );

  private:
    void computeLastChild(const Model& model);
    void computeParents_fromRow(const Model& model);

  };

} // namespace se3

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
#include "pinocchio/multibody/model.hxx"

#endif // ifndef __se3_model_hpp__
