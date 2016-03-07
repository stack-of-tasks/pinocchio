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
#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/multibody/joint/joint-variant.hpp"
#include "pinocchio/tools/string-generator.hpp"
#include <iostream>

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(se3::SE3)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(se3::Inertia)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(se3::Force)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(se3::Motion)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix<double,6,Eigen::Dynamic>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(se3::SE3::Vector3)

namespace se3
{
  class Model;
  class Data;
  
  class Model
  {
  public:
    typedef se3::Index Index;
    typedef se3::JointIndex JointIndex;
    typedef se3::GeomIndex GeomIndex;
    typedef se3::FrameIndex FrameIndex;

    int nq;                               // Dimension of the configuration representation
    int nv;                               // Dimension of the velocity vector space
    int nbody;                            // Number of bodies (= number of joints + 1)
    int nFixBody;                         // Number of fixed-bodies (= number of fixed-joints)
    int nOperationalFrames;               // Number of operational frames

    std::vector<Inertia> inertias;        // Spatial inertias of the body <i> in the supporting joint frame <i>
    std::vector<SE3> jointPlacements;     // Placement (SE3) of the input of joint <i> in parent joint output <li>
    JointModelVector joints;              // Model of joint <i>
    std::vector<JointIndex> parents;           // Joint parent of joint <i>, denoted <li> (li==parents[i])
    std::vector<std::string> names;       // Name of joint <i>
    std::vector<std::string> bodyNames;   // Name of the body attached to the output of joint <i>
    std::vector<bool> hasVisual;          // True iff body <i> has a visual mesh.

    std::vector<SE3> fix_lmpMi;           // Fixed-body relative placement (wrt last moving parent)
    std::vector<Model::JointIndex> fix_lastMovingParent; // Fixed-body index of the last moving parent
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
    JointIndex addBody(  JointIndex parent,const JointModelBase<D> & j,const SE3 & placement,
		                const Inertia & Y,
                    const std::string & jointName = "", const std::string & bodyName = "",
                    bool visual = false);
    template<typename D>
    JointIndex addBody(  JointIndex parent,const JointModelBase<D> & j,const SE3 & placement,
                    const Inertia & Y,
                    const Eigen::VectorXd & effort, const Eigen::VectorXd & velocity,
                    const Eigen::VectorXd & lowPos, const Eigen::VectorXd & upPos,
                    const std::string & jointName = "", const std::string & bodyName = "",
                    bool visual = false);
    JointIndex addFixedBody( JointIndex fix_lastMovingParent,
                        const SE3 & placementFromLastMoving,
                        const std::string &jointName = "",
                        bool visual=false);
    void mergeFixedBody(JointIndex parent, const SE3 & placement, const Inertia & Y);
    JointIndex getBodyId( const std::string & name ) const;
    bool existBodyName( const std::string & name ) const;
    const std::string& getBodyName( const JointIndex index ) const;
    JointIndex getJointId( const std::string & name ) const;
    bool existJointName( const std::string & name ) const;
    const std::string& getJointName( const JointIndex index ) const;

    FrameIndex getFrameId ( const std::string & name ) const;
    bool existFrame ( const std::string & name ) const;
    const std::string & getFrameName ( const FrameIndex index ) const;
    const JointIndex& getFrameParent( const std::string & name ) const;
    const JointIndex& getFrameParent( const FrameIndex index ) const;
    const SE3 & getFramePlacement( const std::string & name ) const;
    const SE3 & getFramePlacement( const FrameIndex index ) const;

    bool addFrame ( const Frame & frame );
    bool addFrame ( const std::string & name, JointIndex index_parent, const SE3 & placement );

  };

  class Data
  {
  public:
    /// \brief The 6d jacobian type (temporary)
    typedef Eigen::Matrix<double,6,Eigen::Dynamic> Matrix6x;
    /// \brief The 3d jacobian type (temporary)
    typedef Eigen::Matrix<double,3,Eigen::Dynamic> Matrix3x;
    typedef SE3::Vector3 Vector3;
    
  public:
    /// \brief A const reference to the reference model.
    const Model & model;
    
    /// \brief Vector of se3::JointData associated to the se3::JointModel stored in model.
    JointDataVector joints;
    
    /// \brief Vector of joint accelerations.
    std::vector<Motion> a;
    
    /// \brief Vector of joint accelerations due to the gravity field.
    std::vector<Motion> a_gf;
    
    /// \brief Vector of joint velocities.
    std::vector<Motion> v;
    
    /// \brief Vector of body forces. For each body, the force represents the sum of
    ///        all external forces acting on the body.
    std::vector<Force> f;
    
    /// \brief Vector of absolute joint placements (wrt the world).
    std::vector<SE3> oMi;

    /// \brief Vector of relative joint placements (wrt the body parent).
    std::vector<SE3> liMi;
    
    /// \brief Vector of joint torques (dim model.nv).
    Eigen::VectorXd tau;
    
    /// \brief Vector of Non Linear Effects (dim model.nv). It corresponds to the Coriolis, centrifugal and gravitational effects.
    /// \note  In the equation \f$ M\ddot{q} + b = \tau \f$,
    ///        the non linear effects are associated to the \f$b\f$ term.
    Eigen::VectorXd nle;

    /// \brief Vector of absolute operationnel frame placements (wrt the world).
    std::vector<SE3> oMof;

    /// \brief Vector of sub-tree composite rigid body inertias, i.e. the apparent inertia of the subtree supported by the joint.
    std::vector<Inertia> Ycrb;
    
    /// \brief The joint space inertia matrix (a square matrix of dim model.nv).
    Eigen::MatrixXd M;
    
    /// \brief The joint accelerations computed from ABA
    Eigen::VectorXd ddq;
    
    // ABA internal data
    /// \brief Inertia matrix of the subtree expressed as dense matrix [ABA]
    std::vector<Inertia::Matrix6> Yaba;
    
    /// \brief Intermediate quantity corresponding to apparent torque [ABA]
    Eigen::VectorXd u;                  // Joint Inertia
    
    // CCRBA return quantities
    /// \brief Centroidal Momentum Matrix
    /// \note \f$ hg = Ag \dot{q}\f$ maps the joint velocity set to the centroidal momentum.
    Matrix6x Ag;
    
    /// \brief Centroidal momentum quantity.
    /// \note The centroidal momentum is expressed in the frame centered at the CoM and aligned with the inertial frame.
    ///
    Force hg;
    
    /// \brief Centroidal Composite Rigid Body Inertia.
    /// \note \f$ hg = Ig v_{\text{mean}}\f$ map a mean velocity to the current centroil momentum quantity.
    Inertia Ig;

    /// \brief Spatial forces set, used in CRBA and CCRBA
    std::vector<Matrix6x> Fcrb;

    /// \brief Index of the last child (for CRBA)
    std::vector<int> lastChild;
    /// \brief Dimension of the subtree motion space (for CRBA)
    std::vector<int> nvSubtree;

    /// \brief Joint space intertia matrix square root (upper trianglular part) computed with a Cholesky Decomposition.
    Eigen::MatrixXd U;
    
    /// \brief Diagonal of the joint space intertia matrix obtained by a Cholesky Decomposition.
    Eigen::VectorXd D;
    
    /// \brief Temporary of size NV used in Cholesky Decomposition.
    Eigen::VectorXd tmp;
    
    /// \brief First previous non-zero row in M (used in Cholesky Decomposition).
    std::vector<int> parents_fromRow;
    
    /// \brief Subtree of the current row index (used in Cholesky Decomposition).
    std::vector<int> nvSubtree_fromRow;
    
    /// \brief Jacobian of joint placements.
    /// \note The columns of J corresponds to the basis of the spatial velocities of each joint and expressed at the origin of the inertial frame. In other words, if \f$ v_{J_{i}} = S_{i} \dot{q}_{i}\f$ is the relative velocity of the joint i regarding to its parent, then \f$J = \begin{bmatrix} ^{0}X_{1} S_{1} & \cdots & ^{0}X_{i} S_{i} & \cdots & ^{0}X_{\text{nj}} S_{\text{nj}} \end{bmatrix} \f$. This Jacobian has no special meaning. To get the jacobian of a precise joint, you need to call se3::getJacobian
    Matrix6x J;
    
    /// \brief Vector of joint placements wrt to algorithm end effector.
    std::vector<SE3> iMf;

    /// \brief Vector of subtree center of mass positions expressed in the root joint of the subtree. In other words, com[j] is the CoM position of the subtree supported by joint \f$ j \f$ and expressed in the joint frame \f$ j \f$. The element com[0] corresponds to the center of mass position of the whole model and expressed in the global frame.
    std::vector<Eigen::Vector3d> com;
    
    /// \brief Vector of subtree center of mass linear velocities expressed in the root joint of the subtree. In other words, vcom[j] is the CoM linear velocity of the subtree supported by joint \f$ j \f$ and expressed in the joint frame \f$ j \f$. The element vcom[0] corresponds to the velocity of the CoM of the whole model expressed in the global frame.
    std::vector<Eigen::Vector3d> vcom;
    
    /// \brief Vector of subtree center of mass linear accelerations expressed in the root joint of the subtree. In other words, acom[j] is the CoM linear acceleration of the subtree supported by joint \f$ j \f$ and expressed in the joint frame \f$ j \f$. The element acom[0] corresponds to the acceleration of the CoM of the whole model expressed in the global frame.
    std::vector<Eigen::Vector3d> acom;
    
    /// \brief Vector of subtree mass. In other words, mass[j] is the mass of the subtree supported by joint \f$ j \f$. The element mass[0] corrresponds to the total mass of the model.
    std::vector<double> mass;
    
    /// \brief Jacobien of center of mass.
    /// \note This Jacobian maps the joint velocity vector to the velocity of the center of mass, expressed in the inertia frame. In other words, \f$ v_{\text{CoM}} = J_{\text{CoM}} \dot{q}\f$.
    Matrix3x Jcom;

    /// \brief Vector of maximal joint torques
    Eigen::VectorXd effortLimit;
    /// \brief Vector of maximal joint velocities
    Eigen::VectorXd velocityLimit;

    /// \brief Lower joint configuration limit
    Eigen::VectorXd lowerPositionLimit;
    /// \brief Upper joint configuration limit
    Eigen::VectorXd upperPositionLimit;
    
    /// \brief Kinetic energy of the model.
    double kinetic_energy;
    
    /// \brief Potential energy of the model.
    double potential_energy;
    
    ///
    /// \brief Default constructor of se3::Data from a se3::Model.
    ///
    /// \param[in] model The model structure of the rigid body system.
    ///
    Data (const Model & model);

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
