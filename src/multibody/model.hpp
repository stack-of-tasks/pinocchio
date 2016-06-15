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
#include <Eigen/Cholesky>

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(se3::SE3)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(se3::Inertia)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(se3::Force)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(se3::Motion)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix<double,6,Eigen::Dynamic>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix<double,6,6>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(se3::SE3::Vector3)

namespace se3
{
  class Model;
  class Data;
  
  class Model
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef se3::Index Index;
    typedef se3::JointIndex JointIndex;
    typedef se3::GeomIndex GeomIndex;
    typedef se3::FrameIndex FrameIndex;

    /// \brief Dimension of the configuration vector representation.
    int nq;
    
    /// \brief Dimension of the velocity vector space.
    int nv;
    
    /// \brief Number of joints .
    int njoint;

    /// \brief Number of bodies .
    int nbody;
    
    
    /// \brief Number of operational frames.
    int nOperationalFrames;

    /// \brief Spatial inertias of the body <i> expressed in the supporting joint frame <i>.
    std::vector<Inertia> inertias;
    
    /// \brief Placement (SE3) of the input of joint <i> regarding to the parent joint output <li>.
    std::vector<SE3> jointPlacements;

    /// \brief Placement (SE3) of the body <i> regarding to the parent joint output <li>.
    std::vector<SE3> bodyPlacements;
    
    /// \brief Model of joint <i>.
    JointModelVector joints;
    
    /// \brief Joint parent of joint <i>, denoted <li> (li==parents[i]).
    std::vector<JointIndex> parents;

    /// \brief Joint parent of body <i>
    std::vector<JointIndex> bodyParents;
    
    /// \brief Name of joint <i>
    std::vector<std::string> names;
    
    /// \brief Name of the body attached to the output of joint <i>.
    std::vector<std::string> bodyNames;
    
    /// \brief Vector of maximal joint torques
    Eigen::VectorXd effortLimit;
    /// \brief Vector of maximal joint velocities
    Eigen::VectorXd velocityLimit;

    /// \brief Lower joint configuration limit
    Eigen::VectorXd lowerPositionLimit;
    /// \brief Upper joint configuration limit
    Eigen::VectorXd upperPositionLimit;

    /// \brief Vector of operational frames registered on the model.
    std::vector<Frame> operational_frames;

    /// \brief Spatial gravity
    Motion gravity;
    /// \brief Default 3D gravity vector (=(0,0,-9.81)).
    static const Eigen::Vector3d gravity981;

    /// \brief Default constructor
    Model()
      : nq(0)
      , nv(0)
      , njoint(1)
      , nbody(1)
      , nOperationalFrames(0)
      , inertias(1)
      , jointPlacements(1)
      , joints(1)
      , parents(1)
      , bodyParents(1)
      , names(1)
      , bodyNames(1)
      , gravity( gravity981,Eigen::Vector3d::Zero() )
    {
      names[0]     = "universe";     // Should be "universe joint (trivial)"
      bodyNames[0] = "universe";
    }
    ~Model() {} // std::cout << "Destroy model" << std::endl; }
    
    ///
    /// \brief Add a Joint along with body to the kinematic tree.
    ///
    /// \param[in] parent Index of the parent joint.
    /// \param[in] j The joint model.
    /// \param[in] jointPlacement The relative placement of the joint j regarding to the parent joint.
    /// \param[in] Y Spatial inertia of the body.
    /// \param[in] jointName Name of the joint.
    /// \param[in] bodyName Name of the body.
    ///
    /// \return The index of the new added joint.
    ///
    template<typename D>
    JointIndex addJointAndBody(JointIndex parent, const JointModelBase<D> & j, const SE3 & jointPlacement,
                               const Inertia & Y, const std::string & jointName = "",
                               const std::string & bodyName = "");
    
    ///
    /// \brief Add a Joint along with body to the kinematic tree, specifying limits
    ///
    /// \param[in] parent Index of the parent joint.
    /// \param[in] j The joint model.
    /// \param[in] jointPlacement The relative placement of the joint j regarding to the parent joint.
    /// \param[in] Y Spatial inertia of the body.
    /// \param[in] effort Maximal joint torque.
    /// \param[in] velocity Maximal joint velocity.
    /// \param[in] lowPos Lower joint configuration.
    /// \param[in] upPos Upper joint configuration.
    /// \param[in] jointName Name of the joint.
    /// \param[in] bodyName Name of the body.
    ///
    /// \return The index of the new added joint.
    ///
    template<typename D>
    JointIndex addJointAndBody(JointIndex parent,const JointModelBase<D> & j,const SE3 & jointPlacement,
                               const Inertia & Y,
                               const Eigen::VectorXd & effort, const Eigen::VectorXd & velocity,
                               const Eigen::VectorXd & lowPos, const Eigen::VectorXd & upPos,
                               const std::string & jointName = "", const std::string & bodyName = "");
    
    ///
    /// \brief Add a Joint with no body (Zero inertia) to the kinematic tree.
    ///
    /// \param[in] parent Index of the parent joint.
    /// \param[in] j The joint model.
    /// \param[in] jointPlacement The relative placement of the joint j regarding to the parent joint.
    /// \param[in] jointName Name of the joint.
    ///
    /// \return The index of the new added joint.
    ///
    template<typename D>
    JointIndex addJoint(JointIndex parent,const JointModelBase<D> & j,const SE3 & jointPlacement,
                        const std::string & jointName = "");

    ///
    /// \brief Add a Joint with no body (Zero inertia) to the kinematic tree, specifying limits
    ///
    /// \param[in] parent Index of the parent joint.
    /// \param[in] j The joint model.
    /// \param[in] jointPlacement The relative placement of the joint j regarding to the parent joint.
    /// \param[in] effort Maximal joint torque.
    /// \param[in] velocity Maximal joint velocity.
    /// \param[in] lowPos Lower joint configuration.
    /// \param[in] upPos Upper joint configuration.
    /// \param[in] jointName Name of the joint.
    ///
    /// \return The index of the new added joint.
    ///
    template<typename D>
    JointIndex addJoint(JointIndex parent,const JointModelBase<D> & j,const SE3 & jointPlacement,
                       const Eigen::VectorXd & effort, const Eigen::VectorXd & velocity,
                       const Eigen::VectorXd & lowPos, const Eigen::VectorXd & upPos,
                       const std::string & jointName = "");



    ///
    /// \brief Append a body to a given Joint of the kinematic tree.
    ///
    /// \param[in] parent Index of the parent joint.
    /// \param[in] bodyPlacement The relative placement of the body regarding to the parent joint.
    /// \param[in] Y Spatial inertia of the body.
    /// \param[in] bodyName Name of the body.
    ///
    ///
    void appendBodyToJoint (const JointIndex parent, const SE3 & bodyPlacement, const Inertia & Y,
                            const std::string & bodyName = "");

    
    ///
    /// \brief Return the index of a body given by its name.
    ///
    /// \param[in] name Name of the body.
    ///
    /// \return Index of the body.
    ///
    JointIndex getBodyId(const std::string & name) const;
    
    ///
    /// \brief Check if a body given by its name exists.
    ///
    /// \param[in] name Name of the body.
    ///
    /// \return True if the body exists in the kinematic tree.
    ///
    bool existBodyName(const std::string & name) const;
    
    ///
    /// \brief Get the name of a body given by its index.
    ///
    /// \param[in] index Index of the body.
    ///
    /// \return Name of the body.
    ///
    const std::string & getBodyName(const JointIndex index) const;
    
    ///
    /// \brief Return the index of a joint given by its name.
    ///
    /// \param[in] index Index of the joint.
    ///
    /// \return Index of the joint.
    ///
    JointIndex getJointId(const std::string & name) const;
    
    ///
    /// \brief Check if a joint given by its name exists.
    ///
    /// \param[in] name Name of the joint.
    ///
    /// \return True if the joint exists in the kinematic tree.
    ///
    bool existJointName(const std::string & name) const;
    
    ///
    /// \brief Get the name of a joint given by its index.
    ///
    /// \param[in] index Index of the joint.
    ///
    /// \return Name of the joint.
    ///
    const std::string & getJointName(const JointIndex index) const;

    ///
    /// \brief Return the index of a frame given by its name.
    ///
    /// \param[in] index Index of the frame.
    ///
    /// \return Index of the frame.
    ///
    FrameIndex getFrameId (const std::string & name) const;
    
    ///
    /// \brief Check if a frame given by its name exists.
    ///
    /// \param[in] name Name of the frame.
    ///
    /// \return Return true if the frame exists.
    ///
    bool existFrame (const std::string & name) const;
    
    ///
    /// \brief Get the name of a frame given by its index.
    ///
    /// \param[in] index Index of the frame.
    ///
    /// \return The name of the frame.
    ///
    const std::string & getFrameName (const FrameIndex index) const;
    
    ///
    /// \brief Get the index of the joint supporting the frame given by its name.
    ///
    /// \param[in] name Name of the frame.
    ///
    /// \return
    ///
    JointIndex getFrameParent(const std::string & name) const;
    
    ///
    /// \brief Get the index of the joint supporting the frame given by its index.
    ///
    /// \param[in] index Index of the frame.
    ///
    /// \return
    ///
    JointIndex getFrameParent(const FrameIndex index) const;
    
    ///
    /// \brief Return the relative placement between a frame and its supporting joint.
    ///
    /// \param[in] name Name of the frame.
    ///
    /// \return The frame placement regarding the supporing joint.
    ///
    const SE3 & getFramePlacement(const std::string & name) const;
    
    ///
    /// \brief Return the relative placement between a frame and its supporting joint.
    ///
    /// \param[in] index Index of the frame.
    ///
    /// \return The frame placement regarding the supporing joint.
    ///
    const SE3 & getFramePlacement(const FrameIndex index) const;

    ///
    /// \brief Add a frame to the kinematic tree.
    ///
    /// \param[in] frame The frame to add to the kinematic tree.
    ///
    /// \return Return true if the frame has been successfully added.
    ///
    bool addFrame(const Frame & frame);
    
    ///
    /// \brief Create and add a frame to the kinematic tree.
    ///
    /// \param[in] name Name of the frame.
    /// \param[in] parent Index of the supporting joint.
    /// \param[in] placement Placement of the frame regarding to the joint frame.
    ///
    /// \return Return true if the frame has been successfully added.
    ///
    bool addFrame(const std::string & name, const JointIndex parent, const SE3 & placement);

  };

  class Data
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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
    /// \note This Jacobian maps the joint velocity vector to the velocity of the center of mass, expressed in the inertial frame. In other words, \f$ v_{\text{CoM}} = J_{\text{CoM}} \dot{q}\f$.
    Matrix3x Jcom;

    
    /// \brief Kinetic energy of the model.
    double kinetic_energy;
    
    /// \brief Potential energy of the model.
    double potential_energy;
    
    // Temporary variables used in forward dynamics
    
    /// \brief Inverse of the operational-space inertia matrix
    Eigen::MatrixXd JMinvJt;
    
    /// \brief Cholesky decompostion of \JMinvJt.
    Eigen::LLT<Eigen::MatrixXd> llt_JMinvJt;
    
    /// \brief Lagrange Multipliers corresponding to the contact forces in se3::forwardDynamics.
    Eigen::VectorXd lambda_c;
    
    /// \brief Temporary corresponding to \f$ \sqrt{D} U^{-1} J^{\top} \f$.
    Eigen::MatrixXd sDUiJt;
    
    /// \brief Temporary corresponding to the residual torque \f$ \tau - b(q,\dot{q}) \f$.
    Eigen::VectorXd torque_residual;
    
    /// \brief Generalized velocity after impact.
    Eigen::VectorXd dq_after;
    
    /// \brief Lagrange Multipliers corresponding to the contact impulses in se3::impulseDynamics.
    Eigen::VectorXd impulse_c;
    
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
