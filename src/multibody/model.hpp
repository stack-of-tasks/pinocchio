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
#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/multibody/frame.hpp"
#include "pinocchio/multibody/joint/joint.hpp"
#include "pinocchio/multibody/joint/joint-composite.hpp"
#include "pinocchio/deprecated.hh"
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
  struct Model
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef se3::Index Index;
    typedef se3::JointIndex JointIndex;
    typedef se3::GeomIndex GeomIndex;
    typedef se3::FrameIndex FrameIndex;
    typedef std::vector<Index> IndexVector;

    /// \brief Dimension of the configuration vector representation.
    int nq;
    
    /// \brief Dimension of the velocity vector space.
    int nv;
    
    /// \brief Number of joints.
    int njoint;

    /// \brief Number of bodies.
    int nbody;
    
    /// \brief Number of operational frames.
    int nFrames;

    /// \brief Spatial inertias of the body <i> expressed in the supporting joint frame <i>.
    std::vector<Inertia> inertias;
    
    /// \brief Placement (SE3) of the input of joint <i> regarding to the parent joint output <li>.
    std::vector<SE3> jointPlacements;

    /// \brief Model of joint <i>, encapsulated in a JointModelAccessor.
    JointModelVector joints;
    
    /// \brief Joint parent of joint <i>, denoted <li> (li==parents[i]).
    std::vector<JointIndex> parents;

    /// \brief Name of joint <i>
    std::vector<std::string> names;
    
    /// \brief Vector of joint's neutral configurations
    Eigen::VectorXd neutralConfiguration;

    /// \brief Vector of maximal joint torques
    Eigen::VectorXd effortLimit;
    /// \brief Vector of maximal joint velocities
    Eigen::VectorXd velocityLimit;

    /// \brief Lower joint configuration limit
    Eigen::VectorXd lowerPositionLimit;
    /// \brief Upper joint configuration limit
    Eigen::VectorXd upperPositionLimit;

    /// \brief Vector of operational frames registered on the model.
    std::vector<Frame> frames;
    
    /// \brief Vector of subtrees.
    /// subtree[j] corresponds to the subtree supported by the joint j.
    /// The first element of subtree[j] is the index of the joint j itself.
    std::vector<IndexVector> subtrees;

    /// \brief Spatial gravity of the model.
    Motion gravity;
    
    /// \brief Default 3D gravity vector (=(0,0,-9.81)).
    static const Eigen::Vector3d gravity981;

    /// \brief Default constructor. Builds an empty model with no joints.
    Model()
      : nq(0)
      , nv(0)
      , njoint(1)
      , nbody(1)
      , nFrames(0)
      , inertias(1)
      , jointPlacements(1)
      , joints(1)
      , parents(1)
      , names(1)
      , subtrees(1)
      , gravity( gravity981,Eigen::Vector3d::Zero() )
    {
      names[0]     = "universe";     // Should be "universe joint (trivial)"
    }
    ~Model() {} // std::cout << "Destroy model" << std::endl; }
    
    ///
    /// \brief Add a joint to the kinematic tree.
    ///
    /// \remark This method also adds a Frame of same name to the vector of frames.
    /// \remark The inertia supported by the joint is set to Zero.
    ///
    /// \tparam JointModelDerived The type of the joint model.
    ///
    /// \param[in] parent Index of the parent joint.
    /// \param[in] joint_model The joint model.
    /// \param[in] joint_placement Placement of the joint inside its parent joint.
    /// \param[in] joint_name Name of the joint. If empty, the name is random.
    /// \param[in] max_effort Maximal joint torque. (Default set to infinity).
    /// \param[in] max_velocity Maximal joint velocity. (Default set to infinity).
    /// \param[in] min_config Lower joint configuration. (Default set to infinity).
    /// \param[in] max_config Upper joint configuration. (Default set to infinity).
    ///
    /// \return The index of the new joint.
    ///
    /// \sa Model::appendBodyToJoint
    ///
    template<typename JointModelDerived>
    JointIndex addJoint(const JointIndex parent, const JointModelBase<JointModelDerived> & joint_model, const SE3 & joint_placement,
                        const std::string & joint_name = "",
                        const Eigen::VectorXd & max_effort = Eigen::VectorXd::Constant(JointModelDerived::NV,std::numeric_limits<double>::max()),
                        const Eigen::VectorXd & max_velocity = Eigen::VectorXd::Constant(JointModelDerived::NV,std::numeric_limits<double>::max()),
                        const Eigen::VectorXd & min_config = Eigen::VectorXd::Constant(JointModelDerived::NQ,std::numeric_limits<double>::min()),
                        const Eigen::VectorXd & max_config = Eigen::VectorXd::Constant(JointModelDerived::NQ,std::numeric_limits<double>::max())
                        );

    ///
    /// \brief Append a body to a given joint of the kinematic tree.
    ///
    /// \remark This method also adds a Frame of same name to the vector of frames.
    ///
    /// \param[in] joint_index Index of the supporting joint.
    /// \param[in] Y Spatial inertia of the body.
    /// \param[in] body_placement The relative placement of the body regarding to the parent joint. Set default to the Identity placement.
    /// \param[in] body_name Name of the body. If empty, the name is random.
    ///
    /// \sa Model::addJoint
    ///
    void appendBodyToJoint(const JointIndex joint_index, const Inertia & Y,
                           const SE3 & body_placement = SE3::Identity(),
                           const std::string & body_name = "");

    ///
    /// \brief Return the index of a body given by its name.
    /// 
    /// \warning If no body is found, return the number of elements at time T.
    /// This can lead to errors if the model is expanded after this method is called
    /// (for example to get the id of a parent body)
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
    /// \warning If no joint is found, return the number of elements at time T.
    /// This can lead to errors if the model is expanded after this method is called
    /// (for example to get the id of a parent joint)
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
    /// \brief Returns the index of a frame given by its name.
    ///        \sa Model::existFrame to check if the frame exists or not.
    ///
    /// \warning If no frame is found, returns the size of the vector of Model::frames.
    /// This can lead to errors if the model is expanded after this method is called
    /// (for example to get the id of a parent frame).
    /// 
    /// \param[in] name Name of the frame.
    ///
    /// \return Index of the frame.
    ///
    FrameIndex getFrameId (const std::string & name) const;
    
    ///
    /// \brief Checks if a frame given by its name exists.
    ///
    /// \param[in] name Name of the frame.
    ///
    /// \return Returns true if the frame exists.
    ///
    bool existFrame (const std::string & name) const;
    
    ///
    /// \brief Get the name of a frame given by its index.
    ///
    /// \param[in] index Index of the frame.
    ///
    /// \return The name of the frame.
    ///
    PINOCCHIO_DEPRECATED const std::string & getFrameName (const FrameIndex index) const;
    
    ///
    /// \brief Get the index of the joint supporting the frame given by its name.
    ///
    /// \param[in] name Name of the frame.
    ///
    /// \return
    ///
    PINOCCHIO_DEPRECATED JointIndex getFrameParent(const std::string & name) const;
    
    ///
    /// \brief Get the index of the joint supporting the frame given by its index.
    ///
    /// \param[in] index Index of the frame.
    ///
    /// \return
    ///
    PINOCCHIO_DEPRECATED JointIndex getFrameParent(const FrameIndex index) const;

    ///
    /// \brief Get the type of the frame given by its index.
    ///
    /// \param[in] name Name of the frame.
    ///
    /// \return
    ///
    PINOCCHIO_DEPRECATED FrameType getFrameType(const std::string & name) const;
    
    ///
    /// \brief Get the type of the frame given by its index.
    ///
    /// \param[in] index Index of the frame.
    ///
    /// \return
    ///
    PINOCCHIO_DEPRECATED FrameType getFrameType(const FrameIndex index) const;
    
    ///
    /// \brief Return the relative placement between a frame and its supporting joint.
    ///
    /// \param[in] name Name of the frame.
    ///
    /// \return The frame placement regarding the supporing joint.
    ///
    PINOCCHIO_DEPRECATED const SE3 & getFramePlacement(const std::string & name) const;
    
    ///
    /// \brief Return the relative placement between a frame and its supporting joint.
    ///
    /// \param[in] index Index of the frame.
    ///
    /// \return The frame placement regarding the supporing joint.
    ///
    PINOCCHIO_DEPRECATED const SE3 & getFramePlacement(const FrameIndex index) const;

    ///
    /// \brief Adds a frame to the kinematic tree.
    ///
    /// \param[in] frame The frame to add to the kinematic tree.
    ///
    /// \return Returns true if the frame has been successfully added.
    ///
    bool addFrame(const Frame & frame);
    
    ///
    /// \brief Creates and adds a frame to the kinematic tree.
    ///
    /// \param[in] name Name of the frame.
    /// \param[in] parent Index of the supporting joint.
    /// \param[in] placement Placement of the frame regarding to the joint frame.
    /// \param[in] type The type of the frame
    ///
    /// \return Returns true if the frame has been successfully added.
    ///
    PINOCCHIO_DEPRECATED bool addFrame(const std::string & name, const JointIndex parent, const SE3 & placement, const FrameType type = OP_FRAME);

  protected:
    
    /// \brief Add the joint_id to its parent subtrees.
    ///
    /// \param[in] joint_id The id of the joint to add to the subtrees
    ///
    void addJointIndexToParentSubtrees(const JointIndex joint_id);
  };

  struct Data
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /// \brief The 6d jacobian type (temporary)
    typedef Eigen::Matrix<double,6,Eigen::Dynamic> Matrix6x;
    /// \brief The 3d jacobian type (temporary)
    typedef Eigen::Matrix<double,3,Eigen::Dynamic> Matrix3x;
    typedef SE3::Vector3 Vector3;
    
    /// \brief Vector of se3::JointData associated to the se3::JointModel stored in model, encapsulated in JointDataAccessor.
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
    std::vector<SE3> oMf;

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
