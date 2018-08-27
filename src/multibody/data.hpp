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

#ifndef __se3_data_hpp__
#define __se3_data_hpp__

#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/force.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/multibody/frame.hpp"
#include "pinocchio/multibody/joint/joint.hpp"
#include "pinocchio/deprecated.hh"
#include "pinocchio/utils/string-generator.hpp"
#include "pinocchio/container/aligned-vector.hpp"

#include <iostream>
#include <Eigen/Cholesky>

namespace se3
{
  
  struct Data
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    /// \brief The 6d jacobian type (temporary)
    typedef Eigen::Matrix<double,6,Eigen::Dynamic> Matrix6x;
    /// \brief The 3d jacobian type (temporary)
    typedef Eigen::Matrix<double,3,Eigen::Dynamic> Matrix3x;
    typedef SE3::Vector3 Vector3;
    
    typedef Eigen::Matrix<double,6,6,Eigen::RowMajor> RowMatrix6;
    typedef Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> RowMatrixXd;
    
    /// \brief Vector of se3::JointData associated to the se3::JointModel stored in model, 
    /// encapsulated in JointDataAccessor.
    JointDataVector joints;
    
    /// \brief Vector of joint accelerations expressed at the centers of the joints.
    container::aligned_vector<Motion> a;
    
    /// \brief Vector of joint accelerations expressed at the origin.
    container::aligned_vector<Motion> oa;
    
    /// \brief Vector of joint accelerations due to the gravity field.
    container::aligned_vector<Motion> a_gf;
    
    /// \brief Vector of joint velocities expressed at the centers of the joints.
    container::aligned_vector<Motion> v;
    
    /// \brief Vector of joint velocities expressed at the origin.
    container::aligned_vector<Motion> ov;
    
    /// \brief Vector of body forces expressed in the local frame of the joint. For each body, the force represents the sum of
    ///        all external forces acting on the body.
    container::aligned_vector<Force> f;
    
    /// \brief Vector of body forces expressed in the world frame. For each body, the force represents the sum of
    ///        all external forces acting on the body.
    container::aligned_vector<Force> of;
    
    /// \brief Vector of spatial momenta expressed in the local frame of the joint.
    container::aligned_vector<Force> h;
    
    /// \brief Vector of spatial momenta expressed in the world frame.
    container::aligned_vector<Force> oh;
    
    /// \brief Vector of absolute joint placements (wrt the world).
    container::aligned_vector<SE3> oMi;

    /// \brief Vector of relative joint placements (wrt the body parent).
    container::aligned_vector<SE3> liMi;
    
    /// \brief Vector of joint torques (dim model.nv).
    Eigen::VectorXd tau;
    
    /// \brief Vector of Non Linear Effects (dim model.nv). It corresponds to concatenation of the Coriolis, centrifugal and gravitational effects.
    /// \note  In the multibody dynamics equation \f$ M\ddot{q} + b(q, \dot{q}) = \tau \f$,
    ///        the non linear effects are associated to the term \f$b\f$.
    Eigen::VectorXd nle;
    
    /// \brief Vector of generalized gravity (dim model.nv).
    /// \note  In the multibody dynamics equation \f$ M\ddot{q} + c(q, \dot{q}) + g(q) = \tau \f$,
    ///        the gravity effect is associated to the \f$g\f$ term.
    Eigen::VectorXd g;

    /// \brief Vector of absolute operationnel frame placements (wrt the world).
    container::aligned_vector<SE3> oMf;

    /// \brief Vector of sub-tree composite rigid body inertias, i.e. the apparent inertia of the subtree supported by the joint
    ///        and expressed in the local frame of the joint..
    container::aligned_vector<Inertia> Ycrb;
    
    /// \brief Vector of sub-tree composite rigid body inertia time derivatives \f$ \dot{Y}_{crb}\f$. See Data::Ycrb for more details.
    container::aligned_vector<Inertia::Matrix6> dYcrb; // TODO: change with dense symmetric matrix6
    
    /// \brief The joint space inertia matrix (a square matrix of dim model.nv).
    Eigen::MatrixXd M;
    
    /// \brief The inverse of the joint space inertia matrix (a square matrix of dim model.nv).
    RowMatrixXd Minv;
    
    /// \brief The Coriolis matrix (a square matrix of dim model.nv).
    Eigen::MatrixXd C;
    
    /// \brief Variation of the forceset with respect to the joint configuration.
    Matrix6x dFdq;
    
    /// \brief Variation of the forceset with respect to the joint velocity.
    Matrix6x dFdv;
    
    /// \brief Variation of the forceset with respect to the joint acceleration.
    Matrix6x dFda;

    /// \brief Used in computeMinverse
    Matrix6x SDinv;

    /// \brief Used in computeMinverse
    Matrix6x UDinv;

    /// \brief Used in computeMinverse
    Matrix6x IS;

    /// \brief Right variation of the inertia matrix
    container::aligned_vector<Inertia::Matrix6> vxI;
    
    /// \brief Left variation of the inertia matrix
    container::aligned_vector<Inertia::Matrix6> Ivx;
    
    /// \brief Inertia quantities expressed in the world frame
    container::aligned_vector<Inertia> oYcrb;
    
    /// \brief Time variation of the inertia quantities expressed in the world frame
    container::aligned_vector<Inertia::Matrix6> doYcrb;
    
    /// \brief Temporary for derivative algorithms
    Inertia::Matrix6 Itmp;
    
    /// \brief Temporary for derivative algorithms
    RowMatrix6 M6tmpR;
    
    /// \brief The joint accelerations computed from ABA
    Eigen::VectorXd ddq;
    
    // ABA internal data
    /// \brief Inertia matrix of the subtree expressed as dense matrix [ABA]
    container::aligned_vector<Inertia::Matrix6> Yaba;  // TODO: change with dense symmetric matrix6
    
    /// \brief Intermediate quantity corresponding to apparent torque [ABA]
    Eigen::VectorXd u;                  // Joint Inertia
    
    // CCRBA return quantities
    /// \brief Centroidal Momentum Matrix
    /// \note \f$ hg = A_g \dot{q}\f$ maps the joint velocity set to the centroidal momentum.
    Matrix6x Ag;
    
    // dCCRBA return quantities
    /// \brief Centroidal Momentum Matrix Time Variation
    /// \note \f$ \dot{h_g} = A_g \ddot{q}\ + \dot{A_g}\dot{q}\f$ maps the joint velocity and acceleration vectors to the time variation of the centroidal momentum.
    Matrix6x dAg;
    
    /// \brief Centroidal momentum quantity.
    /// \note The centroidal momentum is expressed in the frame centered at the CoM and aligned with the inertial frame (i.e. the world frame).
    ///
    Force hg;
    
    /// \brief Centroidal Composite Rigid Body Inertia.
    /// \note \f$ hg = Ig v_{\text{mean}}\f$ map a mean velocity to the current centroil momentum quantity.
    Inertia Ig;

    /// \brief Spatial forces set, used in CRBA and CCRBA
    container::aligned_vector<Matrix6x> Fcrb;

    /// \brief Index of the last child (for CRBA)
    std::vector<int> lastChild;
    /// \brief Dimension of the subtree motion space (for CRBA)
    std::vector<int> nvSubtree;

    /// \brief Joint space intertia matrix square root (upper trianglular part) computed with a Cholesky Decomposition.
    Eigen::MatrixXd U;
    
    /// \brief Diagonal of the joint space intertia matrix obtained by a Cholesky Decomposition.
    Eigen::VectorXd D;
    
    /// \brief Diagonal inverse of the joint space intertia matrix obtained by a Cholesky Decomposition.
    Eigen::VectorXd Dinv;
    
    /// \brief Temporary of size NV used in Cholesky Decomposition.
    Eigen::VectorXd tmp;
    
    /// \brief First previous non-zero row in M (used in Cholesky Decomposition).
    std::vector<int> parents_fromRow;
    
    /// \brief Subtree of the current row index (used in Cholesky Decomposition).
    std::vector<int> nvSubtree_fromRow;
    
    /// \brief Jacobian of joint placements.
    /// \note The columns of J corresponds to the basis of the spatial velocities of each joint and expressed at the origin of the inertial frame. In other words, if \f$ v_{J_{i}} = S_{i} \dot{q}_{i}\f$ is the relative velocity of the joint i regarding to its parent, then \f$J = \begin{bmatrix} ^{0}X_{1} S_{1} & \cdots & ^{0}X_{i} S_{i} & \cdots & ^{0}X_{\text{nj}} S_{\text{nj}} \end{bmatrix} \f$. This Jacobian has no special meaning. To get the jacobian of a precise joint, you need to call se3::getJointJacobian
    Matrix6x J;
    
    /// \brief Derivative of the Jacobian with respect to the time.
    Matrix6x dJ;
    
    /// \brief Variation of the spatial velocity set with respect to the joint configuration.
    Matrix6x dVdq;
    
    /// \brief Variation of the spatial acceleration set with respect to the joint configuration.
    Matrix6x dAdq;
    
    /// \brief Variation of the spatial acceleration set with respect to the joint velocity.
    Matrix6x dAdv;
    
    /// \brief Partial derivative of the joint torque vector with respect to the joint configuration.
    Eigen::MatrixXd dtau_dq;
    
    /// \brief Partial derivative of the joint torque vector with respect to the joint velocity.
    Eigen::MatrixXd dtau_dv;
    
    /// \brief Partial derivative of the joint acceleration vector with respect to the joint configuration.
    Eigen::MatrixXd ddq_dq;
    
    /// \brief Partial derivative of the joint acceleration vector with respect to the joint velocity.
    Eigen::MatrixXd ddq_dv;
    
    /// \brief Vector of joint placements wrt to algorithm end effector.
    container::aligned_vector<SE3> iMf;

    /// \brief Vector of subtree center of mass positions expressed in the root joint of the subtree. In other words, com[j] is the CoM position of the subtree supported by joint \f$ j \f$ and expressed in the joint frame \f$ j \f$. The element com[0] corresponds to the center of mass position of the whole model and expressed in the global frame.
    container::aligned_vector<Eigen::Vector3d> com;
    
    /// \brief Vector of subtree center of mass linear velocities expressed in the root joint of the subtree. In other words, vcom[j] is the CoM linear velocity of the subtree supported by joint \f$ j \f$ and expressed in the joint frame \f$ j \f$. The element vcom[0] corresponds to the velocity of the CoM of the whole model expressed in the global frame.
    container::aligned_vector<Eigen::Vector3d> vcom;
    
    /// \brief Vector of subtree center of mass linear accelerations expressed in the root joint of the subtree. In other words, acom[j] is the CoM linear acceleration of the subtree supported by joint \f$ j \f$ and expressed in the joint frame \f$ j \f$. The element acom[0] corresponds to the acceleration of the CoM of the whole model expressed in the global frame.
    container::aligned_vector<Eigen::Vector3d> acom;
    
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
    
    /// \brief Cholesky decompostion of \f$\JMinvJt\f$.
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
    
    // data related to regressor
    Matrix3x staticRegressor;
    
    ///
    /// \brief Default constructor of se3::Data from a se3::Model.
    ///
    /// \param[in] model The model structure of the rigid body system.
    ///
    explicit Data(const Model & model);

  private:
    void computeLastChild(const Model& model);
    void computeParents_fromRow(const Model& model);

  };

} // namespace se3

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
#include "pinocchio/multibody/data.hxx"

#endif // ifndef __se3_data_hpp__

