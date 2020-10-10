//
// Copyright (c) 2015-2020 CNRS INRIA
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_multibody_data_hpp__
#define __pinocchio_multibody_data_hpp__

#include "pinocchio/math/tensor.hpp"

#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/force.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/multibody/joint/joint-generic.hpp"
#include "pinocchio/container/aligned-vector.hpp"

#include "pinocchio/serialization/serializable.hpp"

#include <iostream>
#include <Eigen/Cholesky>

namespace pinocchio
{
 
  template<typename _Scalar, int _Options, template<typename,int> class JointCollectionTpl>
  struct DataTpl
  : serialization::Serializable< DataTpl<_Scalar,_Options,JointCollectionTpl> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef _Scalar Scalar;
    enum { Options = _Options };
    
    typedef JointCollectionTpl<Scalar,Options> JointCollection;
    
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    
    typedef SE3Tpl<Scalar,Options> SE3;
    typedef MotionTpl<Scalar,Options> Motion;
    typedef ForceTpl<Scalar,Options> Force;
    typedef InertiaTpl<Scalar,Options> Inertia;
    typedef FrameTpl<Scalar,Options> Frame;
    
    typedef pinocchio::Index Index;
    typedef pinocchio::JointIndex JointIndex;
    typedef pinocchio::GeomIndex GeomIndex;
    typedef pinocchio::FrameIndex FrameIndex;
    typedef std::vector<Index> IndexVector;
    
    typedef JointModelTpl<Scalar,Options,JointCollectionTpl> JointModel;
    typedef JointDataTpl<Scalar,Options,JointCollectionTpl> JointData;
    
    typedef PINOCCHIO_ALIGNED_STD_VECTOR(JointModel) JointModelVector;
    typedef PINOCCHIO_ALIGNED_STD_VECTOR(JointData) JointDataVector;
    
    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic,Options> MatrixXs;
    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1,Options> VectorXs;
    typedef Eigen::Matrix<Scalar,3,1,Options> Vector3;
    
    /// \brief Dense vectorized version of a joint configuration vector.
    typedef VectorXs ConfigVectorType;
    
    /// \brief Dense vectorized version of a joint tangent vector (e.g. velocity, acceleration, etc).
    ///        It also handles the notion of co-tangent vector (e.g. torque, etc).
    typedef VectorXs TangentVectorType;
    
    /// \brief The 6d jacobian type (temporary)
    typedef Eigen::Matrix<Scalar,6,Eigen::Dynamic,Options> Matrix6x;
    /// \brief The 3d jacobian type (temporary)
    typedef Eigen::Matrix<Scalar,3,Eigen::Dynamic,Options> Matrix3x;
    
    typedef Eigen::Matrix<Scalar,6,6,Options> Matrix6;
    typedef Eigen::Matrix<Scalar,6,6,Eigen::RowMajor | Options> RowMatrix6;
    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor | Options> RowMatrixXs;

    /// \brief The type of the body regressor
    typedef Eigen::Matrix<Scalar,6,10,Options> BodyRegressorType;
    
    /// \brief The type of Tensor for Kinematics and Dynamics second order derivatives
    typedef Tensor<Scalar,3,Options> Tensor3x;

    /// \brief Vector of pinocchio::JointData associated to the pinocchio::JointModel stored in model, 
    /// encapsulated in JointDataAccessor.
    JointDataVector joints;
    
    /// \brief Vector of joint accelerations expressed at the centers of the joints frames.
    PINOCCHIO_ALIGNED_STD_VECTOR(Motion) a;
    
    /// \brief Vector of joint accelerations expressed at the origin of the world.
    PINOCCHIO_ALIGNED_STD_VECTOR(Motion) oa;
    
    /// \brief Vector of joint accelerations due to the gravity field.
    PINOCCHIO_ALIGNED_STD_VECTOR(Motion) a_gf;
    
    /// \brief Vector of joint accelerations expressed at the origin of the world including gravity contribution.
    PINOCCHIO_ALIGNED_STD_VECTOR(Motion) oa_gf;
    
    /// \brief Vector of joint velocities expressed at the centers of the joints.
    PINOCCHIO_ALIGNED_STD_VECTOR(Motion) v;
    
    /// \brief Vector of joint velocities expressed at the origin.
    PINOCCHIO_ALIGNED_STD_VECTOR(Motion) ov;
    
    /// \brief Vector of body forces expressed in the local frame of the joint. For each body, the force represents the sum of
    ///        all external forces acting on the body.
    PINOCCHIO_ALIGNED_STD_VECTOR(Force) f;
    
    /// \brief Vector of body forces expressed in the world frame. For each body, the force represents the sum of
    ///        all external forces acting on the body.
    PINOCCHIO_ALIGNED_STD_VECTOR(Force) of;
    
    /// \brief Vector of spatial momenta expressed in the local frame of the joint.
    PINOCCHIO_ALIGNED_STD_VECTOR(Force) h;
    
    /// \brief Vector of spatial momenta expressed in the world frame.
    PINOCCHIO_ALIGNED_STD_VECTOR(Force) oh;
    
    /// \brief Vector of absolute joint placements (wrt the world).
    PINOCCHIO_ALIGNED_STD_VECTOR(SE3) oMi;

    /// \brief Vector of relative joint placements (wrt the body parent).
    PINOCCHIO_ALIGNED_STD_VECTOR(SE3) liMi;
    
    /// \brief Vector of joint torques (dim model.nv).
    TangentVectorType tau;
    
    /// \brief Vector of Non Linear Effects (dim model.nv). It corresponds to concatenation of the Coriolis, centrifugal and gravitational effects.
    /// \note  In the multibody dynamics equation \f$ M\ddot{q} + b(q, \dot{q}) = \tau \f$,
    ///        the non linear effects are associated to the term \f$b\f$.
    VectorXs nle;
    
    /// \brief Vector of generalized gravity (dim model.nv).
    /// \note  In the multibody dynamics equation \f$ M\ddot{q} + c(q, \dot{q}) + g(q) = \tau \f$,
    ///        the gravity effect is associated to the \f$g\f$ term.
    VectorXs g;

    /// \brief Vector of absolute operationnel frame placements (wrt the world).
    PINOCCHIO_ALIGNED_STD_VECTOR(SE3) oMf;

    /// \brief Vector of sub-tree composite rigid body inertias, i.e. the apparent inertia of the subtree supported by the joint
    ///        and expressed in the local frame of the joint..
    PINOCCHIO_ALIGNED_STD_VECTOR(Inertia) Ycrb;
    
    /// \brief Vector of sub-tree composite rigid body inertia time derivatives \f$ \dot{Y}_{crb}\f$. See Data::Ycrb for more details.
    PINOCCHIO_ALIGNED_STD_VECTOR(Matrix6) dYcrb; // TODO: change with dense symmetric matrix6
    
    /// \brief The joint space inertia matrix (a square matrix of dim model.nv).
    MatrixXs M;
    
    /// \brief The inverse of the joint space inertia matrix (a square matrix of dim model.nv).
    RowMatrixXs Minv;
    
    /// \brief The Coriolis matrix (a square matrix of dim model.nv).
    MatrixXs C;

    /// \brief Variation of the spatial momenta with respect to the joint configuration.
    Matrix6x dHdq;
    
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
    PINOCCHIO_ALIGNED_STD_VECTOR(Matrix6) vxI;
    
    /// \brief Left variation of the inertia matrix
    PINOCCHIO_ALIGNED_STD_VECTOR(Matrix6) Ivx;
    
    /// \brief Rigid Body Inertia supported by the joint expressed in the world frame
    PINOCCHIO_ALIGNED_STD_VECTOR(Inertia) oinertias;
    
    /// \brief Composite Rigid Body Inertia expressed in the world frame
    PINOCCHIO_ALIGNED_STD_VECTOR(Inertia) oYcrb;
    
    /// \brief Time variation of Composite Rigid Body Inertia expressed in the world frame
    PINOCCHIO_ALIGNED_STD_VECTOR(Matrix6) doYcrb;
    
    /// \brief Temporary for derivative algorithms
    Matrix6 Itmp;
    
    /// \brief Temporary for derivative algorithms
    Matrix6 M6tmp;
    RowMatrix6 M6tmpR;
    RowMatrix6 M6tmpR2;
    
    /// \brief The joint accelerations computed from ABA
    TangentVectorType ddq;
    
    // ABA internal data
    /// \brief Inertia matrix of the subtree expressed as dense matrix [ABA]
    PINOCCHIO_ALIGNED_STD_VECTOR(Matrix6) Yaba;  // TODO: change with dense symmetric matrix6
    
    /// \brief Intermediate quantity corresponding to apparent torque [ABA]
    TangentVectorType u;                  // Joint Inertia
    
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
    /// \note \f$ h_g = \left( m\dot{c}, L_{g} \right); \f$. \f$ h_g \f$ is the stack of the linear momentum and the angular momemtum vectors.
    ///
    Force hg;
    
    /// \brief Centroidal momentum time derivative.
    /// \note The centroidal momentum time derivative is expressed in the frame centered at the CoM and aligned with the inertial frame (i.e. the world frame).
    /// \note \f$ \dot{h_g} = \left( m\ddot{c}, \dot{L}_{g} \right); \f$. \f$ \dot{h_g} \f$ is the stack of the linear momentum variation and the angular momemtum variation.
    ///
    Force dhg;
    
    /// \brief Centroidal Composite Rigid Body Inertia.
    /// \note \f$ hg = Ig v_{\text{mean}}\f$ map a mean velocity to the current centroil momentum quantity.
    Inertia Ig;

    /// \brief Spatial forces set, used in CRBA and CCRBA
    PINOCCHIO_ALIGNED_STD_VECTOR(Matrix6x) Fcrb;

    /// \brief Index of the last child (for CRBA)
    std::vector<int> lastChild;
    
    /// \brief Dimension of the subtree motion space (for CRBA)
    std::vector<int> nvSubtree;
    
    /// \brief Starting index of the Joint motion subspace
    std::vector<int> start_idx_v_fromRow;
    
    /// \brief End index of the Joint motion subspace
    std::vector<int> end_idx_v_fromRow;

    /// \brief Joint space intertia matrix square root (upper trianglular part) computed with a Cholesky Decomposition.
    MatrixXs U;
    
    /// \brief Diagonal of the joint space intertia matrix obtained by a Cholesky Decomposition.
    VectorXs D;
    
    /// \brief Diagonal inverse of the joint space intertia matrix obtained by a Cholesky Decomposition.
    VectorXs Dinv;
    
    /// \brief Temporary of size NV used in Cholesky Decomposition.
    VectorXs tmp;
    
    /// \brief First previous non-zero row in M (used in Cholesky Decomposition).
    std::vector<int> parents_fromRow;
    
    /// \brief Each element of this vector corresponds to the ordered list of indexes belonging to the supporting tree of the
    ///        given index at the row level. It may be helpful to retrieve the sparsity pattern through it.
    std::vector< std::vector<int> > supports_fromRow;
    
    /// \brief Subtree of the current row index (used in Cholesky Decomposition).
    std::vector<int> nvSubtree_fromRow;
    
    /// \brief Jacobian of joint placements.
    /// \note The columns of J corresponds to the basis of the spatial velocities of each joint and expressed at the origin of the inertial frame. In other words, if \f$ v_{J_{i}} = S_{i} \dot{q}_{i}\f$ is the relative velocity of the joint i regarding to its parent, then \f$J = \begin{bmatrix} ^{0}X_{1} S_{1} & \cdots & ^{0}X_{i} S_{i} & \cdots & ^{0}X_{\text{nj}} S_{\text{nj}} \end{bmatrix} \f$. This Jacobian has no special meaning. To get the jacobian of a precise joint, you need to call pinocchio::getJointJacobian
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
    MatrixXs dtau_dq;
    
    /// \brief Partial derivative of the joint torque vector with respect to the joint velocity.
    MatrixXs dtau_dv;
    
    /// \brief Partial derivative of the joint acceleration vector with respect to the joint configuration.
    MatrixXs ddq_dq;
    
    /// \brief Partial derivative of the joint acceleration vector with respect to the joint velocity.
    MatrixXs ddq_dv;
    
    /// \brief Vector of joint placements wrt to algorithm end effector.
    PINOCCHIO_ALIGNED_STD_VECTOR(SE3) iMf;

    /// \brief Vector of subtree center of mass positions expressed in the root joint of the subtree. In other words, com[j] is the CoM position of the subtree supported by joint \f$ j \f$ and expressed in the joint frame \f$ j \f$. The element com[0] corresponds to the center of mass position of the whole model and expressed in the global frame.
    PINOCCHIO_ALIGNED_STD_VECTOR(Vector3) com;
    
    /// \brief Vector of subtree center of mass linear velocities expressed in the root joint of the subtree. In other words, vcom[j] is the CoM linear velocity of the subtree supported by joint \f$ j \f$ and expressed in the joint frame \f$ j \f$. The element vcom[0] corresponds to the velocity of the CoM of the whole model expressed in the global frame.
    PINOCCHIO_ALIGNED_STD_VECTOR(Vector3) vcom;
    
    /// \brief Vector of subtree center of mass linear accelerations expressed in the root joint of the subtree. In other words, acom[j] is the CoM linear acceleration of the subtree supported by joint \f$ j \f$ and expressed in the joint frame \f$ j \f$. The element acom[0] corresponds to the acceleration of the CoM of the whole model expressed in the global frame.
    PINOCCHIO_ALIGNED_STD_VECTOR(Vector3) acom;
    
    /// \brief Vector of subtree mass. In other words, mass[j] is the mass of the subtree supported by joint \f$ j \f$. The element mass[0] corresponds to the total mass of the model.
    std::vector<Scalar> mass;
    
    /// \brief Jacobian of center of mass.
    /// \note This Jacobian maps the joint velocity vector to the velocity of the center of mass, expressed in the inertial frame. In other words, \f$ v_{\text{CoM}} = J_{\text{CoM}} \dot{q}\f$.
    Matrix3x Jcom;

    /// \brief Kinetic energy of the model.
    Scalar kinetic_energy;
    
    /// \brief Potential energy of the model.
    Scalar potential_energy;
    
    // Temporary variables used in forward dynamics
    
    /// \brief Inverse of the operational-space inertia matrix
    MatrixXs JMinvJt;
    
    /// \brief Cholesky decompostion of \f$JMinvJt\f$.
    Eigen::LLT<MatrixXs> llt_JMinvJt;
    
    /// \brief Lagrange Multipliers corresponding to the contact forces in pinocchio::forwardDynamics.
    VectorXs lambda_c;
    
    /// \brief Temporary corresponding to \f$ \sqrt{D} U^{-1} J^{\top} \f$.
    MatrixXs sDUiJt;
    
    /// \brief Temporary corresponding to the residual torque \f$ \tau - b(q,\dot{q}) \f$.
    VectorXs torque_residual;
    
    /// \brief Generalized velocity after impact.
    TangentVectorType dq_after;
    
    /// \brief Lagrange Multipliers corresponding to the contact impulses in pinocchio::impulseDynamics.
    VectorXs impulse_c;
    
    /// \brief Matrix related to static regressor
    Matrix3x staticRegressor;

    /// \brief Body regressor
    BodyRegressorType bodyRegressor;

    /// \brief Matrix related to joint torque regressor
    MatrixXs jointTorqueRegressor;
    
    /// \brief Tensor containing the kinematic Hessian of all the joints.
    Tensor3x kinematic_hessians;
    
    ///
    /// \brief Default constructor of pinocchio::Data from a pinocchio::Model.
    ///
    /// \param[in] model The model structure of the rigid body system.
    ///
    explicit DataTpl(const Model & model);
    
    ///
    /// \brief Default constructor
    ///
    DataTpl() {}

  private:
    void computeLastChild(const Model & model);
    void computeParents_fromRow(const Model & model);
    void computeSupports_fromRow(const Model & model);

  };

} // namespace pinocchio

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
#include "pinocchio/multibody/data.hxx"

#endif // ifndef __pinocchio_multibody_data_hpp__

