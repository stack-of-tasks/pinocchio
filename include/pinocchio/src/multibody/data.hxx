//
// Copyright (c) 2015-2025 CNRS INRIA
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#pragma once

// IWYU pragma: private, include "pinocchio/multibody.hpp"

#ifdef PINOCCHIO_LSP
  #undef PINOCCHIO_LSP
  #include "pinocchio/multibody.hpp"
#endif // PINOCCHIO_LSP

namespace pinocchio
{

  template<typename _Scalar, int _Options, template<typename, int> class JointCollectionTpl>
  struct traits<DataTpl<_Scalar, _Options, JointCollectionTpl>>
  {
    typedef _Scalar Scalar;
    static constexpr int Options = _Options;
    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef JointCollectionTpl<Scalar, Options> JointCollection;
  };

  template<typename _Scalar, int _Options, template<typename, int> class JointCollectionTpl>
  struct DataTpl
  : serialization::Serializable<DataTpl<_Scalar, _Options, JointCollectionTpl>>
  , NumericalBase<DataTpl<_Scalar, _Options, JointCollectionTpl>>
  , DataEntity<DataTpl<_Scalar, _Options, JointCollectionTpl>>
  {

    typedef typename traits<DataTpl>::Scalar Scalar;
    static constexpr int Options = traits<DataTpl>::Options;

    typedef typename traits<DataTpl>::JointCollection JointCollection;
    typedef typename traits<DataTpl>::Model Model;

    typedef SE3Tpl<Scalar, Options> SE3;
    typedef MotionTpl<Scalar, Options> Motion;
    typedef ForceTpl<Scalar, Options> Force;
    typedef InertiaTpl<Scalar, Options> Inertia;
    typedef FrameTpl<Scalar, Options> Frame;

    typedef pinocchio::Index Index;
    typedef pinocchio::JointIndex JointIndex;
    typedef pinocchio::GeomIndex GeomIndex;
    typedef pinocchio::FrameIndex FrameIndex;
    typedef std::vector<Index> IndexVector;

    typedef JointModelTpl<Scalar, Options, JointCollectionTpl> JointModel;
    typedef JointDataTpl<Scalar, Options, JointCollectionTpl> JointData;

    typedef std::vector<JointModel> JointModelVector;
    typedef std::vector<JointData> JointDataVector;

    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Options> MatrixXs;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1, Options> VectorXs;
    typedef Eigen::Matrix<Scalar, 1, Eigen::Dynamic, Options | Eigen::RowMajor> RowVectorXs;
    typedef Eigen::Matrix<Scalar, 3, 1, Options> Vector3;
    typedef Eigen::Matrix<Scalar, 6, 1, Options> Vector6;
    typedef Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic, Options> MatrixXb;

    /// \brief Dense vectorized version of a joint configuration vector.
    typedef VectorXs ConfigVectorType;

    /// \brief Dense vectorized version of a joint tangent vector (e.g. velocity, acceleration,
    /// etc).
    ///        It also handles the notion of co-tangent vector (e.g. torque, etc).
    typedef VectorXs TangentVectorType;

    /// \brief Colmajor matrix with fixed rows (dimension 6) and dynamic cols.
    typedef Eigen::Matrix<Scalar, 6, Eigen::Dynamic, Options> Matrix6x;
    /// \brief Colmajor matrix with fixed rows (dimension 3) and dynamic cols.
    typedef Eigen::Matrix<Scalar, 3, Eigen::Dynamic, Options> Matrix3x;

    typedef Eigen::Matrix<Scalar, 6, 6, Options> Matrix6;
    typedef Eigen::Matrix<Scalar, 6, 6, Eigen::RowMajor | Options> RowMatrix6;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor | Options>
      RowMatrixXs;

    /// \brief The type of the body regressor
    typedef Eigen::Matrix<Scalar, 6, 10, Options> BodyRegressorType;

    ///  \brief The type of Tensor for Kinematics and Dynamics second order derivatives
    typedef Tensor<Scalar, 3, Options> Tensor3x;

    typedef internal::MatrixStackTpl<MatrixXs> DynamicMatrixStack;

    typedef ConstraintCholeskyDecompositionTpl<Scalar, Options> ConstraintCholeskyDecomposition;

    /// \brief Vector of pinocchio::JointData associated to the pinocchio::JointModel stored in
    /// model
    JointDataVector joints;

    /// \brief Vector of pinocchio::JointData associated to the pinocchio::JointModel stored in
    /// model augmented by constraints
    JointDataVector joints_augmented;

    /// \brief Input configuration vector
    ConfigVectorType q_in;

    /// \brief Input velocity vector
    TangentVectorType v_in;

    /// \brief Input acceleration vector
    TangentVectorType a_in;

    /// \brief Input torque vector
    TangentVectorType tau_in;

    /// \brief Vector of joint accelerations expressed in the local frame of the joint.
    std::vector<Motion> a;

    /// \brief Vector of joint accelerations expressed at the origin of the world.
    std::vector<Motion> oa;

    /// \brief Vector of joint accelerations expressed at the origin of the world.
    ///        These accelerations are used in the context of augmented Lagrangian algorithms.
    std::vector<Motion> oa_drift;

    /// \brief Vector of joint accelerations expressed at the origin of the world.
    ///        These accelerations are used in the context of augmented Lagrangian algorithms.
    std::vector<Motion> oa_augmented;

    /// \brief Vector of joint accelerations due to the gravity field.
    std::vector<Motion> a_gf;

    /// \brief Vector of joint accelerations expressed at the origin of the world including the
    /// gravity contribution.
    std::vector<Motion> oa_gf;

    /// \brief Vector of joint velocities expressed in the local frame of the joint.
    std::vector<Motion> v;

    /// \brief Vector of joint velocities expressed at the origin of the world.
    std::vector<Motion> ov;

    /// \brief Vector of body forces expressed in the local frame of the joint. For each body, the
    /// force represents the sum of
    ///        all external forces acting on the body.
    std::vector<Force> f;

    /// \brief Vector of body forces expressed at the origin of the world. For each body, the force
    /// represents the sum of
    ///        all external forces acting on the body.
    std::vector<Force> of;

    /// \brief Vector of body forces expressed in the world frame. For each body, the force
    /// represents the sum of
    ///        all external forces acting on the body. These forces are used in the context of
    ///        augmented Lagrangian algorithms.
    std::vector<Force> of_augmented;

    /// \brief Vector of spatial momenta expressed in the local frame of the joint.
    std::vector<Force> h;

    /// \brief Vector of spatial momenta expressed at the origin of the world.
    std::vector<Force> oh;

    /// \brief Vector of absolute joint placements (wrt the world).
    std::vector<SE3> oMi;

    /// \brief Vector of relative joint placements (wrt the body parent).
    std::vector<SE3> liMi;

    /// \brief Vector of joint torques (dim model.nv).
    TangentVectorType tau;

    /// \brief Vector of Non Linear Effects (dim model.nv). It corresponds to concatenation of the
    /// Coriolis, centrifugal and gravitational effects. \note  In the multibody dynamics equation
    /// \f$ M\ddot{q} + b(q, \dot{q}) = \tau \f$,
    ///        the non linear effects are associated to the term \f$b\f$.
    VectorXs nle;

    /// \brief Vector of generalized gravity (dim model.nv).
    /// \note  In the multibody dynamics equation \f$ M\ddot{q} + c(q, \dot{q}) + g(q) = \tau \f$,
    ///        the gravity effect is associated to the \f$g\f$ term.
    VectorXs g;

    /// \brief Vector of absolute operationnel frame placements (wrt the world).
    std::vector<SE3> oMf;

    /// \brief Vector of sub-tree composite rigid body inertias, i.e. the apparent inertia of the
    /// subtree supported by the joint
    ///        and expressed in the local frame of the joint..
    std::vector<Inertia> Ycrb;

    /// \brief Vector of sub-tree composite rigid body inertia time derivatives \f$
    /// \dot{Y}_{crb}\f$. See Data::Ycrb for more details.
    std::vector<Matrix6> dYcrb; // TODO: change with dense symmetric matrix6

    /// \brief The joint space inertia matrix (a square matrix of dim model.nv).
    MatrixXs M;

    /// \brief The inverse of the joint space inertia matrix (a square matrix of dim model.nv).
    RowMatrixXs Minv;

    /// \brief The Coriolis matrix (a square matrix of dim model.nv).
    MatrixXs C;

    /// \brief Variation of the spatial momenta set with respect to the joint configuration.
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
    std::vector<Matrix6> vxI;

    /// \brief Left variation of the inertia matrix
    std::vector<Matrix6> Ivx;

    /// \brief Combined variations of the inertia matrix \f$ B_i =  \frac{1}{2} [(v_i\times∗)I_i +
    /// (I_i v_i)\times^{∗} − I_i(v_i\times)] \f$  consistent with Christoffel symbols.
    std::vector<Matrix6> B;

    /// \brief Rigid Body Inertia supported by the joint expressed in the world frame
    std::vector<Inertia> oinertias;

    /// \brief Composite Rigid Body Inertia expressed in the world frame
    std::vector<Inertia> oYcrb;

    /// \brief Time variation of Composite Rigid Body Inertia expressed in the world frame
    std::vector<Matrix6> doYcrb;

    /// \brief Temporary for derivative algorithms
    Matrix6 Itmp;

    /// \brief Temporary for derivative algorithms
    Matrix6 M6tmp;
    RowMatrix6 M6tmpR;
    RowMatrix6 M6tmpR2;

    /// \brief The joint accelerations computed from ABA
    TangentVectorType ddq;

    // ABA internal data
    /// \brief Articulated Body Inertia matrix of the subtree expressed in the LOCAL coordinate
    /// frame of the joint
    std::vector<Matrix6> Yaba; // TODO: change with dense symmetric matrix6

    /// \brief Articulated Body Inertia matrix of the subtree expressed in the WORLD coordinate
    /// frame
    std::vector<Matrix6> oYaba; // TODO: change with dense symmetric matrix6

    /// \brief Articulated Body Inertia matrix with constraint augmented inertia, expressed in the
    /// WORLD coordinate frame
    std::vector<Matrix6> oYaba_augmented; // TODO: change with dense symmetric matrix6

    /// \brief Acceleration propagator
    std::vector<Matrix6> oL; // TODO: change with dense symmetric matrix6

    /// \brief Inverse articulated inertia
    std::vector<Matrix6> oK; // TODO: change with dense symmetric matrix6

    /// \brief Intermediate quantity corresponding to apparent torque [ABA]
    TangentVectorType u; // Joint Inertia

    // CCRBA return quantities
    /// \brief Centroidal Momentum Matrix
    /// \note \f$ hg = A_g \dot{q}\f$ maps the joint velocity set to the centroidal momentum.
    Matrix6x Ag;

    // dCCRBA return quantities
    /// \brief Centroidal Momentum Matrix Time Variation
    /// \note \f$ \dot{h_g} = A_g \ddot{q}\ + \dot{A_g}\dot{q}\f$ maps the joint velocity and
    /// acceleration vectors to the time variation of the centroidal momentum.
    Matrix6x dAg;

    /// \brief Centroidal momentum quantity.
    /// \note The centroidal momentum is expressed in the frame centered at the CoM and aligned with
    /// the inertial frame (i.e. the world frame). \note \f$ h_g = \left( m\dot{c}, L_{g} \right);
    /// \f$. \f$ h_g \f$ is the stack of the linear momentum and the angular momemtum vectors.
    ///
    Force hg;

    /// \brief Centroidal momentum time derivative.
    /// \note The centroidal momentum time derivative is expressed in the frame centered at the CoM
    /// and aligned with the inertial frame (i.e. the world frame). \note \f$ \dot{h_g} = \left(
    /// m\ddot{c}, \dot{L}_{g} \right); \f$. \f$ \dot{h_g} \f$ is the stack of the linear momentum
    /// variation and the angular momemtum variation.
    ///
    Force dhg;

    /// \brief Centroidal Composite Rigid Body Inertia.
    /// \note \f$ hg = Ig v_{\text{mean}}\f$ map a mean velocity to the current centroidal momentum
    /// quantity.
    Inertia Ig;

    /// \brief Spatial forces set, used in CRBA and CCRBA
    std::vector<Matrix6x> Fcrb;

    /// \brief Index of the last child (for CRBA)
    PINOCCHIO_DEPRECATED std::vector<int> lastChild;

    /// \brief Dimension of the subtree motion space (for CRBA)
    std::vector<int> nvSubtree;

    /// \brief Starting index of the Joint motion subspace
    std::vector<int> start_idx_v_fromRow;

    /// \brief End index of the Joint motion subspace
    std::vector<int> end_idx_v_fromRow;

    /// \brief Extended model mapping of the joint rows
    /// (idx_vExtended_to_idx_v_fromRow[idx_vExtended] = idx_v)
    std::vector<int> idx_vExtended_to_idx_v_fromRow;

    /// \brief Store the index of the first non mimic child of each mimic joint (for CRBA).
    /// Store 0 if there is no non mimic child.
    std::vector<JointIndex> mimic_subtree_joint;

    /// \brief Joint space inertia matrix square root (upper triangular part) computed with a
    /// Cholesky Decomposition.
    MatrixXs U;

    /// \brief Diagonal of the joint space inertia matrix obtained by a Cholesky Decomposition.
    VectorXs D;

    /// \brief Diagonal inverse of the joint space inertia matrix obtained by a Cholesky
    /// Decomposition.
    VectorXs Dinv;

    /// \brief Temporary of size NV used in Cholesky Decomposition.
    VectorXs tmp;

    /// \brief First previous non-zero row in M (used in Cholesky Decomposition).
    std::vector<int> parents_fromRow;

    /// \brief First previous non-zero row belonging to a mimic joint in M (used in Jacobian).
    std::vector<int> mimic_parents_fromRow;

    /// \brief First previous non-zero row belonging to a non mimic joint in M (used in Jacobian).
    std::vector<int> non_mimic_parents_fromRow;

    /// \brief Each element of this vector corresponds to the ordered list of indexes
    /// belonging to the supporting tree of the given index at the row level.
    /// It may be helpful to retrieve the sparsity pattern through it.
    std::vector<std::vector<int>> supports_fromRow;

    /// \brief Subtree of the current row index (used in Cholesky Decomposition).
    std::vector<int> nvSubtree_fromRow;

    /// \brief Jacobian of joint placements.
    /// \note The columns of J corresponds to the basis of the spatial velocities of each joint and
    /// expressed at the origin of the inertial frame. In other words, if \f$ v_{J_{i}} = S_{i}
    /// \dot{q}_{i}\f$ is the relative velocity of the joint i regarding to its parent, then \f$J =
    /// \begin{bmatrix} ^{0}X_{1} S_{1} & \cdots & ^{0}X_{i} S_{i} & \cdots &
    /// ^{0}X_{\text{nvExtended}} S_{\text{nvExtended}} \end{bmatrix} \f$. This Jacobian has no
    /// special meaning. To get the jacobian of a precise joint, you need to call
    /// pinocchio::getJointJacobian
    Matrix6x J;

    /// \brief Derivative of the Jacobian with respect to the time.
    Matrix6x dJ;

    /// \brief Second derivative of the Jacobian with respect to the time.
    Matrix6x ddJ;

    /// \brief psidot Derivative of Jacobian w.r.t to the parent body moving
    /// v(p(j)) x Sj
    Matrix6x psid;

    /// \brief psiddot Second Derivative of Jacobian w.r.t to the parent body
    /// moving a(p(j)) x Sj + v(p(j)) x psidj
    Matrix6x psidd;

    /// \brief Variation of the spatial velocity set with respect to the joint configuration.
    Matrix6x dVdq;

    /// \brief Variation of the spatial acceleration set with respect to the joint configuration.
    Matrix6x dAdq;

    /// \brief Variation of the spatial acceleration set with respect to the joint velocity.
    Matrix6x dAdv;

    /// \brief Partial derivative of the joint torque vector with respect to the joint
    /// configuration.
    RowMatrixXs dtau_dq;

    /// \brief Partial derivative of the joint torque vector with respect to the joint velocity.
    RowMatrixXs dtau_dv;

    /// \brief Partial derivative of the joint acceleration vector with respect to the joint
    /// configuration.
    RowMatrixXs ddq_dq;

    /// \brief Partial derivative of the joint acceleration vector with respect to the joint
    /// velocity.
    RowMatrixXs ddq_dv;

    /// \brief Partial derivative of the joint acceleration vector with respect to the joint
    /// torques.
    RowMatrixXs ddq_dtau;

    /// \brief Stack of partial derivative of the contact frame acceleration with respect to the
    /// joint parameters.
    MatrixXs dvc_dq;
    MatrixXs dac_dq;
    MatrixXs dac_dv;
    MatrixXs dac_da;

    /// \brief Operational space inertia matrix;
    MatrixXs osim;

    /// \brief Partial derivatives of the constraints forces with respect to the joint
    /// configuration, velocity and torque;
    MatrixXs dlambda_dq;
    MatrixXs dlambda_dv;
    MatrixXs dlambda_dtau;
    MatrixXs dlambda_dx_prox, drhs_prox;

    /// \brief Vector of joint placements wrt to algorithm end effector.
    std::vector<SE3> iMf;

    /// \brief Vector of subtree center of mass positions expressed in the root joint of the
    /// subtree. In other words, com[j] is the CoM position of the subtree supported by joint \f$ j
    /// \f$ and expressed in the joint frame \f$ j \f$. The element com[0] corresponds to the center
    /// of mass position of the whole model and expressed in the global frame.
    std::vector<Vector3> com;

    /// \brief Vector of subtree center of mass linear velocities expressed in the root joint of the
    /// subtree. In other words, vcom[j] is the CoM linear velocity of the subtree supported by
    /// joint \f$ j \f$ and expressed in the joint frame \f$ j \f$. The element vcom[0] corresponds
    /// to the velocity of the CoM of the whole model expressed in the global frame.
    std::vector<Vector3> vcom;

    /// \brief Vector of subtree center of mass linear accelerations expressed in the root joint of
    /// the subtree. In other words, acom[j] is the CoM linear acceleration of the subtree supported
    /// by joint \f$ j \f$ and expressed in the joint frame \f$ j \f$. The element acom[0]
    /// corresponds to the acceleration of the CoM of the whole model expressed in the global frame.
    std::vector<Vector3> acom;

    /// \brief Vector of subtree mass. In other words, mass[j] is the mass of the subtree supported
    /// by joint \f$ j \f$. The element mass[0] corresponds to the total mass of the model.
    std::vector<Scalar> mass;

    /// \brief Jacobian of center of mass.
    /// \note This Jacobian maps the joint velocity vector to the velocity of the center of mass,
    /// expressed in the inertial frame. In other words, \f$ v_{\text{CoM}} = J_{\text{CoM}}
    /// \dot{q}\f$.
    Matrix3x Jcom;

    /// \brief Kinetic energy of the system.
    Scalar kinetic_energy;

    /// \brief Potential energy of the system.
    Scalar potential_energy;

    /// \brief Mechanical energy of the system.
    Scalar mechanical_energy;

    // Temporary variables used in forward dynamics

    /// \brief Inverse of the operational-space inertia matrix
    MatrixXs JMinvJt;

    /// \brief Cholesky decompostion of \f$JMinvJt\f$.
    Eigen::LLT<MatrixXs> llt_JMinvJt;

    /// \brief Lagrange Multipliers corresponding to the contact forces in
    /// pinocchio::forwardDynamics.
    VectorXs lambda_c;

    /// \brief Proximal Lagrange Multipliers used in the computation of the Forward Dynamics
    /// computations.
    VectorXs lambda_c_prox;

    /// \brief Difference between two consecutive iterations of the proxy algorithm.
    VectorXs diff_lambda_c;

    /// \brief Temporary corresponding to \f$ \sqrt{D} U^{-1} J^{\top} \f$.
    MatrixXs sDUiJt;

    /// \brief Temporary corresponding to the residual torque \f$ \tau - b(q,\dot{q}) \f$.
    VectorXs torque_residual;

    /// \brief Generalized velocity after impact.
    TangentVectorType dq_after;

    /// \brief Lagrange Multipliers corresponding to the contact impulses in
    /// pinocchio::impulseDynamics.
    VectorXs impulse_c;

    /// \brief Matrix related to static regressor
    Matrix3x staticRegressor;

    /// \brief Body regressor
    BodyRegressorType bodyRegressor;

    /// \brief Matrix related to joint torque regressor
    MatrixXs jointTorqueRegressor;

    /// \brief Matrix related to kinetic energy regressor
    RowVectorXs kineticEnergyRegressor;

    /// \brief Matrix related to potential energy regressor
    RowVectorXs potentialEnergyRegressor;

    std::vector<Matrix6x> KA;
    std::vector<MatrixXs> LA;
    std::vector<VectorXs> lA;
    std::vector<VectorXs> lambdaA;
    std::vector<int> par_cons_ind;
    std::vector<Motion> a_bias;
    std::vector<MatrixXs> KAS;
    std::vector<int> constraint_ind;
    Eigen::LLT<MatrixXs> osim_llt;

#if defined(_MSC_VER)
    // Eigen tensor warning: Eigen\CXX11\src/Tensor/Tensor.h(76,1): warning C4554: '&': check
    // operator precedence for possible error
  #pragma warning(disable : 4554)
#endif

    /// \brief Tensor containing the kinematic Hessian of all the joints.
    Tensor3x kinematic_hessians;

#if defined(_MSC_VER)
  #pragma warning(default : 4554) // C4554 enabled after tensor definition
#endif

    /// \brief Cholesky decomposition of the KKT contact matrix
    ConstraintCholeskyDecomposition constraint_chol;

    /// \brief Cholesky decomposition of the KKT contact matrix
    /// Deprecated in favor of lowerDryFrictionLimit and upperDryFrictionLimit
    ConstraintCholeskyDecomposition contact_chol;

    /// \brief RHS vector when solving the contact dynamics KKT problem
    VectorXs primal_dual_contact_solution;

    /// \brief Primal RHS in contact dynamic equations
    VectorXs primal_rhs_contact;

#if defined(_MSC_VER)
    // Eigen tensor warning: Eigen\CXX11\src/Tensor/Tensor.h(76,1): warning C4554: '&': check
    // operator precedence for possible error
  #pragma warning(disable : 4554)
#endif

    /// \brief SO Partial derivative of the joint torque vector with respect to
    /// the joint configuration.
    Tensor3x d2tau_dqdq;

    /// \brief SO Partial derivative of the joint torque vector with respect to
    /// the joint velocity.
    Tensor3x d2tau_dvdv;

    /// \brief SO Cross-Partial derivative of the joint torque vector with
    /// respect to the joint configuration/velocity.
    Tensor3x d2tau_dqdv;

    /// \brief SO Cross-Partial derivative of the joint torque vector with
    /// respect to the joint acceleration/configuration. This also equals to the
    /// first-order partial derivative of the Mass Matrix w.r.t joint
    /// configuration.
    Tensor3x d2tau_dadq;

    /// \brief SO Partial derivative of the joint acceleration vector with
    /// respect to the joint configuration.
    Tensor3x d2ddq_dqdq;

    /// \brief SO Partial derivative of the joint acceleration vector with
    /// respect to the joint velocity.
    Tensor3x d2ddq_dvdv;

    /// \brief SO Cross-Partial derivative of the joint acceleration vector
    /// with respect to the joint configuration/velocity.
    Tensor3x d2ddq_dqdv;

    /// \brief SO Cross-Partial derivative of the joint acceleration vector
    /// with respect to the joint torque/configuration.
    Tensor3x d2ddq_dtaudq;

#if defined(_MSC_VER)
  #pragma warning(default : 4554) // C4554 enabled after tensor definition
#endif

    std::vector<Matrix6> extended_motion_propagator; // Stores force propagator to the base link
    std::vector<Matrix6> extended_motion_propagator2;
    std::vector<Matrix6> spatial_inv_inertia; // Stores spatial inverse inertia
    std::vector<size_t> accumulation_descendant;
    std::vector<size_t> accumulation_ancestor;
    std::vector<int> constraints_supported_dim;
    std::vector<std::set<size_t>> constraints_supported;
    std::vector<size_t> joints_supporting_constraints;
    std::vector<size_t> accumulation_joints;
    std::vector<std::vector<size_t>> constraints_on_joint;

    typedef std::vector<JointIndex> JointIndexVector;

    /// \brief Bookkeeping neighbouring vertices during LC-ABA or proxBBO
    std::vector<JointIndexVector> joint_neighbours;

    typedef std::pair<JointIndex, JointIndex> JointIndexPair;

    /// \brief Stores the cross-coupling inertias between links in LC-ABA
    internal::DoubleEntryContainer<std::vector<Matrix6>> joint_cross_coupling;

    /// \brief Coupling relation between joints in the presence of coupling constraints.
    MatrixXb joint_coupling_info;

    /// \brief Stores the projected cross-coupling between links as
    /// `projected_joint_cross_coupling(j,i) = cross_coupling(j,i) * J_i`.
    internal::DoubleEntryContainer<internal::MatrixStackTpl<Matrix6x>>
      projected_joint_cross_coupling;

    /// \brief Stores the elimination ordering of LC-ABA
    std::vector<JointIndex> joint_elimination_order;

    /// \brief Joint apparent inertia vector (related to model.armarture, joint-wise constraints,
    /// etc.)
    DynamicMatrixStack joint_apparent_inertia;

    PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
    PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
    ///
    /// \brief Default constructor of pinocchio::Data from a pinocchio::Model.
    ///
    /// \param[in] model The model structure of the rigid body system.
    ///
    explicit DataTpl(const Model & model);

    ///
    /// \brief Default constructor
    ///
    DataTpl()
    {
    }

    // Avoid deprecation warning
    DataTpl(const DataTpl &) = default;
    ~DataTpl() = default;
    DataTpl & operator=(const DataTpl &) = default;

    PINOCCHIO_COMPILER_DIAGNOSTIC_POP

  private:
    void computeLastChild(const Model & model); // TODO Remove when lastChild is removed
    void computeNvSubtree(const Model & model);
    void computeParents_fromRow(const Model & model);
    void computeSupports_fromRow(const Model & model);
  };

} // namespace pinocchio

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */

namespace pinocchio
{

  PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
  PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  DataTpl<Scalar, Options, JointCollectionTpl>::DataTpl(const Model & model)
  : q_in(neutral(model))
  , v_in(VectorXs::Zero(model.nv))
  , a_in(VectorXs::Zero(model.nv))
  , tau_in(VectorXs::Zero(model.nv))
  , a((std::size_t)model.njoints, Motion::Zero())
  , oa((std::size_t)model.njoints, Motion::Zero())
  , oa_drift((std::size_t)model.njoints, Motion::Zero())
  , oa_augmented((std::size_t)model.njoints, Motion::Zero())
  , a_gf((std::size_t)model.njoints, Motion::Zero())
  , oa_gf((std::size_t)model.njoints, Motion::Zero())
  , v((std::size_t)model.njoints, Motion::Zero())
  , ov((std::size_t)model.njoints, Motion::Zero())
  , f((std::size_t)model.njoints, Force::Zero())
  , of((std::size_t)model.njoints, Force::Zero())
  , of_augmented((std::size_t)model.njoints, Force::Zero())
  , h((std::size_t)model.njoints, Force::Zero())
  , oh((std::size_t)model.njoints, Force::Zero())
  , oMi((std::size_t)model.njoints, SE3::Identity())
  , liMi((std::size_t)model.njoints, SE3::Identity())
  , tau(VectorXs::Zero(model.nv))
  , nle(VectorXs::Zero(model.nv))
  , g(VectorXs::Zero(model.nv))
  , oMf((std::size_t)model.nframes, SE3::Identity())
  , Ycrb((std::size_t)model.njoints, Inertia::Zero())
  , dYcrb((std::size_t)model.njoints, Inertia::Zero())
  , M(MatrixXs::Zero(model.nv, model.nv))
  , Minv(MatrixXs::Zero(model.nv, model.nv))
  , C(MatrixXs::Zero(model.nv, model.nv))
  , dHdq(Matrix6x::Zero(6, model.nv))
  , dFdq(Matrix6x::Zero(6, model.nv))
  , dFdv(Matrix6x::Zero(6, model.nv))
  , dFda(Matrix6x::Zero(6, model.nv))
  , SDinv(Matrix6x::Zero(6, model.nv))
  , UDinv(Matrix6x::Zero(6, model.nv))
  , IS(MatrixXs::Zero(6, model.nv))
  , vxI((std::size_t)model.njoints, Inertia::Matrix6::Zero())
  , Ivx((std::size_t)model.njoints, Inertia::Matrix6::Zero())
  , B((std::size_t)model.njoints, Inertia::Matrix6::Zero())
  , oinertias((std::size_t)model.njoints, Inertia::Zero())
  , oYcrb((std::size_t)model.njoints, Inertia::Zero())
  , doYcrb((std::size_t)model.njoints, Inertia::Matrix6::Zero())
  , ddq(VectorXs::Zero(model.nv))
  , Yaba((std::size_t)model.njoints, Inertia::Matrix6::Zero())
  , oYaba((std::size_t)model.njoints, Inertia::Matrix6::Zero())
  , oYaba_augmented((std::size_t)model.njoints, Inertia::Matrix6::Zero())
  , oL((std::size_t)model.njoints, Inertia::Matrix6::Zero())
  , oK((std::size_t)model.njoints, Inertia::Matrix6::Zero())
  , u(VectorXs::Zero(model.nv))
  , Ag(Matrix6x::Zero(6, model.nv))
  , dAg(Matrix6x::Zero(6, model.nv))
  , hg(Force::Zero())
  , dhg(Force::Zero())
  , Ig(Inertia::Zero())
  , Fcrb((std::size_t)model.njoints, Matrix6x::Zero(6, model.nv))
  , lastChild((std::size_t)model.njoints, -1)
  , nvSubtree((std::size_t)model.njoints, 0)
  , start_idx_v_fromRow((std::size_t)model.nvExtended, -1)
  , end_idx_v_fromRow((std::size_t)model.nvExtended, -1)
  , idx_vExtended_to_idx_v_fromRow((std::size_t)model.nvExtended, -1)
  , U(MatrixXs::Identity(model.nv, model.nv))
  , D(VectorXs::Zero(model.nv))
  , Dinv(VectorXs::Zero(model.nv))
  , tmp(VectorXs::Zero(model.nv))
  , parents_fromRow((std::size_t)model.nvExtended, -1)
  , mimic_parents_fromRow((std::size_t)model.nvExtended, -1)
  , non_mimic_parents_fromRow((std::size_t)model.nvExtended, -1)
  , supports_fromRow((std::size_t)model.nv)
  , nvSubtree_fromRow((std::size_t)model.nvExtended, -1)
  , J(Matrix6x::Zero(6, model.nvExtended))
  , dJ(Matrix6x::Zero(6, model.nvExtended))
  , ddJ(Matrix6x::Zero(6, model.nvExtended))
  , psid(Matrix6x::Zero(6, model.nv))
  , psidd(Matrix6x::Zero(6, model.nv))
  , dVdq(Matrix6x::Zero(6, model.nv))
  , dAdq(Matrix6x::Zero(6, model.nv))
  , dAdv(Matrix6x::Zero(6, model.nv))
  , dtau_dq(RowMatrixXs::Zero(model.nv, model.nv))
  , dtau_dv(RowMatrixXs::Zero(model.nv, model.nv))
  , ddq_dq(RowMatrixXs::Zero(model.nv, model.nv))
  , ddq_dv(RowMatrixXs::Zero(model.nv, model.nv))
  , ddq_dtau(RowMatrixXs::Zero(model.nv, model.nv))
  , iMf((std::size_t)model.njoints, SE3::Identity())
  , com((std::size_t)model.njoints, Vector3::Zero())
  , vcom((std::size_t)model.njoints, Vector3::Zero())
  , acom((std::size_t)model.njoints, Vector3::Zero())
  , mass((std::size_t)model.njoints, (Scalar)(-1))
  , Jcom(Matrix3x::Zero(3, model.nv))
  , kinetic_energy(Scalar(0))
  , potential_energy(Scalar(0))
  , mechanical_energy(Scalar(0))
  , JMinvJt()
  , llt_JMinvJt()
  , lambda_c()
  , lambda_c_prox()
  , diff_lambda_c()
  , sDUiJt(MatrixXs::Zero(model.nv, model.nv))
  , torque_residual(VectorXs::Zero(model.nv))
  , dq_after(VectorXs::Zero(model.nv))
  , impulse_c()
  , staticRegressor(Matrix3x::Zero(3, 4 * (model.njoints - 1)))
  , bodyRegressor(BodyRegressorType::Zero())
  , jointTorqueRegressor(MatrixXs::Zero(model.nv, 10 * (model.njoints - 1)))
  , kineticEnergyRegressor(RowVectorXs::Zero(10 * (model.njoints - 1)))
  , potentialEnergyRegressor(RowVectorXs::Zero(10 * (model.njoints - 1)))
  , KA((std::size_t)model.njoints, Matrix6x::Zero(6, 0))
  , LA((std::size_t)model.njoints, MatrixXs::Zero(0, 0))
  , lA((std::size_t)model.njoints, VectorXs::Zero(0))
  , lambdaA((std::size_t)model.njoints, VectorXs::Zero(0))
  , par_cons_ind((std::size_t)model.njoints, 0)
  , a_bias((std::size_t)model.njoints, Motion::Zero())
  , KAS((std::size_t)model.njoints, MatrixXs::Zero(0, 0))
  , kinematic_hessians(6, model.nv, model.nv)
  , constraint_chol()
  , contact_chol(constraint_chol)
  , d2tau_dqdq(model.nv, model.nv, model.nv)
  , d2tau_dvdv(model.nv, model.nv, model.nv)
  , d2tau_dqdv(model.nv, model.nv, model.nv)
  , d2tau_dadq(model.nv, model.nv, model.nv)
  , d2ddq_dqdq(model.nv, model.nv, model.nv)
  , d2ddq_dvdv(model.nv, model.nv, model.nv)
  , d2ddq_dqdv(model.nv, model.nv, model.nv)
  , d2ddq_dtaudq(model.nv, model.nv, model.nv)
  , extended_motion_propagator((std::size_t)model.njoints, Matrix6::Zero())
  , extended_motion_propagator2((std::size_t)model.njoints, Matrix6::Zero())
  , spatial_inv_inertia((std::size_t)model.njoints, Matrix6::Zero())
  , accumulation_descendant((std::size_t)model.njoints, 0)
  , accumulation_ancestor((std::size_t)model.njoints, 0)
  , constraints_supported_dim((std::size_t)model.njoints, 0)
  , constraints_supported((std::size_t)model.njoints)
  , constraints_on_joint((std::size_t)model.njoints)
  , joint_neighbours((std::size_t)model.njoints)
  , joint_cross_coupling(model.njoints, model.njoints)
  // Boolean value must be initialized to avoid error when serializing
  , joint_coupling_info(MatrixXb::Zero(model.njoints, model.njoints))
  , projected_joint_cross_coupling(model.njoints, model.njoints)
  , joint_apparent_inertia(
      std::size_t(model.njoints),
      std::size_t(PINOCCHIO_SQUARE(*std::max_element(model.nvs.begin(), model.nvs.end()))))
  {
    typedef typename Model::JointIndex JointIndex;

    /* Create data structure associated to the joints */
    joints.reserve(JointIndex(model.njoints));
    for (JointIndex i = 0; i < JointIndex(model.njoints); ++i)
    {
      typedef CreateJointData<Scalar, Options, JointCollectionTpl> Constructor;
      joints.push_back(Constructor::run(model.joints[i]));
      const auto joint_nv = model.nvs[i];
      joint_apparent_inertia.push_back(joint_nv, joint_nv);
      joint_apparent_inertia.back().setZero();
    }
    joints_augmented = joints;

    /* Init for CRBA */
    M.setZero();
    Minv.setZero();

    computeLastChild(model);

    computeNvSubtree(model);

    /* Init for Cholesky */
    computeParents_fromRow(model);
    computeSupports_fromRow(model);

    /* Init universe states relatively to itself */
    a_gf[0] = -model.gravity;

    kinematic_hessians.setZero();
    d2tau_dqdq.setZero();
    d2tau_dvdv.setZero();
    d2tau_dqdv.setZero();
    d2tau_dadq.setZero();
    d2ddq_dqdq.setZero();
    d2ddq_dvdv.setZero();
    d2ddq_dqdv.setZero();
    d2ddq_dtaudq.setZero();
  }
  PINOCCHIO_COMPILER_DIAGNOSTIC_POP

  PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
  PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  inline void DataTpl<Scalar, Options, JointCollectionTpl>::computeLastChild(const Model & model)
  {
    typedef typename Model::Index Index;

    std::fill(lastChild.begin(), lastChild.end(), -1);
    for (int i = model.njoints - 1; i >= 0; --i)
    {
      if (lastChild[(Index)i] == -1)
        lastChild[(Index)i] = i;
      const Index & parent = model.parents[(Index)i];

      lastChild[parent] = std::max<int>(lastChild[(Index)i], lastChild[parent]);
    }
  }
  PINOCCHIO_COMPILER_DIAGNOSTIC_POP

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  inline void DataTpl<Scalar, Options, JointCollectionTpl>::computeNvSubtree(const Model & model)
  {
    for (int i = model.njoints - 1; i >= 0; --i)
    {
      const Index parent = model.parents[(Index)i];
      nvSubtree[(Index)i] += model.joints[(Index)i].nv();
      nvSubtree[parent] += nvSubtree[(Index)i];
    }
    // fill mimic data
    for (const JointIndex mimicking_id : model.mimicking_joints)
    {
      const auto & mimicking_sub = model.subtrees[mimicking_id];
      size_t j = 1;
      bool found = false;
      for (; j < mimicking_sub.size(); j++)
      {
        if (model.nvs[mimicking_sub[j]] != 0)
        {
          found = true;
          break;
        }
      }
      if (mimicking_sub.size() == 1 || !found)
        mimic_subtree_joint.push_back(0);
      else
        mimic_subtree_joint.push_back(mimicking_sub[j]);
    }
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  inline void
  DataTpl<Scalar, Options, JointCollectionTpl>::computeParents_fromRow(const Model & model)
  {
    typedef typename Model::Index Index;

    for (Index joint = 1; joint < (Index)(model.njoints); joint++)
    {
      const Index & parent = model.parents[joint];
      const int idx_vj = model.joints[joint].idx_v();
      const int nvExtended_j = model.joints[joint].nvExtended();
      const int idx_vExtended_j = model.joints[joint].idx_vExtended();

      assert(idx_vExtended_j >= 0 && idx_vExtended_j < model.nvExtended);
      assert(idx_vj >= 0 && idx_vj < model.nv);

      if (parent > 0)
        parents_fromRow[(Index)idx_vExtended_j] =
          model.joints[parent].idx_vExtended() + model.joints[parent].nvExtended() - 1;
      else
        parents_fromRow[(Index)idx_vExtended_j] = -1;

      JointIndex first_non_mimic_parent_id = parent;
      while (first_non_mimic_parent_id > 0
             && boost::get<JointModelMimicTpl<Scalar, Options, JointCollectionTpl>>(
               &model.joints[first_non_mimic_parent_id]))
      {
        first_non_mimic_parent_id = model.parents[first_non_mimic_parent_id];
      }

      if (first_non_mimic_parent_id > 0)
        non_mimic_parents_fromRow[(Index)idx_vExtended_j] =
          model.joints[first_non_mimic_parent_id].idx_vExtended()
          + model.joints[first_non_mimic_parent_id].nvExtended() - 1;
      else
        non_mimic_parents_fromRow[(Index)idx_vExtended_j] = -1;

      JointIndex first_mimic_parent_id = parent;
      while (first_mimic_parent_id > 0
             && !boost::get<JointModelMimicTpl<Scalar, Options, JointCollectionTpl>>(
               &model.joints[first_mimic_parent_id]))
      {
        first_mimic_parent_id = model.parents[first_mimic_parent_id];
      }

      if (first_mimic_parent_id > 0)
        mimic_parents_fromRow[(Index)idx_vExtended_j] =
          model.joints[first_mimic_parent_id].idx_vExtended()
          + model.joints[first_mimic_parent_id].nvExtended() - 1;
      else
        mimic_parents_fromRow[(Index)idx_vExtended_j] = -1;

      nvSubtree_fromRow[(Index)idx_vExtended_j] = nvSubtree[joint];
      start_idx_v_fromRow[(size_t)idx_vj] = idx_vj;
      end_idx_v_fromRow[(size_t)idx_vj] = idx_vj + nvExtended_j - 1;
      idx_vExtended_to_idx_v_fromRow[(size_t)idx_vExtended_j] = idx_vj;

      for (int row = 1; row < nvExtended_j; ++row)
      {
        parents_fromRow[(size_t)(idx_vExtended_j + row)] = idx_vExtended_j + row - 1;
        mimic_parents_fromRow[(size_t)(idx_vExtended_j + row)] = idx_vExtended_j + row - 1;
        non_mimic_parents_fromRow[(size_t)(idx_vExtended_j + row)] = idx_vExtended_j + row - 1;
        nvSubtree_fromRow[(size_t)(idx_vExtended_j + row)] = nvSubtree[joint] - row;
        start_idx_v_fromRow[(size_t)(idx_vExtended_j + row)] =
          start_idx_v_fromRow[(size_t)idx_vExtended_j];
        end_idx_v_fromRow[(size_t)(idx_vExtended_j + row)] =
          end_idx_v_fromRow[(size_t)idx_vExtended_j];
        idx_vExtended_to_idx_v_fromRow[(size_t)(idx_vExtended_j + row)] =
          idx_vExtended_to_idx_v_fromRow[(size_t)idx_vExtended_j] + row;
      }
    }
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  inline void
  DataTpl<Scalar, Options, JointCollectionTpl>::computeSupports_fromRow(const Model & model)
  {
    typedef typename Model::JointIndex JointIndex;

    for (JointIndex joint_id = 1; joint_id < (JointIndex)(model.njoints); joint_id++)
    {
      const int nvj = nv(model.joints[joint_id]);
      const int idx_vj = idx_v(model.joints[joint_id]);

      assert(idx_vj >= 0 && idx_vj < model.nv);

      const int parent_fromRow = parents_fromRow[(size_t)idx_vj];

      if (parent_fromRow >= 0)
        supports_fromRow[(size_t)idx_vj] = supports_fromRow[(size_t)parent_fromRow];

      supports_fromRow[(size_t)idx_vj].push_back(idx_vj);

      for (int row = 1; row < nvj; ++row)
      {
        supports_fromRow[(size_t)(idx_vj + row)] = supports_fromRow[(size_t)(idx_vj + row - 1)];
        supports_fromRow[(size_t)(idx_vj + row)].push_back(idx_vj + row);
      }
    }
  }

  PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
  PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  bool operator==(
    const DataTpl<Scalar, Options, JointCollectionTpl> & data1,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data2)
  {
    bool value =
      data1.joints == data2.joints && data1.joints_augmented == data2.joints_augmented
      && data1.q_in == data2.q_in && data1.v_in == data2.v_in && data1.a_in == data2.a_in
      && data1.tau_in == data2.tau_in && data1.a == data2.a && data1.oa == data2.oa
      && data1.oa_drift == data2.oa_drift && data1.oa_augmented == data2.oa_augmented
      && data1.a_gf == data2.a_gf && data1.oa_gf == data2.oa_gf && data1.v == data2.v
      && data1.ov == data2.ov && data1.f == data2.f && data1.of == data2.of
      && data1.of_augmented == data2.of_augmented && data1.h == data2.h && data1.oh == data2.oh
      && data1.oMi == data2.oMi && data1.liMi == data2.liMi && data1.tau == data2.tau
      && data1.nle == data2.nle && data1.g == data2.g && data1.oMf == data2.oMf
      && data1.Ycrb == data2.Ycrb && data1.dYcrb == data2.dYcrb && data1.M == data2.M
      && data1.Minv == data2.Minv && data1.C == data2.C && data1.dHdq == data2.dHdq
      && data1.dFdq == data2.dFdq && data1.dFdv == data2.dFdv && data1.dFda == data2.dFda
      && data1.SDinv == data2.SDinv && data1.UDinv == data2.UDinv && data1.IS == data2.IS
      && data1.vxI == data2.vxI && data1.Ivx == data2.Ivx && data1.oinertias == data2.oinertias
      && data1.oYcrb == data2.oYcrb && data1.doYcrb == data2.doYcrb && data1.ddq == data2.ddq
      && data1.Yaba == data2.Yaba && data1.oYaba == data2.oYaba
      && data1.oYaba_augmented == data2.oYaba_augmented && data1.oL == data2.oL
      && data1.oK == data2.oK && data1.u == data2.u && data1.Ag == data2.Ag
      && data1.dAg == data2.dAg && data1.hg == data2.hg && data1.dhg == data2.dhg
      && data1.Ig == data2.Ig && data1.Fcrb == data2.Fcrb && data1.lastChild == data2.lastChild
      && data1.nvSubtree == data2.nvSubtree
      && data1.start_idx_v_fromRow == data2.start_idx_v_fromRow
      && data1.end_idx_v_fromRow == data2.end_idx_v_fromRow && data1.U == data2.U
      && data1.D == data2.D && data1.Dinv == data2.Dinv
      && data1.parents_fromRow == data2.parents_fromRow
      && data1.mimic_parents_fromRow == data2.mimic_parents_fromRow
      && data1.non_mimic_parents_fromRow == data2.non_mimic_parents_fromRow
      && data1.idx_vExtended_to_idx_v_fromRow == data2.idx_vExtended_to_idx_v_fromRow
      && data1.mimic_subtree_joint == data2.mimic_subtree_joint
      && data1.supports_fromRow == data2.supports_fromRow
      && data1.nvSubtree_fromRow == data2.nvSubtree_fromRow && data1.J == data2.J
      && data1.dJ == data2.dJ && data1.ddJ == data2.ddJ && data1.psid == data2.psid
      && data1.psidd == data2.psidd && data1.dVdq == data2.dVdq && data1.dAdq == data2.dAdq
      && data1.dAdv == data2.dAdv && data1.dtau_dq == data2.dtau_dq
      && data1.dtau_dv == data2.dtau_dv && data1.ddq_dq == data2.ddq_dq
      && data1.ddq_dv == data2.ddq_dv && data1.dvc_dq == data2.dvc_dq
      && data1.dac_dq == data2.dac_dq && data1.dac_dv == data2.dac_dv
      && data1.dac_da == data2.dac_da && data1.osim == data2.osim
      && data1.dlambda_dq == data2.dlambda_dq && data1.dlambda_dv == data2.dlambda_dv
      && data1.dlambda_dtau == data2.dlambda_dtau && data1.dlambda_dx_prox == data2.dlambda_dx_prox
      && data1.drhs_prox == data2.drhs_prox && data1.iMf == data2.iMf && data1.com == data2.com
      && data1.vcom == data2.vcom && data1.acom == data2.acom && data1.mass == data2.mass
      && data1.Jcom == data2.Jcom && data1.kinetic_energy == data2.kinetic_energy
      && data1.potential_energy == data2.potential_energy
      && data1.mechanical_energy == data2.mechanical_energy && data1.JMinvJt == data2.JMinvJt
      && data1.lambda_c == data2.lambda_c && data1.lambda_c_prox == data2.lambda_c_prox
      && data1.diff_lambda_c == data2.diff_lambda_c && data1.sDUiJt == data2.sDUiJt
      && data1.torque_residual == data2.torque_residual && data1.dq_after == data2.dq_after
      && data1.impulse_c == data2.impulse_c && data1.staticRegressor == data2.staticRegressor
      && data1.bodyRegressor == data2.bodyRegressor
      && data1.jointTorqueRegressor == data2.jointTorqueRegressor
      // && data1.constraint_chol == data2.constraint_chol
      && data1.primal_dual_contact_solution == data2.primal_dual_contact_solution
      && data1.extended_motion_propagator == data2.extended_motion_propagator
      && data1.extended_motion_propagator2 == data2.extended_motion_propagator2
      && data1.spatial_inv_inertia == data2.spatial_inv_inertia
      && data1.accumulation_descendant == data2.accumulation_descendant
      && data1.accumulation_ancestor == data2.accumulation_ancestor
      && data1.constraints_supported_dim == data2.constraints_supported_dim
      && data1.constraints_supported == data2.constraints_supported
      && data1.constraints_on_joint == data2.constraints_on_joint
      && data1.joint_neighbours == data2.joint_neighbours
      && data1.joint_cross_coupling == data2.joint_cross_coupling
      && data1.joint_coupling_info == data2.joint_coupling_info
      && data1.projected_joint_cross_coupling == data2.projected_joint_cross_coupling
      && data1.joint_apparent_inertia == data2.joint_apparent_inertia;

    // operator== for Eigen::Tensor provides an Expression which might be not evaluated as a boolean
    value &= Tensor<bool, 0>((data1.kinematic_hessians == data2.kinematic_hessians).all())(0)
             && Tensor<bool, 0>((data1.d2tau_dqdq == data2.d2tau_dqdq).all())(0)
             && Tensor<bool, 0>((data1.d2tau_dvdv == data2.d2tau_dvdv).all())(0)
             && Tensor<bool, 0>((data1.d2tau_dqdv == data2.d2tau_dqdv).all())(0)
             && Tensor<bool, 0>((data1.d2tau_dadq == data2.d2tau_dadq).all())(0)
             && Tensor<bool, 0>((data1.d2ddq_dqdq == data2.d2ddq_dqdq).all())(0)
             && Tensor<bool, 0>((data1.d2ddq_dvdv == data2.d2ddq_dvdv).all())(0)
             && Tensor<bool, 0>((data1.d2ddq_dqdv == data2.d2ddq_dqdv).all())(0)
             && Tensor<bool, 0>((data1.d2ddq_dtaudq == data2.d2ddq_dtaudq).all())(0);

    return value;
  }
  PINOCCHIO_COMPILER_DIAGNOSTIC_POP

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  bool operator!=(
    const DataTpl<Scalar, Options, JointCollectionTpl> & data1,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data2)
  {
    return !(data1 == data2);
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  typename ModelTpl<Scalar, Options, JointCollectionTpl>::Data
  ModelTpl<Scalar, Options, JointCollectionTpl>::createData() const
  {
    return Data(*this);
  }

} // namespace pinocchio

#ifdef PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION

namespace pinocchio
{

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI
  DataTpl<context::Scalar, context::Options, JointCollectionDefaultTpl>::DataTpl();

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI
  DataTpl<context::Scalar, context::Options, JointCollectionDefaultTpl>::DataTpl(const Model &);

} // namespace pinocchio

#endif // ifdef PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
