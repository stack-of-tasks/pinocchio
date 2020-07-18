//
// Copyright (c) 2017-2019 CNRS INRIA
//

#ifndef __pinocchio_algorithm_kinematics_derivatives_hpp__
#define __pinocchio_algorithm_kinematics_derivatives_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

#include "pinocchio/algorithm/jacobian.hpp"

namespace pinocchio
{
  
  ///
  /// \brief Computes all the terms required to compute the derivatives of the placement, spatial velocity and acceleration
  ///        for any joint of the model.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  /// \tparam TangentVectorType1 Type of the joint velocity vector.
  /// \tparam TangentVectorType2 Type of the joint acceleration vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration (vector dim model.nq).
  /// \param[in] v The joint velocity (vector dim model.nv).
  ///
  /// \remarks This function is similar to do a forwardKinematics(model,data,q,v) followed by a computeJointJacobians(model,data,q).
  ///          In addition, it computes the spatial velocity of the joint expressed in the world frame (see data.ov).
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2>
  inline void computeForwardKinematicsDerivatives(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                                  DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                                  const Eigen::MatrixBase<ConfigVectorType> & q,
                                                  const Eigen::MatrixBase<TangentVectorType1> & v,
                                                  const Eigen::MatrixBase<TangentVectorType2> & a);
  
  ///
  /// \brief Computes the partial derivaties of the spatial velocity of a given with respect to
  ///        the joint configuration and velocity.
  ///        You must first call computForwardKinematicsDerivatives before calling this function.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam Matrix6xOut1 Matrix6x containing the partial derivatives with respect to the joint configuration vector.
  /// \tparam Matrix6xOut2 Matrix6x containing the partial derivatives with respect to the joint velocity vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] rf Reference frame in which the Jacobian is expressed.
  /// \param[out] v_partial_dq Partial derivative of the joint velociy w.r.t. \f$ q \f$.
  /// \param[out] v_partial_dv Partial derivative of the joint velociy w.r.t. \f$ \dot{q} \f$.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename Matrix6xOut1, typename Matrix6xOut2>
  inline void getJointVelocityDerivatives(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                          const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                          const Model::JointIndex jointId,
                                          const ReferenceFrame rf,
                                          const Eigen::MatrixBase<Matrix6xOut1> & v_partial_dq,
                                          const Eigen::MatrixBase<Matrix6xOut2> & v_partial_dv);
  
  ///
  /// \brief Computes the partial derivaties of the spatial acceleration of a given with respect to
  ///        the joint configuration, velocity and acceleration.
  ///        You must first call computForwardKinematicsDerivatives before calling this function.
  ///        It is important to notice that a direct outcome (for free) of this algo is v_partial_dq and v_partial_dv which is equal to a_partial_da.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam Matrix6xOut1 Matrix6x containing the partial derivatives of the spatial velocity with respect to the joint configuration vector.
  /// \tparam Matrix6xOut2 Matrix6x containing the partial derivatives of the spatial acceleration with respect to the joint configuration vector.
  /// \tparam Matrix6xOut3 Matrix6x containing the partial derivatives of the spatial acceleration with respect to the joint velocity vector.
  /// \tparam Matrix6xOut4 Matrix6x containing the partial derivatives of the spatial acceleration with respect to the joint acceleration vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] jointId Index of the joint in model.
  /// \param[in] rf Reference frame in which the Jacobian is expressed.
  /// \param[out] v_partial_dq Partial derivative of the joint spatial velocity w.r.t. \f$ q \f$.
  /// \param[out] a_partial_dq Partial derivative of the joint spatial acceleration w.r.t. \f$ q \f$.
  /// \param[out] a_partial_dq Partial derivative of the joint spatial acceleration w.r.t. \f$ \dot{q} \f$.
  /// \param[out] a_partial_dq Partial derivative of the joint spatial acceleration w.r.t. \f$ \ddot{q} \f$.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename Matrix6xOut1, typename Matrix6xOut2, typename Matrix6xOut3, typename Matrix6xOut4>
  inline void getJointAccelerationDerivatives(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                              const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                              const Model::JointIndex jointId,
                                              const ReferenceFrame rf,
                                              const Eigen::MatrixBase<Matrix6xOut1> & v_partial_dq,
                                              const Eigen::MatrixBase<Matrix6xOut2> & a_partial_dq,
                                              const Eigen::MatrixBase<Matrix6xOut3> & a_partial_dv,
                                              const Eigen::MatrixBase<Matrix6xOut4> & a_partial_da);
  
  ///
  /// \brief Computes the partial derivaties of the spatial acceleration of a given with respect to
  ///        the joint configuration, velocity and acceleration.
  ///        You must first call computForwardKinematicsDerivatives before calling this function.
  ///        It is important to notice that a direct outcome (for free) of this algo is v_partial_dq and v_partial_dv which is equal to a_partial_da.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam Matrix6xOut1 Matrix6x containing the partial derivatives of the spatial velocity with respect to the joint configuration vector.
  /// \tparam Matrix6xOut2 Matrix6x containing the partial derivatives of the spatial velocity with respect to the joint velocity vector.
  /// \tparam Matrix6xOut3 Matrix6x containing the partial derivatives of the spatial acceleration with respect to the joint configuration vector.
  /// \tparam Matrix6xOut4 Matrix6x containing the partial derivatives of the spatial acceleration with respect to the joint velocity vector.
  /// \tparam Matrix6xOut5 Matrix6x containing the partial derivatives of the spatial acceleration with respect to the joint acceleration vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] jointId Index of the joint in model.
  /// \param[in] rf Reference frame in which the Jacobian is expressed.
  /// \param[out] v_partial_dq Partial derivative of the joint spatial velocity w.r.t. \f$ q \f$.
  /// \param[out] v_partial_dv Partial derivative of the joint spatial velociy w.r.t. \f$ \dot{q} \f$.
  /// \param[out] a_partial_dq Partial derivative of the joint spatial acceleration w.r.t. \f$ q \f$.
  /// \param[out] a_partial_dq Partial derivative of the joint spatial acceleration w.r.t. \f$ \dot{q} \f$.
  /// \param[out] a_partial_dq Partial derivative of the joint spatial acceleration w.r.t. \f$ \ddot{q} \f$.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename Matrix6xOut1, typename Matrix6xOut2, typename Matrix6xOut3, typename Matrix6xOut4, typename Matrix6xOut5>
  inline void getJointAccelerationDerivatives(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                              const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                              const Model::JointIndex jointId,
                                              const ReferenceFrame rf,
                                              const Eigen::MatrixBase<Matrix6xOut1> & v_partial_dq,
                                              const Eigen::MatrixBase<Matrix6xOut2> & v_partial_dv,
                                              const Eigen::MatrixBase<Matrix6xOut3> & a_partial_dq,
                                              const Eigen::MatrixBase<Matrix6xOut4> & a_partial_dv,
                                              const Eigen::MatrixBase<Matrix6xOut5> & a_partial_da);

  ///
  /// \brief Computes all the terms required to compute the second order derivatives of the placement information, also know as the
  ///        kinematic Hessian. This function assumes that the joint Jacobians (a.k.a data.J) has been computed first. See \ref computeJointJacobians
  ///        for such a function.
  ///
  /// \tparam Scalar Scalar type of the kinematic model.
  /// \tparam Options Alignement options of the kinematic model.
  /// \tparam JointCollection Collection of Joint types.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  ///
  /// \remarks This function is also related to \see getJointKinematicHessian.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline void
  computeJointKinematicHessians(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                DataTpl<Scalar,Options,JointCollectionTpl> & data);

  ///
  /// \brief Computes all the terms required to compute the second order derivatives of the placement information, also know as the
  ///        kinematic Hessian.
  ///
  /// \tparam Scalar Scalar type of the kinematic model.
  /// \tparam Options Alignement options of the kinematic model.
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration (vector dim model.nq).
  ///
  /// \remarks This function is also related to \see getJointKinematicHessian.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
  inline void
  computeJointKinematicHessians(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                const Eigen::MatrixBase<ConfigVectorType> & q)
  {
    computeJointJacobians(model,data,q);
    computeJointKinematicHessians(model,data);
  }

  ///
  /// \brief Retrieves the kinematic Hessian of a given joint according to the values aleardy computed by computeJointKinematicHessians
  ///        and stored in data. While the kinematic Jacobian of a given joint frame corresponds to the first order derivative of the placement variation with
  ///        respect to \f$ q \f$, the kinematic Hessian corresponds to the second order derivation of placement variation, which in turns also corresponds
  ///        to the first order derivative of the kinematic Jacobian. The frame in which the kinematic Hessian is precised by the input argument rf.
  ///
  /// \tparam Scalar Scalar type of the kinematic model.
  /// \tparam Options Alignement options of the kinematic model.
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] joint_id Index of the joint in model.
  /// \param[in] rf Reference frame with respect to which the derivative of the Jacobian is expressed
  /// \param[out] kinematic_hessian Second order derivative of the joint placement w.r.t. \f$ q \f$ expressed in the frame given by rf.
  ///
  /// \remarks This function is also related to \see computeJointKinematicHessians. kinematic_hessian has to be initialized with zero when calling this function
  ///          for the first time and there is no dynamic memory allocation.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline void
  getJointKinematicHessian(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                           const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                           const Model::JointIndex joint_id,
                           const ReferenceFrame rf,
                           Tensor<Scalar,3,Options> & kinematic_hessian);

  ///
  /// \brief Retrieves the kinematic Hessian of a given joint according to the values aleardy computed by computeJointKinematicHessians
  ///        and stored in data. While the kinematic Jacobian of a given joint frame corresponds to the first order derivative of the placement variation with
  ///        respect to \f$ q \f$, the kinematic Hessian corresponds to the second order derivation of placement variation, which in turns also corresponds
  ///        to the first order derivative of the kinematic Jacobian.
  ///
  /// \tparam Scalar Scalar type of the kinematic model.
  /// \tparam Options Alignement options of the kinematic model.
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] joint_id Index of the joint in model.
  /// \param[in] rf Reference frame with respect to which the derivative of the Jacobian is expressed.
  ///
  /// \returns The kinematic Hessian of the joint provided by its joint_id and expressed in the frame precised by the variable rf.
  ///
  /// \remarks This function is also related to \see computeJointKinematicHessians. This function will proceed to some dynamic memory allocation for the return type.
  ///          Please refer to getJointKinematicHessian for a version without dynamic memory allocation.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline Tensor<Scalar,3,Options>
  getJointKinematicHessian(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                           const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                           const Model::JointIndex joint_id,
                           const ReferenceFrame rf)
  {
    typedef Tensor<Scalar,3,Options> ReturnType;
    ReturnType res(6,model.nv,model.nv); res.setZero();
    getJointKinematicHessian(model,data,joint_id,rf,res);
    return res;
  }

} // namespace pinocchio 

#include "pinocchio/algorithm/kinematics-derivatives.hxx"

#endif // ifndef __pinocchio_algorithm_kinematics_derivatives_hpp__
