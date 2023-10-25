//
// Copyright (c) 2018-2020 CNRS INRIA
//

#ifndef __pinocchio_algorithm_regressor_hpp__
#define __pinocchio_algorithm_regressor_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

namespace pinocchio
{
  
  ///
  /// \copydoc computeJointKinematicRegressor(const ModelTpl<Scalar,Options,JointCollectionTpl> &,const DataTpl<Scalar,Options,JointCollectionTpl> &, const JointIndex, const ReferenceFrame, const SE3Tpl<Scalar,Options> &)
  ///
  /// \param[out] kinematic_regressor The kinematic regressor containing the result. Matrix of size 6*(model.njoints-1) initialized to 0.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename Matrix6xReturnType>
  void computeJointKinematicRegressor(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                      const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                      const JointIndex joint_id,
                                      const ReferenceFrame rf,
                                      const SE3Tpl<Scalar,Options> & placement,
                                      const Eigen::MatrixBase<Matrix6xReturnType> & kinematic_regressor);


  ///
  /// \brief Computes the kinematic regressor that links the joint placements variations of the whole kinematic tree
  ///        to the placement variation of the frame rigidly attached to the joint and given by its placement w.r.t. to the joint frame.
  ///
  /// \remarks It assumes that the \ref forwardKinematics(const ModelTpl<Scalar,Options,JointCollectionTpl> &, DataTpl<Scalar,Options,JointCollectionTpl> &, const Eigen::MatrixBase<ConfigVectorType> &) has been called first.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] joint_id Index of the joint.
  /// \param[in] rf Reference frame in which the result is expressed (LOCAL, LOCAL_WORLD_ALIGNED or WORLD).
  /// \param[in] placement Relative placement to the joint frame.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  typename DataTpl<Scalar,Options,JointCollectionTpl>::Matrix6x
  computeJointKinematicRegressor(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                 const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                 const JointIndex joint_id,
                                 const ReferenceFrame rf,
                                 const SE3Tpl<Scalar,Options> & placement)
  {
    typedef typename DataTpl<Scalar,Options,JointCollectionTpl>::Matrix6x ReturnType;
    ReturnType res(ReturnType::Zero(6,(model.njoints-1)*6));
    
    computeJointKinematicRegressor(model,data,joint_id,rf,placement,res);
    
    return res;
  }

  ///
  /// \copydoc computeJointKinematicRegressor(const ModelTpl<Scalar,Options,JointCollectionTpl> &,const DataTpl<Scalar,Options,JointCollectionTpl> &, const JointIndex, const ReferenceFrame)
  ///
  /// \param[out] kinematic_regressor The kinematic regressor containing the result. Matrix of size 6*(model.njoints-1) initialized to 0.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename Matrix6xReturnType>
  void computeJointKinematicRegressor(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                      const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                      const JointIndex joint_id,
                                      const ReferenceFrame rf,
                                      const Eigen::MatrixBase<Matrix6xReturnType> & kinematic_regressor);

  ///
  /// \brief Computes the kinematic regressor that links the joint placement variations of the whole kinematic tree
  ///        to the placement variation of the joint given as input.
  ///
  /// \remarks It assumes that the \ref forwardKinematics(const ModelTpl<Scalar,Options,JointCollectionTpl> &, DataTpl<Scalar,Options,JointCollectionTpl> &, const Eigen::MatrixBase<ConfigVectorType> &) has been called first.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] joint_id Index of the joint.
  /// \param[in] rf Reference frame in which the result is expressed (LOCAL, LOCAL_WORLD_ALIGNED or WORLD).
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  typename DataTpl<Scalar,Options,JointCollectionTpl>::Matrix6x
  computeJointKinematicRegressor(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                 const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                 const JointIndex joint_id,
                                 const ReferenceFrame rf)
  {
    typedef typename DataTpl<Scalar,Options,JointCollectionTpl>::Matrix6x ReturnType;
    ReturnType res(ReturnType::Zero(6,(model.njoints-1)*6));
    
    computeJointKinematicRegressor(model,data,joint_id,rf,res);
    
    return res;
  }
  
  ///
  /// \copydoc computeFrameKinematicRegressor(const ModelTpl<Scalar,Options,JointCollectionTpl> &, DataTpl<Scalar,Options,JointCollectionTpl> &, const FrameIndex, const ReferenceFrame)
  ///
  /// \param[out] kinematic_regressor The kinematic regressor containing the result. Matrix of size 6*(model.njoints-1) initialized to 0.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename Matrix6xReturnType>
  void computeFrameKinematicRegressor(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                      DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                      const FrameIndex frame_id,
                                      const ReferenceFrame rf,
                                      const Eigen::MatrixBase<Matrix6xReturnType> & kinematic_regressor);
  
  ///
  /// \brief Computes the kinematic regressor that links the joint placement variations of the whole kinematic tree
  ///        to the placement variation of the frame given as input.
  ///
  /// \remarks It assumes that the \ref framesForwardKinematics(const ModelTpl<Scalar,Options,JointCollectionTpl> &, DataTpl<Scalar,Options,JointCollectionTpl> &, const Eigen::MatrixBase<ConfigVectorType> &) has been called first.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] frame_id Index of the frame.
  /// \param[in] rf Reference frame in which the result is expressed (LOCAL, LOCAL_WORLD_ALIGNED or WORLD).
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  typename DataTpl<Scalar,Options,JointCollectionTpl>::Matrix6x
  computeFrameKinematicRegressor(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                 DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                 const FrameIndex frame_id,
                                 const ReferenceFrame rf)
  {
    typedef typename DataTpl<Scalar,Options,JointCollectionTpl>::Matrix6x ReturnType;
    ReturnType res(ReturnType::Zero(6,(model.njoints-1)*6));
    
    computeFrameKinematicRegressor(model,data,frame_id,rf,res);
    
    return res;
  }

  ///
  /// \brief Computes the static regressor that links the center of mass positions of all the links
  ///        to the center of mass of the complete model according to the current configuration of the robot.
  ///
  /// The result is stored in `data.staticRegressor` and it corresponds to a matrix \f$ Y \f$ such that
  /// \f$ c = Y(q,\dot{q},\ddot{q})\tilde{\pi} \f$
  /// where \f$ \tilde{\pi} = (\tilde{\pi}_1^T\ \dots\ \tilde{\pi}_n^T)^T \f$ and
  /// \f$ \tilde{\pi}_i = \text{model.inertias[i].toDynamicParameters().head<4>()} \f$
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  ///
  /// \return The static regressor of the system.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
  inline typename DataTpl<Scalar,Options,JointCollectionTpl>::Matrix3x &
  computeStaticRegressor(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                         DataTpl<Scalar,Options,JointCollectionTpl> & data,
                         const Eigen::MatrixBase<ConfigVectorType> & q);

  namespace regressor
  {
    
    ///
    /// \brief Computes the static regressor that links the center of mass positions of all the links
    ///        to the center of mass of the complete model according to the current configuration of the robot.
    ///
    /// \tparam JointCollection Collection of Joint types.
    /// \tparam ConfigVectorType Type of the joint configuration vector.
    ///
    /// \param[in] model The model structure of the rigid body system.
    /// \param[in] data The data structure of the rigid body system.
    /// \param[in] q The joint configuration vector (dim model.nq).
    ///
    /// \return The static regressor of the system.
    ///
    /// \deprecated This function is now in the main pinocchio namespace
    ///
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
    PINOCCHIO_DEPRECATED typename DataTpl<Scalar,Options,JointCollectionTpl>::Matrix3x &
    computeStaticRegressor(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                           DataTpl<Scalar,Options,JointCollectionTpl> & data,
                           const Eigen::MatrixBase<ConfigVectorType> & q)
    {
        return ::pinocchio::computeStaticRegressor(model,data,q);
    }
  }

  ///
  /// \brief Computes the regressor for the dynamic parameters of a single rigid body.
  ///
  /// The result is such that
  /// \f$ I a + v \times I v = bodyRegressor(v,a) * I.toDynamicParameters() \f$
  ///
  /// \param[in] v Velocity of the rigid body
  /// \param[in] a Acceleration of the rigid body
  /// \param[out] regressor The resulting regressor of the body.
  ///
  template<typename MotionVelocity, typename MotionAcceleration, typename OutputType>
  inline void
  bodyRegressor(const MotionDense<MotionVelocity> & v,
                const MotionDense<MotionAcceleration> & a,
                const Eigen::MatrixBase<OutputType> & regressor);

  ///
  /// \brief Computes the regressor for the dynamic parameters of a single rigid body.
  ///
  /// The result is such that
  /// \f$ I a + v \times I v = bodyRegressor(v,a) * I.toDynamicParameters() \f$
  ///
  /// \param[in] v Velocity of the rigid body
  /// \param[in] a Acceleration of the rigid body
  ///
  /// \return The regressor of the body.
  ///
  template<typename MotionVelocity, typename MotionAcceleration>
  inline Eigen::Matrix<typename MotionVelocity::Scalar,6,10,PINOCCHIO_EIGEN_PLAIN_TYPE(typename MotionVelocity::Vector3)::Options>
  bodyRegressor(const MotionDense<MotionVelocity> & v,
                const MotionDense<MotionAcceleration> & a);

  ///
  /// \brief Computes the regressor for the dynamic parameters of a rigid body attached to a given joint,
  ///        puts the result in data.bodyRegressor and returns it.
  ///
  /// This algorithm assumes RNEA has been run to compute the acceleration and gravitational effects.
  ///
  /// The result is such that
  /// \f$ f = \text{jointBodyRegressor(model,data,jointId) * I.toDynamicParameters()} \f$
  /// where \f$ f \f$ is the net force acting on the body, including gravity
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] jointId The id of the joint.
  ///
  /// \return The regressor of the body.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline typename DataTpl<Scalar,Options,JointCollectionTpl>::BodyRegressorType &
  jointBodyRegressor(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                     DataTpl<Scalar,Options,JointCollectionTpl> & data,
                     JointIndex jointId);

  ///
  /// \brief Computes the regressor for the dynamic parameters of a rigid body attached to a given frame,
  ///        puts the result in data.bodyRegressor and returns it.
  ///
  /// This algorithm assumes RNEA has been run to compute the acceleration and gravitational effects.
  ///
  /// The result is such that
  /// \f$ f = \text{frameBodyRegressor(model,data,frameId) * I.toDynamicParameters()} \f$
  /// where \f$ f \f$ is the net force acting on the body, including gravity
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] frameId The id of the frame.
  ///
  /// \return The dynamic regressor of the body.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline typename DataTpl<Scalar,Options,JointCollectionTpl>::BodyRegressorType &
  frameBodyRegressor(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                     DataTpl<Scalar,Options,JointCollectionTpl> & data,
                     FrameIndex frameId);

  ///
  /// \brief Computes the joint torque regressor that links the joint torque
  ///        to the dynamic parameters of each link according to the current the robot motion.
  ///
  /// The result is stored in `data.jointTorqueRegressor` and it corresponds to a matrix \f$ Y \f$ such that
  /// \f$ \tau = Y(q,\dot{q},\ddot{q})\pi \f$
  /// where \f$ \pi = (\pi_1^T\ \dots\ \pi_n^T)^T \f$ and \f$ \pi_i = \text{model.inertias[i].toDynamicParameters()} \f$
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  /// \tparam TangentVectorType1 Type of the joint velocity vector.
  /// \tparam TangentVectorType2 Type of the joint acceleration vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] v The joint velocity vector (dim model.nv).
  /// \param[in] a The joint acceleration vector (dim model.nv).
  ///
  /// \return The joint torque regressor of the system.
  ///
  /// \warning This function writes temporary information in data.bodyRegressor. This means if you have valuable data in it it will be overwritten.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2>
  inline typename DataTpl<Scalar,Options,JointCollectionTpl>::MatrixXs &
  computeJointTorqueRegressor(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                              DataTpl<Scalar,Options,JointCollectionTpl> & data,
                              const Eigen::MatrixBase<ConfigVectorType> & q,
                              const Eigen::MatrixBase<TangentVectorType1> & v,
                              const Eigen::MatrixBase<TangentVectorType2> & a);

} // namespace pinocchio

/* --- Details -------------------------------------------------------------------- */
#include "pinocchio/algorithm/regressor.hxx"

#endif // ifndef __pinocchio_algorithm_regressor_hpp__
