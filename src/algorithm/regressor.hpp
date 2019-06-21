//
// Copyright (c) 2018 CNRS
//

#ifndef __pinocchio_regressor_hpp__
#define __pinocchio_regressor_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

namespace pinocchio
{
    
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
    inline PINOCCHIO_DEPRECATED typename DataTpl<Scalar,Options,JointCollectionTpl>::Matrix3x &
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
  bodyRegressor(const MotionDense<MotionVelocity> & v, const MotionDense<MotionAcceleration> & a, const Eigen::MatrixBase<OutputType> & regressor);

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
  bodyRegressor(const MotionDense<MotionVelocity> & v, const MotionDense<MotionAcceleration> & a);

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
  /// \return The regressor of the body.
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

#endif // ifndef __pinocchio_regressor_hpp__
