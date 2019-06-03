//
// Copyright (c) 2018 CNRS
//

#ifndef __pinocchio_regressor_hpp__
#define __pinocchio_regressor_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

namespace pinocchio
{
  
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
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
    inline typename DataTpl<Scalar,Options,JointCollectionTpl>::Matrix3x &
    computeStaticRegressor(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                           DataTpl<Scalar,Options,JointCollectionTpl> & data,
                           const Eigen::MatrixBase<ConfigVectorType> & q);
  }

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
  /// \brief Computes the regressor for the dynamic parameters of a rigid body attached to a given joint.
  ///
  /// This algorithm assumes RNEA has been run to compute the acceleration and gravitational effects.
  ///
  /// The result is such that
  /// \f$ f = jointBodyRegressor(model,data,jointId) * I.toDynamicParameters() \f$
  /// where \f$ f \f$ is the net force acting on the body, including gravity
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] jointId The id of the joint.
  ///
  /// \return The regressor of the body.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline Eigen::Matrix<Scalar,6,10,Options>
  jointBodyRegressor(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                     DataTpl<Scalar,Options,JointCollectionTpl> & data,
                     JointIndex jointId);
  
} // namespace pinocchio

/* --- Details -------------------------------------------------------------------- */
#include "pinocchio/algorithm/regressor.hxx"

#endif // ifndef __pinocchio_regressor_hpp__
