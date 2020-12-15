//
// Copyright (c) 2016-2020 CNRS INRIA
//

#ifndef __pinocchio_math_rpy_hpp__
#define __pinocchio_math_rpy_hpp__

#include "pinocchio/math/fwd.hpp"
#include "pinocchio/math/comparison-operators.hpp"
#include "pinocchio/multibody/fwd.hpp"

namespace pinocchio
{
  namespace rpy
  {
    ///
    /// \brief Convert from Roll, Pitch, Yaw to rotation Matrix
    ///
    /// Given \f$r, p, y\f$, the rotation is given as \f$ R = R_z(y)R_y(p)R_x(r) \f$,
    /// where \f$R_{\alpha}(\theta)\f$ denotes the rotation of \f$\theta\f$ degrees
    /// around axis \f$\alpha\f$.
    ///
    template<typename Scalar>
    Eigen::Matrix<Scalar,3,3>
    rpyToMatrix(const Scalar& r,
                const Scalar& p,
                const Scalar& y);

    ///
    /// \brief Convert from Roll, Pitch, Yaw to rotation Matrix
    ///
    /// Given a vector \f$(r, p, y)\f$, the rotation is given as \f$ R = R_z(y)R_y(p)R_x(r) \f$,
    /// where \f$R_{\alpha}(\theta)\f$ denotes the rotation of \f$\theta\f$ degrees
    /// around axis \f$\alpha\f$.
    ///
    template<typename Vector3Like>
    Eigen::Matrix<typename Vector3Like::Scalar,3,3,PINOCCHIO_EIGEN_PLAIN_TYPE(Vector3Like)::Options>
    rpyToMatrix(const Eigen::MatrixBase<Vector3Like> & rpy);

    ///
    /// \brief Convert from Transformation Matrix to Roll, Pitch, Yaw
    ///
    /// Given a rotation matrix \f$R\f$, the angles \f$r, p, y\f$ are given
    /// so that \f$ R = R_z(y)R_y(p)R_x(r) \f$,
    /// where \f$R_{\alpha}(\theta)\f$ denotes the rotation of \f$\theta\f$ degrees
    /// around axis \f$\alpha\f$.
    /// The angles are guaranteed to be in the ranges \f$r\in[-\pi,\pi]\f$
    /// \f$p\in[-\frac{\pi}{2},\frac{\pi}{2}]\f$ \f$y\in[-\pi,\pi]\f$,
    /// unlike Eigen's eulerAngles() function
    ///
    /// \warning the method assumes \f$R\f$ is a rotation matrix. If it is not, the result is undefined.
    ///
    template<typename Matrix3Like>
    Eigen::Matrix<typename Matrix3Like::Scalar,3,1,PINOCCHIO_EIGEN_PLAIN_TYPE(Matrix3Like)::Options>
    matrixToRpy(const Eigen::MatrixBase<Matrix3Like> & R);

    ///
    /// \brief Compute the Jacobian of the Roll-Pitch-Yaw conversion
    ///
    /// Given \f$\phi = (r, p, y)\f$ and reference frame F (either LOCAL or WORLD),
    /// the Jacobian is such that \f$ {}^F\omega = J_F(\phi)\dot{\phi} \f$,
    /// where \f$ {}^F\omega \f$ is the angular velocity expressed in frame F
    /// and \f$ J_F \f$ is the Jacobian computed with reference frame F
    ///
    /// \param[in] rpy Roll-Pitch-Yaw vector
    /// \param[in] rf  Reference frame in which the angular velocity is expressed
    ///
    /// \return The Jacobian of the Roll-Pitch-Yaw conversion in the appropriate frame
    ///
    /// \note for the purpose of this function, WORLD and LOCAL_WORLD_ALIGNED are equivalent
    ///
    template<typename Vector3Like>
    Eigen::Matrix<typename Vector3Like::Scalar,3,3,PINOCCHIO_EIGEN_PLAIN_TYPE(Vector3Like)::Options>
    computeRpyJacobian(const Eigen::MatrixBase<Vector3Like> & rpy, const ReferenceFrame rf=LOCAL);
  
    ///
    /// \brief Compute the inverse Jacobian of the Roll-Pitch-Yaw conversion
    ///
    /// Given \f$\phi = (r, p, y)\f$ and reference frame F (either LOCAL or WORLD),
    /// the Jacobian is such that \f$ {}^F\omega = J_F(\phi)\dot{\phi} \f$,
    /// where \f$ {}^F\omega \f$ is the angular velocity expressed in frame F
    /// and \f$ J_F \f$ is the Jacobian computed with reference frame F
    ///
    /// \param[in] rpy Roll-Pitch-Yaw vector
    /// \param[in] rf  Reference frame in which the angular velocity is expressed
    ///
    /// \return The inverse of the Jacobian of the Roll-Pitch-Yaw conversion in the appropriate frame
    ///
    /// \note for the purpose of this function, WORLD and LOCAL_WORLD_ALIGNED are equivalent
    ///
    template<typename Vector3Like>
    Eigen::Matrix<typename Vector3Like::Scalar,3,3,PINOCCHIO_EIGEN_PLAIN_TYPE(Vector3Like)::Options>
    computeRpyJacobianInverse(const Eigen::MatrixBase<Vector3Like> & rpy, const ReferenceFrame rf=LOCAL);

    ///
    /// \brief Compute the time derivative Jacobian of the Roll-Pitch-Yaw conversion
    ///
    /// Given \f$\phi = (r, p, y)\f$ and reference frame F (either LOCAL or WORLD),
    /// the Jacobian is such that \f$ {}^F\omega = J_F(\phi)\dot{\phi} \f$,
    /// where \f$ {}^F\omega \f$ is the angular velocity expressed in frame F
    /// and \f$ J_F \f$ is the Jacobian computed with reference frame F
    ///
    /// \param[in] rpy     Roll-Pitch-Yaw vector
    /// \param[in] rpydot  Time derivative of the Roll-Pitch-Yaw vector
    /// \param[in] rf      Reference frame in which the angular velocity is expressed
    ///
    /// \return The time derivative of the Jacobian of the Roll-Pitch-Yaw conversion in the appropriate frame
    ///
    /// \note for the purpose of this function, WORLD and LOCAL_WORLD_ALIGNED are equivalent
    ///
    template<typename Vector3Like0, typename Vector3Like1>
    Eigen::Matrix<typename Vector3Like0::Scalar,3,3,PINOCCHIO_EIGEN_PLAIN_TYPE(Vector3Like0)::Options>
    computeRpyJacobianTimeDerivative(const Eigen::MatrixBase<Vector3Like0> & rpy, const Eigen::MatrixBase<Vector3Like1> & rpydot, const ReferenceFrame rf=LOCAL);
  } // namespace rpy
}

/* --- Details -------------------------------------------------------------------- */
#include "pinocchio/math/rpy.hxx"

#endif //#ifndef __pinocchio_math_rpy_hpp__
