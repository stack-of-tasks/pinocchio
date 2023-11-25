//
// Copyright (c) 2015-2020 CNRS INRIA
//

#ifndef __pinocchio_algorithm_center_of_mass_hpp__
#define __pinocchio_algorithm_center_of_mass_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

namespace pinocchio
{

  ///
  /// \brief Compute the total mass of the model and return it.
  ///
  /// \param[in] model The model structure of the rigid body system.
  ///
  /// \return Total mass of the model.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline Scalar computeTotalMass(const ModelTpl<Scalar,Options,JointCollectionTpl> & model);

  ///
  /// \brief Compute the total mass of the model, put it in data.mass[0] and return it.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  ///
  /// \warning This method does not fill the whole data.mass vector. Only data.mass[0] is updated.
  /// If you need the whole data.mass vector to be computed, use computeSubtreeMasses
  ///
  /// \return Total mass of the model.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline Scalar computeTotalMass(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                 DataTpl<Scalar,Options,JointCollectionTpl> & data);

  ///
  /// \brief Compute the mass of each kinematic subtree and store it in data.mass. The element mass[0] corresponds to the total mass of the model.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  ///
  /// \note If you are only interested in knowing the total mass of the model, computeTotalMass will probably be slightly faster.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline void computeSubtreeMasses(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                   DataTpl<Scalar,Options,JointCollectionTpl> & data);

  ///
  /// \brief Computes the center of mass position of a given model according to a particular joint configuration.
  ///        The result is accessible through data.com[0] for the full body com and data.com[i] for the subtree supported by joint i (expressed in the joint i frame).
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] computeSubtreeComs If true, the algorithm computes also the center of mass of the subtrees.
  ///
  /// \return The center of mass position of the full rigid body system expressed in the world frame.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::Vector3 &
  centerOfMass(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
               DataTpl<Scalar,Options,JointCollectionTpl> & data,
               const Eigen::MatrixBase<ConfigVectorType> & q,
               const bool computeSubtreeComs = true);

  ///
  /// \brief Computes the center of mass position and velocity of a given model according to a particular joint configuration and velocity.
  ///        The result is accessible through data.com[0], data.vcom[0] for the full body com position and velocity.
  ///        And data.com[i] and data.vcom[i] for the subtree supported by joint i (expressed in the joint i frame).
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  /// \tparam TangentVectorType Type of the joint velocity vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] v The joint velocity vector (dim model.nv).
  /// \param[in] computeSubtreeComs If true, the algorithm computes also the center of mass of the subtrees.
  ///
  /// \return The center of mass position of the full rigid body system expressed in the world frame.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::Vector3 &
  centerOfMass(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
               DataTpl<Scalar,Options,JointCollectionTpl> & data,
               const Eigen::MatrixBase<ConfigVectorType> & q,
               const Eigen::MatrixBase<TangentVectorType> & v,
               const bool computeSubtreeComs = true);

  ///
  /// \brief Computes the center of mass position, velocity and acceleration of a given model according to a particular joint configuration, velocity and acceleration.
  ///        The result is accessible through data.com[0], data.vcom[0], data.acom[0] for the full body com position, velocity and acceleation.
  ///        And data.com[i], data.vcom[i] and data.acom[i] for the subtree supported by joint i (expressed in the joint i frame).
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
  /// \param[in] computeSubtreeComs If true, the algorithm computes also the center of mass of the subtrees.
  ///
  /// \return The center of mass position of the full rigid body system expressed in the world frame.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::Vector3 &
  centerOfMass(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
               DataTpl<Scalar,Options,JointCollectionTpl> & data,
               const Eigen::MatrixBase<ConfigVectorType> & q,
               const Eigen::MatrixBase<TangentVectorType1> & v,
               const Eigen::MatrixBase<TangentVectorType2> & a,
               const bool computeSubtreeComs = true);

  ///
  /// \brief Computes the center of mass position, velocity and acceleration of a given model according to the current kinematic values contained in data and the requested kinematic_level.
  ///        The result is accessible through data.com[0], data.vcom[0] and data.acom[0] for the full body com position and velocity.
  ///        And data.com[i] and data.vcom[i] for the subtree supported by joint i (expressed in the joint i frame).
  ///
  /// \tparam JointCollection Collection of Joint types.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] kinematic_level if = POSITION, computes the CoM position, if = VELOCITY, also computes the CoM velocity and if = ACCELERATION, it also computes the CoM acceleration.
  /// \param[in] computeSubtreeComs If true, the algorithm computes also the center of mass of the subtrees.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  const typename DataTpl<Scalar,Options,JointCollectionTpl>::Vector3 &
  centerOfMass(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
               DataTpl<Scalar,Options,JointCollectionTpl> & data,
               KinematicLevel kinematic_level,
               const bool computeSubtreeComs = true);

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  PINOCCHIO_DEPRECATED
  inline void centerOfMass(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                           DataTpl<Scalar,Options,JointCollectionTpl> & data,
                           int kinematic_level,
                           const bool computeSubtreeComs = true)
  {
    centerOfMass(model,data,static_cast<KinematicLevel>(kinematic_level),computeSubtreeComs);
  }

  ///
  /// \brief Computes the center of mass position, velocity and acceleration of a given model according to the current kinematic values contained in data.
  ///        The result is accessible through data.com[0], data.vcom[0] and data.acom[0] for the full body com position and velocity.
  ///        And data.com[i] and data.vcom[i] for the subtree supported by joint i (expressed in the joint i frame).
  ///
  /// \tparam JointCollection Collection of Joint types.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] computeSubtreeComs If true, the algorithm computes also the center of mass of the subtrees, expressed in the local coordinate frame of each joint.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  const typename DataTpl<Scalar,Options,JointCollectionTpl>::Vector3 &
  centerOfMass(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
               DataTpl<Scalar,Options,JointCollectionTpl> & data,
               const bool computeSubtreeComs = true)
  { return centerOfMass(model,data,ACCELERATION,computeSubtreeComs); }

  ///
  /// \brief Computes both the jacobian and the the center of mass position of a given model according to a particular joint configuration.
  ///        The results are accessible through data.Jcom and data.com[0] and are both expressed in the world frame. In addition, the algorithm also computes the Jacobian of all the joints (\sa pinocchio::computeJointJacobians).
  ///        And data.com[i] gives the center of mass of the subtree supported by joint i (expressed in the world frame).
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] computeSubtreeComs If true, the algorithm also computes the centers of mass of the subtrees, expressed in the world coordinate frame.
  ///
  /// \return The jacobian of center of mass position of the rigid body system expressed in the world frame (matrix 3 x model.nv).
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::Matrix3x &
  jacobianCenterOfMass(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                       DataTpl<Scalar,Options,JointCollectionTpl> & data,
                       const Eigen::MatrixBase<ConfigVectorType> & q,
                       const bool computeSubtreeComs = true);

  ///
  /// \brief Computes both the jacobian and the the center of mass position of a given model according to the current value stored in data.
  ///        It assumes that forwardKinematics has been called first.
  ///        The results are accessible through data.Jcom and data.com[0] and are both expressed in the world frame. In addition, the algorithm also computes the Jacobian of all the joints (\sa pinocchio::computeJointJacobians).
  ///        And data.com[i] gives the center of mass of the subtree supported by joint i (expressed in the world frame).
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] computeSubtreeComs If true, the algorithm also computes the center of mass of the subtrees, expressed in the world coordinate frame.
  ///
  /// \return The jacobian of center of mass position of the rigid body system expressed in the world frame (matrix 3 x model.nv).
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::Matrix3x &
  jacobianCenterOfMass(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                       DataTpl<Scalar,Options,JointCollectionTpl> & data,
                       const bool computeSubtreeComs = true);

  ///
  /// \brief Computes the Jacobian of the center of mass of the given subtree according to a particular joint configuration.
  ///        In addition, the algorithm also computes the Jacobian of all the joints (\sa pinocchio::computeJointJacobians).
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.
  /// \tparam Matrix3xLike Type of the output Jacobian matrix.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] rootSubtreeId Index of the parent joint supporting the subtree.
  /// \param[out] res The Jacobian matrix where the results will be stored in (dim 3 x model.nv). You must first fill J with zero elements, e.g. J.setZero().
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename Matrix3xLike>
  inline void
  jacobianSubtreeCenterOfMass(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                              DataTpl<Scalar,Options,JointCollectionTpl> & data,
                              const Eigen::MatrixBase<ConfigVectorType> & q,
                              const JointIndex & rootSubtreeId,
                              const Eigen::MatrixBase<Matrix3xLike> & res);

  ///
  /// \brief Computes the Jacobian of the center of mass of the given subtree according to the current value stored in data.
  ///        It assumes that forwardKinematics has been called first.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam Matrix3xLike Type of the output Jacobian matrix.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] rootSubtreeId Index of the parent joint supporting the subtree.
  /// \param[out] res The Jacobian matrix where the results will be stored in (dim 3 x model.nv). You must first fill J with zero elements, e.g. J.setZero().
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename Matrix3xLike>
  inline void
  jacobianSubtreeCenterOfMass(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                              DataTpl<Scalar,Options,JointCollectionTpl> & data,
                              const JointIndex & rootSubtreeId,
                              const Eigen::MatrixBase<Matrix3xLike> & res);
  
  ///
  /// \brief Retrieves the Jacobian of the center of mass of the given subtree according to the current value stored in data.
  ///        It assumes that pinocchio::jacobianCenterOfMass has been called first with computeSubtreeComs equals to true.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam Matrix3xLike Type of the output Jacobian matrix.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] rootSubtreeId Index of the parent joint supporting the subtree.
  /// \param[out] res The Jacobian matrix where the results will be stored in (dim 3 x model.nv). You must first fill J with zero elements, e.g. J.setZero().
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename Matrix3xLike>
  inline void
  getJacobianSubtreeCenterOfMass(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                 const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                 const JointIndex & rootSubtreeId,
                                 const Eigen::MatrixBase<Matrix3xLike> & res);


  /* If the CRBA has been run, then both COM and Jcom are easily available from
   * the joint space mass matrix (data.M).
   * Use the following function to infer them directly. In that case,
   * the COM subtrees (also easily available from CRBA data) are not
   * explicitely set. Use data.Ycrb[i].lever() to get them. */
  ///
  /// \brief Extracts the center of mass position from the joint space inertia matrix (also called the mass matrix).
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam Matrix3xLike Type of the output Jacobian matrix.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  ///
  /// \return The center of mass position of the rigid body system expressed in the world frame (vector 3).
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::Vector3 &
  getComFromCrba(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                 DataTpl<Scalar,Options,JointCollectionTpl> & data);

  ///
  /// \brief Extracts both the jacobian of the center of mass (CoM), the total mass of the system and the CoM position from the joint space inertia matrix (also called the mass matrix).
  ///        The results are accessible through data.Jcom, data.mass[0] and data.com[0] and are both expressed in the world frame.
  ///
  /// \tparam JointCollection Collection of Joint types.
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  ///
  /// \return The jacobian of the CoM expressed in the world frame (matrix 3 x model.nv).
  ///
  /// \remark This extraction of inertial quantities is only valid for free-floating base systems.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::Matrix3x &
  getJacobianComFromCrba(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                         DataTpl<Scalar,Options,JointCollectionTpl> & data);

} // namespace pinocchio

/* --- Details -------------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------------- */
#include "pinocchio/algorithm/center-of-mass.hxx"

#endif // ifndef __pinocchio_algorithm_center_of_mass_hpp__
