//
// Copyright (c) 2016-2021 CNRS INRIA
//

#ifndef __pinocchio_algorithm_joint_configuration_hpp__
#define __pinocchio_algorithm_joint_configuration_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/liegroup/liegroup.hpp"

namespace pinocchio
{

  /// \name API with return value as argument
  /// \{

  /**
   *
   * @brief      Integrate a configuration vector for the specified model for a tangent vector during one unit time
   *
   * @details This function corresponds to the exponential map of the joint configuration Lie Group.
   *          Its output can be interpreted as the "sum" from the Lie algebra to the joint configuration space \f$ q \oplus v \f$.
   *
   * @param[in]  model   Model of the kinematic tree on which the integration is performed.
   * @param[in]  q       Initial configuration (size model.nq)
   * @param[in]  v       Joint velocity (size model.nv)
   *
   * @param[out] qout    The integrated configuration (size model.nq)
   *
   */
  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType, typename ReturnType>
  void
  integrate(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
            const Eigen::MatrixBase<ConfigVectorType> & q,
            const Eigen::MatrixBase<TangentVectorType> & v,
            const Eigen::MatrixBase<ReturnType> & qout);

  /**
   *
   * @brief      Integrate a configuration vector for the specified model for a tangent vector during one unit time
   *
   * @details This function corresponds to the exponential map of the joint configuration Lie Group.
   *          Its output can be interpreted as the "sum" from the Lie algebra to the joint configuration space \f$ q \oplus v \f$.
   *
   * @param[in]  model  Model of the kinematic tree on which the integration is performed.
   * @param[in]  q       Initial configuration (size model.nq)
   * @param[in]  v       Joint velocity (size model.nv)
   *
   * @param[out] qout    The integrated configuration (size model.nq)
   *
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType, typename ReturnType>
  void
  integrate(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
            const Eigen::MatrixBase<ConfigVectorType> & q,
            const Eigen::MatrixBase<TangentVectorType> & v,
            const Eigen::MatrixBase<ReturnType> & qout)
  {
    integrate<LieGroupMap,Scalar,Options,JointCollectionTpl,ConfigVectorType,TangentVectorType,ReturnType>(model, q.derived(), v.derived(), qout.derived());
  }

  /**
   *
   * @brief      Interpolate two configurations for a given model
   *
   * @param[in]  model   Model of the kinematic tree on which the interpolation is performed.
   * @param[in]  q0      Initial configuration vector (size model.nq)
   * @param[in]  q1      Final configuration vector (size model.nq)
   * @param[in]  u       u in [0;1] position along the interpolation.
   *
   * @param[out] qout    The interpolated configuration (q0 if u = 0, q1 if u = 1)
   *
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2, typename ReturnType>
  void
  interpolate(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
              const Eigen::MatrixBase<ConfigVectorIn1> & q0,
              const Eigen::MatrixBase<ConfigVectorIn2> & q1,
              const Scalar & u,
              const Eigen::MatrixBase<ReturnType> & qout);

  /**
   *
   * @brief      Interpolate two configurations for a given model
   *
   * @param[in]  model   Model of the kinematic tree on which the interpolation is performed.
   * @param[in]  q0      Initial configuration vector (size model.nq)
   * @param[in]  q1      Final configuration vector (size model.nq)
   * @param[in]  u       u in [0;1] position along the interpolation.
   *
   * @param[out] qout    The interpolated configuration (q0 if u = 0, q1 if u = 1)
   *
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2, typename ReturnType>
  void
  interpolate(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
              const Eigen::MatrixBase<ConfigVectorIn1> & q0,
              const Eigen::MatrixBase<ConfigVectorIn2> & q1,
              const Scalar & u,
              const Eigen::MatrixBase<ReturnType> & qout)
  {
    interpolate<LieGroupMap,Scalar,Options,JointCollectionTpl,ConfigVectorIn1,ConfigVectorIn2,ReturnType>(model, q0.derived(), q1.derived(), u, PINOCCHIO_EIGEN_CONST_CAST(ReturnType,qout));
  }

  /**
   *
   * @brief      Compute the tangent vector that must be integrated during one unit time to go from q0 to q1
   *
   * @details This function corresponds to the log map of the joint configuration Lie Group.
   *          Its output can be interpreted as a difference from the joint configuration space to the Lie algebra \f$ q_1 \ominus q_0 \f$.
   *
   * @param[in]  model   Model of the system on which the difference operation is performed.
   * @param[in]  q0      Initial configuration (size model.nq)
   * @param[in]  q1      Desired configuration (size model.nq)
   *
   * @param[out] dvout   The corresponding velocity (size model.nv)
   *
   */
  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2, typename ReturnType>
  void
  difference(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
             const Eigen::MatrixBase<ConfigVectorIn1> & q0,
             const Eigen::MatrixBase<ConfigVectorIn2> & q1,
             const Eigen::MatrixBase<ReturnType> & dvout);

  /**
   *
   * @brief      Compute the tangent vector that must be integrated during one unit time to go from q0 to q1
   *
   * @details This function corresponds to the log map of the joint configuration Lie Group.
   *          Its output can be interpreted as a difference from the joint configuration space to the Lie algebra \f$ q_1 \ominus q_0 \f$.
   *
   * @param[in]  model   Model of the system on which the difference operation is performed.
   * @param[in]  q0      Initial configuration (size model.nq)
   * @param[in]  q1      Desired configuration (size model.nq)
   *
   * @param[out] dvout   The corresponding velocity (size model.nv).
   *
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2, typename ReturnType>
  void
  difference(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
             const Eigen::MatrixBase<ConfigVectorIn1> & q0,
             const Eigen::MatrixBase<ConfigVectorIn2> & q1,
             const Eigen::MatrixBase<ReturnType> & dvout)
  {
    difference<LieGroupMap,Scalar,Options,JointCollectionTpl,ConfigVectorIn1,ConfigVectorIn2,ReturnType>(model,q0.derived(),q1.derived(),PINOCCHIO_EIGEN_CONST_CAST(ReturnType,dvout));
  }

  /**
   *
   * @brief      Squared distance between two configuration vectors
   *
   * @param[in]  model      Model of the system on which the squared distance operation is performed.
   * @param[in]  q0             Configuration 0 (size model.nq)
   * @param[in]  q1             Configuration 1 (size model.nq)
   * @param[out] out          The corresponding squared distances for each joint (size model.njoints-1 = number of joints).
   *
   */
  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2, typename ReturnType>
  void
  squaredDistance(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  const Eigen::MatrixBase<ConfigVectorIn1> & q0,
                  const Eigen::MatrixBase<ConfigVectorIn2> & q1,
                  const Eigen::MatrixBase<ReturnType> & out);

  /**
   *
   * @brief      Squared distance between two configuration vectors
   *
   * @param[in]  model      Model of the system on which the squared distance operation is performed.
   * @param[in]  q0             Configuration 0 (size model.nq)
   * @param[in]  q1             Configuration 1 (size model.nq)
   * @param[out] out          The corresponding squared distances for each joint (size model.njoints-1 = number of joints).
   *
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2, typename ReturnType>
  void
  squaredDistance(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  const Eigen::MatrixBase<ConfigVectorIn1> & q0,
                  const Eigen::MatrixBase<ConfigVectorIn2> & q1,
                  const Eigen::MatrixBase<ReturnType> & out)
  {
    squaredDistance<LieGroupMap,Scalar,Options,JointCollectionTpl,ConfigVectorIn1,ConfigVectorIn2,ReturnType>(model,q0.derived(),q1.derived(),PINOCCHIO_EIGEN_CONST_CAST(ReturnType,out));
  }

  /**
   *
   * @brief      Generate a configuration vector uniformly sampled among provided limits.
   *
   * @remarks    Limits are not taken into account for rotational transformations (typically SO(2),SO(3)), because they are by definition unbounded.
   *
   * @warning     If limits are infinite, exceptions may be thrown in the joint implementation of uniformlySample.
   *
   * @param[in]  model                Model of the system on which the random configuration operation is performed.
   * @param[in]  lowerLimits  Joints lower limits (size model.nq).
   * @param[in]  upperLimits  Joints upper limits (size model.nq).
   * @param[out] qout                  The resulting configuration vector (size model.nq).
   *
   */
  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2, typename ReturnType>
  void
  randomConfiguration(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                      const Eigen::MatrixBase<ConfigVectorIn1> & lowerLimits,
                      const Eigen::MatrixBase<ConfigVectorIn2> & upperLimits,
                      const Eigen::MatrixBase<ReturnType> & qout);

 /**
  *
  * @brief      Generate a configuration vector uniformly sampled among provided limits.
  *
  * @remarks    Limits are not taken into account for rotational transformations (typically SO(2),SO(3)), because they are by definition unbounded.
  *
  * @warning     If limits are infinite, exceptions may be thrown in the joint implementation of uniformlySample
  *
  * @param[in]  model               Model of the system on which the random configuration operation is performed.
  * @param[in]  lowerLimits  Joints lower limits (size model.nq).
  * @param[in]  upperLimits  Joints upper limits (size model.nq).
  * @param[out] qout                  The resulting configuration vector (size model.nq).
  *
  */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2, typename ReturnType>
  void
  randomConfiguration(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                      const Eigen::MatrixBase<ConfigVectorIn1> & lowerLimits,
                      const Eigen::MatrixBase<ConfigVectorIn2> & upperLimits,
                      const Eigen::MatrixBase<ReturnType> & qout)
  {
    randomConfiguration<LieGroupMap,Scalar,Options,JointCollectionTpl,ConfigVectorIn1,ConfigVectorIn2,ReturnType>(model, lowerLimits.derived(), upperLimits.derived(), PINOCCHIO_EIGEN_CONST_CAST(ReturnType,qout));
  }

  /**
   *
   * @brief         Return the neutral configuration element related to the model configuration space.
   *
   * @param[in]     model      Model of the kinematic tree on which the neutral element is computed
   *
   * @param[out]    qout        The neutral configuration element (size model.nq).
   *
   */
  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ReturnType>
  void
  neutral(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
          const Eigen::MatrixBase<ReturnType> & qout);

  /**
   *
   * @brief         Return the neutral configuration element related to the model configuration space.
   *
   * @param[in]     model      Model of the kinematic tree on which the neutral element is computed.
   *
   * @param[out]    qout        The neutral configuration element (size model.nq).
   *
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ReturnType>
  void
  neutral(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
          const Eigen::MatrixBase<ReturnType> & qout)
  {
    neutral<LieGroupMap,Scalar,Options,JointCollectionTpl,ReturnType>(model,PINOCCHIO_EIGEN_CONST_CAST(ReturnType,qout));
  }

  /**
   *
   * @brief   Computes the Jacobian of a small variation of the configuration vector or the tangent vector into the tangent space at identity.
   *
   * @details This jacobian has to be interpreted in terms of Lie group, not vector space: as such,
   *          it is expressed in the tangent space only, not the configuration space.
   *          Calling \f$ f(q, v) \f$ the integrate function, these jacobians satisfy the following relationships in the
   *          tangent space:
   *           - Jacobian relative to q: \f$ f(q \oplus \delta q, v) \ominus f(q, v) = J_q \delta q + o(\delta q)\f$.
   *           - Jacobian relative to v: \f$ f(q, v + \delta v) \ominus f(q, v) = J_v \delta v + o(\delta v)\f$.
   *
   * @param[in]  model   Model of the kinematic tree on which the integration operation is performed.
   * @param[in]  q       Initial configuration (size model.nq)
   * @param[in]  v       Joint velocity (size model.nv)
   * @param[out] J       Jacobian of the Integrate operation, either with respect to q or v (size model.nv x model.nv).
   * @param[in]  arg     Argument (either q or v) with respect to which the differentiation is performed.
   *
   */
  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType, typename JacobianMatrixType>
  void dIntegrate(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  const Eigen::MatrixBase<ConfigVectorType> & q,
                  const Eigen::MatrixBase<TangentVectorType> & v,
                  const Eigen::MatrixBase<JacobianMatrixType> & J,
                  const ArgumentPosition arg,
                  const AssignmentOperatorType op=SETTO);

  /**
   *
   * @brief   Computes the Jacobian of a small variation of the configuration vector or the tangent vector into the tangent space at identity.
   *
   * @details This jacobian has to be interpreted in terms of Lie group, not vector space: as such,
   *          it is expressed in the tangent space only, not the configuration space.
   *          Calling \f$ f(q, v) \f$ the integrate function, these jacobians satisfy the following relationships in the
   *          tangent space:
   *           - Jacobian relative to q: \f$ f(q \oplus \delta q, v) \ominus f(q, v) = J_q(q, v) \delta q + o(\delta q)\f$.
   *           - Jacobian relative to v: \f$ f(q, v + \delta v) \ominus f(q, v) = J_v(q, v) \delta v + o(\delta v)\f$.
   *
   * @param[in]  model   Model of the kinematic tree on which the integration operation is performed.
   * @param[in]  q            Initial configuration (size model.nq)
   * @param[in]  v            Joint velocity (size model.nv)
   * @param[out] J            Jacobian of the Integrate operation, either with respect to q or v (size model.nv x model.nv).
   * @param[in]  arg        Argument (either q or v) with respect to which the differentiation is performed.
   *
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType, typename JacobianMatrixType>
  void dIntegrate(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  const Eigen::MatrixBase<ConfigVectorType> & q,
                  const Eigen::MatrixBase<TangentVectorType> & v,
                  const Eigen::MatrixBase<JacobianMatrixType> & J,
                  const ArgumentPosition arg)
  {
    dIntegrate<LieGroupMap,Scalar,Options,JointCollectionTpl,ConfigVectorType,TangentVectorType,JacobianMatrixType>(model, q.derived(), v.derived(), PINOCCHIO_EIGEN_CONST_CAST(JacobianMatrixType,J),arg,SETTO);
  }


  /**
   *
   * @brief   Computes the Jacobian of a small variation of the configuration vector or the tangent vector into the tangent space at identity.
   *
   * @details This jacobian has to be interpreted in terms of Lie group, not vector space: as such,
   *          it is expressed in the tangent space only, not the configuration space.
   *          Calling \f$ f(q, v) \f$ the integrate function, these jacobians satisfy the following relationships in the
   *          tangent space:
   *           - Jacobian relative to q: \f$ f(q \oplus \delta q, v) \ominus f(q, v) = J_q(q, v) \delta q + o(\delta q)\f$.
   *           - Jacobian relative to v: \f$ f(q, v + \delta v) \ominus f(q, v) = J_v(q, v) \delta v + o(\delta v)\f$.
   *
   * @param[in]  model   Model of the kinematic tree on which the integration operation is performed.
   * @param[in]  q            Initial configuration (size model.nq)
   * @param[in]  v            Joint velocity (size model.nv)
   * @param[out] J            Jacobian of the Integrate operation, either with respect to q or v (size model.nv x model.nv).
   * @param[in]  arg        Argument (either q or v) with respect to which the differentiation is performed.
   *
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType, typename JacobianMatrixType>
  void dIntegrate(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  const Eigen::MatrixBase<ConfigVectorType> & q,
                  const Eigen::MatrixBase<TangentVectorType> & v,
                  const Eigen::MatrixBase<JacobianMatrixType> & J,
                  const ArgumentPosition arg,
                  const AssignmentOperatorType op)
  {
    dIntegrate<LieGroupMap,Scalar,Options,JointCollectionTpl,ConfigVectorType,TangentVectorType,JacobianMatrixType>(model, q.derived(), v.derived(), PINOCCHIO_EIGEN_CONST_CAST(JacobianMatrixType,J),arg,op);
  }

  /**
   *
   * @brief   Transport a matrix from the terminal to the originate tangent space of the integrate operation, with respect to the configuration or the velocity arguments.
   *
   * @details This function performs the parallel transportation of an input matrix whose columns are expressed in the tangent space of the integrated element \f$ q \oplus v \f$,
   *          to the tangent space at \f$ q \f$.
   *          It performs the product with the Jacobian of integrate by exploiting at best the sparsity of the underlying operations.
   *          In other words, this functions transforms a tangent vector expressed at \f$ q \oplus v \f$ to a tangent vector expressed at \f$ q \f$, considering that the change of configuration between
   *          \f$ q \oplus v \f$ and \f$ q \f$ may alter the value of this tangent vector.
   *          A typical example of parallel transportation is the action operated by a rigid transformation \f$ M \in \text{SE}(3)\f$ on a spatial velocity \f$ v \in \text{se}(3)\f$.
   *          In the context of configuration spaces assimilated as vectorial spaces, this operation corresponds to Identity.
   *          For Lie groups, its corresponds to the canonical vector field transportation.
   *
   * @param[in]  model   Model of the kinematic tree on which the integration operation is performed.
   * @param[in]  q            Initial configuration (size model.nq)
   * @param[in]  v            Joint velocity (size model.nv)
   * @param[out] Jin        Input matrix (number of rows = model.nv).
   * @param[out] Jout      Output matrix (same size as Jin).
   * @param[in]  arg        Argument (either ARG0 for q or ARG1 for v) with respect to which the differentation is performed.
   *
   */
  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType, typename JacobianMatrixType1, typename JacobianMatrixType2>
  void dIntegrateTransport(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                           const Eigen::MatrixBase<ConfigVectorType> & q,
                           const Eigen::MatrixBase<TangentVectorType> & v,
                           const Eigen::MatrixBase<JacobianMatrixType1> & Jin,
                           const Eigen::MatrixBase<JacobianMatrixType2> & Jout,
                           const ArgumentPosition arg);

  /**
   *
   * @brief   Transport a matrix from the terminal to the originate tangent space of the integrate operation, with respect to the configuration or the velocity arguments.
   *
   * @details This function performs the parallel transportation of an input matrix whose columns are expressed in the tangent space of the integrated element \f$ q \oplus v \f$,
   *          to the tangent space at \f$ q \f$.
   *          It performs the product with the Jacobian of integrate by exploiting at best the sparsity of the underlying operations.
   *          In other words, this functions transforms a tangent vector expressed at \f$ q \oplus v \f$ to a tangent vector expressed at \f$ q \f$, considering that the change of configuration between
   *          \f$ q \oplus v \f$ and \f$ q \f$ may alter the value of this tangent vector.
   *          A typical example of parallel transportation is the action operated by a rigid transformation \f$ M \in \text{SE}(3)\f$ on a spatial velocity \f$ v \in \text{se}(3)\f$.
   *          In the context of configuration spaces assimilated as vectorial spaces, this operation corresponds to Identity.
   *          For Lie groups, its corresponds to the canonical vector field transportation.
   *
   * @param[in]  model   Model of the kinematic tree on which the integration operation is performed.
   * @param[in]  q            Initial configuration (size model.nq)
   * @param[in]  v            Joint velocity (size model.nv)
   * @param[out] Jin        Input matrix (number of rows = model.nv).
   * @param[out] Jout      Output matrix (same size as Jin).
   * @param[in]  arg        Argument (either ARG0 for q or ARG1 for v) with respect to which the differentation is performed.
   *
  */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType, typename JacobianMatrixType1, typename JacobianMatrixType2>
  void dIntegrateTransport(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                           const Eigen::MatrixBase<ConfigVectorType> & q,
                           const Eigen::MatrixBase<TangentVectorType> & v,
                           const Eigen::MatrixBase<JacobianMatrixType1> & Jin,
                           const Eigen::MatrixBase<JacobianMatrixType2> & Jout,
                           const ArgumentPosition arg)
  {
    dIntegrateTransport<LieGroupMap,Scalar,Options,JointCollectionTpl,ConfigVectorType,TangentVectorType,JacobianMatrixType1,JacobianMatrixType2>(model, q.derived(), v.derived(), Jin.derived(), PINOCCHIO_EIGEN_CONST_CAST(JacobianMatrixType2,Jout),arg);
  }

  /**
   *
   * @brief   Transport in place a matrix from the terminal to the originate tangent space of the integrate operation, with respect to the configuration or the velocity arguments.
   *
   * @details This function performs the parallel transportation of an input matrix whose columns are expressed in the tangent space of the integrated element \f$ q \oplus v \f$,
   *          to the tangent space at \f$ q \f$.
   *          In other words, this functions transforms a tangent vector expressed at \f$ q \oplus v \f$ to a tangent vector expressed at \f$ q \f$, considering that the change of configuration between
   *          \f$ q \oplus v \f$ and \f$ q \f$ may alter the value of this tangent vector.
   *          A typical example of parallel transportation is the action operated by a rigid transformation \f$ M \in \text{SE}(3)\f$ on a spatial velocity \f$ v \in \text{se}(3)\f$.
   *          In the context of configuration spaces assimilated as vectorial spaces, this operation corresponds to Identity.
   *          For Lie groups, its corresponds to the canonical vector field transportation.
   *
   * @param[in]     model   Model of the kinematic tree on which the integration operation is performed.
   * @param[in]     q            Initial configuration (size model.nq)
   * @param[in]     v            Joint velocity (size model.nv)
   * @param[in,out] J            Input/output matrix (number of rows = model.nv).
   * @param[in]     arg        Argument (either ARG0 for q or ARG1 for v) with respect to which the differentation is performed.
   *
  */
  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType, typename JacobianMatrixType>
  void dIntegrateTransport(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                           const Eigen::MatrixBase<ConfigVectorType> & q,
                           const Eigen::MatrixBase<TangentVectorType> & v,
                           const Eigen::MatrixBase<JacobianMatrixType> & J,
                           const ArgumentPosition arg);

  /**
   *
   * @brief   Transport in place a matrix from the terminal to the originate tangent space of the integrate operation, with respect to the configuration or the velocity arguments.
   *
   * @details This function performs the parallel transportation of an input matrix whose columns are expressed in the tangent space of the integrated element \f$ q \oplus v \f$,
   *          to the tangent space at \f$ q \f$.
   *          In other words, this functions transforms a tangent vector expressed at \f$ q \oplus v \f$ to a tangent vector expressed at \f$ q \f$, considering that the change of configuration between
   *          \f$ q \oplus v \f$ and \f$ q \f$ may alter the value of this tangent vector.
   *          A typical example of parallel transportation is the action operated by a rigid transformation \f$ M \in \text{SE}(3)\f$ on a spatial velocity \f$ v \in \text{se}(3)\f$.
   *          In the context of configuration spaces assimilated as vectorial spaces, this operation corresponds to Identity.
   *          For Lie groups, its corresponds to the canonical vector field transportation.
   *
   * @param[in]     model   Model of the kinematic tree on which the integration operation is performed.
   * @param[in]     q            Initial configuration (size model.nq)
   * @param[in]     v            Joint velocity (size model.nv)
   * @param[in,out] J            Input/output matrix (number of rows = model.nv).
   * @param[in]     arg        Argument (either ARG0 for q or ARG1 for v) with respect to which the differentation is performed.
   *
  */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType, typename JacobianMatrixType>
  void dIntegrateTransport(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                           const Eigen::MatrixBase<ConfigVectorType> & q,
                           const Eigen::MatrixBase<TangentVectorType> & v,
                           const Eigen::MatrixBase<JacobianMatrixType> & J,
                           const ArgumentPosition arg)
  {
    dIntegrateTransport<LieGroupMap,Scalar,Options,JointCollectionTpl,ConfigVectorType,TangentVectorType,JacobianMatrixType>(model, q.derived(), v.derived(), PINOCCHIO_EIGEN_CONST_CAST(JacobianMatrixType,J),arg);
  }

  /**
   *
   * @brief   Computes the Jacobian of a small variation of the configuration vector into the tangent space at identity.
   *
   * @details This jacobian has to be interpreted in terms of Lie group, not vector space: as such,
   *          it is expressed in the tangent space only, not the configuration space.
   *          Calling \f$ d(q0, q1) \f$ the difference function, these jacobians satisfy the following relationships in the
   *          tangent space:
   *           - Jacobian relative to q0: \f$ d(q_0 \oplus \delta q_0, q_1) \ominus d(q_0, q_1) = J_{q_0} \delta q_0 + o(\| \delta q_0 \|)\f$.
   *           - Jacobian relative to q1: \f$ d(q_0, q_1 \oplus \delta q_1) \ominus d(q_0, q_1) = J_{q_1} \delta q_1 + o(\| \delta q_1 \|)\f$.
   *
   * @param[in]  model   Model of the kinematic tree on which the difference operation is performed.
   * @param[in]  q0          Initial configuration (size model.nq)
   * @param[in]  q1          Joint velocity (size model.nv)
   * @param[out] J            Jacobian of the Difference operation, either with respect to q0 or q1 (size model.nv x model.nv).
   * @param[in]  arg        Argument (either q0 or q1) with respect to which the differentiation is performed.
   *
   */
  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVector1, typename ConfigVector2, typename JacobianMatrix>
  void dDifference(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                   const Eigen::MatrixBase<ConfigVector1> & q0,
                   const Eigen::MatrixBase<ConfigVector2> & q1,
                   const Eigen::MatrixBase<JacobianMatrix> & J,
                   const ArgumentPosition arg);

  /**
   *
   * @brief   Computes the Jacobian of a small variation of the configuration vector into the tangent space at identity.
   *
   * @details This jacobian has to be interpreted in terms of Lie group, not vector space: as such,
   *          it is expressed in the tangent space only, not the configuration space.
   *          Calling \f$ d(q0, q1) \f$ the difference function, these jacobians satisfy the following relationships in the
   *          tangent space:
   *           - Jacobian relative to q0: \f$ d(q_0 \oplus \delta q_0, q_1) \ominus d(q_0, q_1) = J_{q_0} \delta q_0 + o(\| \delta q_0 \|)\f$.
   *           - Jacobian relative to q1: \f$ d(q_0, q_1 \oplus \delta q_1) \ominus d(q_0, q_1) = J_{q_1} \delta q_1 + o(\| \delta q_1 \|)\f$.
   *
   * @param[in]  model   Model of the kinematic tree on which the difference operation is performed.
   * @param[in]  q0          Initial configuration (size model.nq)
   * @param[in]  q1          Joint velocity (size model.nv)
   * @param[out] J            Jacobian of the Difference operation, either with respect to q0 or q1 (size model.nv x model.nv).
   * @param[in]  arg        Argument (either q0 or q1) with respect to which the differentiation is performed.
   *
  */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVector1, typename ConfigVector2, typename JacobianMatrix>
  void dDifference(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  const Eigen::MatrixBase<ConfigVector1> & q0,
                  const Eigen::MatrixBase<ConfigVector2> & q1,
                  const Eigen::MatrixBase<JacobianMatrix> & J,
                  const ArgumentPosition arg)
  {
    dDifference<LieGroupMap,Scalar,Options,JointCollectionTpl,ConfigVector1,ConfigVector2,JacobianMatrix>
    (model, q0.derived(), q1.derived(), PINOCCHIO_EIGEN_CONST_CAST(JacobianMatrix,J),arg);
  }
  /**
   *
   * @brief      Overall squared distance between two configuration vectors
   *
   * @param[in]  model      Model we want to compute the distance
   * @param[in]  q0         Configuration 0 (size model.nq)
   * @param[in]  q1         Configuration 1 (size model.nq)
   *
   * @return     The squared distance between the two configurations
   *
   */
  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2>
  Scalar squaredDistanceSum(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                            const Eigen::MatrixBase<ConfigVectorIn1> & q0,
                            const Eigen::MatrixBase<ConfigVectorIn2> & q1);

  /**
   *
   * @brief      Overall squared distance between two configuration vectors, namely \f$ || q_{1} \ominus q_{0} ||_2^{2} \f$.
   *
   * @param[in]  model  Model of the kinematic tree
   * @param[in]  q0         Configuration from (size model.nq)
   * @param[in]  q1         Configuration to (size model.nq)
   *
   * @return     The squared distance between the two configurations q0 and q1.
   *
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2>
  inline Scalar
  squaredDistanceSum(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                     const Eigen::MatrixBase<ConfigVectorIn1> & q0,
                     const Eigen::MatrixBase<ConfigVectorIn2> & q1)
  {
    return squaredDistanceSum<LieGroupMap,Scalar,Options,JointCollectionTpl,ConfigVectorIn1,ConfigVectorIn2>(model, q0.derived(), q1.derived());
  }

  /**
   *
   * @brief      Distance between two configuration vectors, namely \f$ || q_{1} \ominus q_{0} ||_2 \f$.
   *
   * @param[in]  model      Model we want to compute the distance
   * @param[in]  q0         Configuration 0 (size model.nq)
   * @param[in]  q1         Configuration 1 (size model.nq)
   *
   * @return     The distance between the two configurations q0 and q1.
   *
   */
  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2>
  Scalar distance(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  const Eigen::MatrixBase<ConfigVectorIn1> & q0,
                  const Eigen::MatrixBase<ConfigVectorIn2> & q1);

  /**
   *
   * @brief      Distance between two configuration vectors
   *
   * @param[in]  model      Model we want to compute the distance
   * @param[in]  q0         Configuration 0 (size model.nq)
   * @param[in]  q1         Configuration 1 (size model.nq)
   *
   * @return     The distance between the two configurations q0 and q1.
   *
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2>
  inline Scalar
  distance(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
           const Eigen::MatrixBase<ConfigVectorIn1> & q0,
           const Eigen::MatrixBase<ConfigVectorIn2> & q1)
  {
    return distance<LieGroupMap,Scalar,Options,JointCollectionTpl,ConfigVectorIn1,ConfigVectorIn2>(model, q0.derived(), q1.derived());
  }

  /**
   *
   * @brief         Normalize a configuration vector.
   *
   * @param[in]     model      Model of the kinematic tree.
   * @param[in,out] q               Configuration to normalize (size model.nq).
   *
   */
  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
  inline void normalize(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                        const Eigen::MatrixBase<ConfigVectorType> & qout);

  /**
   *
   * @brief         Normalize a configuration vector.
   *
   * @param[in]     model      Model of the kinematic tree.
   * @param[in,out] q               Configuration to normalize (size model.nq).
   *
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
  inline void normalize(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                        const Eigen::MatrixBase<ConfigVectorType> & qout)
  {
    normalize<LieGroupMap,Scalar,Options,JointCollectionTpl,ConfigVectorType>(model,PINOCCHIO_EIGEN_CONST_CAST(ConfigVectorType,qout));
  }

  /**
   *
   * @brief         Check whether a configuration vector is normalized within the given precision provided by prec.
   *
   * @param[in]     model      Model of the kinematic tree.
   * @param[in]     q          Configuration to check (size model.nq).
   * @param[in]     prec       Precision.
   *
   * @return     Whether the configuration is normalized or not, within the given precision.
   */
  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
  inline bool isNormalized(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                           const Eigen::MatrixBase<ConfigVectorType> & q,
                           const Scalar& prec = Eigen::NumTraits<Scalar>::dummy_precision());

  /**
   *
   * @brief         Check whether a configuration vector is normalized within the given precision provided by prec.
   *
   * @param[in]     model      Model of the kinematic tree.
   * @param[in]     q          Configuration to check (size model.nq).
   * @param[in]     prec       Precision.
   *
   * @return     Whether the configuration is normalized or not, within the given precision.
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
  inline bool isNormalized(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                           const Eigen::MatrixBase<ConfigVectorType> & q,
                           const Scalar& prec = Eigen::NumTraits<Scalar>::dummy_precision())
  {
    return isNormalized<LieGroupMap,Scalar,Options,JointCollectionTpl,ConfigVectorType>(model,q,prec);
  }

  /**
   *
   * @brief         Return true if the given configurations are equivalents, within the given precision.
   * @remarks       Two configurations can be equivalent but not equally coefficient wise (e.g two quaternions with opposite coefficients give rise to the same orientation, i.e. they are equivalent.).
   *
   * @param[in]     model     Model of the kinematic tree.
   * @param[in]     q1        The first configuraiton to compare.
   * @param[in]     q2        The second configuration to compare.
   * @param[in]     prec      precision of the comparison.
   *
   * @return     Whether the configurations are equivalent or not, within the given precision.
   *
   */
  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2>
  inline bool
  isSameConfiguration(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                      const Eigen::MatrixBase<ConfigVectorIn1> & q1,
                      const Eigen::MatrixBase<ConfigVectorIn2> & q2,
                      const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision());

  /**
   *
   * @brief         Return true if the given configurations are equivalents, within the given precision.
   * @remarks       Two configurations can be equivalent but not equally coefficient wise (e.g two quaternions with opposite coefficients give rise to the same orientation, i.e. they are equivalent.).
   *
   * @param[in]     model     Model of the kinematic tree.
   * @param[in]     q1           The first configuraiton to compare
   * @param[in]     q2           The second configuration to compare
   * @param[in]     prec      precision of the comparison.
   *
   * @return     Whether the configurations are equivalent or not
   *
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2>
  inline bool
  isSameConfiguration(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                      const Eigen::MatrixBase<ConfigVectorIn1> & q1,
                      const Eigen::MatrixBase<ConfigVectorIn2> & q2,
                      const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision())
  {
    return isSameConfiguration<LieGroupMap,Scalar,Options,JointCollectionTpl,ConfigVectorIn1,ConfigVectorIn2>(model, q1.derived(), q2.derived(), prec);
  }

  /**
   *
   * @brief         Return the Jacobian of the integrate function for the components of the config vector.
   *
   * @param[in]     model          Model of the kinematic tree.
   * @param[out]    jacobian   The Jacobian of the integrate operation.
   *
   * @details       This function is often required for the numerical solvers that are working on the
   *                tangent of the configuration space, instead of the configuration space itself.
   *
   */
  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVector, typename JacobianMatrix>
  inline void
  integrateCoeffWiseJacobian(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                             const Eigen::MatrixBase<ConfigVector> & q,
                             const Eigen::MatrixBase<JacobianMatrix> & jacobian);

  /**
   *
   * @brief         Return the Jacobian of the integrate function for the components of the config vector.
   *
   * @param[in]     model          Model of the kinematic tree.
   * @param[out]    jacobian   The Jacobian of the integrate operation.
   *
   * @details       This function is often required for the numerical solvers that are working on the
   *                tangent of the configuration space, instead of the configuration space itself.
   *
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVector, typename JacobianMatrix>
  inline void
  integrateCoeffWiseJacobian(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                             const Eigen::MatrixBase<ConfigVector> & q,
                             const Eigen::MatrixBase<JacobianMatrix> & jacobian)
  {
    integrateCoeffWiseJacobian<LieGroupMap,Scalar,Options,JointCollectionTpl,ConfigVector,JacobianMatrix>(model,q.derived(),PINOCCHIO_EIGEN_CONST_CAST(JacobianMatrix,jacobian));
  }

  /// \}

  /// \name API that allocates memory
  /// \{

  /**
   *
   * @brief      Integrate a configuration vector for the specified model for a tangent vector during one unit time
   *
   * @param[in]  model   Model of the kinematic tree on which the integration operation is performed.
   * @param[in]  q            Initial configuration (size model.nq)
   * @param[in]  v            Joint velocity (size model.nv)
   *
   * @return     The integrated configuration (size model.nq)
   *
   */
  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType>
  inline typename PINOCCHIO_EIGEN_PLAIN_TYPE(ConfigVectorType)
  integrate(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
            const Eigen::MatrixBase<ConfigVectorType> & q,
            const Eigen::MatrixBase<TangentVectorType> & v);

  /**
   *
   * @brief      Integrate a configuration vector for the specified model for a tangent vector during one unit time.
   *
   * @param[in]  model   Model of the kinematic tree on which the integration operation is performed.
   * @param[in]  q            Initial configuration (size model.nq)
   * @param[in]  v            Joint velocity (size model.nv)
   *
   * @return     The integrated configuration (size model.nq)
   *
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType>
  inline typename PINOCCHIO_EIGEN_PLAIN_TYPE(ConfigVectorType)
  integrate(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
            const Eigen::MatrixBase<ConfigVectorType> & q,
            const Eigen::MatrixBase<TangentVectorType> & v)
  {
    return integrate<LieGroupMap,Scalar,Options,JointCollectionTpl,ConfigVectorType,TangentVectorType>(model, q.derived(), v.derived());
  }

  /**
   *
   * @brief      Interpolate two configurations for a given model.
   *
   * @param[in]  model   Model of the kinematic tree on which the interpolation operation is performed.
   * @param[in]  q0          Initial configuration vector (size model.nq)
   * @param[in]  q1          Final configuration vector (size model.nq)
   * @param[in]  u            u in [0;1] position along the interpolation.
   *
   * @return     The interpolated configuration (q0 if u = 0, q1 if u = 1)
   *
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2>
  inline typename PINOCCHIO_EIGEN_PLAIN_TYPE(ConfigVectorIn1)
  interpolate(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
              const Eigen::MatrixBase<ConfigVectorIn1> & q0,
              const Eigen::MatrixBase<ConfigVectorIn2> & q1,
              const Scalar & u);

  /**
   *
   * @brief      Interpolate two configurations for a given model.
   *
   * @param[in]  model   Model of the kinematic tree on which the interpolation operation is performed.
   * @param[in]  q0          Initial configuration vector (size model.nq)
   * @param[in]  q1          Final configuration vector (size model.nq)
   * @param[in]  u            u in [0;1] position along the interpolation.
   *
   * @return     The interpolated configuration (q0 if u = 0, q1 if u = 1)
   *
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2>
  inline typename PINOCCHIO_EIGEN_PLAIN_TYPE(ConfigVectorIn1)
  interpolate(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
              const Eigen::MatrixBase<ConfigVectorIn1> & q0,
              const Eigen::MatrixBase<ConfigVectorIn2> & q1,
              const Scalar & u)
  {
    return interpolate<LieGroupMap,Scalar,Options,JointCollectionTpl,ConfigVectorIn1,ConfigVectorIn2>(model, q0.derived(), q1.derived(), u);
  }

  /**
   *
   * @brief      Compute the tangent vector that must be integrated during one unit time to go from q0 to q1
   *
   * @param[in]  model   Model of the kinematic tree on which the difference operation is performed.
   * @param[in]  q0          Initial configuration (size model.nq)
   * @param[in]  q1          Finial configuration (size model.nq)
   *
   * @return     The corresponding velocity (size model.nv)
   *
   */
  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2>
  inline typename PINOCCHIO_EIGEN_PLAIN_TYPE(ConfigVectorIn1)
  difference(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
             const Eigen::MatrixBase<ConfigVectorIn1> & q0,
             const Eigen::MatrixBase<ConfigVectorIn2> & q1);

  /**
   *
   * @brief      Compute the tangent vector that must be integrated during one unit time to go from q0 to q1.
   *
   * @param[in]  model   Model of the kinematic tree on which the difference operation is performed.
   * @param[in]  q0          Initial configuration (size model.nq)
   * @param[in]  q1          Final configuration (size model.nq)
   *
   * @return     The corresponding velocity (size model.nv)
   *
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2>
  inline typename PINOCCHIO_EIGEN_PLAIN_TYPE(ConfigVectorIn1)
  difference(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
             const Eigen::MatrixBase<ConfigVectorIn1> & q0,
             const Eigen::MatrixBase<ConfigVectorIn2> & q1)
  {
    return difference<LieGroupMap,Scalar,Options,JointCollectionTpl,ConfigVectorIn1,ConfigVectorIn2>(model,q0.derived(),q1.derived());
  }

  /**
   *
   * @brief      Squared distance between two configurations.
   *
   * @param[in]  model      Model of the kinematic tree on which the squared distance operation is performed.
   * @param[in]  q0             Configuration 0 (size model.nq)
   * @param[in]  q1             Configuration 1 (size model.nq)
   *
   * @return     The corresponding squared distances for each joint (size model.njoints-1, corresponding to the number of joints)
   *
   */
  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2>
  inline typename PINOCCHIO_EIGEN_PLAIN_TYPE(ConfigVectorIn1)
  squaredDistance(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  const Eigen::MatrixBase<ConfigVectorIn1> & q0,
                  const Eigen::MatrixBase<ConfigVectorIn2> & q1);

  /**
   *
   * @brief      Squared distance between two configuration vectors
   *
   * @param[in]  model      Model of the kinematic tree on which the squared distance operation is performed.
   * @param[in]  q0             Configuration 0 (size model.nq)
   * @param[in]  q1             Configuration 1 (size model.nq)
   *
   * @return     The corresponding squared distances for each joint (size model.njoints-1, corresponding to the number of joints)
   *
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2>
  inline typename PINOCCHIO_EIGEN_PLAIN_TYPE(ConfigVectorIn1)
  squaredDistance(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  const Eigen::MatrixBase<ConfigVectorIn1> & q0,
                  const Eigen::MatrixBase<ConfigVectorIn2> & q1)
  {
    return squaredDistance<LieGroupMap,Scalar,Options,JointCollectionTpl,ConfigVectorIn1,ConfigVectorIn2>(model,q0.derived(),q1.derived());
  }

  /**
   *
   * @brief      Generate a configuration vector uniformly sampled among given limits.
   *
   * @remarks    Limits are not taken into account for rotational transformations (typically SO(2),SO(3)), because they are by definition unbounded.
   *
   * @warning     If limits are infinite, exceptions may be thrown in the joint implementation of uniformlySample
   *
   * @param[in]  model                Model of the kinematic tree on which the uniform sampling operation is performed.
   * @param[in]  lowerLimits   Joints lower limits (size model.nq).
   * @param[in]  upperLimits   Joints upper limits (size model.nq).
   *
   * @return     The resulting configuration vector (size model.nq).
   *
   */
  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2>
  typename PINOCCHIO_EIGEN_PLAIN_TYPE_NO_PARENS((typename ModelTpl<Scalar,Options,JointCollectionTpl>::ConfigVectorType))
  randomConfiguration(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                      const Eigen::MatrixBase<ConfigVectorIn1> & lowerLimits,
                      const Eigen::MatrixBase<ConfigVectorIn2> & upperLimits);

 /**
  *
  * @brief      Generate a configuration vector uniformly sampled among provided limits.
  *
  * @remarks    Limits are not taken into account for rotational transformations (typically SO(2),SO(3)), because they are by definition unbounded.
  *
  * @warning     If limits are infinite, exceptions may be thrown in the joint implementation of uniformlySample
  *
  * @param[in]  model               Model of the kinematic tree on which the uniform sampling operation is performed.
  * @param[in]  lowerLimits  Joints lower limits (size model.nq).
  * @param[in]  upperLimits  Joints upper limits (size model.nq).
  *
  * @return     The resulting configuration vector (size model.nq)
  
  */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2>
  typename PINOCCHIO_EIGEN_PLAIN_TYPE_NO_PARENS((typename ModelTpl<Scalar,Options,JointCollectionTpl>::ConfigVectorType))
  randomConfiguration(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                      const Eigen::MatrixBase<ConfigVectorIn1> & lowerLimits,
                      const Eigen::MatrixBase<ConfigVectorIn2> & upperLimits)
  {
    return randomConfiguration<LieGroupMap,Scalar,Options,JointCollectionTpl,ConfigVectorIn1,ConfigVectorIn2>(model, lowerLimits.derived(), upperLimits.derived());
  }

  /**
   *
   * @brief      Generate a configuration vector uniformly sampled among the joint limits of the specified Model.
   *
   * @remarks    Limits are not taken into account for rotational transformations (typically SO(2),SO(3)), because they are by definition unbounded.
   *
   * @warning    If limits are infinite (no one specified when adding a body or no modification directly in my_model.{lowerPositionLimit,upperPositionLimit},
   *             exceptions may be thrown in the joint implementation of uniformlySample
   *
   * @param[in]  model   Model of the kinematic tree on which the uniform sampling operation is performed.
   *
   * @return     The resulting configuration vector (size model.nq)
   *
   */
  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  typename PINOCCHIO_EIGEN_PLAIN_TYPE_NO_PARENS((typename ModelTpl<Scalar,Options,JointCollectionTpl>::ConfigVectorType))
  randomConfiguration(const ModelTpl<Scalar,Options,JointCollectionTpl> & model);

  /**
   *
   * @brief      Generate a configuration vector uniformly sampled among the joint limits of the specified Model.
   *
   * @remarks    Limits are not taken into account for rotational transformations (typically SO(2),SO(3)), because they are by definition unbounded.
   *
   * @warning    If limits are infinite (no one specified when adding a body or no modification directly in my_model.{lowerPositionLimit,upperPositionLimit},
   *             exceptions may be thrown in the joint implementation of uniformlySample
   *
   * @param[in]  model   Model of the kinematic tree on which the uniform sampling operation is performed.
   *
   * @return     The resulting configuration vector (size model.nq).
   *
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  typename PINOCCHIO_EIGEN_PLAIN_TYPE_NO_PARENS((typename ModelTpl<Scalar,Options,JointCollectionTpl>::ConfigVectorType))
  randomConfiguration(const ModelTpl<Scalar,Options,JointCollectionTpl> & model)
  {
    return randomConfiguration<LieGroupMap,Scalar,Options,JointCollectionTpl>(model);
  }

  /**
   *
   * @brief         Return the neutral configuration element related to the model configuration space.
   *
   * @param[in]     model      Model of the kinematic tree on which the neutral element is computed.
   *
   * @return        The neutral configuration element (size model.nq).
   *
   */
  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline Eigen::Matrix<Scalar,Eigen::Dynamic,1,Options>
  neutral(const ModelTpl<Scalar,Options,JointCollectionTpl> & model);

  /**
   * @brief         Return the neutral configuration element related to the model configuration space.
   *
   * @param[in]     model      Model of the kinematic tree on which the neutral element is computed.
   *
   * @return        The neutral configuration element (size model.nq).
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline Eigen::Matrix<Scalar,Eigen::Dynamic,1,Options>
  neutral(const ModelTpl<Scalar,Options,JointCollectionTpl> & model)
  {
    return neutral<LieGroupMap,Scalar,Options,JointCollectionTpl>(model);
  }

  /// \}

} // namespace pinocchio

/* --- Details -------------------------------------------------------------------- */
#include "pinocchio/algorithm/joint-configuration.hxx"

#endif // ifndef __pinocchio_algorithm_joint_configuration_hpp__
