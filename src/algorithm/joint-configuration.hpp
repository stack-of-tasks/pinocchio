//
// Copyright (c) 2016-2018 CNRS INRIA
//

#ifndef __pinocchio_joint_configuration_hpp__
#define __pinocchio_joint_configuration_hpp__

#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/multibody/liegroup/liegroup.hpp"

namespace pinocchio
{

  /// \name API with return value as argument
  /// \{

  /**
   * @brief      Integrate a configuration vector for the specified model for a tangent vector during one unit time
   *
   * @details This function corresponds to the exponential map of the joint configuration Lie Group.
   *          Its output can be interpreted as the "sum" from the Lie algebra to the joint configuration space \f$ q \oplus v \f$.
   *
   * @param[in]  model   Model that must be integrated
   * @param[in]  q       Initial configuration (size model.nq)
   * @param[in]  v       Joint velocity (size model.nv)
   * @param[out] qout    The integrated configuration (size model.nq)
   */
  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType, typename ReturnType>
  void
  integrate(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
            const Eigen::MatrixBase<ConfigVectorType> & q,
            const Eigen::MatrixBase<TangentVectorType> & v,
            const Eigen::MatrixBase<ReturnType> & qout);

  /**
   * @brief      Integrate a configuration vector for the specified model for a tangent vector during one unit time
   *
   * @details This function corresponds to the exponential map of the joint configuration Lie Group.
   *          Its output can be interpreted as the "sum" from the Lie algebra to the joint configuration space \f$ q \oplus v \f$.
   *
   * @param[in]  model   Model that must be integrated
   * @param[in]  q       Initial configuration (size model.nq)
   * @param[in]  v       Joint velocity (size model.nv)
   * @param[out] qout    The integrated configuration (size model.nq)
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
   * @brief      Interpolate two configurations for a given model
   *
   * @param[in]  model   Model to be interpolated
   * @param[in]  q0      Initial configuration vector (size model.nq)
   * @param[in]  q1      Final configuration vector (size model.nq)
   * @param[in]  u       u in [0;1] position along the interpolation.
   * @param[out] qout    The interpolated configuration (q0 if u = 0, q1 if u = 1)
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2, typename ReturnType>
  void
  interpolate(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
              const Eigen::MatrixBase<ConfigVectorIn1> & q0,
              const Eigen::MatrixBase<ConfigVectorIn2> & q1,
              const Scalar & u,
              const Eigen::MatrixBase<ReturnType> & qout);

  /**
   * @brief      Interpolate two configurations for a given model
   *
   * @param[in]  model   Model to be interpolated
   * @param[in]  q0      Initial configuration vector (size model.nq)
   * @param[in]  q1      Final configuration vector (size model.nq)
   * @param[in]  u       u in [0;1] position along the interpolation.
   * @param[out] qout    The interpolated configuration (q0 if u = 0, q1 if u = 1)
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2, typename ReturnType>
  void
  interpolate(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
              const Eigen::MatrixBase<ConfigVectorIn1> & q0,
              const Eigen::MatrixBase<ConfigVectorIn2> & q1,
              const Scalar & u,
              const Eigen::MatrixBase<ReturnType> & qout)
  {
    interpolate<LieGroupMap,Scalar,Options,JointCollectionTpl,ConfigVectorIn1,ConfigVectorIn2,ReturnType>(model, q0, q1, u, qout);
  }

  /**
   * @brief      Compute the tangent vector that must be integrated during one unit time to go from q0 to q1
   *
   * @details This function corresponds to the log map of the joint configuration Lie Group.
   *          Its output can be interpreted as a difference from the joint configuration space to the Lie algebra \f$ q_1 \ominus q_0 \f$.
   *
   * @param[in]  model   Model of the system
   * @param[in]  q0      Initial configuration (size model.nq)
   * @param[in]  q1      Wished configuration (size model.nq)
   * @param[out] dvout   The corresponding velocity (size model.nv)
   */
  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2, typename ReturnType>
  void
  difference(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
             const Eigen::MatrixBase<ConfigVectorIn1> & q0,
             const Eigen::MatrixBase<ConfigVectorIn2> & q1,
             const Eigen::MatrixBase<ReturnType> & dvout);

  /**
   * @brief      Compute the tangent vector that must be integrated during one unit time to go from q0 to q1
   *
   * @details This function corresponds to the log map of the joint configuration Lie Group.
   *          Its output can be interpreted as a difference from the joint configuration space to the Lie algebra \f$ q_1 \ominus q_0 \f$.
   *
   * @param[in]  model   Model of the system
   * @param[in]  q0      Initial configuration (size model.nq)
   * @param[in]  q1      Wished configuration (size model.nq)
   * @param[out] dvout   The corresponding velocity (size model.nv)
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2, typename ReturnType>
  void
  difference(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
             const Eigen::MatrixBase<ConfigVectorIn1> & q0,
             const Eigen::MatrixBase<ConfigVectorIn2> & q1,
             const Eigen::MatrixBase<ReturnType> & dvout)
  {
    difference<LieGroupMap,Scalar,Options,JointCollectionTpl,ConfigVectorIn1,ConfigVectorIn2,ReturnType>(model,q0.derived(),q1.derived(),dvout.derived());
  }

  /**
   * @brief      Squared distance between two configuration vectors
   *
   * @param[in]  model      Model we want to compute the distance
   * @param[in]  q0         Configuration 0 (size model.nq)
   * @param[in]  q1         Configuration 1 (size model.nq)
   * @param[out] out        The corresponding squared distances for each joint (size model.njoints-1 = number of joints)
   */
  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2, typename ReturnType>
  void
  squaredDistance(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  const Eigen::MatrixBase<ConfigVectorIn1> & q0,
                  const Eigen::MatrixBase<ConfigVectorIn2> & q1,
                  const Eigen::MatrixBase<ReturnType> & out);

  /**
   * @brief      Squared distance between two configuration vectors
   *
   * @param[in]  model      Model we want to compute the distance
   * @param[in]  q0         Configuration 0 (size model.nq)
   * @param[in]  q1         Configuration 1 (size model.nq)
   * @param[out] out        The corresponding squared distances for each joint (size model.njoints-1 = number of joints)
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2, typename ReturnType>
  void
  squaredDistance(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  const Eigen::MatrixBase<ConfigVectorIn1> & q0,
                  const Eigen::MatrixBase<ConfigVectorIn2> & q1,
                  const Eigen::MatrixBase<ReturnType> & out)
  {
    squaredDistance<LieGroupMap,Scalar,Options,JointCollectionTpl,ConfigVectorIn1,ConfigVectorIn2,ReturnType>(model,q0.derived(),q1.derived(),out.derived());
  }

  /**
   * @brief      Generate a configuration vector uniformly sampled among provided limits.
   *
   * @remarks    Limits are not taken into account for rotational transformations (typically SO(2),SO(3)), because they are by definition unbounded.
   *
   * @warning     If limits are infinite, exceptions may be thrown in the joint implementation of uniformlySample
   *
   * @param[in]  model        Model for which we want to generate a configuration vector.
   * @param[in]  lowerLimits  Joints lower limits
   * @param[in]  upperLimits  Joints upper limits
   * @param[out] qout         The resulted configuration vector (size model.nq)
   */
  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2, typename ReturnType>
  void
  randomConfiguration(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                      const Eigen::MatrixBase<ConfigVectorIn1> & lowerLimits,
                      const Eigen::MatrixBase<ConfigVectorIn2> & upperLimits,
                      const Eigen::MatrixBase<ReturnType> & qout);

 /**
   * @brief      Generate a configuration vector uniformly sampled among provided limits.
   *
   * @remarks    Limits are not taken into account for rotational transformations (typically SO(2),SO(3)), because they are by definition unbounded.
   *
   * @warning     If limits are infinite, exceptions may be thrown in the joint implementation of uniformlySample
   *
   * @param[in]  model        Model for which we want to generate a configuration vector.
   * @param[in]  lowerLimits  Joints lower limits
   * @param[in]  upperLimits  Joints upper limits
   * @param[out] qout         The resulted configuration vector (size model.nq)
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2, typename ReturnType>
  void
  randomConfiguration(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                      const Eigen::MatrixBase<ConfigVectorIn1> & lowerLimits,
                      const Eigen::MatrixBase<ConfigVectorIn2> & upperLimits,
                      const Eigen::MatrixBase<ReturnType> & qout)
  {
    randomConfiguration<LieGroupMap,Scalar,Options,JointCollectionTpl,ConfigVectorIn1,ConfigVectorIn2,ReturnType>(model, lowerLimits.derived(), upperLimits.derived(), qout.derived());
  }

  /**
   * @brief         Return the neutral configuration element related to the model configuration space.
   *
   * @param[in]     model      Model
   * @param[out]    qout        The neutral configuration element.
   */
  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ReturnType>
  void
  neutral(const ModelTpl<Scalar,Options,JointCollectionTpl> & model, const Eigen::MatrixBase<ReturnType> & qout);

  /**
   * @brief         Return the neutral configuration element related to the model configuration space.
   *
   * @param[in]     model      Model
   * @param[out]    qout        The neutral configuration element.
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ReturnType>
  void
  neutral(const ModelTpl<Scalar,Options,JointCollectionTpl> & model, const Eigen::MatrixBase<ReturnType> & qout)
  {
    neutral<LieGroupMap,Scalar,Options,JointCollectionTpl,ReturnType>(model,qout.derived());
  }

  /**
   * @brief      Computes the Jacobian of a small variation of the configuration vector or the tangent vector into the tangent space at identity.
   *
   * @details This jacobian has to be interpreted in terms of Lie group, not vector space: as such,
   *          it is expressed in the tangent space only, not the configuration space.
   *          Calling \f$ f(q, v) \f$ the integrate function, these jacobians satisfy the following relationships in the
   *          tangent space:
   *           - Jacobian relative to q: \f$ f(q \oplus \delta q, v) \ominus f(q, v) = J_q \delta q + o(\delta q)\f$.
   *           - Jacobian relative to v: \f$ f(q, v + \delta v) \ominus f(q, v) = J_v \delta v + o(\delta v)\f$.
   *
   * @param[in]  model   Model that must be integrated
   * @param[in]  q       Initial configuration (size model.nq)
   * @param[in]  v       Joint velocity (size model.nv)
   * @param[out] J       Jacobian of the Integrate operation, either with respect to q or v (size model.nv x model.nv).
   * @param[in]  arg     Argument (either q or v) with respect to which the
   *
   */
  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType, typename JacobianMatrixType>
  void dIntegrate(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  const Eigen::MatrixBase<ConfigVectorType> & q,
                  const Eigen::MatrixBase<TangentVectorType> & v,
                  const Eigen::MatrixBase<JacobianMatrixType> & J,
                  const ArgumentPosition arg);

  /**
   * @brief      Computes the Jacobian of a small variation of the configuration vector or the tangent vector into the tangent space at identity.
   *
   * @details This jacobian has to be interpreted in terms of Lie group, not vector space: as such,
   *          it is expressed in the tangent space only, not the configuration space.
   *          Calling \f$ f(q, v) \f$ the integrate function, these jacobians satisfy the following relationships in the
   *          tangent space:
   *           - Jacobian relative to q: \f$ f(q \oplus \delta q, v) \ominus f(q, v) = J_q(q, v) \delta q + o(\delta q)\f$.
   *           - Jacobian relative to v: \f$ f(q, v + \delta v) \ominus f(q, v) = J_v(q, v) \delta v + o(\delta v)\f$.
   *
   * @param[in]  model   Model that must be integrated
   * @param[in]  q       Initial configuration (size model.nq)
   * @param[in]  v       Joint velocity (size model.nv)
   * @param[out] J       Jacobian of the Integrate operation, either with respect to q or v (size model.nv x model.nv).
   * @param[in]  arg     Argument (either q or v) with respect to which the
   *
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType, typename JacobianMatrixType>
  void dIntegrate(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  const Eigen::MatrixBase<ConfigVectorType> & q,
                  const Eigen::MatrixBase<TangentVectorType> & v,
                  const Eigen::MatrixBase<JacobianMatrixType> & J,
                  const ArgumentPosition arg)
  {
    dIntegrate<LieGroupMap,Scalar,Options,JointCollectionTpl,ConfigVectorType,TangentVectorType,JacobianMatrixType>(model, q.derived(), v.derived(), J.derived(),arg);
  }

  /**
   * @brief      Overall squared distance between two configuration vectors
   *
   * @param[in]  model      Model we want to compute the distance
   * @param[in]  q0         Configuration 0 (size model.nq)
   * @param[in]  q1         Configuration 1 (size model.nq)
   *
   * @return     The squared distance between the two configurations
   */
  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2>
  Scalar squaredDistanceSum(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                            const Eigen::MatrixBase<ConfigVectorIn1> & q0,
                            const Eigen::MatrixBase<ConfigVectorIn2> & q1);

  /**
   * @brief      Overall squared distance between two configuration vectors
   *
   * @param[in]  model      Model we want to compute the distance
   * @param[in]  q0         Configuration 0 (size model.nq)
   * @param[in]  q1         Configuration 1 (size model.nq)
   *
   * @return     The squared distance between the two configurations
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
   * @brief      Distance between two configuration vectors
   *
   * @param[in]  model      Model we want to compute the distance
   * @param[in]  q0         Configuration 0 (size model.nq)
   * @param[in]  q1         Configuration 1 (size model.nq)
   *
   * @return     The distance between the two configurations
   */
  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2>
  Scalar distance(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  const Eigen::MatrixBase<ConfigVectorIn1> & q0,
                  const Eigen::MatrixBase<ConfigVectorIn2> & q1);

  /**
   * @brief      Distance between two configuration vectors
   *
   * @param[in]  model      Model we want to compute the distance
   * @param[in]  q0         Configuration 0 (size model.nq)
   * @param[in]  q1         Configuration 1 (size model.nq)
   *
   * @return     The distance between the two configurations
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
   * @brief         Normalize a configuration vector
   *
   * @param[in]     model      Model
   * @param[in,out] q          Configuration to normalize
   *
   */
  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
  inline void normalize(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                        const Eigen::MatrixBase<ConfigVectorType> & qout);

  /**
   * @brief         Normalize a configuration vector
   *
   * @param[in]     model      Model
   * @param[in,out] q          Configuration to normalize
   *
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
  inline void normalize(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                        const Eigen::MatrixBase<ConfigVectorType> & qout)
  {
    normalize<LieGroupMap,Scalar,Options,JointCollectionTpl,ConfigVectorType>(model,qout.derived());
  }

  /**
   * @brief         Return true if the given configurations are equivalents
   * @warning       Two configurations can be equivalent but not equally coefficient wise (e.g for quaternions)
   *
   * @param[in]     model     Model
   * @param[in]     q1        The first configuraiton to compare
   * @param[in]     q2        The second configuration to compare
   * @param[in]     prec      precision of the comparison
   *
   * @return     Whether the configurations are equivalent or not
   */
  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2>
  inline bool
  isSameConfiguration(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                      const Eigen::MatrixBase<ConfigVectorIn1> & q1,
                      const Eigen::MatrixBase<ConfigVectorIn2> & q2,
                      const Scalar & prec);

  /**
   * @brief         Return true if the given configurations are equivalents
   * @warning       Two configurations can be equivalent but not equally coefficient wise (e.g for quaternions)
   *
   * @param[in]     model     Model
   * @param[in]     q1        The first configuraiton to compare
   * @param[in]     q2        The second configuration to compare
   * @param[in]     prec      precision of the comparison
   *
   * @return     Whether the configurations are equivalent or not
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
   * @brief         Return the Jacobian of the integrate function for the components of the config vector.
   *
   * @param[in]     model      Model of rigid body system.
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
   * @brief         Return the Jacobian of the integrate function for the components of the config vector.
   *
   * @param[in]     model      Model of rigid body system.
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
    integrateCoeffWiseJacobian<LieGroupMap,Scalar,Options,JointCollectionTpl,ConfigVector,JacobianMatrix>(model,q,jacobian);
  }

  /// \}

  /// \name API that allocates memory
  /// \{

  /**
   * @brief      Integrate a configuration vector for the specified model for a tangent vector during one unit time
   *
   * @param[in]  model   Model that must be integrated
   * @param[in]  q       Initial configuration (size model.nq)
   * @param[in]  v       Joint velocity (size model.nv)
   *
   * @return     The integrated configuration (size model.nq)
   */
  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType>
  inline typename PINOCCHIO_EIGEN_PLAIN_TYPE(ConfigVectorType)
  integrate(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
            const Eigen::MatrixBase<ConfigVectorType> & q,
            const Eigen::MatrixBase<TangentVectorType> & v);

  /**
   * @brief      Integrate a configuration vector for the specified model for a tangent vector during one unit time
   *
   * @param[in]  model   Model that must be integrated
   * @param[in]  q       Initial configuration (size model.nq)
   * @param[in]  v       Joint velocity (size model.nv)
   *
   * @return     The integrated configuration (size model.nq)
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
   * @brief      Interpolate two configurations for a given model
   *
   * @param[in]  model   Model to be interpolated
   * @param[in]  q0      Initial configuration vector (size model.nq)
   * @param[in]  q1      Final configuration vector (size model.nq)
   * @param[in]  u       u in [0;1] position along the interpolation.
   *
   * @return     The interpolated configuration (q0 if u = 0, q1 if u = 1)
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2>
  inline typename PINOCCHIO_EIGEN_PLAIN_TYPE(ConfigVectorIn1)
  interpolate(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
              const Eigen::MatrixBase<ConfigVectorIn1> & q0,
              const Eigen::MatrixBase<ConfigVectorIn2> & q1,
              const Scalar & u);

  /**
   * @brief      Interpolate two configurations for a given model
   *
   * @param[in]  model   Model to be interpolated
   * @param[in]  q0      Initial configuration vector (size model.nq)
   * @param[in]  q1      Final configuration vector (size model.nq)
   * @param[in]  u       u in [0;1] position along the interpolation.
   *
   * @return     The interpolated configuration (q0 if u = 0, q1 if u = 1)
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2>
  inline typename PINOCCHIO_EIGEN_PLAIN_TYPE(ConfigVectorIn1)
  interpolate(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
              const Eigen::MatrixBase<ConfigVectorIn1> & q0,
              const Eigen::MatrixBase<ConfigVectorIn2> & q1,
              const Scalar & u)
  {
    return interpolate<LieGroupMap,Scalar,Options,JointCollectionTpl,ConfigVectorIn1,ConfigVectorIn2>(model, q0, q1, u);
  }

  /**
   * @brief      Compute the tangent vector that must be integrated during one unit time to go from q0 to q1
   *
   * @param[in]  model   Model of the system
   * @param[in]  q0      Initial configuration (size model.nq)
   * @param[in]  q1      Wished configuration (size model.nq)
   *
   * @return     The corresponding velocity (size model.nv)
   */
  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2>
  inline typename PINOCCHIO_EIGEN_PLAIN_TYPE(ConfigVectorIn1)
  difference(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
             const Eigen::MatrixBase<ConfigVectorIn1> & q0,
             const Eigen::MatrixBase<ConfigVectorIn2> & q1);

  /**
   * @brief      Compute the tangent vector that must be integrated during one unit time to go from q0 to q1
   *
   * @param[in]  model   Model of the system
   * @param[in]  q0      Initial configuration (size model.nq)
   * @param[in]  q1      Wished configuration (size model.nq)
   *
   * @return     The corresponding velocity (size model.nv)
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
   * @brief      Squared distance between two configuration vectors
   *
   * @param[in]  model      Model we want to compute the distance
   * @param[in]  q0         Configuration 0 (size model.nq)
   * @param[in]  q1         Configuration 1 (size model.nq)
   *
   * @return     The corresponding squared distances for each joint (size model.njoints-1 = number of joints)
   */
  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2>
  inline typename PINOCCHIO_EIGEN_PLAIN_TYPE(ConfigVectorIn1)
  squaredDistance(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  const Eigen::MatrixBase<ConfigVectorIn1> & q0,
                  const Eigen::MatrixBase<ConfigVectorIn2> & q1);

  /**
   * @brief      Squared distance between two configuration vectors
   *
   * @param[in]  model      Model we want to compute the distance
   * @param[in]  q0         Configuration 0 (size model.nq)
   * @param[in]  q1         Configuration 1 (size model.nq)
   *
   * @return     The corresponding squared distances for each joint (size model.njoints-1 = number of joints)
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
   * @brief      Generate a configuration vector uniformly sampled among provided limits.
   *
   * @remarks    Limits are not taken into account for rotational transformations (typically SO(2),SO(3)), because they are by definition unbounded.
   *
   * @warning     If limits are infinite, exceptions may be thrown in the joint implementation of uniformlySample
   *
   * @param[in]  model        Model for which we want to generate a configuration vector.
   * @param[in]  lowerLimits  Joints lower limits
   * @param[in]  upperLimits  Joints upper limits
   *
   * @return     The resulted configuration vector (size model.nq)
   */
  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2>
  typename PINOCCHIO_EIGEN_PLAIN_TYPE((typename ModelTpl<Scalar,Options,JointCollectionTpl>::ConfigVectorType))
  randomConfiguration(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                      const Eigen::MatrixBase<ConfigVectorIn1> & lowerLimits,
                      const Eigen::MatrixBase<ConfigVectorIn2> & upperLimits);

 /**
   * @brief      Generate a configuration vector uniformly sampled among provided limits.
   *
   * @remarks    Limits are not taken into account for rotational transformations (typically SO(2),SO(3)), because they are by definition unbounded.
   *
   * @warning     If limits are infinite, exceptions may be thrown in the joint implementation of uniformlySample
   *
   * @param[in]  model        Model for which we want to generate a configuration vector.
   * @param[in]  lowerLimits  Joints lower limits
   * @param[in]  upperLimits  Joints upper limits
   *
   * @return     The resulted configuration vector (size model.nq)
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2>
  typename PINOCCHIO_EIGEN_PLAIN_TYPE((typename ModelTpl<Scalar,Options,JointCollectionTpl>::ConfigVectorType))
  randomConfiguration(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                      const Eigen::MatrixBase<ConfigVectorIn1> & lowerLimits,
                      const Eigen::MatrixBase<ConfigVectorIn2> & upperLimits)
  {
    return randomConfiguration<LieGroupMap,Scalar,Options,JointCollectionTpl,ConfigVectorIn1,ConfigVectorIn2>(model, lowerLimits.derived(), upperLimits.derived());
  }

  /**
   * @brief      Generate a configuration vector uniformly sampled among the joint limits of the specified Model.
   *
   * @remarks    Limits are not taken into account for rotational transformations (typically SO(2),SO(3)), because they are by definition unbounded.
   *
   * @warning    If limits are infinite (no one specified when adding a body or no modification directly in my_model.{lowerPositionLimit,upperPositionLimit},
   *             exceptions may be thrown in the joint implementation of uniformlySample
   *
   * @param[in]  model   Model for which we want to generate a configuration vector.
   *
   * @return     The resulted configuration vector (size model.nq)
   */
  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  typename PINOCCHIO_EIGEN_PLAIN_TYPE((typename ModelTpl<Scalar,Options,JointCollectionTpl>::ConfigVectorType))
  randomConfiguration(const ModelTpl<Scalar,Options,JointCollectionTpl> & model);

  /**
   * @brief      Generate a configuration vector uniformly sampled among the joint limits of the specified Model.
   *
   * @remarks    Limits are not taken into account for rotational transformations (typically SO(2),SO(3)), because they are by definition unbounded.
   *
   * @warning    If limits are infinite (no one specified when adding a body or no modification directly in my_model.{lowerPositionLimit,upperPositionLimit},
   *             exceptions may be thrown in the joint implementation of uniformlySample
   *
   * @param[in]  model   Model for which we want to generate a configuration vector.
   *
   * @return     The resulted configuration vector (size model.nq)
   */
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  typename PINOCCHIO_EIGEN_PLAIN_TYPE((typename ModelTpl<Scalar,Options,JointCollectionTpl>::ConfigVectorType))
  randomConfiguration(const ModelTpl<Scalar,Options,JointCollectionTpl> & model)
  {
    return randomConfiguration<LieGroupMap,Scalar,Options,JointCollectionTpl>(model);
  }

  /**
   * @brief         Return the neutral configuration element related to the model configuration space.
   *
   * @param[in]     model      Model
   *
   * @return        The neutral configuration element.
   */
  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline Eigen::Matrix<Scalar,Eigen::Dynamic,1,Options>
  neutral(const ModelTpl<Scalar,Options,JointCollectionTpl> & model);

  /**
   * @brief         Return the neutral configuration element related to the model configuration space.
   *
   * @param[in]     model      Model
   *
   * @return        The neutral configuration element.
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

#endif // ifndef __pinocchio_joint_configuration_hpp__

