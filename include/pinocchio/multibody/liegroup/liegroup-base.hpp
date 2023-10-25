//
// Copyright (c) 2016-2020 CNRS INRIA
//

#ifndef __pinocchio_multibody_liegroup_liegroup_operation_base_hpp__
#define __pinocchio_multibody_liegroup_liegroup_operation_base_hpp__

#include "pinocchio/multibody/liegroup/fwd.hpp"

#include <limits>

namespace pinocchio
{
#ifdef PINOCCHIO_WITH_CXX11_SUPPORT
  constexpr int SELF = 0;
#else
  enum { SELF = 0 };
#endif
  
#define PINOCCHIO_LIE_GROUP_PUBLIC_INTERFACE_GENERIC(Derived,TYPENAME)               \
  typedef          LieGroupBase<Derived> Base;                        \
  typedef TYPENAME Base::Index Index;                                          \
  typedef TYPENAME traits<Derived>::Scalar Scalar;                             \
  enum {                                                                       \
    Options = traits<Derived>::Options,                                        \
    NQ = Base::NQ,                                                             \
    NV = Base::NV                                                              \
  };                                                                           \
  typedef TYPENAME Base::ConfigVector_t ConfigVector_t;                        \
  typedef TYPENAME Base::TangentVector_t TangentVector_t;                      \
  typedef TYPENAME Base::JacobianMatrix_t JacobianMatrix_t
  
#define PINOCCHIO_LIE_GROUP_PUBLIC_INTERFACE(Derived)                                \
PINOCCHIO_LIE_GROUP_PUBLIC_INTERFACE_GENERIC(Derived,PINOCCHIO_MACRO_EMPTY_ARG)
  
#define PINOCCHIO_LIE_GROUP_TPL_PUBLIC_INTERFACE(Derived)                            \
PINOCCHIO_LIE_GROUP_PUBLIC_INTERFACE_GENERIC(Derived,typename)

  template<typename Derived>
  struct LieGroupBase
  {
    typedef Derived LieGroupDerived;
    typedef int Index;
    typedef typename traits<LieGroupDerived>::Scalar Scalar;
    enum
    {
      Options = traits<LieGroupDerived>::Options,
      NQ = traits<LieGroupDerived>::NQ,
      NV = traits<LieGroupDerived>::NV
    };

    typedef Eigen::Matrix<Scalar,NQ,1,Options> ConfigVector_t;
    typedef Eigen::Matrix<Scalar,NV,1,Options> TangentVector_t;
    typedef Eigen::Matrix<Scalar,NV,NV,Options> JacobianMatrix_t;

    /// \name API with return value as argument
    /// \{

    /**
     * @brief      Integrate a joint's configuration with a tangent vector during one unit time duration
     *
     * @param[in]  q     the initial configuration.
     * @param[in]  v     the tangent velocity.
     *
     * @param[out] qout  the configuration integrated.
     */
    template <class ConfigIn_t, class Tangent_t, class ConfigOut_t>
    void integrate(const Eigen::MatrixBase<ConfigIn_t> & q,
                   const Eigen::MatrixBase<Tangent_t>  & v,
                   const Eigen::MatrixBase<ConfigOut_t> & qout) const;
    
    /**
     * @brief      Computes the Jacobian of the integrate operator around zero.
     *
     * @details    This function computes the Jacobian of the configuration vector variation (component-vise) with respect to a small variation
     *             in the tangent.
     *
     * @param[in]  q    configuration vector.
     *
     * @param[out] J    the Jacobian of the log of the Integrate operation w.r.t. the configuration vector q.
     *
     * @remarks    This function might be useful for instance when using google-ceres to compute the Jacobian of the integrate operation.
     */
    template<class Config_t, class Jacobian_t>
    void integrateCoeffWiseJacobian(const Eigen::MatrixBase<Config_t >  & q,
                                    const Eigen::MatrixBase<Jacobian_t> & J) const;

    /**
     * @brief      Computes the Jacobian of a small variation of the configuration vector or the tangent vector into tangent space at identity.
     *
     * @details    This Jacobian corresponds to the Jacobian of \f$ (\mathbf{q} \oplus \delta \mathbf{q}) \oplus \mathbf{v} \f$ with
     *             \f$ \delta \mathbf{q} \rightarrow 0 \f$ if arg == ARG0 or \f$ \delta \mathbf{v} \rightarrow 0 \f$ if arg == ARG1.
     *
     * @param[in]  q    configuration vector.
     * @param[in]  v    tangent vector.
     * @param[in]  op   assignment operator (SETTO, ADDTO or RMTO).
     * @tparam     arg  ARG0 (resp. ARG1) to get the Jacobian with respect to q (resp. v).
     *
     * @param[out] J    the Jacobian of the Integrate operation w.r.t. the argument arg.
     */
    template <ArgumentPosition arg, class Config_t, class Tangent_t, class JacobianOut_t>
    void dIntegrate(const Eigen::MatrixBase<Config_t >  & q,
                    const Eigen::MatrixBase<Tangent_t>  & v,
                    const Eigen::MatrixBase<JacobianOut_t> & J,
                    AssignmentOperatorType op = SETTO) const
    {
      PINOCCHIO_STATIC_ASSERT(arg==ARG0||arg==ARG1, arg_SHOULD_BE_ARG0_OR_ARG1);
      return dIntegrate(q.derived(),v.derived(),PINOCCHIO_EIGEN_CONST_CAST(JacobianOut_t,J),arg,op);
    }
    
    /**
     * @brief      Computes the Jacobian of a small variation of the configuration vector or the tangent vector into tangent space at identity.
     *
     * @details    This Jacobian corresponds to the Jacobian of \f$ (\mathbf{q} \oplus \delta \mathbf{q}) \oplus \mathbf{v} \f$ with
     *             \f$ \delta \mathbf{q} \rightarrow 0 \f$ if arg == ARG0 or \f$ \delta \mathbf{v} \rightarrow 0 \f$ if arg == ARG1.
     *
     * @param[in]  q    configuration vector.
     * @param[in]  v    tangent vector.
     * @param[in] arg  ARG0 (resp. ARG1) to get the Jacobian with respect to q (resp. v).
     * @param[in]  op   assignment operator (SETTO, ADDTO and RMTO).
     *
     * @param[out] J    the Jacobian of the Integrate operation w.r.t. the argument arg.
     */
    template<class Config_t, class Tangent_t, class JacobianOut_t>
    void dIntegrate(const Eigen::MatrixBase<Config_t >  & q,
                    const Eigen::MatrixBase<Tangent_t>  & v,
                    const Eigen::MatrixBase<JacobianOut_t> & J,
                    const ArgumentPosition arg,
                    const AssignmentOperatorType op = SETTO) const;

    /**
     * @brief      Computes the Jacobian of a small variation of the configuration vector into tangent space at identity.
     *
     * @details    This Jacobian corresponds to the Jacobian of \f$ (\mathbf{q} \oplus \delta \mathbf{q}) \oplus \mathbf{v} \f$ with
     *             \f$ \delta \mathbf{q} \rightarrow 0 \f$.
     *
     * @param[in]  q    configuration vector.
     * @param[in]  v    tangent vector.
     * @param[in]  op   assignment operator (SETTO, ADDTO or RMTO).
     *
     * @param[out] J    the Jacobian of the Integrate operation w.r.t. the configuration vector q.
     */
    template <class Config_t, class Tangent_t, class JacobianOut_t>
    void dIntegrate_dq(const Eigen::MatrixBase<Config_t >  & q,
                       const Eigen::MatrixBase<Tangent_t>  & v,
                       const Eigen::MatrixBase<JacobianOut_t> & J,
                       const AssignmentOperatorType op = SETTO) const;

    template <class Config_t, class Tangent_t, class JacobianIn_t, class JacobianOut_t>
    void dIntegrate_dq(const Eigen::MatrixBase<Config_t >  & q,
                       const Eigen::MatrixBase<Tangent_t>  & v,
                       const Eigen::MatrixBase<JacobianIn_t> & Jin,
                       int self,
                       const Eigen::MatrixBase<JacobianOut_t> & Jout,
                       const AssignmentOperatorType op = SETTO) const;

    template <class Config_t, class Tangent_t, class JacobianIn_t, class JacobianOut_t>
    void dIntegrate_dq(const Eigen::MatrixBase<Config_t >  & q,
                       const Eigen::MatrixBase<Tangent_t>  & v,
                       int self,
                       const Eigen::MatrixBase<JacobianIn_t> & Jin,
                       const Eigen::MatrixBase<JacobianOut_t> & Jout,
                       const AssignmentOperatorType op = SETTO) const;

    /**
     * @brief      Computes the Jacobian of a small variation of the tangent vector into tangent space at identity.
     *
     * @details    This Jacobian corresponds to the Jacobian of \f$ \mathbf{q} \oplus (\mathbf{v}  + \delta \mathbf{v}) \f$ with
     *             \f$ \delta \mathbf{v} \rightarrow 0 \f$.
     *
     * @param[in]  q    configuration vector.
     * @param[in]  v    tangent vector.
     * @param[in]  op   assignment operator (SETTO, ADDTO or RMTO).
     *
     * @param[out] J    the Jacobian of the Integrate operation w.r.t. the tangent vector v.
     */
    template <class Config_t, class Tangent_t, class JacobianOut_t>
    void dIntegrate_dv(const Eigen::MatrixBase<Config_t >  & q,
                       const Eigen::MatrixBase<Tangent_t>  & v,
                       const Eigen::MatrixBase<JacobianOut_t> & J,
                       const AssignmentOperatorType op = SETTO) const;

    template <class Config_t, class Tangent_t, class JacobianIn_t, class JacobianOut_t>
    void dIntegrate_dv(const Eigen::MatrixBase<Config_t >  & q,
                       const Eigen::MatrixBase<Tangent_t>  & v,
                       int self,
                       const Eigen::MatrixBase<JacobianIn_t> & Jin,
                       const Eigen::MatrixBase<JacobianOut_t> & Jout,
                       const AssignmentOperatorType op = SETTO) const;

    template <class Config_t, class Tangent_t, class JacobianIn_t, class JacobianOut_t>
    void dIntegrate_dv(const Eigen::MatrixBase<Config_t >  & q,
                       const Eigen::MatrixBase<Tangent_t>  & v,
                       const Eigen::MatrixBase<JacobianIn_t> & Jin,
                       int self,
                       const Eigen::MatrixBase<JacobianOut_t> & Jout,
                       const AssignmentOperatorType op = SETTO) const;

    /**
     *
     * @brief   Transport a matrix from the terminal to the originate tangent space of the integrate operation, with respect to the configuration or the velocity arguments.
     *
     * @details This function performs the parallel transportation of an input matrix whose columns are expressed in the tangent space of the integrated element \f$ q \oplus v \f$,
     *          to the tangent space at \f$ q \f$.
   *          In other words, this functions transforms a tangent vector expressed at \f$ q \oplus v \f$ to a tangent vector expressed at \f$ q \f$, considering that the change of configuration between
   *          \f$ q \oplus v \f$ and \f$ q \f$ may alter the value of this tangent vector.
   *          A typical example of parallel transportation is the action operated by a rigid transformation \f$ M \in \text{SE}(3)\f$ on a spatial velocity \f$ v \in \text{se}(3)\f$.
     *          In the context of configuration spaces assimilated as vectorial spaces, this operation corresponds to Identity.
     *          For Lie groups, its corresponds to the canonical vector field transportation.
     *
     * @param[in]  q        configuration vector.
     * @param[in]  v        tangent vector
     * @param[in]  Jin    the input matrix
     * @param[in]  arg    argument with respect to which the differentiation is performed (ARG0 corresponding to q, and ARG1 to v)
     *
     * @param[out] Jout    Transported matrix
     *
     */
    template<class Config_t, class Tangent_t, class JacobianIn_t, class JacobianOut_t>
    void dIntegrateTransport(const Eigen::MatrixBase<Config_t >  & q,
                             const Eigen::MatrixBase<Tangent_t>  & v,
                             const Eigen::MatrixBase<JacobianIn_t> & Jin,
                             const Eigen::MatrixBase<JacobianOut_t> & Jout,
                             const ArgumentPosition arg) const;

    /**
     *
     * @brief   Transport a matrix from the terminal to the originate tangent space of the integrate operation, with respect to the configuration argument.
     *
     * @details This function performs the parallel transportation of an input matrix whose columns are expressed in the tangent space of the integrated element \f$ q \oplus v \f$,
     *          to the tangent space at \f$ q \f$.
     *          In other words, this functions transforms a tangent vector expressed at \f$ q \oplus v \f$ to a tangent vector expressed at \f$ q \f$, considering that the change of configuration between
     *          \f$ q \oplus v \f$ and \f$ q \f$ may alter the value of this tangent vector.
     *          A typical example of parallel transportation is the action operated by a rigid transformation \f$ M \in \text{SE}(3)\f$ on a spatial velocity \f$ v \in \text{se}(3)\f$.
     *          In the context of configuration spaces assimilated as vectorial spaces, this operation corresponds to Identity.
     *          For Lie groups, its corresponds to the canonical vector field transportation.
     *
     * @param[in]  q    configuration vector.
     * @param[in]  v    tangent vector
     * @param[in]  Jin    the input matrix
     *
     * @param[out] Jout    Transported matrix
     */
    template <class Config_t, class Tangent_t, class JacobianIn_t, class JacobianOut_t>
    void dIntegrateTransport_dq(const Eigen::MatrixBase<Config_t >  & q,
                                const Eigen::MatrixBase<Tangent_t>  & v,
                                const Eigen::MatrixBase<JacobianIn_t> & Jin,
                                const Eigen::MatrixBase<JacobianOut_t> & Jout) const;
    /**
     *
     * @brief   Transport a matrix from the terminal to the originate tangent space of the integrate operation, with respect to the velocity argument.
     *
     * @details This function performs the parallel transportation of an input matrix whose columns are expressed in the tangent space of the integrated element \f$ q \oplus v \f$,
     *          to the tangent space at \f$ q \f$.
   *          In other words, this functions transforms a tangent vector expressed at \f$ q \oplus v \f$ to a tangent vector expressed at \f$ q \f$, considering that the change of configuration between
   *          \f$ q \oplus v \f$ and \f$ q \f$ may alter the value of this tangent vector.
   *          A typical example of parallel transportation is the action operated by a rigid transformation \f$ M \in \text{SE}(3)\f$ on a spatial velocity \f$ v \in \text{se}(3)\f$.
     *          In the context of configuration spaces assimilated as vectorial spaces, this operation corresponds to Identity.
     *          For Lie groups, its corresponds to the canonical vector field transportation.
     *
     * @param[in]  q    configuration vector.
     * @param[in]  v    tangent vector
     * @param[in]  Jin    the input matrix
     *
     * @param[out] Jout    Transported matrix
     */
    template <class Config_t, class Tangent_t, class JacobianIn_t, class JacobianOut_t>
    void dIntegrateTransport_dv(const Eigen::MatrixBase<Config_t >  & q,
                                const Eigen::MatrixBase<Tangent_t>  & v,
                                const Eigen::MatrixBase<JacobianIn_t> & Jin,
                                const Eigen::MatrixBase<JacobianOut_t> & Jout) const;
  

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
     * @param[in]     q       configuration vector.
     * @param[in]     v       tangent vector
     * @param[in,out] J       the inplace matrix
     * @param[in]     arg  argument with respect to which the differentiation is performed (ARG0 corresponding to q, and ARG1 to v)
     */
    template<class Config_t, class Tangent_t, class Jacobian_t>
    void dIntegrateTransport(const Eigen::MatrixBase<Config_t >  & q,
                             const Eigen::MatrixBase<Tangent_t>  & v,
                             const Eigen::MatrixBase<Jacobian_t> & J,
                             const ArgumentPosition arg) const;

    /**
     *
     * @brief   Transport in place a matrix from the terminal to the originate tangent space of the integrate operation, with respect to the configuration argument.
     *
     * @details This function performs the parallel transportation of an input matrix whose columns are expressed in the tangent space of the integrated element \f$ q \oplus v \f$,
     *          to the tangent space at \f$ q \f$.
   *          In other words, this functions transforms a tangent vector expressed at \f$ q \oplus v \f$ to a tangent vector expressed at \f$ q \f$, considering that the change of configuration between
   *          \f$ q \oplus v \f$ and \f$ q \f$ may alter the value of this tangent vector.
   *          A typical example of parallel transportation is the action operated by a rigid transformation \f$ M \in \text{SE}(3)\f$ on a spatial velocity \f$ v \in \text{se}(3)\f$.
     *          In the context of configuration spaces assimilated as vectorial spaces, this operation corresponds to Identity.
     *          For Lie groups, its corresponds to the canonical vector field transportation.
     *
     * @param[in]  q    configuration vector.
     * @param[in]  v    tangent vector
     * @param[in,out]  Jin    the inplace matrix
     *
    */
    template <class Config_t, class Tangent_t, class Jacobian_t>
    void dIntegrateTransport_dq(const Eigen::MatrixBase<Config_t >  & q,
                                const Eigen::MatrixBase<Tangent_t>  & v,
                                const Eigen::MatrixBase<Jacobian_t> & J) const;
    /**
     *
     * @brief   Transport in place a matrix from the terminal to the originate tangent space of the integrate operation, with respect to the velocity argument.
     *
     * @details This function performs the parallel transportation of an input matrix whose columns are expressed in the tangent space of the integrated element \f$ q \oplus v \f$,
     *          to the tangent space at \f$ q \f$.
   *          In other words, this functions transforms a tangent vector expressed at \f$ q \oplus v \f$ to a tangent vector expressed at \f$ q \f$, considering that the change of configuration between
   *          \f$ q \oplus v \f$ and \f$ q \f$ may alter the value of this tangent vector.
   *          A typical example of parallel transportation is the action operated by a rigid transformation \f$ M \in \text{SE}(3)\f$ on a spatial velocity \f$ v \in \text{se}(3)\f$.
     *          In the context of configuration spaces assimilated as vectorial spaces, this operation corresponds to Identity.
     *          For Lie groups, its corresponds to the canonical vector field transportation.
     *
     * @param[in]  q    configuration vector.
     * @param[in]  v    tangent vector
     * @param[in,out]  J    the inplace matrix
     *
    */
    template <class Config_t, class Tangent_t, class Jacobian_t>
    void dIntegrateTransport_dv(const Eigen::MatrixBase<Config_t >  & q,
                                const Eigen::MatrixBase<Tangent_t>  & v,
                                const Eigen::MatrixBase<Jacobian_t> & J) const;

    /**
     * @brief      Interpolation between two joint's configurations
     *
     * @param[in]  q0    the initial configuration to interpolate.
     * @param[in]  q1    the final configuration to interpolate.
     * @param[in]  u     in [0;1] the absicca along the interpolation.
     *
     * @param[out] qout  the interpolated configuration (q0 if u = 0, q1 if u = 1)
     */
    template <class ConfigL_t, class ConfigR_t, class ConfigOut_t>
    void interpolate(const Eigen::MatrixBase<ConfigL_t> & q0,
                     const Eigen::MatrixBase<ConfigR_t> & q1,
                     const Scalar& u,
                     const Eigen::MatrixBase<ConfigOut_t> & qout) const;
    
    /**
     * @brief      Normalize the joint configuration given as input.
     *             For instance, the quaternion must be unitary.
     *
     * @note       If the input vector is too small (i.e., qout.norm()==0), then it is left unchanged.
     *             It is therefore possible that after this method is called `isNormalized(qout)` is still false.
     * 
     * @param[in,out]     qout  the normalized joint configuration.
     */
    template <class Config_t>
    void normalize(const Eigen::MatrixBase<Config_t> & qout) const;

    /**
     * @brief      Check whether the input joint configuration is normalized.
     *             For instance, the quaternion must be unitary.
     * 
     * @param[in]     qin  The joint configuration to check.
     * 
     * @return true if the input vector is normalized, false otherwise.
     */
    template <class Config_t>
    bool isNormalized(const Eigen::MatrixBase<Config_t> & qin,
                      const Scalar& prec = Eigen::NumTraits<Scalar>::dummy_precision()) const;

    /**
     * @brief      Generate a random joint configuration, normalizing quaternions when necessary.
     *
     * \warning    Do not take into account the joint limits. To shoot a configuration uniformingly
     *             depending on joint limits, see randomConfiguration.
     *
     * @param[out] qout  the random joint configuration.
     */
    template <class Config_t>
    void random(const Eigen::MatrixBase<Config_t> & qout) const;

    /**
     * @brief      Generate a configuration vector uniformly sampled among
     *             provided limits.
     *
     * @param[in]  lower_pos_limit  the lower joint limit vector.
     * @param[in]  upper_pos_limit  the upper joint limit vector.
     *
     * @param[out] qout             the random joint configuration in the two bounds.
     */
    template <class ConfigL_t, class ConfigR_t, class ConfigOut_t>
    void randomConfiguration(const Eigen::MatrixBase<ConfigL_t> & lower_pos_limit,
                             const Eigen::MatrixBase<ConfigR_t> & upper_pos_limit,
                             const Eigen::MatrixBase<ConfigOut_t> & qout) const;

    /**
     * @brief      Computes the tangent vector that must be integrated during one unit time to go from q0 to q1.
     *
     * @param[in]  q0    the initial configuration vector.
     * @param[in]  q1    the terminal configuration vector.
     *
     * @param[out] v     the corresponding velocity.
     *
     * @note             Both inputs must be well-formed configuration vectors. The output of this function is
     *                   unspecified if inputs contain NaNs or extremal values for the underlying scalar type.
     *
     * \cheatsheet \f$ q_1 \ominus q_0 = - \left( q_0 \ominus q_1 \right) \f$
     */
    template <class ConfigL_t, class ConfigR_t, class Tangent_t>
    void difference(const Eigen::MatrixBase<ConfigL_t> & q0,
                    const Eigen::MatrixBase<ConfigR_t> & q1,
                    const Eigen::MatrixBase<Tangent_t> & v) const;

    /**
     *
     * @brief      Computes the Jacobian of the difference operation with respect to q0 or q1.
     *
     * @tparam     arg   ARG0 (resp. ARG1) to get the Jacobian with respect to q0 (resp. q1).
     *
     * @param[in]  q0    the initial configuration vector.
     * @param[in]  q1    the terminal configuration vector.
     *
     * @param[out] J     the Jacobian of the difference operation.
     *
     * \warning because \f$ q_1 \ominus q_0 = - \left( q_0 \ominus q_1 \right) \f$,
     * you may be tempted to write
     * \f$ \frac{\partial\ominus}{\partial q_1} = - \frac{\partial\ominus}{\partial q_0} \f$.
     * This is **false** in the general case because
     * \f$ \frac{\partial\ominus}{\partial q_i} \f$ expects an input velocity relative to frame i.
     * See SpecialEuclideanOperationTpl<3,_Scalar,_Options>::dDifference_impl.
     *
     * \cheatsheet \f$ \frac{\partial\ominus}{\partial q_1} \frac{\partial\oplus}{\partial v} = I \f$
     */
    template <ArgumentPosition arg, class ConfigL_t, class ConfigR_t, class JacobianOut_t>
    void dDifference(const Eigen::MatrixBase<ConfigL_t> & q0,
                     const Eigen::MatrixBase<ConfigR_t> & q1,
                     const Eigen::MatrixBase<JacobianOut_t> & J) const;

    /**
     *
     * @brief      Computes the Jacobian of the difference operation with respect to q0 or q1.
     *
     * @param[in]  q0    the initial configuration vector.
     * @param[in]  q1    the terminal configuration vector.
     * @param[in]  arg   ARG0 (resp. ARG1) to get the Jacobian with respect to q0 (resp. q1).
     *
     * @param[out] J     the Jacobian of the difference operation.
     *
     */
    template<class ConfigL_t, class ConfigR_t, class JacobianOut_t>
    void dDifference(const Eigen::MatrixBase<ConfigL_t> & q0,
                     const Eigen::MatrixBase<ConfigR_t> & q1,
                     const Eigen::MatrixBase<JacobianOut_t> & J,
                     const ArgumentPosition arg) const;

    template<ArgumentPosition arg, class ConfigL_t, class ConfigR_t, class JacobianIn_t, class JacobianOut_t>
    void dDifference(const Eigen::MatrixBase<ConfigL_t> & q0,
                     const Eigen::MatrixBase<ConfigR_t> & q1,
                     const Eigen::MatrixBase<JacobianIn_t> & Jin,
                     int self,
                     const Eigen::MatrixBase<JacobianOut_t> & Jout,
                     const AssignmentOperatorType op = SETTO) const;

    template<ArgumentPosition arg, class ConfigL_t, class ConfigR_t, class JacobianIn_t, class JacobianOut_t>
    void dDifference(const Eigen::MatrixBase<ConfigL_t> & q0,
                     const Eigen::MatrixBase<ConfigR_t> & q1,
                     int self,
                     const Eigen::MatrixBase<JacobianIn_t> & Jin,
                     const Eigen::MatrixBase<JacobianOut_t> & Jout,
                     const AssignmentOperatorType op = SETTO) const;

    /**
     * @brief      Squared distance between two joint configurations.
     *
     * @param[in]  q0    the initial configuration vector.
     * @param[in]  q1    the terminal configuration vector.
     *
     * @param[out] d     the corresponding distance betwenn q0 and q1.
     */
    template <class ConfigL_t, class ConfigR_t>
    Scalar squaredDistance(const Eigen::MatrixBase<ConfigL_t> & q0,
                           const Eigen::MatrixBase<ConfigR_t> & q1) const;

    /**
     * @brief      Distance between two configurations of the joint
     *
     * @param[in]  q0    the initial configuration vector.
     * @param[in]  q1    the terminal configuration vector.
     *
     * @return     The corresponding distance.
     */
    template <class ConfigL_t, class ConfigR_t>
    Scalar distance(const Eigen::MatrixBase<ConfigL_t> & q0,
                    const Eigen::MatrixBase<ConfigR_t> & q1) const;

    /**
     * @brief      Check if two configurations are equivalent within the given precision.
     *
     * @param[in]  q0    Configuration 0
     * @param[in]  q1    Configuration 1
     *
     * \cheatsheet \f$ q_1 \equiv  q_0 \oplus \left( q_1 \ominus q_0 \right) \f$ (\f$\equiv\f$ means equivalent, not equal).
     */
    template <class ConfigL_t, class ConfigR_t>
    bool isSameConfiguration(const Eigen::MatrixBase<ConfigL_t> & q0,
                             const Eigen::MatrixBase<ConfigR_t> & q1,
                             const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision()) const;

    bool operator== (const LieGroupBase& other) const
    {
      return derived().isEqual_impl(other.derived());
    }

    bool operator!= (const LieGroupBase& other) const
    {
      return derived().isDifferent_impl(other.derived());
    }
    /// \}

    /// \name API that allocates memory
    /// \{

    template <class Config_t, class Tangent_t>
    ConfigVector_t integrate(const Eigen::MatrixBase<Config_t>  & q,
                             const Eigen::MatrixBase<Tangent_t> & v) const ;

    template <class ConfigL_t, class ConfigR_t>
    ConfigVector_t interpolate(const Eigen::MatrixBase<ConfigL_t> & q0,
                               const Eigen::MatrixBase<ConfigR_t> & q1,
                               const Scalar& u) const;

    ConfigVector_t random() const;

    template <class ConfigL_t, class ConfigR_t>
    ConfigVector_t randomConfiguration
    (const Eigen::MatrixBase<ConfigL_t> & lower_pos_limit,
     const Eigen::MatrixBase<ConfigR_t> & upper_pos_limit) const;

    template <class ConfigL_t, class ConfigR_t>
    TangentVector_t difference(const Eigen::MatrixBase<ConfigL_t> & q0,
                               const Eigen::MatrixBase<ConfigR_t> & q1) const;
    /// \}


    /// \name Default implementations
    /// \{

    template <class Config_t, class Tangent_t, class JacobianIn_t, class JacobianOut_t>
    void dIntegrate_product_impl(const Config_t & q,
                                 const Tangent_t & v,
                                 const JacobianIn_t & Jin,
                                 JacobianOut_t & Jout,
                                 bool dIntegrateOnTheLeft,
                                 const ArgumentPosition arg,
                                 const AssignmentOperatorType op) const;

    template <ArgumentPosition arg, class ConfigL_t, class ConfigR_t, class JacobianIn_t, class JacobianOut_t>
    void dDifference_product_impl(const ConfigL_t & q0,
                                  const ConfigR_t & q1,
                                  const JacobianIn_t & Jin,
                                  JacobianOut_t & Jout,
                                  bool dDifferenceOnTheLeft,
                                  const AssignmentOperatorType op) const;

    template <class ConfigL_t, class ConfigR_t, class ConfigOut_t>
    void interpolate_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                          const Eigen::MatrixBase<ConfigR_t> & q1,
                          const Scalar& u,
                          const Eigen::MatrixBase<ConfigOut_t> & qout) const;
    
    template <class Config_t>
    void normalize_impl(const Eigen::MatrixBase<Config_t> & qout) const;

    template <class Config_t>
    bool isNormalized_impl(const Eigen::MatrixBase<Config_t> & qin,
                           const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision()) const;

    template <class ConfigL_t, class ConfigR_t>
    Scalar squaredDistance_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                const Eigen::MatrixBase<ConfigR_t> & q1) const;

    template <class ConfigL_t, class ConfigR_t>
    bool isSameConfiguration_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                  const Eigen::MatrixBase<ConfigR_t> & q1,
                                  const Scalar & prec) const;
 
    /// \brief Default equality check.
    /// By default, two LieGroupBase of same type are considered equal.
    bool isEqual_impl (const LieGroupBase& /*other*/) const { return true; }
    bool isDifferent_impl (const LieGroupBase& other) const
    {
      return !derived().isEqual_impl(other.derived());
    }

    /// Get dimension of Lie Group vector representation
    ///
    /// For instance, for SO(3), the dimension of the vector representation is
    /// 4 (quaternion) while the dimension of the tangent space is 3.
    Index nq () const;
    /// Get dimension of Lie Group tangent space
    Index nv () const;
    /// Get neutral element as a vector
    ConfigVector_t neutral () const;

    /// Get name of instance
    std::string name () const;

    Derived& derived ()
    {
      return static_cast <Derived&> (*this);
    }

    const Derived& derived () const
    {
      return static_cast <const Derived&> (*this);
    }
    /// \}

  protected:
    /// Default constructor.
    ///
    /// Prevent the construction of derived class.
    LieGroupBase() {}

    /// Copy constructor
    ///
    /// Prevent the copy of derived class.
    LieGroupBase( const LieGroupBase & /*clone*/) {}
    LieGroupBase& operator=(const LieGroupBase & /*x*/) { return *this; }

    // C++11
    // LieGroupBase(const LieGroupBase &) = delete;
    // LieGroupBase& operator=(const LieGroupBase & /*x*/) = delete;
  }; // struct LieGroupBase

} // namespace pinocchio

#include "pinocchio/multibody/liegroup/liegroup-base.hxx"

#endif // ifndef __pinocchio_multibody_liegroup_liegroup_operation_base_hpp__
