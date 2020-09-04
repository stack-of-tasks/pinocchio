//
// Copyright (c) 2018 CNRS
//

#ifndef __pinocchio_cartesian_product_variant_hpp__
#define __pinocchio_cartesian_product_variant_hpp__

#include "pinocchio/multibody/liegroup/liegroup-base.hpp"
#include "pinocchio/multibody/liegroup/liegroup-collection.hpp"

#include "pinocchio/container/aligned-vector.hpp"

namespace pinocchio
{
  
  template<typename Scalar, int Options = 0,
    template<typename,int> class LieGroupCollectionTpl = LieGroupCollectionDefaultTpl>
      struct CartesianProductOperationVariantTpl;
  typedef CartesianProductOperationVariantTpl<double, 0, LieGroupCollectionDefaultTpl> CartesianProductOperationVariant;
  
  template<typename _Scalar, int _Options, template<typename,int> class LieGroupCollectionTpl>
  struct traits<CartesianProductOperationVariantTpl<_Scalar, _Options, LieGroupCollectionTpl> >
  {
    typedef _Scalar Scalar;
    enum {
      Options = _Options,
      NQ = Eigen::Dynamic,
      NV = Eigen::Dynamic
    };
  };
  
  ///
  /// \brief Dynamic Cartesian product composed of elementary Lie groups defined in LieGroupVariant
  ///
  template<typename _Scalar, int _Options, template<typename,int> class LieGroupCollectionTpl>
  struct CartesianProductOperationVariantTpl
  : public LieGroupBase<CartesianProductOperationVariantTpl<_Scalar, _Options, LieGroupCollectionTpl> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    PINOCCHIO_LIE_GROUP_TPL_PUBLIC_INTERFACE(CartesianProductOperationVariantTpl);

    typedef LieGroupCollectionTpl<Scalar, Options> LieGroupCollection;
    typedef typename LieGroupCollection::LieGroupVariant LieGroupVariant;
    typedef LieGroupGenericTpl<LieGroupCollection> LieGroupGeneric;
    
    /// \brief Default constructor
    CartesianProductOperationVariantTpl()
    : m_nq(0), m_nv(0)
    , lg_nqs(0), lg_nvs(0)
    , m_neutral(0)
    {};
    
    ///
    /// \brief Constructor with one single Lie group
    ///
    /// \param[in] lg Lie group variant to insert inside the Cartesian product
    ///
    explicit CartesianProductOperationVariantTpl(const LieGroupGeneric & lg)
    : m_nq(0), m_nv(0)
    , lg_nqs(0), lg_nvs(0)
    , m_neutral(0)
    {
      append(lg);
    };
    
    ///
    /// \brief Constructor with two Lie groups
    ///
    /// \param[in] lg1 Lie group variant to insert inside the Cartesian product
    /// \param[in] lg2 Lie group variant to insert inside the Cartesian product
    ///
    CartesianProductOperationVariantTpl(const LieGroupGeneric & lg1,
                                        const LieGroupGeneric & lg2)
    : m_nq(0), m_nv(0)
    , lg_nqs(0), lg_nvs(0)
    , m_neutral(0)
    {
      append(lg1); append(lg2);
    };
    
    ///
    /// \brief Append a Lie group to the Cartesian product
    ///
    /// \param[in] lg Lie group variant to insert inside the Cartesian product
    ///
    void append(const LieGroupGeneric & lg);

    ///
    /// \brief Cartesian product between *this and other.
    ///
    /// \param[in] other CartesianProductOperation to compose with this
    ///
    /// \returns A new Cartesian product betwenn *this and other.
    ///
    CartesianProductOperationVariantTpl operator* (const CartesianProductOperationVariantTpl& other) const;

    ///
    /// \brief Append other to *this.
    ///
    /// \param[in] other CartesianProductOperation to append to *this.
    ///
    CartesianProductOperationVariantTpl& operator*= (const CartesianProductOperationVariantTpl& other);

    ///
    /// \brief Append a Lie group to *this.
    ///
    /// \param[in] lg LieGroupGeneric to append to *this.
    ///
    inline CartesianProductOperationVariantTpl& operator*= (const LieGroupGeneric& lg)
    {
      append(lg);
      return *this;
    }
    
    int nq() const { return m_nq; }
    int nv() const { return m_nv; }
    
    std::string name() const { return m_name; }
    
    ConfigVector_t neutral() const { return m_neutral; }
    
    template <class ConfigL_t, class ConfigR_t, class Tangent_t>
    void difference_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                         const Eigen::MatrixBase<ConfigR_t> & q1,
                         const Eigen::MatrixBase<Tangent_t> & d) const;

    template <ArgumentPosition arg, class ConfigL_t, class ConfigR_t, class JacobianOut_t>
    void dDifference_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                          const Eigen::MatrixBase<ConfigR_t> & q1,
                          const Eigen::MatrixBase<JacobianOut_t> & J) const;

    template <ArgumentPosition arg, class ConfigL_t, class ConfigR_t, class JacobianIn_t, class JacobianOut_t>
    void dDifference_product_impl(const ConfigL_t & q0,
                                  const ConfigR_t & q1,
                                  const JacobianIn_t & Jin,
                                  JacobianOut_t & Jout,
                                  bool dDifferenceOnTheLeft,
                                  const AssignmentOperatorType op) const;

    template <class ConfigIn_t, class Velocity_t, class ConfigOut_t>
    void integrate_impl(const Eigen::MatrixBase<ConfigIn_t> & q,
                        const Eigen::MatrixBase<Velocity_t> & v,
                        const Eigen::MatrixBase<ConfigOut_t> & qout) const;

    template <class Config_t, class Jacobian_t>
    void integrateCoeffWiseJacobian_impl(const Eigen::MatrixBase<Config_t> & q,
                                         const Eigen::MatrixBase<Jacobian_t> & J) const;

    template <class Config_t, class Tangent_t, class JacobianOut_t>
    void dIntegrate_dq_impl(const Eigen::MatrixBase<Config_t > & q,
                            const Eigen::MatrixBase<Tangent_t> & v,
                            const Eigen::MatrixBase<JacobianOut_t> & J,
                            const AssignmentOperatorType op=SETTO) const;

    template <class Config_t, class Tangent_t, class JacobianOut_t>
    void dIntegrate_dv_impl(const Eigen::MatrixBase<Config_t > & q,
                            const Eigen::MatrixBase<Tangent_t> & v,
                            const Eigen::MatrixBase<JacobianOut_t> & J,
                            const AssignmentOperatorType op=SETTO) const;

    template <class Config_t, class Tangent_t, class JacobianIn_t, class JacobianOut_t>
    void dIntegrate_product_impl(const Config_t & q,
                                 const Tangent_t & v,
                                 const JacobianIn_t & Jin,
                                 JacobianOut_t & Jout,
                                 bool dIntegrateOnTheLeft,
                                 const ArgumentPosition arg,
                                 const AssignmentOperatorType op) const;

    template <class Config_t, class Tangent_t, class JacobianIn_t, class JacobianOut_t>
    void dIntegrateTransport_dq_impl(const Eigen::MatrixBase<Config_t > & q,
                                     const Eigen::MatrixBase<Tangent_t> & v,
                                     const Eigen::MatrixBase<JacobianIn_t> & J_in,
                                     const Eigen::MatrixBase<JacobianOut_t> & J_out) const;

    template <class Config_t, class Tangent_t, class JacobianIn_t, class JacobianOut_t>
    void dIntegrateTransport_dv_impl(const Eigen::MatrixBase<Config_t > & q,
                                     const Eigen::MatrixBase<Tangent_t> & v,
                                     const Eigen::MatrixBase<JacobianIn_t> & J_in,
                                     const Eigen::MatrixBase<JacobianOut_t> & J_out) const;

    template <class Config_t, class Tangent_t, class JacobianOut_t>
    void dIntegrateTransport_dq_impl(const Eigen::MatrixBase<Config_t > & q,
                                     const Eigen::MatrixBase<Tangent_t> & v,
                                     const Eigen::MatrixBase<JacobianOut_t> & J) const;

    template <class Config_t, class Tangent_t, class JacobianOut_t>
    void dIntegrateTransport_dv_impl(const Eigen::MatrixBase<Config_t > & q,
                                     const Eigen::MatrixBase<Tangent_t> & v,
                                     const Eigen::MatrixBase<JacobianOut_t> & J) const;

    template <class ConfigL_t, class ConfigR_t>
    Scalar squaredDistance_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                const Eigen::MatrixBase<ConfigR_t> & q1) const;

    template <class Config_t>
    void normalize_impl (const Eigen::MatrixBase<Config_t>& qout) const;

    template <class Config_t>
    void random_impl (const Eigen::MatrixBase<Config_t>& qout) const;

    template <class ConfigL_t, class ConfigR_t, class ConfigOut_t>
    void randomConfiguration_impl(const Eigen::MatrixBase<ConfigL_t> & lower,
                                  const Eigen::MatrixBase<ConfigR_t> & upper,
                                  const Eigen::MatrixBase<ConfigOut_t> & qout) const;

    template <class ConfigL_t, class ConfigR_t>
    bool isSameConfiguration_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                  const Eigen::MatrixBase<ConfigR_t> & q1,
                                  const Scalar & prec) const;

    bool isEqual_impl (const CartesianProductOperationVariantTpl& other) const;

    template <typename LieGroup1, typename LieGroup2>
    bool isEqual(const CartesianProductOperation<LieGroup1, LieGroup2> & other) const;

  protected:
    
    PINOCCHIO_ALIGNED_STD_VECTOR(LieGroupGeneric) liegroups;
    Index m_nq, m_nv;
    std::vector<Index> lg_nqs, lg_nvs;
    std::string m_name;
    
    ConfigVector_t m_neutral;
    
  };
  
} // namespace pinocchio

#include <pinocchio/multibody/liegroup/cartesian-product-variant.hxx>

#endif // ifndef __pinocchio_cartesian_product_variant_hpp__
