//
// Copyright (c) 2018 CNRS
//

#ifndef __pinocchio_cartesian_product_variant_hpp__
#define __pinocchio_cartesian_product_variant_hpp__

#include "pinocchio/multibody/liegroup/operation-base.hpp"
#include "pinocchio/multibody/liegroup/liegroup-variant.hpp"
#include "pinocchio/multibody/liegroup/liegroup-variant-visitors.hpp"

#include <vector>

namespace pinocchio
{
  
  struct CartesianProductOperationVariant;
  
  template<>
  struct traits<CartesianProductOperationVariant>
  {
    typedef double Scalar;
    enum {
      NQ = Eigen::Dynamic,
      NV = Eigen::Dynamic
    };
  };
  
  ///
  /// \brief Dynamic Cartesian product composed of elementary Lie groups defined in LieGroupVariant
  ///
  struct CartesianProductOperationVariant : public LieGroupOperationBase<CartesianProductOperationVariant>
  {
    PINOCCHIO_LIE_GROUP_PUBLIC_INTERFACE(CartesianProductOperationVariant);
    
    /// \brief Default constructor
    CartesianProductOperationVariant()
    : m_nq(0), m_nv(0)
    , lg_nqs(0), lg_nvs(0)
    , m_neutral(0)
    {};
    
    ///
    /// \brief Constructor with one single Lie group
    ///
    /// \param[in] lg Lie group variant to insert inside the Cartesian product
    ///
    CartesianProductOperationVariant(const LieGroupVariant & lg)
    {
      append(lg);
    };
    
    ///
    /// \brief Constructor with two Lie groups
    ///
    /// \param[in] lg1 Lie group variant to insert inside the Cartesian product
    /// \param[in] lg2 Lie group variant to insert inside the Cartesian product
    ///
    CartesianProductOperationVariant(const LieGroupVariant & lg1,
                                     const LieGroupVariant & lg2)
    {
      append(lg1); append(lg2);
    };
    
    ///
    /// \brief Append a Lie group to the Cartesian product
    ///
    /// \param[in] lg Lie group variant to insert inside the Cartesian product
    ///
    void append(const LieGroupVariant & lg)
    {
      liegroups.push_back(lg);
      const Index lg_nq = ::pinocchio::nq(lg); lg_nqs.push_back(lg_nq); m_nq += lg_nq;
      const Index lg_nv = ::pinocchio::nv(lg); lg_nvs.push_back(lg_nv); m_nv += lg_nv;
      
      if(liegroups.size() > 1)
        m_name += " x ";
      m_name += ::pinocchio::name(lg);
      
      m_neutral.conservativeResize(m_nq);
      m_neutral.tail(lg_nq) = ::pinocchio::neutral(lg);
      
    }
    
    int nq() const { return m_nq; }
    int nv() const { return m_nv; }
    
    std::string name() const { return m_name; }
    
    ConfigVector_t neutral() const { return m_neutral; }
    
    template <class ConfigIn_t, class Velocity_t, class ConfigOut_t>
    void integrate_impl(const Eigen::MatrixBase<ConfigIn_t> & q,
                        const Eigen::MatrixBase<Velocity_t> & v,
                        const Eigen::MatrixBase<ConfigOut_t> & qout) const
    {
      assert(q.size() == m_nq);
      assert(v.size() == m_nv);
      assert(qout.size() == m_nq);
     
      ConfigOut_t & qout_ = const_cast< ConfigOut_t& >(qout.derived());
      Index id_q = 0, id_v = 0;
      for(size_t k = 0; k < liegroups.size(); ++k)
      {
        const Index & nq = lg_nqs[k];
        const Index & nv = lg_nvs[k];
        ::pinocchio::integrate(liegroups[k],
                         q.segment(id_q,lg_nqs[k]),
                         v.segment(id_v,lg_nvs[k]),
                         qout_.segment(id_q,lg_nqs[k]));
        
        id_q += nq; id_v += nv;
      }

    }
    
//    template <class Config_t>
//    void random_impl (const Eigen::MatrixBase<Config_t>& qout) const
//    {
//      R3crossSO3_t().random(qout);
//    }
//    
//    template <class ConfigL_t, class ConfigR_t, class ConfigOut_t>
//    void randomConfiguration_impl(const Eigen::MatrixBase<ConfigL_t> & lower,
//                                  const Eigen::MatrixBase<ConfigR_t> & upper,
//                                  const Eigen::MatrixBase<ConfigOut_t> & qout)
//    const
//    {
//      R3crossSO3_t ().randomConfiguration(lower, upper, qout);
//    }
    
  protected:
    
    std::vector<LieGroupVariant> liegroups;
    Index m_nq, m_nv;
    std::vector<Index> lg_nqs, lg_nvs;
    std::string m_name;
    
    ConfigVector_t m_neutral;
    
  };
  
} // namespace pinocchio

#endif // ifndef __pinocchio_cartesian_product_variant_hpp__
