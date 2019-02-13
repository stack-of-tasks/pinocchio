//
// Copyright (c) 2016-2018 CNRS
//

#ifndef __pinocchio_vector_space_operation_hpp__
#define __pinocchio_vector_space_operation_hpp__

#include <stdexcept>

#include "pinocchio/multibody/liegroup/liegroup-base.hpp"

#include <boost/integer/static_min_max.hpp>

namespace pinocchio
{
  template<int Dim, typename Scalar, int Options = 0> struct VectorSpaceOperationTpl;
  
  template<int Dim, typename _Scalar, int _Options>
  struct traits< VectorSpaceOperationTpl<Dim,_Scalar,_Options> >
  {
    typedef _Scalar Scalar;
    enum {
      Options = _Options,
      NQ = Dim,
      NV = Dim
    };
  };

  template<int Dim, typename _Scalar, int _Options>
  struct VectorSpaceOperationTpl
  : public LieGroupBase< VectorSpaceOperationTpl<Dim,_Scalar,_Options> >
  {
    PINOCCHIO_LIE_GROUP_TPL_PUBLIC_INTERFACE(VectorSpaceOperationTpl);

    /// Constructor
    /// \param size size of the vector space: should be the equal to template
    ///        argument for static sized vector-spaces.
    VectorSpaceOperationTpl(int size = boost::static_signed_max<0,Dim>::value)
    : size_(size)
    {
      assert(size_.value() >= 0);
    }

    /// Constructor
    /// \param other other VectorSpaceOperationTpl from which to retrieve size
    VectorSpaceOperationTpl(const VectorSpaceOperationTpl & other)
    : Base(), size_(other.size_.value())
    {
      assert(size_.value() >= 0);
    }

    Index nq () const
    {
      return size_.value();
    }
    Index nv () const
    {
      return size_.value();
    }

    ConfigVector_t neutral () const
    {
      return ConfigVector_t::Zero(size_.value());
    }

    std::string name () const
    {
      std::ostringstream oss; oss << "R^" << nq();
      return oss.str ();
    }

    template <class ConfigL_t, class ConfigR_t, class Tangent_t>
    static void difference_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                const Eigen::MatrixBase<ConfigR_t> & q1,
                                const Eigen::MatrixBase<Tangent_t> & d)
    {
      PINOCCHIO_EIGEN_CONST_CAST(Tangent_t,d) = q1 - q0;
    }

    template <ArgumentPosition arg, class ConfigL_t, class ConfigR_t, class JacobianOut_t>
    void dDifference_impl (const Eigen::MatrixBase<ConfigL_t> &,
                           const Eigen::MatrixBase<ConfigR_t> &,
                           const Eigen::MatrixBase<JacobianOut_t>& J) const
    {
      if (arg == ARG0)
        PINOCCHIO_EIGEN_CONST_CAST(JacobianOut_t,J).noalias() = - JacobianMatrix_t::Identity();
      else if (arg == ARG1)
        PINOCCHIO_EIGEN_CONST_CAST(JacobianOut_t,J).setIdentity();
    }

    template <class ConfigIn_t, class Velocity_t, class ConfigOut_t>
    static void integrate_impl(const Eigen::MatrixBase<ConfigIn_t> & q,
                               const Eigen::MatrixBase<Velocity_t> & v,
                               const Eigen::MatrixBase<ConfigOut_t> & qout)
    {
      PINOCCHIO_EIGEN_CONST_CAST(ConfigOut_t,qout) = q + v;
    }

    template <class Config_t, class Jacobian_t>
    static void integrateCoeffWiseJacobian_impl(const Eigen::MatrixBase<Config_t> &,
                                                const Eigen::MatrixBase<Jacobian_t> & J)
    {
      PINOCCHIO_EIGEN_CONST_CAST(Jacobian_t,J).setIdentity();
    }

    template <class Config_t, class Tangent_t, class JacobianOut_t>
    static void dIntegrate_dq_impl(const Eigen::MatrixBase<Config_t >  & /*q*/,
                                   const Eigen::MatrixBase<Tangent_t>  & /*v*/,
                                   const Eigen::MatrixBase<JacobianOut_t>& J)
    {
      PINOCCHIO_EIGEN_CONST_CAST(JacobianOut_t,J).setIdentity();
    }

    template <class Config_t, class Tangent_t, class JacobianOut_t>
    static void dIntegrate_dv_impl(const Eigen::MatrixBase<Config_t >  & /*q*/,
                                   const Eigen::MatrixBase<Tangent_t>  & /*v*/,
                                   const Eigen::MatrixBase<JacobianOut_t>& J)
    {
      PINOCCHIO_EIGEN_CONST_CAST(JacobianOut_t,J).setIdentity();
    }


    // template <class ConfigL_t, class ConfigR_t>
    // static double squaredDistance_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                       // const Eigen::MatrixBase<ConfigR_t> & q1)

    template <class Config_t>
    static void normalize_impl (const Eigen::MatrixBase<Config_t>& /*qout*/)
    {}

    template <class Config_t>
    void random_impl (const Eigen::MatrixBase<Config_t>& qout) const
    {
      PINOCCHIO_EIGEN_CONST_CAST(Config_t,qout).setRandom();
    }

    template <class ConfigL_t, class ConfigR_t, class ConfigOut_t>
    void randomConfiguration_impl
    (const Eigen::MatrixBase<ConfigL_t> & lower_pos_limit,
     const Eigen::MatrixBase<ConfigR_t> & upper_pos_limit,
     const Eigen::MatrixBase<ConfigOut_t> & qout) const
    {
      ConfigOut_t & res = PINOCCHIO_EIGEN_CONST_CAST(ConfigOut_t,qout).derived();
      for (int i = 0; i < nq (); ++i)
      {
        if(lower_pos_limit[i] == -std::numeric_limits<typename ConfigL_t::Scalar>::infinity() ||
           upper_pos_limit[i] ==  std::numeric_limits<typename ConfigR_t::Scalar>::infinity() )
        {
          std::ostringstream error;
          error << "non bounded limit. Cannot uniformly sample joint at rank " << i;
          // assert(false && "non bounded limit. Cannot uniformly sample joint revolute");
          throw std::range_error(error.str());
        }
        res[i] = lower_pos_limit[i] + (( upper_pos_limit[i] - lower_pos_limit[i]) * rand())/RAND_MAX;
      }
    }
    
  private:
    
    Eigen::internal::variable_if_dynamic<Index, Dim> size_;
  }; // struct VectorSpaceOperationTpl

} // namespace pinocchio

#endif // ifndef __pinocchio_vector_space_operation_hpp__
