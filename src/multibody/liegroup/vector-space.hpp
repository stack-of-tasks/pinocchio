//
// Copyright (c) 2016-2018 CNRS
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.

#ifndef __se3_vector_space_operation_hpp__
#define __se3_vector_space_operation_hpp__

#include <stdexcept>

#include "pinocchio/multibody/liegroup/operation-base.hpp"

#include <boost/integer/static_min_max.hpp>

namespace se3
{
  template<int Size> struct VectorSpaceOperation;
  template<int Size> struct traits<VectorSpaceOperation<Size> > {
    typedef double Scalar;
    enum {
      NQ = Size,
      NV = Size
    };
  };

  template<int Size = Eigen::Dynamic>
  struct VectorSpaceOperation : public LieGroupBase <VectorSpaceOperation<Size> >
  {
    SE3_LIE_GROUP_TPL_PUBLIC_INTERFACE(VectorSpaceOperation);

    /// Constructor
    /// \param size size of the vector space: should be the equal to template
    ///        argument for static sized vector-spaces.
    VectorSpaceOperation (int size = boost::static_signed_max<0,Size>::value) : size_ (size)
    {
      assert (size_.value() >= 0);
    }

    /// Constructor
    /// \param other other VectorSpaceOperation from which to retrieve size
    VectorSpaceOperation (const VectorSpaceOperation& other) : Base (), size_ (other.size_.value())
    {
      assert (size_.value() >= 0);
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
      return ConfigVector_t::Zero(size_.value());;
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
      const_cast< Eigen::MatrixBase<Tangent_t>& > (d) = q1 - q0;
    }

    template <class ConfigL_t, class ConfigR_t, class JacobianLOut_t, class JacobianROut_t>
    static void Jdifference_impl(const Eigen::MatrixBase<ConfigL_t> &,
                                 const Eigen::MatrixBase<ConfigR_t> &,
                                 const Eigen::MatrixBase<JacobianLOut_t>& J0,
                                 const Eigen::MatrixBase<JacobianROut_t>& J1)
    {
      const_cast< JacobianLOut_t& > (J0.derived()).setZero();
      const_cast< JacobianLOut_t& > (J0.derived()).diagonal().setConstant(-1);
      const_cast< JacobianROut_t& > (J1.derived()).setIdentity();
    }

    template <class ConfigIn_t, class Velocity_t, class ConfigOut_t>
    static void integrate_impl(const Eigen::MatrixBase<ConfigIn_t> & q,
                               const Eigen::MatrixBase<Velocity_t> & v,
                               const Eigen::MatrixBase<ConfigOut_t> & qout)
    {
      const_cast< Eigen::MatrixBase<ConfigOut_t>& > (qout) = q + v;
    }

    template <class Tangent_t, class JacobianOut_t>
    static void Jintegrate_impl(const Eigen::MatrixBase<Tangent_t> &,
                                const Eigen::MatrixBase<JacobianOut_t> & J)
    {
      const_cast< JacobianOut_t& > (J.derived()).setIdentity();
    }

    template <class Config_t, class Tangent_t, class JacobianOut_t>
    static void dIntegrate_dq_impl(const Eigen::MatrixBase<Config_t >  & /*q*/,
                                   const Eigen::MatrixBase<Tangent_t>  & /*v*/,
                                   const Eigen::MatrixBase<JacobianOut_t>& J)
    {
      const_cast< JacobianOut_t& > (J.derived()).setIdentity();
    }

    template <class Config_t, class Tangent_t, class JacobianOut_t>
    static void dIntegrate_dv_impl(const Eigen::MatrixBase<Config_t >  & /*q*/,
                                   const Eigen::MatrixBase<Tangent_t>  & /*v*/,
                                   const Eigen::MatrixBase<JacobianOut_t>& J)
    {
      const_cast< JacobianOut_t& > (J.derived()).setIdentity();
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
      const_cast< Eigen::MatrixBase<Config_t>& > (qout).setRandom();
    }

    template <class ConfigL_t, class ConfigR_t, class ConfigOut_t>
    void randomConfiguration_impl
    (const Eigen::MatrixBase<ConfigL_t> & lower_pos_limit,
     const Eigen::MatrixBase<ConfigR_t> & upper_pos_limit,
     const Eigen::MatrixBase<ConfigOut_t> & qout) const
    {
      ConfigOut_t& res = const_cast< Eigen::MatrixBase<ConfigOut_t>& > (qout).derived();
      for (int i = 0; i < nq (); ++i)
      {
        if(lower_pos_limit[i] == -std::numeric_limits<Scalar>::infinity() ||
           upper_pos_limit[i] ==  std::numeric_limits<Scalar>::infinity() )
        {
          std::ostringstream error;
          error << "non bounded limit. Cannot uniformly sample joint at rank " << i;
          // assert(false && "non bounded limit. Cannot uniformly sample joint revolute");
          throw std::runtime_error(error.str());
        }
        res[i] = lower_pos_limit[i] + (( upper_pos_limit[i] - lower_pos_limit[i]) * rand())/RAND_MAX;
      }
    }
  private:
    Eigen::internal::variable_if_dynamic<Index, Size> size_;
  }; // struct VectorSpaceOperation

} // namespace se3

#endif // ifndef __se3_vector_space_operation_hpp__
