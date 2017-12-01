//
// Copyright (c) 2016-2017 CNRS
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

#ifndef __se3_cartesian_product_operation_hpp__
#define __se3_cartesian_product_operation_hpp__

#include <pinocchio/multibody/liegroup/operation-base.hpp>

namespace se3
{
  template<typename LieGroup1, typename LieGroup2>
  struct CartesianProductOperation;
  template<typename LieGroup1, typename LieGroup2>
  struct traits<CartesianProductOperation<LieGroup1, LieGroup2> > {
    typedef double Scalar;
    enum {
      NQ = LieGroup1::NQ + LieGroup2::NQ,
      NV = LieGroup1::NV + LieGroup2::NV
    };
    typedef Eigen::Matrix<Scalar,NQ,1> ConfigVector_t;
    typedef Eigen::Matrix<Scalar,NV,1> TangentVector_t;
  };

  template<typename LieGroup1, typename LieGroup2>
  struct CartesianProductOperation : public LieGroupOperationBase <CartesianProductOperation<LieGroup1, LieGroup2> >
  {
    typedef CartesianProductOperation<LieGroup1, LieGroup2>  LieGroupDerived;
    SE3_LIE_GROUP_TYPEDEF_TEMPLATE;

    CartesianProductOperation () : lg1_ (), lg2_ ()
    {
    }
    /// Get dimension of Lie Group vector representation
    ///
    /// For instance, for SO(3), the dimension of the vector representation is
    /// 4 (quaternion) while the dimension of the tangent space is 3.
    Index nq () const
    {
      return lg1_.nq () + lg2_.nq ();
    }
    /// Get dimension of Lie Group tangent space
    Index nv () const
    {
      return lg1_.nv () + lg2_.nv ();
    }

    ConfigVector_t neutral () const
    {
      ConfigVector_t n;
      n.resize (nq ());
      n.head (lg1_.nq ()) = lg1_.neutral ();
      n.tail (lg2_.nq ()) = lg2_.neutral ();
      return n;
    }

    std::string name () const
    {
      std::ostringstream oss; oss << lg1_.name () << "*" << lg2_.name ();
      return oss.str ();
    }

    template <class ConfigL_t, class ConfigR_t, class Tangent_t>
    static void difference_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                const Eigen::MatrixBase<ConfigR_t> & q1,
                                const Eigen::MatrixBase<Tangent_t> & d)
    {
      Tangent_t& out = const_cast< Eigen::MatrixBase<Tangent_t>& > (d).derived();
      LieGroup1::difference(q0.template head<LieGroup1::NQ>(), q1.template head<LieGroup1::NQ>(), out.template head<LieGroup1::NV>());
      LieGroup2::difference(q0.template tail<LieGroup2::NQ>(), q1.template tail<LieGroup2::NQ>(), out.template tail<LieGroup2::NV>());
    }

    template <class ConfigIn_t, class Velocity_t, class ConfigOut_t>
    static void integrate_impl(const Eigen::MatrixBase<ConfigIn_t> & q,
                               const Eigen::MatrixBase<Velocity_t> & v,
                               const Eigen::MatrixBase<ConfigOut_t> & qout)
    {
      ConfigOut_t& out = const_cast< Eigen::MatrixBase<ConfigOut_t>& > (qout).derived();
      LieGroup1::integrate(q.template head<LieGroup1::NQ>(), v.template head<LieGroup1::NV>(), out.template head<LieGroup1::NQ>());
      LieGroup2::integrate(q.template tail<LieGroup2::NQ>(), v.template tail<LieGroup2::NV>(), out.template tail<LieGroup2::NQ>());
    }

    template <class ConfigL_t, class ConfigR_t>
    static double squaredDistance_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                       const Eigen::MatrixBase<ConfigR_t> & q1)
    {
      return LieGroup1::squaredDistance(q0.template head<LieGroup1::NQ>(), q1.template head<LieGroup1::NQ>())
        +    LieGroup2::squaredDistance(q0.template tail<LieGroup2::NQ>(), q1.template tail<LieGroup2::NQ>());
    }
    
    template <class Config_t>
    static void normalize_impl (const Eigen::MatrixBase<Config_t>& qout)
    {
      LieGroup1::normalize(qout.derived().template head<LieGroup1::NQ>());
      LieGroup2::normalize(qout.derived().template tail<LieGroup2::NQ>());
    }

    template <class Config_t>
    void random_impl (const Eigen::MatrixBase<Config_t>& qout) const
    {
      Config_t& out = const_cast< Eigen::MatrixBase<Config_t>& > (qout).derived();
      LieGroup1 ().random(out.template head<LieGroup1::NQ>());
      LieGroup2 ().random(out.template tail<LieGroup2::NQ>());
    }

    template <class ConfigL_t, class ConfigR_t, class ConfigOut_t>
    void randomConfiguration_impl(const Eigen::MatrixBase<ConfigL_t> & lower,
                                  const Eigen::MatrixBase<ConfigR_t> & upper,
                                  const Eigen::MatrixBase<ConfigOut_t> & qout)
      const
    {
      ConfigOut_t& out = const_cast< Eigen::MatrixBase<ConfigOut_t>& > (qout).derived();
      LieGroup1 ().randomConfiguration(lower.template head<LieGroup1::NQ>(),
                                       upper.template head<LieGroup1::NQ>(),
                                       out.template head<LieGroup1::NQ>());
      LieGroup2 ().randomConfiguration(lower.template tail<LieGroup2::NQ>(),
                                       upper.template tail<LieGroup2::NQ>(),
                                       out.template tail<LieGroup2::NQ>());
    }

    template <class ConfigL_t, class ConfigR_t>
    static bool isSameConfiguration_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                         const Eigen::MatrixBase<ConfigR_t> & q1,
                                         const Scalar & prec)
    {
      return LieGroup1::isSameConfiguration(q0.template head<LieGroup1::NQ>(), q1.template head<LieGroup1::NQ>(), prec)
        +    LieGroup2::isSameConfiguration(q0.template tail<LieGroup2::NQ>(), q1.template tail<LieGroup2::NQ>(), prec);
    }
  private:
    LieGroup1 lg1_;
    LieGroup2 lg2_;
  }; // struct CartesianProductOperation

} // namespace se3

#endif // ifndef __se3_cartesian_product_operation_hpp__
