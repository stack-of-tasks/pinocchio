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

#ifndef __se3_cartesian_product_operation_hpp__
#define __se3_cartesian_product_operation_hpp__

#include <pinocchio/multibody/liegroup/operation-base.hpp>

namespace se3
{
  template<int dim1, int dim2>
  struct eval_set_dim
  {
    enum { value = dim1 + dim2 };
  };
  
  template<int dim>
  struct eval_set_dim<dim,Eigen::Dynamic>
  {
    enum { value = Eigen::Dynamic };
  };

  template<int dim>
  struct eval_set_dim<Eigen::Dynamic,dim>
  {
    enum { value = Eigen::Dynamic };
  };
  
  template<typename LieGroup1, typename LieGroup2>
  struct CartesianProductOperation;
  template<typename LieGroup1, typename LieGroup2>
  struct traits<CartesianProductOperation<LieGroup1, LieGroup2> > {
    typedef double Scalar;
    enum {
      NQ = eval_set_dim<LieGroup1::NQ,LieGroup2::NQ>::value,
      NV = eval_set_dim<LieGroup1::NV,LieGroup2::NV>::value
    };
  };

  template<typename LieGroup1, typename LieGroup2>
  struct CartesianProductOperation : public LieGroupOperationBase <CartesianProductOperation<LieGroup1, LieGroup2> >
  {
    SE3_LIE_GROUP_TPL_PUBLIC_INTERFACE(CartesianProductOperation);

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
    void difference_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                const Eigen::MatrixBase<ConfigR_t> & q1,
                                const Eigen::MatrixBase<Tangent_t> & d) const
    {
      Tangent_t& out = const_cast< Eigen::MatrixBase<Tangent_t>& > (d).derived();
      lg1_.difference(q0.head(lg1_.nq()), q1.head(lg1_.nq()), out.head(lg1_.nv()));
      lg2_.difference(q0.tail(lg2_.nq()), q1.tail(lg2_.nq()), out.tail(lg2_.nv()));
    }

    template <class ConfigL_t, class ConfigR_t, class JacobianLOut_t, class JacobianROut_t>
    void Jdifference_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                 const Eigen::MatrixBase<ConfigR_t> & q1,
                                 const Eigen::MatrixBase<JacobianLOut_t>& J0,
                                 const Eigen::MatrixBase<JacobianROut_t>& J1) const
    {
      JacobianLOut_t& J0out = const_cast< JacobianLOut_t& >(J0.derived());
      J0out.topRightCorner(lg1_.nv(),lg2_.nv()).setZero();
      J0out.bottomLeftCorner(lg2_.nv(),lg1_.nv()).setZero();

      JacobianROut_t& J1out = const_cast< JacobianROut_t& >(J1.derived());
      J1out.topRightCorner(lg1_.nv(),lg2_.nv()).setZero();
      J1out.bottomLeftCorner(lg2_.nv(),lg1_.nv()).setZero();

      lg1_.Jdifference(
          q0.head(lg1_.nq()),
          q1.head(lg1_.nq()),
          J0out.topLeftCorner(lg1_.nv(),lg1_.nv()),
          J1out.topLeftCorner(lg1_.nv(),lg1_.nv()));
      lg2_.Jdifference(
          q0.tail(lg2_.nq()),
          q1.tail(lg2_.nq()),
          J0out.bottomRightCorner(lg2_.nv(),lg2_.nv()),
          J1out.bottomRightCorner(lg2_.nv(),lg2_.nv()));
    }

    template <class ConfigIn_t, class Velocity_t, class ConfigOut_t>
    void integrate_impl(const Eigen::MatrixBase<ConfigIn_t> & q,
                               const Eigen::MatrixBase<Velocity_t> & v,
                               const Eigen::MatrixBase<ConfigOut_t> & qout) const
    {
      ConfigOut_t& out = const_cast< Eigen::MatrixBase<ConfigOut_t>& > (qout).derived();
      LieGroup1().integrate(q.head(lg1_.nq()), v.head(lg1_.nv()), out.head(lg1_.nq()));
      LieGroup2().integrate(q.tail(lg2_.nq()), v.tail(lg2_.nv()), out.tail(lg2_.nq()));
    }

    template <class Config_t, class Tangent_t, class JacobianOut_t>
    void dIntegrate_dq_impl(const Eigen::MatrixBase<Config_t > & q,
                            const Eigen::MatrixBase<Tangent_t> & v,
                            const Eigen::MatrixBase<JacobianOut_t> & J) const
    {
      JacobianOut_t& Jout = const_cast< JacobianOut_t& >(J.derived());
      Jout.  topRightCorner(lg1_.nv(),lg2_.nv()).setZero();
      Jout.bottomLeftCorner(lg2_.nv(),lg1_.nv()).setZero();
      lg1_.dIntegrate_dq(v.head(lg1_.nv()), Jout.    topLeftCorner(lg1_.nv(),lg1_.nv()));
      lg2_.dIntegrate_dq(v.tail(lg2_.nv()), Jout.bottomRightCorner(lg2_.nv(),lg2_.nv()));
    }

    template <class Config_t, class Tangent_t, class JacobianOut_t>
    void dIntegrate_dv_impl(const Eigen::MatrixBase<Config_t > & q,
                            const Eigen::MatrixBase<Tangent_t> & v,
                            const Eigen::MatrixBase<JacobianOut_t> & J) const
    {
      JacobianOut_t& Jout = const_cast< JacobianOut_t& >(J.derived());
      Jout.  topRightCorner(lg1_.nv(),lg2_.nv()).setZero();
      Jout.bottomLeftCorner(lg2_.nv(),lg1_.nv()).setZero();
      lg1_.dIntegrate_dv(v.head(lg1_.nv()), Jout.    topLeftCorner(lg1_.nv(),lg1_.nv()));
      lg2_.dIntegrate_dv(v.tail(lg2_.nv()), Jout.bottomRightCorner(lg2_.nv(),lg2_.nv()));
    }

    template <class ConfigL_t, class ConfigR_t>
    Scalar squaredDistance_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                const Eigen::MatrixBase<ConfigR_t> & q1) const
    {
      return lg1_.squaredDistance(q0.head(lg1_.nq()), q1.head(lg1_.nq()))
        +    lg2_.squaredDistance(q0.tail(lg2_.nq()), q1.tail(lg2_.nq()));
    }
    
    template <class Config_t>
    void normalize_impl (const Eigen::MatrixBase<Config_t>& qout) const
    {
      lg1_.normalize(qout.derived().head(lg1_.nq()));
      lg2_.normalize(qout.derived().tail(lg2_.nq()));
    }

    template <class Config_t>
    void random_impl (const Eigen::MatrixBase<Config_t>& qout) const
    {
      Config_t& out = const_cast< Eigen::MatrixBase<Config_t>& > (qout).derived();
      lg1_.random(out.head(lg1_.nq()));
      lg2_.random(out.tail(lg2_.nq()));
    }

    template <class ConfigL_t, class ConfigR_t, class ConfigOut_t>
    void randomConfiguration_impl(const Eigen::MatrixBase<ConfigL_t> & lower,
                                  const Eigen::MatrixBase<ConfigR_t> & upper,
                                  const Eigen::MatrixBase<ConfigOut_t> & qout)
      const
    {
      ConfigOut_t& out = const_cast< Eigen::MatrixBase<ConfigOut_t>& > (qout).derived();
      lg1_.randomConfiguration(lower.head(lg1_.nq()),
                                       upper.head(lg1_.nq()),
                                       out.head(lg1_.nq()));
      lg2_.randomConfiguration(lower.tail(lg2_.nq()),
                                       upper.tail(lg2_.nq()),
                                       out.tail(lg2_.nq()));
    }

    template <class ConfigL_t, class ConfigR_t>
    bool isSameConfiguration_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                  const Eigen::MatrixBase<ConfigR_t> & q1,
                                  const Scalar & prec) const
    {
      return LieGroup1().isSameConfiguration(q0.head(lg1_.nq()), q1.head(lg1_.nq()), prec)
        +    LieGroup2().isSameConfiguration(q0.tail(lg2_.nq()), q1.tail(lg2_.nq()), prec);
    }
  private:
    LieGroup1 lg1_;
    LieGroup2 lg2_;
  }; // struct CartesianProductOperation

} // namespace se3

#endif // ifndef __se3_cartesian_product_operation_hpp__
