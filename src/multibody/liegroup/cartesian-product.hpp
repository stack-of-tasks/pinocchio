//
// Copyright (c) 2016-2018 CNRS
//

#ifndef __pinocchio_cartesian_product_operation_hpp__
#define __pinocchio_cartesian_product_operation_hpp__

#include <pinocchio/multibody/liegroup/liegroup-base.hpp>

namespace pinocchio
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
    typedef typename traits<LieGroup1>::Scalar Scalar;
    enum {
      Options = traits<LieGroup1>::Options,
      NQ = eval_set_dim<LieGroup1::NQ,LieGroup2::NQ>::value,
      NV = eval_set_dim<LieGroup1::NV,LieGroup2::NV>::value
    };
  };

  template<typename LieGroup1, typename LieGroup2>
  struct CartesianProductOperation : public LieGroupBase <CartesianProductOperation<LieGroup1, LieGroup2> >
  {
    PINOCCHIO_LIE_GROUP_TPL_PUBLIC_INTERFACE(CartesianProductOperation);

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
      Qo1(n) = lg1_.neutral ();
      Qo2(n) = lg2_.neutral ();
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
      lg1_.difference(Q1(q0), Q1(q1), Vo1(d));
      lg2_.difference(Q2(q0), Q2(q1), Vo2(d));
    }

    template <ArgumentPosition arg, class ConfigL_t, class ConfigR_t, class JacobianOut_t>
    void dDifference (const Eigen::MatrixBase<ConfigL_t> & q0,
                      const Eigen::MatrixBase<ConfigR_t> & q1,
                      const Eigen::MatrixBase<JacobianOut_t>& J) const
    {
      J12(J).setZero();
      J21(J).setZero();

      lg1_.template dDifference<arg> (Q1(q0), Q1(q1), J11(J));
      lg2_.template dDifference<arg> (Q2(q0), Q2(q1), J22(J));
    }

    template <class ConfigIn_t, class Velocity_t, class ConfigOut_t>
    void integrate_impl(const Eigen::MatrixBase<ConfigIn_t> & q,
                               const Eigen::MatrixBase<Velocity_t> & v,
                               const Eigen::MatrixBase<ConfigOut_t> & qout) const
    {
      lg1_.integrate(Q1(q), V1(v), Qo1(qout));
      lg2_.integrate(Q2(q), V2(v), Qo2(qout));
    }
    
    template <class Config_t, class Jacobian_t>
    void integrateCoeffWiseJacobian_impl(const Eigen::MatrixBase<Config_t> & q,
                                         const Eigen::MatrixBase<Jacobian_t> & J) const
    {
      assert(J.rows() == nq() && J.cols() == nv() && "J is not of the right dimension");
      Jacobian_t & J_ = PINOCCHIO_EIGEN_CONST_CAST(Jacobian_t,J);
      J_.topRightCorner(lg1_.nq(),lg2_.nv()).setZero();
      J_.bottomLeftCorner(lg2_.nq(),lg1_.nv()).setZero();
      
      lg1_.integrateCoeffWiseJacobian(Q1(q),
                                      J_.topLeftCorner(lg1_.nq(),lg1_.nv()));
      lg2_.integrateCoeffWiseJacobian(Q2(q), J_.bottomRightCorner(lg2_.nq(),lg2_.nv()));
    }

    template <class Config_t, class Tangent_t, class JacobianOut_t>
    void dIntegrate_dq_impl(const Eigen::MatrixBase<Config_t > & q,
                            const Eigen::MatrixBase<Tangent_t> & v,
                            const Eigen::MatrixBase<JacobianOut_t> & J) const
    {
      J12(J).setZero();
      J21(J).setZero();
      lg1_.dIntegrate_dq(Q1(q), V1(v), J11(J));
      lg2_.dIntegrate_dq(Q2(q), V2(v), J22(J));
    }

    template <class Config_t, class Tangent_t, class JacobianOut_t>
    void dIntegrate_dv_impl(const Eigen::MatrixBase<Config_t > & q,
                            const Eigen::MatrixBase<Tangent_t> & v,
                            const Eigen::MatrixBase<JacobianOut_t> & J) const
    {
      J12(J).setZero();
      J21(J).setZero();
      lg1_.dIntegrate_dv(Q1(q), V1(v), J11(J));
      lg2_.dIntegrate_dv(Q2(q), V2(v), J22(J));
    }

    template <class ConfigL_t, class ConfigR_t>
    Scalar squaredDistance_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                const Eigen::MatrixBase<ConfigR_t> & q1) const
    {
      return lg1_.squaredDistance(Q1(q0), Q1(q1))
        +    lg2_.squaredDistance(Q2(q0), Q2(q1));
    }
    
    template <class Config_t>
    void normalize_impl (const Eigen::MatrixBase<Config_t>& qout) const
    {
      lg1_.normalize(Qo1(qout));
      lg2_.normalize(Qo2(qout));
    }

    template <class Config_t>
    void random_impl (const Eigen::MatrixBase<Config_t>& qout) const
    {
      lg1_.random(Qo1(qout));
      lg2_.random(Qo2(qout));
    }

    template <class ConfigL_t, class ConfigR_t, class ConfigOut_t>
    void randomConfiguration_impl(const Eigen::MatrixBase<ConfigL_t> & lower,
                                  const Eigen::MatrixBase<ConfigR_t> & upper,
                                  const Eigen::MatrixBase<ConfigOut_t> & qout)
      const
    {
      lg1_.randomConfiguration(Q1(lower), Q1(upper), Qo1(qout));
      lg2_.randomConfiguration(Q2(lower), Q2(upper), Qo2(qout));
    }

    template <class ConfigL_t, class ConfigR_t>
    bool isSameConfiguration_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                  const Eigen::MatrixBase<ConfigR_t> & q1,
                                  const Scalar & prec) const
    {
      return lg1_.isSameConfiguration(Q1(q0), Q1(q1), prec)
        &&   lg2_.isSameConfiguration(Q2(q0), Q2(q1), prec);
    }
  private:
    LieGroup1 lg1_;
    LieGroup2 lg2_;

    // VectorSpaceOperationTpl<-1> within CartesianProductOperation will not work
    // if Eigen version is lower than 3.2.1
#if EIGEN_VERSION_AT_LEAST(3,2,1)
# define REMOVE_IF_EIGEN_TOO_LOW(x) x
#else
# define REMOVE_IF_EIGEN_TOO_LOW(x)
#endif

    template <typename Config > typename Config ::template ConstFixedSegmentReturnType<LieGroup1::NQ>::Type Q1 (const Eigen::MatrixBase<Config >& q) const { return q.derived().template head<LieGroup1::NQ>(REMOVE_IF_EIGEN_TOO_LOW(lg1_.nq())); }
    template <typename Config > typename Config ::template ConstFixedSegmentReturnType<LieGroup2::NQ>::Type Q2 (const Eigen::MatrixBase<Config >& q) const { return q.derived().template tail<LieGroup2::NQ>(REMOVE_IF_EIGEN_TOO_LOW(lg2_.nq())); }
    template <typename Tangent> typename Tangent::template ConstFixedSegmentReturnType<LieGroup1::NV>::Type V1 (const Eigen::MatrixBase<Tangent>& v) const { return v.derived().template head<LieGroup1::NV>(REMOVE_IF_EIGEN_TOO_LOW(lg1_.nv())); }
    template <typename Tangent> typename Tangent::template ConstFixedSegmentReturnType<LieGroup2::NV>::Type V2 (const Eigen::MatrixBase<Tangent>& v) const { return v.derived().template tail<LieGroup2::NV>(REMOVE_IF_EIGEN_TOO_LOW(lg2_.nv())); }

    template <typename Config > typename Config ::template      FixedSegmentReturnType<LieGroup1::NQ>::Type Qo1 (const Eigen::MatrixBase<Config >& q) const { return const_cast<Config &>(q.derived()).template head<LieGroup1::NQ>(REMOVE_IF_EIGEN_TOO_LOW(lg1_.nq())); }
    template <typename Config > typename Config ::template      FixedSegmentReturnType<LieGroup2::NQ>::Type Qo2 (const Eigen::MatrixBase<Config >& q) const { return const_cast<Config &>(q.derived()).template tail<LieGroup2::NQ>(REMOVE_IF_EIGEN_TOO_LOW(lg2_.nq())); }
    template <typename Tangent> typename Tangent::template      FixedSegmentReturnType<LieGroup1::NV>::Type Vo1 (const Eigen::MatrixBase<Tangent>& v) const { return const_cast<Tangent&>(v.derived()).template head<LieGroup1::NV>(REMOVE_IF_EIGEN_TOO_LOW(lg1_.nv())); }
    template <typename Tangent> typename Tangent::template      FixedSegmentReturnType<LieGroup2::NV>::Type Vo2 (const Eigen::MatrixBase<Tangent>& v) const { return const_cast<Tangent&>(v.derived()).template tail<LieGroup2::NV>(REMOVE_IF_EIGEN_TOO_LOW(lg2_.nv())); }

    template <typename Jac> Eigen::Block<Jac, LieGroup1::NV, LieGroup1::NV> J11 (const Eigen::MatrixBase<Jac>& J) const { return const_cast<Jac&>(J.derived()).template     topLeftCorner<LieGroup1::NV, LieGroup1::NV>(lg1_.nv(),lg1_.nv()); }
    template <typename Jac> Eigen::Block<Jac, LieGroup1::NV, LieGroup2::NV> J12 (const Eigen::MatrixBase<Jac>& J) const { return const_cast<Jac&>(J.derived()).template    topRightCorner<LieGroup1::NV, LieGroup2::NV>(lg1_.nv(),lg2_.nv()); }
    template <typename Jac> Eigen::Block<Jac, LieGroup2::NV, LieGroup1::NV> J21 (const Eigen::MatrixBase<Jac>& J) const { return const_cast<Jac&>(J.derived()).template  bottomLeftCorner<LieGroup2::NV, LieGroup1::NV>(lg2_.nv(),lg1_.nv()); }
    template <typename Jac> Eigen::Block<Jac, LieGroup2::NV, LieGroup2::NV> J22 (const Eigen::MatrixBase<Jac>& J) const { return const_cast<Jac&>(J.derived()).template bottomRightCorner<LieGroup2::NV, LieGroup2::NV>(lg2_.nv(),lg2_.nv()); }
#undef REMOVE_IF_EIGEN_TOO_LOW

  }; // struct CartesianProductOperation

} // namespace pinocchio

#endif // ifndef __pinocchio_cartesian_product_operation_hpp__
