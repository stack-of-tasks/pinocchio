//
// Copyright (c) 2016-2020 CNRS CNRS
//

#ifndef __pinocchio_multibody_liegroup_cartesian_product_operation_hpp__
#define __pinocchio_multibody_liegroup_cartesian_product_operation_hpp__

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

    CartesianProductOperation () : lg1 (), lg2 ()
    {
    }
    // Get dimension of Lie Group vector representation
    //
    // For instance, for SO(3), the dimension of the vector representation is
    // 4 (quaternion) while the dimension of the tangent space is 3.
    Index nq () const
    {
      return lg1.nq () + lg2.nq ();
    }
    
    // Get dimension of Lie Group tangent space
    Index nv () const
    {
      return lg1.nv () + lg2.nv ();
    }

    ConfigVector_t neutral () const
    {
      ConfigVector_t n;
      n.resize (nq ());
      Qo1(n) = lg1.neutral ();
      Qo2(n) = lg2.neutral ();
      return n;
    }

    std::string name () const
    {
      std::ostringstream oss; oss << lg1.name () << "*" << lg2.name ();
      return oss.str ();
    }

    template <class ConfigL_t, class ConfigR_t, class Tangent_t>
    void difference_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                         const Eigen::MatrixBase<ConfigR_t> & q1,
                         const Eigen::MatrixBase<Tangent_t> & d) const
    {
      lg1.difference(Q1(q0), Q1(q1), Vo1(d));
      lg2.difference(Q2(q0), Q2(q1), Vo2(d));
    }

    template <ArgumentPosition arg, class ConfigL_t, class ConfigR_t, class JacobianOut_t>
    void dDifference_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                          const Eigen::MatrixBase<ConfigR_t> & q1,
                          const Eigen::MatrixBase<JacobianOut_t> & J) const
    {
      J12(J).setZero();
      J21(J).setZero();

      lg1.template dDifference<arg> (Q1(q0), Q1(q1), J11(J));
      lg2.template dDifference<arg> (Q2(q0), Q2(q1), J22(J));
    }

    template <class ConfigIn_t, class Velocity_t, class ConfigOut_t>
    void integrate_impl(const Eigen::MatrixBase<ConfigIn_t> & q,
                        const Eigen::MatrixBase<Velocity_t> & v,
                        const Eigen::MatrixBase<ConfigOut_t> & qout) const
    {
      lg1.integrate(Q1(q), V1(v), Qo1(qout));
      lg2.integrate(Q2(q), V2(v), Qo2(qout));
    }
    
    template <class Config_t, class Jacobian_t>
    void integrateCoeffWiseJacobian_impl(const Eigen::MatrixBase<Config_t> & q,
                                         const Eigen::MatrixBase<Jacobian_t> & J) const
    {
      assert(J.rows() == nq() && J.cols() == nv() && "J is not of the right dimension");
      Jacobian_t & J_ = PINOCCHIO_EIGEN_CONST_CAST(Jacobian_t,J);
      J_.topRightCorner(lg1.nq(),lg2.nv()).setZero();
      J_.bottomLeftCorner(lg2.nq(),lg1.nv()).setZero();
      
      lg1.integrateCoeffWiseJacobian(Q1(q),
                                      J_.topLeftCorner(lg1.nq(),lg1.nv()));
      lg2.integrateCoeffWiseJacobian(Q2(q), J_.bottomRightCorner(lg2.nq(),lg2.nv()));
    }

    template <class Config_t, class Tangent_t, class JacobianOut_t>
    void dIntegrate_dq_impl(const Eigen::MatrixBase<Config_t > & q,
                            const Eigen::MatrixBase<Tangent_t> & v,
                            const Eigen::MatrixBase<JacobianOut_t> & J,
                            const AssignmentOperatorType op=SETTO) const
    {
      switch(op)
        {
        case SETTO:
          J12(J).setZero();
          J21(J).setZero();
          // fallthrough
        case ADDTO:
        case RMTO:
          lg1.dIntegrate_dq(Q1(q), V1(v), J11(J),op);
          lg2.dIntegrate_dq(Q2(q), V2(v), J22(J),op);
          break;
        default:
          assert(false && "Wrong Op requesed value");
          break;
        }      
    }

    template <class Config_t, class Tangent_t, class JacobianOut_t>
    void dIntegrate_dv_impl(const Eigen::MatrixBase<Config_t > & q,
                            const Eigen::MatrixBase<Tangent_t> & v,
                            const Eigen::MatrixBase<JacobianOut_t> & J,
                            const AssignmentOperatorType op=SETTO) const
    {
      switch(op)
        {
        case SETTO:
          J12(J).setZero();
          J21(J).setZero();
          // fallthrough
        case ADDTO:
        case RMTO:
          lg1.dIntegrate_dv(Q1(q), V1(v), J11(J),op);
          lg2.dIntegrate_dv(Q2(q), V2(v), J22(J),op);
          break;
        default:
          assert(false && "Wrong Op requesed value");
          break;
        }
    }

    template <class Config_t, class Tangent_t, class JacobianIn_t, class JacobianOut_t>
    void dIntegrateTransport_dq_impl(const Eigen::MatrixBase<Config_t > & q,
                                     const Eigen::MatrixBase<Tangent_t> & v,
                                     const Eigen::MatrixBase<JacobianIn_t> & J_in,
                                     const Eigen::MatrixBase<JacobianOut_t> & J_out) const
    {
      JacobianOut_t& Jout = PINOCCHIO_EIGEN_CONST_CAST(JacobianOut_t,J_out);
      JacobianOut_t& Jin = PINOCCHIO_EIGEN_CONST_CAST(JacobianIn_t,J_in);
      lg1.dIntegrateTransport_dq(Q1(q), V1(v), Jin.template topRows<LieGroup1::NV>(),Jout.template topRows<LieGroup1::NV>());
      lg2.dIntegrateTransport_dq(Q2(q), V2(v), Jin.template bottomRows<LieGroup2::NV>(),Jout.template bottomRows<LieGroup2::NV>());
    }

    template <class Config_t, class Tangent_t, class JacobianIn_t, class JacobianOut_t>
    void dIntegrateTransport_dv_impl(const Eigen::MatrixBase<Config_t > & q,
                                     const Eigen::MatrixBase<Tangent_t> & v,
                                     const Eigen::MatrixBase<JacobianIn_t> & J_in,
                                     const Eigen::MatrixBase<JacobianOut_t> & J_out) const
    {
      JacobianOut_t& Jout = PINOCCHIO_EIGEN_CONST_CAST(JacobianOut_t,J_out);
      JacobianOut_t& Jin = PINOCCHIO_EIGEN_CONST_CAST(JacobianIn_t,J_in);
      lg1.dIntegrateTransport_dv(Q1(q), V1(v), Jin.template topRows<LieGroup1::NV>(),Jout.template topRows<LieGroup1::NV>());
      lg2.dIntegrateTransport_dv(Q2(q), V2(v), Jin.template bottomRows<LieGroup2::NV>(),Jout.template bottomRows<LieGroup2::NV>());
    }
    

    template <class Config_t, class Tangent_t, class Jacobian_t>
    void dIntegrateTransport_dq_impl(const Eigen::MatrixBase<Config_t > & q,
                                     const Eigen::MatrixBase<Tangent_t> & v,
                                     const Eigen::MatrixBase<Jacobian_t> & Jin) const
    {
      Jacobian_t& J = PINOCCHIO_EIGEN_CONST_CAST(Jacobian_t,Jin);
      lg1.dIntegrateTransport_dq(Q1(q), V1(v), J.template topRows<LieGroup1::NV>());
      lg2.dIntegrateTransport_dq(Q2(q), V2(v), J.template bottomRows<LieGroup2::NV>());
    }

    template <class Config_t, class Tangent_t, class Jacobian_t>
    void dIntegrateTransport_dv_impl(const Eigen::MatrixBase<Config_t > & q,
                                     const Eigen::MatrixBase<Tangent_t> & v,
                                     const Eigen::MatrixBase<Jacobian_t> & Jin) const
    {
      Jacobian_t& J = PINOCCHIO_EIGEN_CONST_CAST(Jacobian_t,Jin);
      lg1.dIntegrateTransport_dv(Q1(q), V1(v), J.template topRows<LieGroup1::NV>());
      lg2.dIntegrateTransport_dv(Q2(q), V2(v), J.template bottomRows<LieGroup2::NV>());
    }

    template <class ConfigL_t, class ConfigR_t>
    Scalar squaredDistance_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                const Eigen::MatrixBase<ConfigR_t> & q1) const
    {
      return lg1.squaredDistance(Q1(q0), Q1(q1))
        +    lg2.squaredDistance(Q2(q0), Q2(q1));
    }
    
    template <class Config_t>
    void normalize_impl (const Eigen::MatrixBase<Config_t>& qout) const
    {
      lg1.normalize(Qo1(qout));
      lg2.normalize(Qo2(qout));
    }

    template <class Config_t>
    void random_impl (const Eigen::MatrixBase<Config_t>& qout) const
    {
      lg1.random(Qo1(qout));
      lg2.random(Qo2(qout));
    }

    template <class ConfigL_t, class ConfigR_t, class ConfigOut_t>
    void randomConfiguration_impl(const Eigen::MatrixBase<ConfigL_t> & lower,
                                  const Eigen::MatrixBase<ConfigR_t> & upper,
                                  const Eigen::MatrixBase<ConfigOut_t> & qout)
      const
    {
      lg1.randomConfiguration(Q1(lower), Q1(upper), Qo1(qout));
      lg2.randomConfiguration(Q2(lower), Q2(upper), Qo2(qout));
    }

    template <class ConfigL_t, class ConfigR_t>
    bool isSameConfiguration_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                  const Eigen::MatrixBase<ConfigR_t> & q1,
                                  const Scalar & prec) const
    {
      return lg1.isSameConfiguration(Q1(q0), Q1(q1), prec)
        &&   lg2.isSameConfiguration(Q2(q0), Q2(q1), prec);
    }

    bool isEqual_impl (const CartesianProductOperation& other) const
    {
      return lg1 == other.lg1 && lg2 == other.lg2;
    }

    LieGroup1 lg1;
    LieGroup2 lg2;

  private:
    // VectorSpaceOperationTpl<-1> within CartesianProductOperation will not work
    // if Eigen version is lower than 3.2.1
#if EIGEN_VERSION_AT_LEAST(3,2,1)
# define REMOVE_IF_EIGEN_TOO_LOW(x) x
#else
# define REMOVE_IF_EIGEN_TOO_LOW(x)
#endif

    template <typename Config > typename Config ::template ConstFixedSegmentReturnType<LieGroup1::NQ>::Type Q1 (const Eigen::MatrixBase<Config >& q) const { return q.derived().template head<LieGroup1::NQ>(REMOVE_IF_EIGEN_TOO_LOW(lg1.nq())); }
    template <typename Config > typename Config ::template ConstFixedSegmentReturnType<LieGroup2::NQ>::Type Q2 (const Eigen::MatrixBase<Config >& q) const { return q.derived().template tail<LieGroup2::NQ>(REMOVE_IF_EIGEN_TOO_LOW(lg2.nq())); }
    template <typename Tangent> typename Tangent::template ConstFixedSegmentReturnType<LieGroup1::NV>::Type V1 (const Eigen::MatrixBase<Tangent>& v) const { return v.derived().template head<LieGroup1::NV>(REMOVE_IF_EIGEN_TOO_LOW(lg1.nv())); }
    template <typename Tangent> typename Tangent::template ConstFixedSegmentReturnType<LieGroup2::NV>::Type V2 (const Eigen::MatrixBase<Tangent>& v) const { return v.derived().template tail<LieGroup2::NV>(REMOVE_IF_EIGEN_TOO_LOW(lg2.nv())); }

    template <typename Config > typename Config ::template      FixedSegmentReturnType<LieGroup1::NQ>::Type Qo1 (const Eigen::MatrixBase<Config >& q) const { return PINOCCHIO_EIGEN_CONST_CAST(Config,q).template head<LieGroup1::NQ>(REMOVE_IF_EIGEN_TOO_LOW(lg1.nq())); }
    template <typename Config > typename Config ::template      FixedSegmentReturnType<LieGroup2::NQ>::Type Qo2 (const Eigen::MatrixBase<Config >& q) const { return PINOCCHIO_EIGEN_CONST_CAST(Config,q).template tail<LieGroup2::NQ>(REMOVE_IF_EIGEN_TOO_LOW(lg2.nq())); }
    template <typename Tangent> typename Tangent::template      FixedSegmentReturnType<LieGroup1::NV>::Type Vo1 (const Eigen::MatrixBase<Tangent>& v) const { return PINOCCHIO_EIGEN_CONST_CAST(Tangent,v).template head<LieGroup1::NV>(REMOVE_IF_EIGEN_TOO_LOW(lg1.nv())); }
    template <typename Tangent> typename Tangent::template      FixedSegmentReturnType<LieGroup2::NV>::Type Vo2 (const Eigen::MatrixBase<Tangent>& v) const { return PINOCCHIO_EIGEN_CONST_CAST(Tangent,v).template tail<LieGroup2::NV>(REMOVE_IF_EIGEN_TOO_LOW(lg2.nv())); }

    template <typename Jac> Eigen::Block<Jac, LieGroup1::NV, LieGroup1::NV> J11 (const Eigen::MatrixBase<Jac>& J) const { return PINOCCHIO_EIGEN_CONST_CAST(Jac,J).template     topLeftCorner<LieGroup1::NV, LieGroup1::NV>(lg1.nv(),lg1.nv()); }
    template <typename Jac> Eigen::Block<Jac, LieGroup1::NV, LieGroup2::NV> J12 (const Eigen::MatrixBase<Jac>& J) const { return PINOCCHIO_EIGEN_CONST_CAST(Jac,J).template    topRightCorner<LieGroup1::NV, LieGroup2::NV>(lg1.nv(),lg2.nv()); }
    template <typename Jac> Eigen::Block<Jac, LieGroup2::NV, LieGroup1::NV> J21 (const Eigen::MatrixBase<Jac>& J) const { return PINOCCHIO_EIGEN_CONST_CAST(Jac,J).template  bottomLeftCorner<LieGroup2::NV, LieGroup1::NV>(lg2.nv(),lg1.nv()); }
    template <typename Jac> Eigen::Block<Jac, LieGroup2::NV, LieGroup2::NV> J22 (const Eigen::MatrixBase<Jac>& J) const { return PINOCCHIO_EIGEN_CONST_CAST(Jac,J).template bottomRightCorner<LieGroup2::NV, LieGroup2::NV>(lg2.nv(),lg2.nv()); }
#undef REMOVE_IF_EIGEN_TOO_LOW

  }; // struct CartesianProductOperation

} // namespace pinocchio

#endif // ifndef __pinocchio_multibody_liegroup_cartesian_product_operation_hpp__
