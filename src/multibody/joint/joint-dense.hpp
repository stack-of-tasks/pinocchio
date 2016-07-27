//
// Copyright (c) 2015 CNRS
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

#ifndef __se3_joint_dense_hpp__
#define __se3_joint_dense_hpp__

#include "pinocchio/multibody/joint/joint-base.hpp"

namespace se3
{

  template<int _NQ, int _NV>
  struct traits< JointDense<_NQ, _NV > >
  {
    enum {
      NQ = _NQ, // pb
      NV = _NV
    };
    typedef JointDataDense<_NQ, _NV> JointDataDerived;
    typedef JointModelDense<_NQ, _NV> JointModelDerived;
    typedef ConstraintXd Constraint_t;
    typedef SE3 Transformation_t;
    typedef Motion Motion_t;
    typedef BiasZero Bias_t;
    typedef Eigen::Matrix<double,6,NV> F_t;
    
    // [ABA]
    typedef Eigen::Matrix<double,6,NV> U_t;
    typedef Eigen::Matrix<double,NV,NV> D_t;
    typedef Eigen::Matrix<double,6,NV> UD_t;

    typedef Eigen::Matrix<double,NQ,1> ConfigVector_t;
    typedef Eigen::Matrix<double,NV,1> TangentVector_t;
  };

  template<int _NQ, int _NV> struct traits< JointDataDense<_NQ, _NV > > { typedef JointDense<_NQ,_NV > JointDerived; };
  template<int _NQ, int _NV> struct traits< JointModelDense<_NQ, _NV > > { typedef JointDense<_NQ,_NV > JointDerived; };

  template <int _NQ, int _NV>
  struct JointDataDense : public JointDataBase< JointDataDense<_NQ, _NV > >
  {
    typedef JointDense<_NQ, _NV > JointDerived;
    typedef JointDataBase< JointDataDense<_NQ,_NV> > Base;
    typedef JointDataDense<_NQ,_NV> Derived;
    SE3_JOINT_TYPEDEF_TEMPLATE;

    Constraint_t S;
    Transformation_t M;
    Motion_t v;
    Bias_t c;

    F_t F;
    
    // [ABA] specific data
    U_t U;
    D_t Dinv;
    UD_t UDinv;

    /// Removed Default constructor of JointDataDense because it was calling default constructor of
    /// ConstraintXd -> eigen_static_assert
    /// JointDataDense should always be instanciated from a JointDataXX.toDense()
    // JointDataDense() : M(1)
    // {
    //   M.translation(SE3::Vector3::Zero());
    // }

    JointDataDense() {};

    JointDataDense(const Constraint_t & S,
                   const Transformation_t & M,
                   const Motion_t & v,
                   const Bias_t & c,
                   const F_t & F,
                   const U_t & U,
                   const D_t & Dinv,
                   const UD_t & UDinv)
    : S(S)
    , M(M)
    , v(v)
    , c(c)
    , F(F)
    , U(U)
    , Dinv(Dinv)
    , UDinv(UDinv)
    {}

    JointDataDense<_NQ, _NV> toDense_impl() const
    {
      assert(false && "Trying to convert a jointDataDense to JointDataDense : useless"); // disapear with release optimizations
      return *this;
    }

  }; // struct JointDataDense

  template <int _NQ, int _NV>
  struct JointModelDense : public JointModelBase< JointModelDense<_NQ, _NV > >
  {
    typedef JointDense<_NQ, _NV > JointDerived;
    typedef JointModelBase<JointModelDense<_NQ, _NV > > Base;
    
    SE3_JOINT_TYPEDEF_TEMPLATE;

    using Base::id;
    using Base::idx_q;
    using Base::idx_v;
    using Base::setIndexes;
    using Base::i_v;
    using Base::i_q;

    int nv_dyn,nq_dyn;
    
    JointDataDerived createData() const
    {
      //assert(false && "JointModelDense is read-only, should not createData");
      return JointDataDerived();
    }
    void calc(JointDataDerived &,
              const Eigen::VectorXd &) const
    {
      assert(false && "JointModelDense is read-only, should not perform any calc");
    }

    void calc(JointDataDerived &,
              const Eigen::VectorXd &,
              const Eigen::VectorXd &) const
    {
      assert(false && "JointModelDense is read-only, should not perform any calc");
    }
    
    void calc_aba(JointDataDerived &,
                  Inertia::Matrix6 &,
                  const bool) const
    {
      assert(false && "JointModelDense is read-only, should not perform any calc");
    }

    ConfigVector_t integrate_impl(const Eigen::VectorXd &,const Eigen::VectorXd &) const
    { 
      ConfigVector_t result;
      assert(false && "JointModelDense is read-only, should not perform any calc");
      return result; 
    } 

    ConfigVector_t interpolate_impl(const Eigen::VectorXd &,const Eigen::VectorXd &, double) const
    { 
      ConfigVector_t result;
      assert(false && "JointModelDense is read-only, should not perform any calc");
      return result; 
    }

    ConfigVector_t random_impl() const
    { 
      ConfigVector_t result;
      assert(false && "JointModelDense is read-only, should not perform any calc");
      return result; 
    }

    ConfigVector_t randomConfiguration_impl(const ConfigVector_t & , const ConfigVector_t & ) const
    { 
      ConfigVector_t result;
      assert(false && "JointModelDense is read-only, should not perform any calc");
      return result; 
    }  

    TangentVector_t difference_impl(const Eigen::VectorXd &,const Eigen::VectorXd &) const
    { 
      TangentVector_t result;
      assert(false && "JointModelDense is read-only, should not perform any calc");
      return result; 
    } 

    double distance_impl(const Eigen::VectorXd &,const Eigen::VectorXd &) const
    { 
      double result = 0;
      assert(false && "JointModelDense is read-only, should not perform any calc");
      return result; 
    }

    ConfigVector_t neutralConfiguration_impl() const
    { 
      ConfigVector_t result;
      assert(false && "JointModelDense is read-only, should not perform any calc");
      return result; 
    }

    void normalize_impl(Eigen::VectorXd &) const
    {
      assert(false && "JointModelDense is read-only, should not perform any calc");
    }

    JointModelDense() {}
    JointModelDense(JointIndex idx, int idx_q, int idx_v)
    {
      setIndexes(idx, idx_q, idx_v);
    }

    int     nv_impl() const { return nv_dyn; }
    int     nq_impl() const { return nq_dyn; }
    
    typename ConfigVector_t::Scalar finiteDifferenceIncrement() const
    {
      using std::sqrt;
      typedef typename ConfigVector_t::Scalar Scalar;
      return sqrt(Eigen::NumTraits<Scalar>::epsilon());
    }

    template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::ConstType
    jointConfigSelector_impl(const Eigen::MatrixBase<D>& a) const { return a.template segment(i_q,nq_dyn); }
    template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::Type
    jointConfigSelector_impl( Eigen::MatrixBase<D>& a) const { return a.template segment(i_q,nq_dyn); }

    template<typename D>
    typename SizeDepType<NV>::template SegmentReturn<D>::ConstType
    jointVelocitySelector_impl(const Eigen::MatrixBase<D>& a) const { return a.template segment(i_v,nv_dyn); }
    template<typename D>
    typename SizeDepType<NV>::template SegmentReturn<D>::Type
    jointVelocitySelector_impl( Eigen::MatrixBase<D>& a) const { return a.template segment(i_v,nv_dyn); }

    template<typename D>
    typename SizeDepType<NV>::template ColsReturn<D>::ConstType 
    jointCols_impl(const Eigen::MatrixBase<D>& A) const { return A.template middleCols<NV>(i_v); }
    template<typename D>
    typename SizeDepType<NV>::template ColsReturn<D>::Type 
    jointCols_impl(Eigen::MatrixBase<D>& A) const { return A.template middleCols<NV>(i_v); }

    JointModelDense<_NQ, _NV> toDense_impl() const
    {
      assert(false && "Trying to convert a jointModelDense to JointModelDense : useless");
      return *this;
    }

    static std::string classname() { return std::string("JointModelDense"); }
    std::string shortname() const { return classname(); }

  }; // struct JointModelDense

  template<>
  template<typename D>
  typename SizeDepType<Eigen::Dynamic>::template SegmentReturn<D>::ConstType
  JointModelDense<Eigen::Dynamic,Eigen::Dynamic>::
  jointConfigSelector_impl(const Eigen::MatrixBase<D>& a) const { return a.segment(i_q,nq_dyn); }
  template<>
  template<typename D>
  typename SizeDepType<Eigen::Dynamic>::template SegmentReturn<D>::Type
  JointModelDense<Eigen::Dynamic,Eigen::Dynamic>::
  jointConfigSelector_impl(Eigen::MatrixBase<D>& a) const { return a.segment(i_q,nq_dyn); }
  template<>
  template<typename D>
  typename SizeDepType<Eigen::Dynamic>::template SegmentReturn<D>::ConstType
  JointModelDense<Eigen::Dynamic,Eigen::Dynamic>::
  jointVelocitySelector_impl(const Eigen::MatrixBase<D>& a) const { return a.segment(i_v,nv_dyn); }
  template<>
  template<typename D>
  typename SizeDepType<Eigen::Dynamic>::template SegmentReturn<D>::Type
  JointModelDense<Eigen::Dynamic,Eigen::Dynamic>::
  jointVelocitySelector_impl(Eigen::MatrixBase<D>& a) const { return a.segment(i_v,nv_dyn); }

  template<>
  template<typename D>
  typename SizeDepType<Eigen::Dynamic>::template ColsReturn<D>::ConstType 
  JointModelDense<Eigen::Dynamic,Eigen::Dynamic>::
  jointCols_impl(const Eigen::MatrixBase<D>& A) const { return A.middleCols(i_v,nv_dyn); }
  template<>
  template<typename D>
  typename SizeDepType<Eigen::Dynamic>::template ColsReturn<D>::Type 
  JointModelDense<Eigen::Dynamic,Eigen::Dynamic>::
  jointCols_impl(Eigen::MatrixBase<D>& A) const { return A.middleCols(i_v,nv_dyn); }

} // namespace se3

#endif // ifndef __se3_joint_dense_hpp__
