//
// Copyright (c) 2015-2016,2018 CNRS
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
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

#ifndef __se3_joint_base_hpp__
#define __se3_joint_base_hpp__

#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/multibody/joint/fwd.hpp"

#include <limits>

namespace se3
{

#define SE3_JOINT_TYPEDEF_GENERIC(TYPENAME)              \
  typedef Eigen::DenseIndex Index;                \
  typedef TYPENAME traits<JointDerived>::Scalar Scalar;    \
  typedef TYPENAME traits<JointDerived>::JointDataDerived JointDataDerived;        \
  typedef TYPENAME traits<JointDerived>::JointModelDerived JointModelDerived;      \
  typedef TYPENAME traits<JointDerived>::Constraint_t Constraint_t;      \
  typedef TYPENAME traits<JointDerived>::Transformation_t Transformation_t; \
  typedef TYPENAME traits<JointDerived>::Motion_t Motion_t;        \
  typedef TYPENAME traits<JointDerived>::Bias_t Bias_t;        \
  typedef TYPENAME traits<JointDerived>::F_t F_t;          \
  typedef TYPENAME traits<JointDerived>::U_t U_t;       \
  typedef TYPENAME traits<JointDerived>::D_t D_t;       \
  typedef TYPENAME traits<JointDerived>::UD_t UD_t;       \
  enum {                  \
    Options = traits<JointDerived>::Options,    \
    NQ = traits<JointDerived>::NQ,              \
    NV = traits<JointDerived>::NV               \
  };                        \
  typedef TYPENAME traits<JointDerived>::ConfigVector_t ConfigVector_t;        \
  typedef TYPENAME traits<JointDerived>::TangentVector_t TangentVector_t
  
#define PINOCCHIO_JOINT_DATA_TYPEDEF_GENERIC(TYPENAME)              \
  SE3_JOINT_TYPEDEF_GENERIC(TYPENAME); \
  typedef TYPENAME traits<JointDerived>::ConstraintTypeConstRef ConstraintTypeConstRef;      \
  typedef TYPENAME traits<JointDerived>::TansformTypeConstRef TansformTypeConstRef;      \
  typedef TYPENAME traits<JointDerived>::MotionTypeConstRef MotionTypeConstRef;      \
  typedef TYPENAME traits<JointDerived>::BiasTypeConstRef BiasTypeConstRef;      \
  typedef TYPENAME traits<JointDerived>::UTypeConstRef UTypeConstRef;      \
  typedef TYPENAME traits<JointDerived>::UTypeRef UTypeRef;      \
  typedef TYPENAME traits<JointDerived>::DTypeConstRef DTypeConstRef;      \
  typedef TYPENAME traits<JointDerived>::UDTypeConstRef UDTypeConstRef
  
#ifdef __clang__

  #define SE3_JOINT_TYPEDEF SE3_JOINT_TYPEDEF_GENERIC(PINOCCHIO_EMPTY_ARG)
  #define PINOCCHIO_JOINT_DATA_TYPEDEF PINOCCHIO_JOINT_DATA_TYPEDEF_GENERIC(PINOCCHIO_EMPTY_ARG)
  
  #define SE3_JOINT_TYPEDEF_TEMPLATE SE3_JOINT_TYPEDEF_GENERIC(typename)
  #define PINOCCHIO_JOINT_DATA_TYPEDEF_TEMPLATE PINOCCHIO_JOINT_DATA_TYPEDEF_GENERIC(typename)

#elif (__GNUC__ == 4) && (__GNUC_MINOR__ == 4) && (__GNUC_PATCHLEVEL__ == 2)

  #define SE3_JOINT_TYPEDEF SE3_JOINT_TYPEDEF_GENERIC(PINOCCHIO_EMPTY_ARG)
  #define PINOCCHIO_JOINT_DATA_TYPEDEF PINOCCHIO_JOINT_DATA_TYPEDEF_GENERIC(PINOCCHIO_EMPTY_ARG)
  
  #define SE3_JOINT_TYPEDEF_TEMPLATE SE3_JOINT_TYPEDEF_GENERIC(typename)
  #define PINOCCHIO_JOINT_DATA_TYPEDEF_TEMPLATE PINOCCHIO_JOINT_DATA_TYPEDEF_GENERIC(typename)

#else

  #define SE3_JOINT_TYPEDEF SE3_JOINT_TYPEDEF_GENERIC(typename)
  #define PINOCCHIO_JOINT_DATA_TYPEDEF PINOCCHIO_JOINT_DATA_TYPEDEF_GENERIC(typename)
  
  #define SE3_JOINT_TYPEDEF_TEMPLATE SE3_JOINT_TYPEDEF_GENERIC(typename)
  #define PINOCCHIO_JOINT_DATA_TYPEDEF_TEMPLATE PINOCCHIO_JOINT_DATA_TYPEDEF_GENERIC(typename)

#endif

#define SE3_JOINT_USE_INDEXES \
  typedef JointModelBase<JointModelDerived> Base; \
  using Base::idx_q; \
  using Base::idx_v

#define JOINT_CAST_TYPE_SPECIALIZATION(JointModelTpl) \
template<typename Scalar, int Options, typename NewScalar> \
struct CastType< NewScalar, JointModelTpl<Scalar,Options> > \
{ typedef JointModelTpl<NewScalar,Options> type; }
  
  
#define JOINT_DATA_BASE_DEFAULT_ACCESSOR \
  ConstraintTypeConstRef S_accessor() const { return S; } \
  TansformTypeConstRef M_accessor() const { return M; } \
  MotionTypeConstRef v_accessor() const { return v; } \
  BiasTypeConstRef c_accessor() const { return c; } \
  UTypeConstRef U_accessor() const { return U; } \
  UTypeRef U_accessor() { return U; } \
  DTypeConstRef Dinv_accessor() const { return Dinv; } \
  UDTypeConstRef UDinv_accessor() const { return UDinv; }
  
#define JOINT_DATA_BASE_ACCESSOR_DEFAULT_RETURN_TYPE \
  typedef const Constraint_t & ConstraintTypeConstRef; \
  typedef const Transformation_t & TansformTypeConstRef; \
  typedef const Motion_t & MotionTypeConstRef; \
  typedef const Bias_t & BiasTypeConstRef; \
  typedef const U_t & UTypeConstRef; \
  typedef U_t & UTypeRef; \
  typedef const D_t & DTypeConstRef; \
  typedef const UD_t & UDTypeConstRef;
  
  template<typename Derived>
  struct JointDataBase
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef typename traits<Derived>::JointDerived JointDerived;
    PINOCCHIO_JOINT_DATA_TYPEDEF_TEMPLATE;

    Derived & derived() { return *static_cast<Derived*>(this); }
    const Derived & derived() const { return *static_cast<const Derived*>(this); }

    ConstraintTypeConstRef S() const     { return derived().S_accessor(); }
    TansformTypeConstRef M() const     { return derived().M_accessor(); }
    MotionTypeConstRef v() const     { return derived().v_accessor(); }
    BiasTypeConstRef c() const     { return derived().c_accessor(); }
    F_t & F()                              { return derived().F; }
    
    UTypeConstRef U() const     { return derived().U_accessor(); }
    UTypeRef U()           { return derived().U_accessor(); }
    DTypeConstRef Dinv() const  { return derived().Dinv_accessor(); }
    UDTypeConstRef UDinv() const { return derived().UDinv_accessor(); }
    
  protected:
    
    /// \brief Default constructor: protected.
    inline JointDataBase() {}

  }; // struct JointDataBase

  template<int NV>
  struct SizeDepType
  {
    template<class Mat>
    struct SegmentReturn 
    {
      typedef typename Mat::template FixedSegmentReturnType<NV>::Type Type;
      typedef typename Mat::template ConstFixedSegmentReturnType<NV>::Type ConstType;
    };
    template<class Mat>
    struct ColsReturn
    {
      typedef typename Mat::template NColsBlockXpr<NV>::Type Type;
      typedef typename Mat::template ConstNColsBlockXpr<NV>::Type ConstType;
    };
    template<class Mat>
    struct RowsReturn
    {
      typedef typename Mat::template NRowsBlockXpr<NV>::Type Type;
      typedef typename Mat::template ConstNRowsBlockXpr<NV>::Type ConstType;
    };
  };
  
  template<>
  struct SizeDepType<Eigen::Dynamic>
  {
    template<class Mat>
    struct SegmentReturn 
    {
      typedef typename Mat::SegmentReturnType Type;
      typedef typename Mat::ConstSegmentReturnType ConstType;
    };
    template<class Mat>
    struct ColsReturn
    {
      typedef typename Mat::ColsBlockXpr Type;
      typedef typename Mat::ConstColsBlockXpr ConstType;
    };
    template<class Mat>
    struct RowsReturn
    {
      typedef typename Mat::RowsBlockXpr Type;
      typedef typename Mat::ConstRowsBlockXpr ConstType;
    };
  };

  template<typename Derived>
  struct JointModelBase
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef typename traits<Derived>::JointDerived JointDerived;
    SE3_JOINT_TYPEDEF_TEMPLATE;

    JointModelDerived & derived() { return *static_cast<Derived*>(this); }
    const JointModelDerived & derived() const { return *static_cast<const Derived*>(this); }

    JointDataDerived createData() const { return derived().createData(); }
    
    template<typename ConfigVectorType>
    void calc(JointDataDerived & data,
              const Eigen::MatrixBase<ConfigVectorType> & qs) const
    {
      derived().calc(data,qs.derived());
    }
    
    template<typename ConfigVectorType, typename TangentVectorType>
    void calc(JointDataDerived & data,
              const Eigen::MatrixBase<ConfigVectorType> & qs,
              const Eigen::MatrixBase<TangentVectorType> & vs) const
    {
      derived().calc(data,qs.derived(),vs.derived());
      
    }
    
    template<typename Matrix6Type>
    void calc_aba(JointDataDerived & data,
                  const Eigen::MatrixBase<Matrix6Type> & I,
                  const bool update_I = false) const
    {
      derived().calc_aba(data, EIGEN_CONST_CAST(Matrix6Type,I), update_I);
    }
    
    ///
    /// \brief Return the resolution of the finite differerence increment according to the Scalar type
    /// \remark Ideally, this function must depend on the value of q
    ///
    /// \returns The finite difference increment.
    ///
    Scalar finiteDifferenceIncrement() const
    { return derived().finiteDifferenceIncrement(); }

    JointIndex i_id; // ID of the joint in the multibody list.
    int i_q;    // Index of the joint configuration in the joint configuration vector.
    int i_v;    // Index of the joint velocity in the joint velocity vector.

    int     nv()    const { return derived().nv_impl(); }
    int     nq()    const { return derived().nq_impl(); }
    
    // Default _impl methods are reimplemented by dynamic-size joints.
    int     nv_impl() const { return NV; }
    int     nq_impl() const { return NQ; }

    int  idx_q() const { return i_q; }
    int  idx_v() const { return i_v; }
    JointIndex id() const { return i_id; }

    void setIndexes(JointIndex id, int q, int v) { derived().setIndexes_impl(id, q, v); }
    
    void setIndexes_impl(JointIndex id,int q,int v) { i_id = id, i_q = q; i_v = v; }
    
    void disp(std::ostream & os) const
    {
      using namespace std;
      os
      << shortname() << endl
      << "  index: " << i_id << endl
      << "  index q: " << i_q << endl
      << "  index v: " << i_v << endl
      << "  nq: " << nq() << endl
      << "  nv: " << nv() << endl
      ;
    }
    
    friend std::ostream & operator << (std::ostream & os, const JointModelBase<Derived> & joint)
    {
      joint.disp(os);
      return os;
    }
    
    std::string shortname() const { return derived().shortname(); }
    static std::string classname() { return Derived::classname(); }
    
    template<typename NewScalar>
    typename CastType<NewScalar,Derived>::type cast() const
    { return derived().template cast<NewScalar>(); }
    
    template <class OtherDerived>
    bool operator==(const JointModelBase<OtherDerived> & other) const
    { return derived().isEqual(other.derived()); }
    
    template <class OtherDerived>
    bool operator!=(const JointModelBase<OtherDerived> & other) const
    { return !(derived() == other.derived()); }
    
    template <class OtherDerived>
    bool isEqual(const JointModelBase<OtherDerived> &) const { return false; }
    
    bool isEqual(const JointModelBase<Derived> & other) const
    {
      return other.id() == id()
      && other.idx_q() == idx_q()
      && other.idx_v() == idx_v();
    }

    /* Acces to dedicated segment in robot config space.  */
    // Const access
    template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::ConstType 
    jointConfigSelector(const Eigen::MatrixBase<D>& a) const       { return derived().jointConfigSelector_impl(a); }
    template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::ConstType 
    jointConfigSelector_impl(const Eigen::MatrixBase<D>& a) const   { return a.template segment<NQ>(i_q); }
    // Non-const access
    template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::Type 
    jointConfigSelector( Eigen::MatrixBase<D>& a) const { return derived().jointConfigSelector_impl(a); }
    template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::Type 
    jointConfigSelector_impl( Eigen::MatrixBase<D>& a) const { return a.template segment<NQ>(i_q); }

    /* Acces to dedicated segment in robot config velocity space.  */
    // Const access
    template<typename D>
    typename SizeDepType<NV>::template SegmentReturn<D>::ConstType 
    jointVelocitySelector(const Eigen::MatrixBase<D>& a) const       { return derived().jointVelocitySelector_impl(a); }
    template<typename D>
    typename SizeDepType<NV>::template SegmentReturn<D>::ConstType 
    jointVelocitySelector_impl(const Eigen::MatrixBase<D>& a) const   { return a.template segment<NV>(i_v); }
    // Non-const access
    template<typename D>
    typename SizeDepType<NV>::template SegmentReturn<D>::Type 
    jointVelocitySelector( Eigen::MatrixBase<D>& a) const { return derived().jointVelocitySelector_impl(a); }
    template<typename D>
    typename SizeDepType<NV>::template SegmentReturn<D>::Type 
    jointVelocitySelector_impl( Eigen::MatrixBase<D>& a) const { return a.template segment<NV>(i_v); }

    template<typename D>
    typename SizeDepType<NV>::template ColsReturn<D>::ConstType 
    jointCols(const Eigen::MatrixBase<D>& A) const       { return derived().jointCols_impl(A); }
    template<typename D>
    typename SizeDepType<NV>::template ColsReturn<D>::ConstType 
    jointCols_impl(const Eigen::MatrixBase<D>& A) const       { return A.template middleCols<NV>(i_v); }
    template<typename D>
    typename SizeDepType<NV>::template ColsReturn<D>::Type 
    jointCols(Eigen::MatrixBase<D>& A) const       { return derived().jointCols_impl(A); }
    template<typename D>
    typename SizeDepType<NV>::template ColsReturn<D>::Type 
    jointCols_impl(Eigen::MatrixBase<D>& A) const       { return A.template middleCols<NV>(i_v); }

  protected:

    /// Default constructor: protected.
    /// 
    /// Prevent the construction of stand-alone JointModelBase.
    inline JointModelBase()
    : i_id(std::numeric_limits<JointIndex>::max()), i_q(-1), i_v(-1) {}
    
    /// Copy constructor: protected.
    ///
    /// Copy of stand-alone JointModelBase are prevented, but can be used from inhereting
    /// objects. Copy is done by calling copy operator.
    inline JointModelBase( const JointModelBase& clone) { *this = clone; }
    
    /// Copy operator: protected.
    ///
    /// Copy of stand-alone JointModelBase are prevented, but can be used from inhereting
    /// objects. 
    inline JointModelBase& operator=(const JointModelBase& clone)
    {
      i_id = clone.i_id;
      i_q = clone.i_q;
      i_v = clone.i_v;
      return *this;
    }

  }; // struct JointModelBase

} // namespace se3

#endif // ifndef __se3_joint_base_hpp__
