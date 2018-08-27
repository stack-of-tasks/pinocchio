//
// Copyright (c) 2015-2016 CNRS
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

#include <Eigen/Core>
#include <limits>

namespace se3
{
#ifdef __clang__

#define SE3_JOINT_TYPEDEF_ARG(prefix)              \
   typedef int Index;                \
   typedef prefix traits<JointDerived>::Scalar Scalar;    \
   typedef prefix traits<JointDerived>::JointDataDerived JointDataDerived;        \
   typedef prefix traits<JointDerived>::JointModelDerived JointModelDerived;      \
   typedef prefix traits<JointDerived>::Constraint_t Constraint_t;      \
   typedef prefix traits<JointDerived>::Transformation_t Transformation_t; \
   typedef prefix traits<JointDerived>::Motion_t Motion_t;        \
   typedef prefix traits<JointDerived>::Bias_t Bias_t;        \
   typedef prefix traits<JointDerived>::F_t F_t;          \
   typedef prefix traits<JointDerived>::U_t U_t;       \
   typedef prefix traits<JointDerived>::D_t D_t;       \
   typedef prefix traits<JointDerived>::UD_t UD_t;       \
   enum {                  \
    NQ = traits<JointDerived>::NQ,              \
    NV = traits<JointDerived>::NV               \
  };                        \
  typedef prefix traits<JointDerived>::ConfigVector_t ConfigVector_t;        \
  typedef prefix traits<JointDerived>::TangentVector_t TangentVector_t

#define SE3_JOINT_TYPEDEF SE3_JOINT_TYPEDEF_ARG()
#define SE3_JOINT_TYPEDEF_TEMPLATE SE3_JOINT_TYPEDEF_ARG(typename)

#elif (__GNUC__ == 4) && (__GNUC_MINOR__ == 4) && (__GNUC_PATCHLEVEL__ == 2)

#define SE3_JOINT_TYPEDEF_NOARG()       \
  typedef int Index;            \
  typedef traits<JointDerived>::Scalar Scalar;    \
  typedef traits<JointDerived>::JointDataDerived JointDataDerived;     \
  typedef traits<JointDerived>::JointModelDerived JointModelDerived;     \
  typedef traits<JointDerived>::Constraint_t Constraint_t;   \
  typedef traits<JointDerived>::Transformation_t Transformation_t; \
  typedef traits<JointDerived>::Motion_t Motion_t;     \
  typedef traits<JointDerived>::Bias_t Bias_t;       \
  typedef traits<JointDerived>::F_t F_t;       \
  typedef traits<JointDerived>::U_t U_t;       \
  typedef traits<JointDerived>::D_t D_t;       \
  typedef traits<JointDerived>::UD_t UD_t;       \
  enum {              \
    NQ = traits<JointDerived>::NQ,         \
    NV = traits<JointDerived>::NV          \
  };                        \
  typedef traits<JointDerived>::ConfigVector_t ConfigVector_t;        \
  typedef traits<JointDerived>::TangentVector_t TangentVector_t

#define SE3_JOINT_TYPEDEF_ARG(prefix)         \
  typedef int Index;              \
  typedef prefix traits<JointDerived>::Scalar Scalar;     \
  typedef prefix traits<JointDerived>::JointDataDerived JointDataDerived;      \
  typedef prefix traits<JointDerived>::JointModelDerived JointModelDerived;      \
  typedef prefix traits<JointDerived>::Constraint_t Constraint_t;    \
  typedef prefix traits<JointDerived>::Transformation_t Transformation_t;  \
  typedef prefix traits<JointDerived>::Motion_t Motion_t;      \
  typedef prefix traits<JointDerived>::Bias_t Bias_t;        \
  typedef prefix traits<JointDerived>::F_t F_t;        \
  typedef prefix traits<JointDerived>::U_t U_t;       \
  typedef prefix traits<JointDerived>::D_t D_t;       \
  typedef prefix traits<JointDerived>::UD_t UD_t;       \
  enum {                \
    NQ = traits<JointDerived>::NQ,           \
    NV = traits<JointDerived>::NV            \
  };                        \
  typedef prefix traits<JointDerived>::ConfigVector_t ConfigVector_t;        \
  typedef prefix traits<JointDerived>::TangentVector_t TangentVector_t

#define SE3_JOINT_TYPEDEF SE3_JOINT_TYPEDEF_NOARG()
#define SE3_JOINT_TYPEDEF_TEMPLATE SE3_JOINT_TYPEDEF_ARG(typename)

#else

#define SE3_JOINT_TYPEDEF_ARG()              \
  typedef int Index;                 \
  typedef typename traits<JointDerived>::Scalar Scalar;    \
  typedef typename traits<JointDerived>::JointDataDerived JointDataDerived;         \
  typedef typename traits<JointDerived>::JointModelDerived JointModelDerived;       \
  typedef typename traits<JointDerived>::Constraint_t Constraint_t;       \
  typedef typename traits<JointDerived>::Transformation_t Transformation_t; \
  typedef typename traits<JointDerived>::Motion_t Motion_t;         \
  typedef typename traits<JointDerived>::Bias_t Bias_t;         \
  typedef typename traits<JointDerived>::F_t F_t;           \
  typedef typename traits<JointDerived>::U_t U_t;       \
  typedef typename traits<JointDerived>::D_t D_t;       \
  typedef typename traits<JointDerived>::UD_t UD_t;       \
  enum {                   \
    NQ = traits<JointDerived>::NQ,              \
    NV = traits<JointDerived>::NV               \
  };                        \
  typedef typename traits<JointDerived>::ConfigVector_t ConfigVector_t;        \
  typedef typename traits<JointDerived>::TangentVector_t TangentVector_t

#define SE3_JOINT_TYPEDEF SE3_JOINT_TYPEDEF_ARG()
#define SE3_JOINT_TYPEDEF_TEMPLATE SE3_JOINT_TYPEDEF_ARG()

#endif

#define SE3_JOINT_USE_INDEXES \
  typedef JointModelBase<JointModelDerived> Base; \
  using Base::idx_q; \
  using Base::idx_v


  template<typename Derived>
  struct JointDataBase
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef typename traits<Derived>::JointDerived JointDerived;
    SE3_JOINT_TYPEDEF_TEMPLATE;

    JointDataDerived& derived() { return *static_cast<Derived*>(this); }
    const JointDataDerived& derived() const { return *static_cast<const Derived*>(this); }

    const Constraint_t     & S() const  { return derived().S;   }
    const Transformation_t & M() const  { return derived().M;   }
    const Motion_t         & v() const  { return derived().v;   }
    const Bias_t           & c() const  { return derived().c;   }
    F_t & F()        { return derived().F; }
    
    const U_t & U() const { return derived().U; }
    U_t & U() { return derived().U; }
    const D_t & Dinv() const { return derived().Dinv; }
    const UD_t & UDinv() const { return derived().UDinv; }

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
  

    JointModelDerived& derived() { return *static_cast<Derived*>(this); }
    const JointModelDerived& derived() const { return *static_cast<const Derived*>(this); }

    JointDataDerived createData() const { return derived().createData(); }
    
    void calc(JointDataDerived& data,
              const Eigen::VectorXd & qs) const
    { derived().calc(data,qs); }
    
    void calc(JointDataDerived& data,
              const Eigen::VectorXd & qs,
              const Eigen::VectorXd & vs) const
    { derived().calc(data,qs,vs); }
    
    void calc_aba(JointDataDerived & data,
                  Inertia::Matrix6 & I,
                  const bool update_I = false) const
    { derived().calc_aba(data, I, update_I); }
    
    ///
    /// \brief Return the resolution of the finite differerence increment according to the Scalar type
    /// \remark Ideally, this function must depend on the value of q
    ///
    /// \returns The finite difference increment.
    ///
    typename ConfigVector_t::Scalar finiteDifferenceIncrement() const
    { return derived().finiteDifferenceIncrement(); }

    JointIndex i_id; // ID of the joint in the multibody list.
    int i_q;    // Index of the joint configuration in the joint configuration vector.
    int i_v;    // Index of the joint velocity in the joint velocity vector.

    int     nv()    const { return derived().nv_impl(); }
    int     nq()    const { return derived().nq_impl(); }
    // Both _impl methods are reimplemented by dynamic-size joints.
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
    
    template <class OtherDerived>
    bool operator==(const JointModelBase<OtherDerived> & other) const { return derived().isEqual(other); }
    
    template <class OtherDerived>
    bool isEqual(const JointModelBase<OtherDerived> &) const { return false; }
    
    bool isEqual(const JointModelBase<Derived> & other) const
    {
      return other.id() == id() && other.idx_q() == idx_q() && other.idx_v() == idx_v();
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
    inline JointModelBase() : i_id(std::numeric_limits<JointIndex>::max()), i_q(-1), i_v(-1) {}
    
    /// Copy constructor: protected.
    ///
    /// Copy of stand-alone JointModelBase are prevented, but can be used from inhereting
    /// objects. Copy is done by calling copy operator.
    inline JointModelBase( const JointModelBase& clone) { *this = clone; }
    
    /// Copy operator: protected.
    ///
    /// Copy of stand-alone JointModelBase are prevented, but can be used from inhereting
    /// objects. 
    inline JointModelBase& operator= (const JointModelBase& clone) 
    {
//      setIndexes(clone.id(),clone.idx_q(),clone.idx_v());
      i_id = clone.i_id;
      i_q = clone.i_q;
      i_v = clone.i_v;
      return *this;
    }

  }; // struct JointModelBase

} // namespace se3

#endif // ifndef __se3_joint_base_hpp__
