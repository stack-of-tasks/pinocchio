//
// Copyright (c) 2015-2019 CNRS INRIA
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_multibody_joint_model_base_hpp__
#define __pinocchio_multibody_joint_model_base_hpp__

#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/joint/joint-common-operations.hpp"

#include "pinocchio/math/matrix-block.hpp"

#include <limits>

#define PINOCCHIO_JOINT_MODEL_TYPEDEF_GENERIC(Joint,TYPENAME)              \
  typedef Eigen::DenseIndex Index;                \
  typedef TYPENAME traits<Joint>::Scalar Scalar;    \
  typedef TYPENAME traits<Joint>::JointDataDerived JointDataDerived;        \
  typedef TYPENAME traits<Joint>::JointModelDerived JointModelDerived;      \
  typedef TYPENAME traits<Joint>::Constraint_t Constraint_t;      \
  typedef TYPENAME traits<Joint>::Transformation_t Transformation_t; \
  typedef TYPENAME traits<Joint>::Motion_t Motion_t;        \
  typedef TYPENAME traits<Joint>::Bias_t Bias_t;        \
  typedef TYPENAME traits<Joint>::U_t U_t;       \
  typedef TYPENAME traits<Joint>::D_t D_t;       \
  typedef TYPENAME traits<Joint>::UD_t UD_t;       \
  enum {                  \
    Options = traits<Joint>::Options,    \
    NQ = traits<Joint>::NQ,              \
    NV = traits<Joint>::NV               \
  };                        \
  typedef TYPENAME traits<Joint>::ConfigVector_t ConfigVector_t;        \
  typedef TYPENAME traits<Joint>::TangentVector_t TangentVector_t

#ifdef __clang__

  #define PINOCCHIO_JOINT_TYPEDEF(Joint) PINOCCHIO_JOINT_MODEL_TYPEDEF_GENERIC(Joint,PINOCCHIO_EMPTY_ARG)
  #define PINOCCHIO_JOINT_TYPEDEF_TEMPLATE(Joint) PINOCCHIO_JOINT_MODEL_TYPEDEF_GENERIC(Joint,typename)

#elif (__GNUC__ == 4) && (__GNUC_MINOR__ == 4) && (__GNUC_PATCHLEVEL__ == 2)

  #define PINOCCHIO_JOINT_TYPEDEF(Joint) PINOCCHIO_JOINT_MODEL_TYPEDEF_GENERIC(Joint,PINOCCHIO_EMPTY_ARG)
  #define PINOCCHIO_JOINT_TYPEDEF_TEMPLATE(Joint) PINOCCHIO_JOINT_MODEL_TYPEDEF_GENERIC(Joint,typename)

#else

  #define PINOCCHIO_JOINT_TYPEDEF(Joint) PINOCCHIO_JOINT_MODEL_TYPEDEF_GENERIC(Joint,typename)
  #define PINOCCHIO_JOINT_TYPEDEF_TEMPLATE(Joint) PINOCCHIO_JOINT_MODEL_TYPEDEF_GENERIC(Joint,typename)

#endif

#define PINOCCHIO_JOINT_USE_INDEXES(Joint) \
  typedef JointModelBase<Joint> Base; \
  using Base::idx_q; \
  using Base::idx_v

#define PINOCCHIO_JOINT_CAST_TYPE_SPECIALIZATION(JointModelTpl) \
template<typename Scalar, int Options, typename NewScalar> \
struct CastType< NewScalar, JointModelTpl<Scalar,Options> > \
{ typedef JointModelTpl<NewScalar,Options> type; }

namespace pinocchio
{

  template<typename Derived>
  struct JointModelBase
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef typename traits<Derived>::JointDerived JointDerived;
    PINOCCHIO_JOINT_TYPEDEF_TEMPLATE(JointDerived);

    JointModelDerived & derived() { return *static_cast<Derived*>(this); }
    const JointModelDerived & derived() const { return *static_cast<const Derived*>(this); }

    JointDataDerived createData() const { return derived().createData(); }
    
    const std::vector<bool> hasConfigurationLimit() const
    {
      return derived().hasConfigurationLimit();
    }

    const std::vector<bool> hasConfigurationLimitInTangent() const
    {
      return derived().hasConfigurationLimitInTangent();
    }

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
      derived().calc_aba(data, PINOCCHIO_EIGEN_CONST_CAST(Matrix6Type,I), update_I);
    }

    int nv()    const { return derived().nv_impl(); }
    int nq()    const { return derived().nq_impl(); }
    
    // Default _impl methods are reimplemented by dynamic-size joints.
    int  nv_impl() const { return NV; }
    int  nq_impl() const { return NQ; }
    
    int  idx_q() const { return derived().idx_q_impl(); }
    int  idx_v() const { return derived().idx_v_impl(); }
    JointIndex id() const { return derived().id_impl(); }

    int  idx_q_impl() const { return i_q; }
    int  idx_v_impl() const { return i_v; }
    JointIndex id_impl() const { return i_id; }

    void setIndexes(JointIndex id, int q, int v)
    { derived().setIndexes_impl(id, q, v); }
    
    void setIndexes_impl(JointIndex id,int q,int v)
    { i_id = id, i_q = q; i_v = v; }
    
    void disp(std::ostream & os) const
    {
      using namespace std;
      os
      << shortname() << endl
      << "  index: " << id() << endl
      << "  index q: " << idx_q() << endl
      << "  index v: " << idx_v() << endl
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
    bool isEqual(const JointModelBase<OtherDerived> &) const
    { return false; }
    
    bool isEqual(const JointModelBase<Derived> & other) const
    {
      return derived().hasSameIndexes(other.derived());
    }
    
    template <class OtherDerived>
    bool hasSameIndexes(const JointModelBase<OtherDerived> & other) const
    {
      return other.id() == id()
      && other.idx_q() == idx_q()
      && other.idx_v() == idx_v();
    }

    /* Acces to dedicated segment in robot config space.  */
    // Const access
    template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::ConstType
    jointConfigSelector(const Eigen::MatrixBase<D> & a) const
    { return derived().jointConfigSelector_impl(a); }
    
    template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::ConstType
    jointConfigSelector_impl(const Eigen::MatrixBase<D> & a) const
    { return SizeDepType<NQ>::segment(a.derived(),idx_q(),nq()); }
    
    // Non-const access
    template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::Type
    jointConfigSelector(Eigen::MatrixBase<D> & a) const
    { return derived().jointConfigSelector_impl(a); }
    
    template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::Type
    jointConfigSelector_impl(Eigen::MatrixBase<D> & a) const
    { return SizeDepType<NQ>::segment(a,idx_q(),nq()); }

    /* Acces to dedicated segment in robot config velocity space.  */
    // Const access
    template<typename D>
    typename SizeDepType<NV>::template SegmentReturn<D>::ConstType
    jointVelocitySelector(const Eigen::MatrixBase<D> & a) const
    { return derived().jointVelocitySelector_impl(a.derived()); }
    
    template<typename D>
    typename SizeDepType<NV>::template SegmentReturn<D>::ConstType
    jointVelocitySelector_impl(const Eigen::MatrixBase<D> & a) const
    { return SizeDepType<NV>::segment(a.derived(),idx_v(),nv()); }
    
    // Non-const access
    template<typename D>
    typename SizeDepType<NV>::template SegmentReturn<D>::Type
    jointVelocitySelector(Eigen::MatrixBase<D> & a) const
    { return derived().jointVelocitySelector_impl(a.derived()); }
    
    template<typename D>
    typename SizeDepType<NV>::template SegmentReturn<D>::Type
    jointVelocitySelector_impl(Eigen::MatrixBase<D> & a) const
    { return SizeDepType<NV>::segment(a.derived(),idx_v(),nv()); }

    /* Acces to dedicated columns in a ForceSet or MotionSet matrix.*/
    // Const access
    template<typename D>
    typename SizeDepType<NV>::template ColsReturn<D>::ConstType
    jointCols(const Eigen::MatrixBase<D>& A) const
    { return derived().jointCols_impl(A.derived()); }
    
    template<typename D>
    typename SizeDepType<NV>::template ColsReturn<D>::ConstType
    jointCols_impl(const Eigen::MatrixBase<D>& A) const
    { return SizeDepType<NV>::middleCols(A.derived(),idx_v(),nv()); }
    
    // Non-const access
    template<typename D>
    typename SizeDepType<NV>::template ColsReturn<D>::Type
    jointCols(Eigen::MatrixBase<D>& A) const
    { return derived().jointCols_impl(A.derived()); }
    
    template<typename D>
    typename SizeDepType<NV>::template ColsReturn<D>::Type
    jointCols_impl(Eigen::MatrixBase<D> & A) const
    { return SizeDepType<NV>::middleCols(A.derived(),idx_v(),nv()); }
    
    /* Acces to dedicated rows in a matrix.*/
    // Const access
    template<typename D>
    typename SizeDepType<NV>::template RowsReturn<D>::ConstType
    jointRows(const Eigen::MatrixBase<D> & A) const
    { return derived().jointRows_impl(A.derived()); }
    
    template<typename D>
    typename SizeDepType<NV>::template RowsReturn<D>::ConstType
    jointRows_impl(const Eigen::MatrixBase<D> & A) const
    { return SizeDepType<NV>::middleRows(A.derived(),idx_v(),nv()); }
    
    // Non-const access
    template<typename D>
    typename SizeDepType<NV>::template RowsReturn<D>::Type
    jointRows(Eigen::MatrixBase<D> & A) const
    { return derived().jointRows_impl(A.derived()); }
    
    template<typename D>
    typename SizeDepType<NV>::template RowsReturn<D>::Type
    jointRows_impl(Eigen::MatrixBase<D> & A) const
    { return SizeDepType<NV>::middleRows(A.derived(),idx_v(),nv()); }
    
    /// \brief Returns a block of dimension nv()xnv() located at position idx_v(),idx_v() in the matrix Mat
    // Const access
    template<typename D>
    typename SizeDepType<NV>::template BlockReturn<D>::ConstType
    jointBlock(const Eigen::MatrixBase<D> & Mat) const
    { return derived().jointBlock_impl(Mat.derived()); }
    
    template<typename D>
    typename SizeDepType<NV>::template BlockReturn<D>::ConstType
    jointBlock_impl(const Eigen::MatrixBase<D> & Mat) const
    { return SizeDepType<NV>::block(Mat.derived(),idx_v(),idx_v(),nv(),nv()); }
    
    // Non-const access
    template<typename D>
    typename SizeDepType<NV>::template BlockReturn<D>::Type
    jointBlock(Eigen::MatrixBase<D> & Mat) const
    { return derived().jointBlock_impl(Mat.derived()); }
    
    template<typename D>
    typename SizeDepType<NV>::template BlockReturn<D>::Type
    jointBlock_impl(Eigen::MatrixBase<D> & Mat) const
    { return SizeDepType<NV>::block(Mat.derived(),idx_v(),idx_v(),nv(),nv()); }

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
    inline JointModelBase(const JointModelBase & clone)
    { *this = clone; }
    
    /// Copy operator: protected.
    ///
    /// Copy of stand-alone JointModelBase are prevented, but can be used from inhereting
    /// objects.
    inline JointModelBase & operator=(const JointModelBase & clone)
    {
      i_id = clone.i_id;
      i_q = clone.i_q;
      i_v = clone.i_v;
      return *this;
    }
    
    // data
    JointIndex i_id; // ID of the joint in the multibody list.
    int i_q;    // Index of the joint configuration in the joint configuration vector.
    int i_v;    // Index of the joint velocity in the joint velocity vector.

  }; // struct JointModelBase

} // namespace pinocchio

#endif // ifndef __pinocchio_multibody_joint_model_base_hpp__

