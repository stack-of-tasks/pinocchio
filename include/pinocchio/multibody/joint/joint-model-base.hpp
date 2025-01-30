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

#define PINOCCHIO_JOINT_MODEL_TYPEDEF_GENERIC(Joint, TYPENAME)                                     \
  typedef Eigen::DenseIndex Index;                                                                 \
  typedef TYPENAME traits<Joint>::Scalar Scalar;                                                   \
  typedef TYPENAME traits<Joint>::JointDataDerived JointDataDerived;                               \
  typedef TYPENAME traits<Joint>::JointModelDerived JointModelDerived;                             \
  typedef TYPENAME traits<Joint>::Constraint_t Constraint_t;                                       \
  typedef TYPENAME traits<Joint>::Transformation_t Transformation_t;                               \
  typedef TYPENAME traits<Joint>::Motion_t Motion_t;                                               \
  typedef TYPENAME traits<Joint>::Bias_t Bias_t;                                                   \
  typedef TYPENAME traits<Joint>::U_t U_t;                                                         \
  typedef TYPENAME traits<Joint>::D_t D_t;                                                         \
  typedef TYPENAME traits<Joint>::UD_t UD_t;                                                       \
  typedef TYPENAME traits<Joint>::is_mimicable_t is_mimicable_t;                                   \
  enum                                                                                             \
  {                                                                                                \
    Options = traits<Joint>::Options,                                                              \
    NQ = traits<Joint>::NQ,                                                                        \
    NV = traits<Joint>::NV,                                                                        \
    NVExtended = traits<Joint>::NVExtended                                                         \
  };                                                                                               \
  typedef TYPENAME traits<Joint>::ConfigVector_t ConfigVector_t;                                   \
  typedef TYPENAME traits<Joint>::TangentVector_t TangentVector_t

#ifdef __clang__

  #define PINOCCHIO_JOINT_TYPEDEF(Joint)                                                           \
    PINOCCHIO_JOINT_MODEL_TYPEDEF_GENERIC(Joint, PINOCCHIO_EMPTY_ARG)
  #define PINOCCHIO_JOINT_TYPEDEF_TEMPLATE(Joint)                                                  \
    PINOCCHIO_JOINT_MODEL_TYPEDEF_GENERIC(Joint, typename)

#elif (__GNUC__ == 4) && (__GNUC_MINOR__ == 4) && (__GNUC_PATCHLEVEL__ == 2)

  #define PINOCCHIO_JOINT_TYPEDEF(Joint)                                                           \
    PINOCCHIO_JOINT_MODEL_TYPEDEF_GENERIC(Joint, PINOCCHIO_EMPTY_ARG)
  #define PINOCCHIO_JOINT_TYPEDEF_TEMPLATE(Joint)                                                  \
    PINOCCHIO_JOINT_MODEL_TYPEDEF_GENERIC(Joint, typename)

#else

  #define PINOCCHIO_JOINT_TYPEDEF(Joint) PINOCCHIO_JOINT_MODEL_TYPEDEF_GENERIC(Joint, typename)
  #define PINOCCHIO_JOINT_TYPEDEF_TEMPLATE(Joint)                                                  \
    PINOCCHIO_JOINT_MODEL_TYPEDEF_GENERIC(Joint, typename)

#endif

#define PINOCCHIO_JOINT_USE_INDEXES(Joint)                                                         \
  typedef JointModelBase<Joint> Base;                                                              \
  using Base::idx_q;                                                                               \
  using Base::idx_v;                                                                               \
  using Base::idx_vExtended

#define PINOCCHIO_JOINT_CAST_TYPE_SPECIALIZATION(JointModelTpl)                                    \
  template<typename Scalar, int Options, typename NewScalar>                                       \
  struct CastType<NewScalar, JointModelTpl<Scalar, Options>>                                       \
  {                                                                                                \
    typedef JointModelTpl<NewScalar, Options> type;                                                \
  }

namespace pinocchio
{

  template<typename Derived>
  struct JointModelBase : NumericalBase<Derived>
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef typename traits<Derived>::JointDerived JointDerived;
    PINOCCHIO_JOINT_TYPEDEF_TEMPLATE(JointDerived);

    JointModelDerived & derived()
    {
      return *static_cast<Derived *>(this);
    }
    const JointModelDerived & derived() const
    {
      return *static_cast<const Derived *>(this);
    }

    JointDataDerived createData() const
    {
      return derived().createData();
    }

    const std::vector<bool> hasConfigurationLimit() const
    {
      return derived().hasConfigurationLimit();
    }

    const std::vector<bool> hasConfigurationLimitInTangent() const
    {
      return derived().hasConfigurationLimitInTangent();
    }

    template<typename ConfigVectorType>
    void calc(JointDataDerived & data, const Eigen::MatrixBase<ConfigVectorType> & qs) const
    {
      derived().calc(data, qs.derived());
    }

    template<typename TangentVectorType>
    void calc(
      JointDataDerived & data,
      const Blank not_used,
      const Eigen::MatrixBase<TangentVectorType> & vs) const
    {
      derived().calc(data, not_used, vs.derived());
    }

    template<typename ConfigVectorType, typename TangentVectorType>
    void calc(
      JointDataDerived & data,
      const Eigen::MatrixBase<ConfigVectorType> & qs,
      const Eigen::MatrixBase<TangentVectorType> & vs) const
    {
      derived().calc(data, qs.derived(), vs.derived());
    }

    template<typename VectorLike, typename Matrix6Like>
    void calc_aba(
      JointDataDerived & data,
      const Eigen::MatrixBase<VectorLike> & armature,
      const Eigen::MatrixBase<Matrix6Like> & I,
      const bool update_I = false) const
    {
      derived().calc_aba(
        data, armature.derived(), PINOCCHIO_EIGEN_CONST_CAST(Matrix6Like, I), update_I);
    }

    int nv() const
    {
      return derived().nv_impl();
    }
    int nq() const
    {
      return derived().nq_impl();
    }
    int nvExtended() const
    {
      return derived().nvExtended_impl();
    }

    // Default _impl methods are reimplemented by dynamic-size joints.
    int nv_impl() const
    {
      return NV;
    }
    int nq_impl() const
    {
      return NQ;
    }
    int nvExtended_impl() const
    {
      return NVExtended;
    }

    int idx_q() const
    {
      return derived().idx_q_impl();
    }
    int idx_v() const
    {
      return derived().idx_v_impl();
    }
    int idx_vExtended() const
    {
      return derived().idx_vExtended_impl();
    }
    JointIndex id() const
    {
      return derived().id_impl();
    }

    int idx_q_impl() const
    {
      return i_q;
    }
    int idx_v_impl() const
    {
      return i_v;
    }
    int idx_vExtended_impl() const
    {
      return i_vExtended;
    }
    JointIndex id_impl() const
    {
      return i_id;
    }

    void setIndexes(JointIndex id, int q, int v)
    {
      derived().setIndexes_impl(id, q, v, v);
    }

    void setIndexes(JointIndex id, int q, int v, int vExtended)
    {
      derived().setIndexes_impl(id, q, v, vExtended);
    }

    void setIndexes_impl(JointIndex id, int q, int v, int vExtended)
    {
      i_id = id, i_q = q;
      i_v = v;
      i_vExtended = vExtended;
    }

    void disp(std::ostream & os) const
    {
      using namespace std;
      os << shortname() << endl
         << "  index: " << id() << endl
         << "  index q: " << idx_q() << endl
         << "  index v: " << idx_v() << endl
         << "  index vExtended: " << idx_vExtended() << endl
         << "  nq: " << nq() << endl
         << "  nv: " << nv() << endl
         << "  nvExtended: " << nvExtended() << endl;
    }

    friend std::ostream & operator<<(std::ostream & os, const JointModelBase<Derived> & jmodel)
    {
      jmodel.derived().disp(os);
      return os;
    }

    std::string shortname() const
    {
      return derived().shortname();
    }
    static std::string classname()
    {
      return Derived::classname();
    }

    template<typename NewScalar>
    typename CastType<NewScalar, Derived>::type cast() const
    {
      return derived().template cast<NewScalar>();
    }

    template<class OtherDerived>
    bool operator==(const JointModelBase<OtherDerived> & other) const
    {
      return derived().isEqual(other.derived());
    }

    template<class OtherDerived>
    bool operator!=(const JointModelBase<OtherDerived> & other) const
    {
      return !(internal::comparison_eq(derived(), other.derived()));
    }

    template<class OtherDerived>
    bool isEqual(const JointModelBase<OtherDerived> &) const
    {
      return false;
    }

    bool isEqual(const JointModelBase<Derived> & other) const
    {
      return derived().hasSameIndexes(other.derived());
    }

    template<class OtherDerived>
    bool hasSameIndexes(const JointModelBase<OtherDerived> & other) const
    {
      return internal::comparison_eq(other.id(), id())
             && internal::comparison_eq(other.idx_q(), idx_q())
             && internal::comparison_eq(other.idx_v(), idx_v())
             && internal::comparison_eq(other.idx_vExtended(), idx_vExtended());
    }

    /* Acces to dedicated segment in robot config space.  */
    // Const access
    template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::ConstType
    JointMappedConfigSelector(const Eigen::MatrixBase<D> & a) const
    {
      return derived().JointMappedConfigSelector_impl(a);
    }

    template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::ConstType
    JointMappedConfigSelector_impl(const Eigen::MatrixBase<D> & a) const
    {
      return SizeDepType<NQ>::segment(a.derived(), idx_q(), nq());
    }

    // Non-const access
    template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::Type
    JointMappedConfigSelector(Eigen::MatrixBase<D> & a) const
    {
      return derived().JointMappedConfigSelector_impl(a);
    }

    template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::Type
    JointMappedConfigSelector_impl(Eigen::MatrixBase<D> & a) const
    {
      return SizeDepType<NQ>::segment(a, idx_q(), nq());
    }

    // Const access
    template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::ConstType
    jointConfigSelector(const Eigen::MatrixBase<D> & a) const
    {
      return derived().jointConfigSelector_impl(a);
    }

    template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::ConstType
    jointConfigSelector_impl(const Eigen::MatrixBase<D> & a) const
    {
      return SizeDepType<NQ>::segment(a.derived(), idx_q(), nq());
    }

    // Non-const access
    template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::Type
    jointConfigSelector(Eigen::MatrixBase<D> & a) const
    {
      return derived().jointConfigSelector_impl(a);
    }

    template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::Type
    jointConfigSelector_impl(Eigen::MatrixBase<D> & a) const
    {
      return SizeDepType<NQ>::segment(a, idx_q(), nq());
    }

    /* Acces to dedicated segment in robot config velocity space.  */
    // Const access
    template<typename D>
    typename SizeDepType<NV>::template SegmentReturn<D>::ConstType
    JointMappedVelocitySelector(const Eigen::MatrixBase<D> & a) const
    {
      return derived().JointMappedVelocitySelector_impl(a.derived());
    }

    template<typename D>
    typename SizeDepType<NV>::template SegmentReturn<D>::ConstType
    JointMappedVelocitySelector_impl(const Eigen::MatrixBase<D> & a) const
    {
      return SizeDepType<NV>::segment(a.derived(), idx_v(), nvExtended());
    }

    // Non-const access
    template<typename D>
    typename SizeDepType<NV>::template SegmentReturn<D>::Type
    JointMappedVelocitySelector(Eigen::MatrixBase<D> & a) const
    {
      return derived().JointMappedVelocitySelector_impl(a.derived());
    }

    template<typename D>
    typename SizeDepType<NV>::template SegmentReturn<D>::Type
    JointMappedVelocitySelector_impl(Eigen::MatrixBase<D> & a) const
    {
      return SizeDepType<NV>::segment(a.derived(), idx_v(), nvExtended());
    }

    // Const access
    template<typename D>
    typename SizeDepType<NV>::template SegmentReturn<D>::ConstType
    jointVelocitySelector(const Eigen::MatrixBase<D> & a) const
    {
      return derived().jointVelocitySelector_impl(a.derived());
    }

    template<typename D>
    typename SizeDepType<NV>::template SegmentReturn<D>::ConstType
    jointVelocitySelector_impl(const Eigen::MatrixBase<D> & a) const
    {
      return SizeDepType<NV>::segment(a.derived(), idx_v(), nv());
    }

    // Non-const access
    template<typename D>
    typename SizeDepType<NV>::template SegmentReturn<D>::Type
    jointVelocitySelector(Eigen::MatrixBase<D> & a) const
    {
      return derived().jointVelocitySelector_impl(a.derived());
    }

    template<typename D>
    typename SizeDepType<NV>::template SegmentReturn<D>::Type
    jointVelocitySelector_impl(Eigen::MatrixBase<D> & a) const
    {
      return SizeDepType<NV>::segment(a.derived(), idx_v(), nv());
    }

    /* Acces to dedicated columns in a ForceSet or MotionSet matrix.*/
    // Const access
    template<typename D>
    typename SizeDepType<NV>::template ColsReturn<D>::ConstType
    jointCols(const Eigen::MatrixBase<D> & A) const
    {
      return derived().jointCols_impl(A.derived());
    }

    template<typename D>
    typename SizeDepType<NVExtended>::template ColsReturn<D>::ConstType
    jointExtendedModelCols(const Eigen::MatrixBase<D> & A) const
    {
      return derived().jointExtendedModelCols_impl(A.derived());
    }

    template<typename D>
    typename SizeDepType<NV>::template ColsReturn<D>::ConstType
    jointCols_impl(const Eigen::MatrixBase<D> & A) const
    {
      return SizeDepType<NV>::middleCols(A.derived(), idx_v(), nv());
    }

    template<typename D>
    typename SizeDepType<NVExtended>::template ColsReturn<D>::ConstType
    jointExtendedModelCols_impl(const Eigen::MatrixBase<D> & A) const
    {
      return SizeDepType<NVExtended>::middleCols(A.derived(), idx_vExtended(), nvExtended());
    }

    // Non-const access
    // TODO rename Jac/Vel into Full/Red (for full system and reduced system ?)
    template<typename D>
    typename SizeDepType<NV>::template ColsReturn<D>::Type jointCols(Eigen::MatrixBase<D> & A) const
    {
      return derived().jointCols_impl(A.derived());
    }

    template<typename D>
    typename SizeDepType<NVExtended>::template ColsReturn<D>::Type
    jointExtendedModelCols(Eigen::MatrixBase<D> & A) const
    {
      return derived().jointExtendedModelCols_impl(A.derived());
    }

    template<typename D>
    typename SizeDepType<NV>::template ColsReturn<D>::Type
    jointCols_impl(Eigen::MatrixBase<D> & A) const
    {
      return SizeDepType<NV>::middleCols(A.derived(), idx_v(), nv());
    }

    template<typename D>
    typename SizeDepType<NVExtended>::template ColsReturn<D>::Type
    jointExtendedModelCols_impl(Eigen::MatrixBase<D> & A) const
    {
      return SizeDepType<NVExtended>::middleCols(A.derived(), idx_vExtended(), nvExtended());
    }

    /* Acces to dedicated rows in a matrix.*/
    // Const access
    template<typename D>
    typename SizeDepType<NV>::template RowsReturn<D>::ConstType
    jointRows(const Eigen::MatrixBase<D> & A) const
    {
      return derived().jointRows_impl(A.derived());
    }

    template<typename D>
    typename SizeDepType<NVExtended>::template RowsReturn<D>::ConstType
    jointExtendedModelRows(const Eigen::MatrixBase<D> & A) const
    {
      return derived().jointExtendedModelRows_impl(A.derived());
    }

    template<typename D>
    typename SizeDepType<NV>::template RowsReturn<D>::ConstType
    jointRows_impl(const Eigen::MatrixBase<D> & A) const
    {
      return SizeDepType<NV>::middleRows(A.derived(), idx_v(), nv());
    }

    template<typename D>
    typename SizeDepType<NVExtended>::template RowsReturn<D>::ConstType
    jointExtendedModelRows_impl(const Eigen::MatrixBase<D> & A) const
    {
      return SizeDepType<NVExtended>::middleRows(A.derived(), idx_vExtended(), nvExtended());
    }

    // Non-const access
    template<typename D>
    typename SizeDepType<NV>::template RowsReturn<D>::Type jointRows(Eigen::MatrixBase<D> & A) const
    {
      return derived().jointRows_impl(A.derived());
    }

    template<typename D>
    typename SizeDepType<NVExtended>::template RowsReturn<D>::Type
    jointExtendedModelRows(Eigen::MatrixBase<D> & A) const
    {
      return derived().jointExtendedModelRows_impl(A.derived());
    }

    template<typename D>
    typename SizeDepType<NV>::template RowsReturn<D>::Type
    jointRows_impl(Eigen::MatrixBase<D> & A) const
    {
      return SizeDepType<NV>::middleRows(A.derived(), idx_v(), nv());
    }

    template<typename D>
    typename SizeDepType<NVExtended>::template RowsReturn<D>::Type
    jointExtendedModelRows_impl(Eigen::MatrixBase<D> & A) const
    {
      return SizeDepType<NVExtended>::middleRows(A.derived(), idx_vExtended(), nvExtended());
    }

    /// \brief Returns a block of dimension nv()xnv() located at position idx_v(),idx_v() in the
    /// matrix Mat
    // Const access
    template<typename D>
    typename SizeDepType<NV>::template BlockReturn<D>::ConstType
    jointBlock(const Eigen::MatrixBase<D> & Mat) const
    {
      return derived().jointBlock_impl(Mat.derived());
    }

    template<typename D>
    typename SizeDepType<NVExtended>::template BlockReturn<D>::ConstType
    jointExtendedModelBlock(const Eigen::MatrixBase<D> & Mat) const
    {
      return derived().jointExtendedModelBlock_impl(Mat.derived());
    }

    template<typename D>
    typename SizeDepType<NV>::template BlockReturn<D>::ConstType
    jointBlock_impl(const Eigen::MatrixBase<D> & Mat) const
    {
      return SizeDepType<NV>::block(Mat.derived(), idx_v(), idx_v(), nv(), nv());
    }

    template<typename D>
    typename SizeDepType<NVExtended>::template BlockReturn<D>::ConstType
    jointExtendedModelBlock_impl(const Eigen::MatrixBase<D> & Mat) const
    {
      return SizeDepType<NVExtended>::block(
        Mat.derived(), idx_vExtended(), idx_vExtended(), nvExtended(), nvExtended());
    }

    // Non-const access
    template<typename D>
    typename SizeDepType<NV>::template BlockReturn<D>::Type
    jointBlock(Eigen::MatrixBase<D> & Mat) const
    {
      return derived().jointBlock_impl(Mat.derived());
    }

    template<typename D>
    typename SizeDepType<NVExtended>::template BlockReturn<D>::Type
    jointExtendedModelBlock(Eigen::MatrixBase<D> & Mat) const
    {
      return derived().jointExtendedModelBlock_impl(Mat.derived());
    }

    template<typename D>
    typename SizeDepType<NV>::template BlockReturn<D>::Type
    jointBlock_impl(Eigen::MatrixBase<D> & Mat) const
    {
      return SizeDepType<NV>::block(Mat.derived(), idx_v(), idx_v(), nv(), nv());
    }

    template<typename D>
    typename SizeDepType<NVExtended>::template BlockReturn<D>::Type
    jointExtendedModelBlock_impl(Eigen::MatrixBase<D> & Mat) const
    {
      return SizeDepType<NVExtended>::block(
        Mat.derived(), idx_vExtended(), idx_vExtended(), nvExtended(), nvExtended());
    }

  protected:
    /// Default constructor: protected.
    ///
    /// Prevent the construction of stand-alone JointModelBase.
    inline JointModelBase()
    : i_id(std::numeric_limits<JointIndex>::max())
    , i_q(-1)
    , i_v(-1)
    , i_vExtended(-1)
    {
    }

    /// Copy constructor: protected.
    ///
    /// Copy of stand-alone JointModelBase are prevented, but can be used from inhereting
    /// objects. Copy is done by calling copy operator.
    inline JointModelBase(const JointModelBase & clone)
    {
      *this = clone;
    }

    /// Copy operator: protected.
    ///
    /// Copy of stand-alone JointModelBase are prevented, but can be used from inhereting
    /// objects.
    inline JointModelBase & operator=(const JointModelBase & clone)
    {
      i_id = clone.i_id;
      i_q = clone.i_q;
      i_v = clone.i_v;
      i_vExtended = clone.i_vExtended;
      return *this;
    }

    // data
    JointIndex i_id; // ID of the joint in the multibody list.
    int i_q;         // Index of the joint configuration in the joint configuration vector.
    int i_v;         // Index of the joint velocity in the joint velocity vector.
    int i_vExtended; // Index of the joint jacobian in the joint jacobian matrix.

  }; // struct JointModelBase

} // namespace pinocchio

#endif // ifndef __pinocchio_multibody_joint_model_base_hpp__
