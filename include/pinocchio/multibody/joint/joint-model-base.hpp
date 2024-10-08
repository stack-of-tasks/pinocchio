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
    NJ = traits<Joint>::NJ                                                                         \
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
  using Base::idx_j

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
    int nj() const
    {
      return derived().nj_impl();
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
    int nj_impl() const
    {
      return NJ;
    }

    int idx_q() const
    {
      return derived().idx_q_impl();
    }
    int idx_v() const
    {
      return derived().idx_v_impl();
    }
    int idx_j() const
    {
      return derived().idx_j_impl();
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
    int idx_j_impl() const
    {
      return i_j;
    }
    JointIndex id_impl() const
    {
      return i_id;
    }

    void setIndexes(JointIndex id, int q, int v, int j)
    {
      derived().setIndexes_impl(id, q, v, j);
    }

    void setIndexes_impl(JointIndex id, int q, int v, int j)
    {
      i_id = id, i_q = q;
      i_v = v;
      i_j = j;
    }

    void disp(std::ostream & os) const
    {
      using namespace std;
      os << shortname() << endl
         << "  index: " << id() << endl
         << "  index q: " << idx_q() << endl
         << "  index v: " << idx_v() << endl
         << "  index j: " << idx_j() << endl
         << "  nq: " << nq() << endl
         << "  nv: " << nv() << endl
         << "  nj: " << nj() << endl;
    }

    friend std::ostream & operator<<(std::ostream & os, const JointModelBase<Derived> & joint)
    {
      joint.disp(os);
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
             && internal::comparison_eq(other.idx_j(), idx_j());
    }

    /* Acces to dedicated segment in robot config space.  */
    // Const access
    template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::ConstType
    jointConfigFromDofSelector(const Eigen::MatrixBase<D> & a) const
    {
      return derived().jointConfigFromDofSelector_impl(a);
    }

    template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::ConstType
    jointConfigFromDofSelector_impl(const Eigen::MatrixBase<D> & a) const
    {
      return SizeDepType<NQ>::segment(a.derived(), idx_q(), nq());
    }

    // Non-const access
    template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::Type
    jointConfigFromDofSelector(Eigen::MatrixBase<D> & a) const
    {
      return derived().jointConfigFromDofSelector_impl(a);
    }

    template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::Type
    jointConfigFromDofSelector_impl(Eigen::MatrixBase<D> & a) const
    {
      return SizeDepType<NQ>::segment(a, idx_q(), nq());
    }

    // Const access
    template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::ConstType
    jointConfigFromNqSelector(const Eigen::MatrixBase<D> & a) const
    {
      return derived().jointConfigFromNqSelector_impl(a);
    }

    template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::ConstType
    jointConfigFromNqSelector_impl(const Eigen::MatrixBase<D> & a) const
    {
      return SizeDepType<NQ>::segment(a.derived(), idx_q(), nq());
    }

    // Non-const access
    template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::Type
    jointConfigFromNqSelector(Eigen::MatrixBase<D> & a) const
    {
      return derived().jointConfigFromNqSelector_impl(a);
    }

    template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::Type
    jointConfigFromNqSelector_impl(Eigen::MatrixBase<D> & a) const
    {
      return SizeDepType<NQ>::segment(a, idx_q(), nq());
    }

    /* Acces to dedicated segment in robot config velocity space.  */
    // Const access
    template<typename D>
    typename SizeDepType<NV>::template SegmentReturn<D>::ConstType
    jointVelocityFromDofSelector(const Eigen::MatrixBase<D> & a) const
    {
      return derived().jointVelocityFromDofSelector_impl(a.derived());
    }

    template<typename D>
    typename SizeDepType<NV>::template SegmentReturn<D>::ConstType
    jointVelocityFromDofSelector_impl(const Eigen::MatrixBase<D> & a) const
    {
      return SizeDepType<NV>::segment(a.derived(), idx_v(), nj());
    }

    // Non-const access
    template<typename D>
    typename SizeDepType<NV>::template SegmentReturn<D>::Type
    jointVelocityFromDofSelector(Eigen::MatrixBase<D> & a) const
    {
      return derived().jointVelocityFromDofSelector_impl(a.derived());
    }

    template<typename D>
    typename SizeDepType<NV>::template SegmentReturn<D>::Type
    jointVelocityFromDofSelector_impl(Eigen::MatrixBase<D> & a) const
    {
      return SizeDepType<NV>::segment(a.derived(), idx_v(), nj());
    }

    // Const access
    template<typename D>
    typename SizeDepType<NV>::template SegmentReturn<D>::ConstType
    jointVelocityFromNvSelector(const Eigen::MatrixBase<D> & a) const
    {
      return derived().jointVelocityFromNvSelector_impl(a.derived());
    }

    template<typename D>
    typename SizeDepType<NV>::template SegmentReturn<D>::ConstType
    jointVelocityFromNvSelector_impl(const Eigen::MatrixBase<D> & a) const
    {
      return SizeDepType<NV>::segment(a.derived(), idx_v(), nv());
    }

    // Non-const access
    template<typename D>
    typename SizeDepType<NV>::template SegmentReturn<D>::Type
    jointVelocityFromNvSelector(Eigen::MatrixBase<D> & a) const
    {
      return derived().jointVelocityFromNvSelector_impl(a.derived());
    }

    template<typename D>
    typename SizeDepType<NV>::template SegmentReturn<D>::Type
    jointVelocityFromNvSelector_impl(Eigen::MatrixBase<D> & a) const
    {
      return SizeDepType<NV>::segment(a.derived(), idx_v(), nv());
    }

    /* Acces to dedicated columns in a ForceSet or MotionSet matrix.*/
    // Const access
    template<typename D>
    typename SizeDepType<NV>::template ColsReturn<D>::ConstType
    jointVelCols(const Eigen::MatrixBase<D> & A) const
    {
      return derived().jointVelCols_impl(A.derived());
    }

    template<typename D>
    typename SizeDepType<NJ>::template ColsReturn<D>::ConstType
    jointJacCols(const Eigen::MatrixBase<D> & A) const
    {
      return derived().jointJacCols_impl(A.derived());
    }

    template<typename D>
    typename SizeDepType<NV>::template ColsReturn<D>::ConstType
    jointVelCols_impl(const Eigen::MatrixBase<D> & A) const
    {
      return SizeDepType<NV>::middleCols(A.derived(), idx_v(), nv());
    }

    template<typename D>
    typename SizeDepType<NJ>::template ColsReturn<D>::ConstType
    jointJacCols_impl(const Eigen::MatrixBase<D> & A) const
    {
      return SizeDepType<NJ>::middleCols(A.derived(), idx_j(), nj());
    }

    // Non-const access
    // TODO rename Jac/Vel into Full/Red (for full system and reduced system ?)
    template<typename D>
    typename SizeDepType<NV>::template ColsReturn<D>::Type
    jointVelCols(Eigen::MatrixBase<D> & A) const
    {
      return derived().jointVelCols_impl(A.derived());
    }

    template<typename D>
    typename SizeDepType<NJ>::template ColsReturn<D>::Type
    jointJacCols(Eigen::MatrixBase<D> & A) const
    {
      return derived().jointJacCols_impl(A.derived());
    }

    template<typename D>
    typename SizeDepType<NV>::template ColsReturn<D>::Type
    jointVelCols_impl(Eigen::MatrixBase<D> & A) const
    {
      return SizeDepType<NV>::middleCols(A.derived(), idx_v(), nv());
    }

    template<typename D>
    typename SizeDepType<NJ>::template ColsReturn<D>::Type
    jointJacCols_impl(Eigen::MatrixBase<D> & A) const
    {
      return SizeDepType<NJ>::middleCols(A.derived(), idx_j(), nj());
    }

    /* Acces to dedicated rows in a matrix.*/
    // Const access
    template<typename D>
    typename SizeDepType<NV>::template RowsReturn<D>::ConstType
    jointVelRows(const Eigen::MatrixBase<D> & A) const
    {
      return derived().jointVelRows_impl(A.derived());
    }

    template<typename D>
    typename SizeDepType<NJ>::template RowsReturn<D>::ConstType
    jointJacRows(const Eigen::MatrixBase<D> & A) const
    {
      return derived().jointJacRows_impl(A.derived());
    }

    template<typename D>
    typename SizeDepType<NV>::template RowsReturn<D>::ConstType
    jointVelRows_impl(const Eigen::MatrixBase<D> & A) const
    {
      return SizeDepType<NV>::middleRows(A.derived(), idx_v(), nv());
    }

    template<typename D>
    typename SizeDepType<NJ>::template RowsReturn<D>::ConstType
    jointJacRows_impl(const Eigen::MatrixBase<D> & A) const
    {
      return SizeDepType<NJ>::middleRows(A.derived(), idx_j(), nj());
    }

    // Non-const access
    template<typename D>
    typename SizeDepType<NV>::template RowsReturn<D>::Type
    jointVelRows(Eigen::MatrixBase<D> & A) const
    {
      return derived().jointVelRows_impl(A.derived());
    }

    template<typename D>
    typename SizeDepType<NJ>::template RowsReturn<D>::Type
    jointJacRows(Eigen::MatrixBase<D> & A) const
    {
      return derived().jointJacRows_impl(A.derived());
    }

    template<typename D>
    typename SizeDepType<NV>::template RowsReturn<D>::Type
    jointVelRows_impl(Eigen::MatrixBase<D> & A) const
    {
      return SizeDepType<NV>::middleRows(A.derived(), idx_v(), nv());
    }

    template<typename D>
    typename SizeDepType<NJ>::template RowsReturn<D>::Type
    jointJacRows_impl(Eigen::MatrixBase<D> & A) const
    {
      return SizeDepType<NJ>::middleRows(A.derived(), idx_j(), nj());
    }

    /// \brief Returns a block of dimension nv()xnv() located at position idx_v(),idx_v() in the
    /// matrix Mat
    // Const access
    template<typename D>
    typename SizeDepType<NV>::template BlockReturn<D>::ConstType
    jointVelBlock(const Eigen::MatrixBase<D> & Mat) const
    {
      return derived().jointVelBlock_impl(Mat.derived());
    }

    template<typename D>
    typename SizeDepType<NJ>::template BlockReturn<D>::ConstType
    jointJacBlock(const Eigen::MatrixBase<D> & Mat) const
    {
      return derived().jointJacBlock_impl(Mat.derived());
    }

    template<typename D>
    typename SizeDepType<NV>::template BlockReturn<D>::ConstType
    jointVelBlock_impl(const Eigen::MatrixBase<D> & Mat) const
    {
      return SizeDepType<NV>::block(Mat.derived(), idx_v(), idx_v(), nv(), nv());
    }

    template<typename D>
    typename SizeDepType<NJ>::template BlockReturn<D>::ConstType
    jointJacBlock_impl(const Eigen::MatrixBase<D> & Mat) const
    {
      return SizeDepType<NJ>::block(Mat.derived(), idx_j(), idx_j(), nj(), nj());
    }

    // Non-const access
    template<typename D>
    typename SizeDepType<NV>::template BlockReturn<D>::Type
    jointVelBlock(Eigen::MatrixBase<D> & Mat) const
    {
      return derived().jointVelBlock_impl(Mat.derived());
    }

    template<typename D>
    typename SizeDepType<NJ>::template BlockReturn<D>::Type
    jointJacBlock(Eigen::MatrixBase<D> & Mat) const
    {
      return derived().jointJacBlock_impl(Mat.derived());
    }

    template<typename D>
    typename SizeDepType<NV>::template BlockReturn<D>::Type
    jointVelBlock_impl(Eigen::MatrixBase<D> & Mat) const
    {
      return SizeDepType<NV>::block(Mat.derived(), idx_v(), idx_v(), nv(), nv());
    }

    template<typename D>
    typename SizeDepType<NJ>::template BlockReturn<D>::Type
    jointJacBlock_impl(Eigen::MatrixBase<D> & Mat) const
    {
      return SizeDepType<NJ>::block(Mat.derived(), idx_j(), idx_j(), nj(), nj());
    }

  protected:
    /// Default constructor: protected.
    ///
    /// Prevent the construction of stand-alone JointModelBase.
    inline JointModelBase()
    : i_id(std::numeric_limits<JointIndex>::max())
    , i_q(-1)
    , i_v(-1)
    , i_j(-1)
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
      i_j = clone.i_j;
      return *this;
    }

    // data
    JointIndex i_id; // ID of the joint in the multibody list.
    int i_q;         // Index of the joint configuration in the joint configuration vector.
    int i_v;         // Index of the joint velocity in the joint velocity vector.
    int i_j;         // Index of the joint jacobian in the joint jacobian matrix.

  }; // struct JointModelBase

} // namespace pinocchio

#endif // ifndef __pinocchio_multibody_joint_model_base_hpp__
