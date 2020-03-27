//
// Copyright (c) 2019 INRIA
//

#ifndef __pinocchio_joint_mimic_hpp__
#define __pinocchio_joint_mimic_hpp__

#include "pinocchio/macros.hpp"
#include "pinocchio/multibody/joint/joint-base.hpp"

namespace pinocchio
{
  
  template<class Constraint> struct ScaledConstraint;
  
  template<class Constraint>
  struct traits< ScaledConstraint<Constraint> >
  {
    typedef typename traits<Constraint>::Scalar Scalar;
    enum { Options = traits<Constraint>::Options };
    enum {
      LINEAR = traits<Constraint>::LINEAR,
      ANGULAR = traits<Constraint>::ANGULAR
    };
    typedef typename traits<Constraint>::JointMotion JointMotion;
    typedef typename traits<Constraint>::JointForce JointForce;
    typedef typename traits<Constraint>::DenseBase DenseBase;
    typedef typename traits<Constraint>::MatrixReturnType MatrixReturnType;
    typedef typename traits<Constraint>::ConstMatrixReturnType ConstMatrixReturnType;
  }; // traits ScaledConstraint
  
  template<class Constraint>
  struct SE3GroupAction< ScaledConstraint<Constraint> >
  { typedef typename SE3GroupAction<Constraint>::ReturnType ReturnType; };
  
  template<class Constraint, typename MotionDerived>
  struct MotionAlgebraAction< ScaledConstraint<Constraint>, MotionDerived >
  { typedef typename MotionAlgebraAction<Constraint,MotionDerived>::ReturnType ReturnType; };
  
  template<class Constraint, typename ForceDerived>
  struct ConstraintForceOp< ScaledConstraint<Constraint>, ForceDerived>
  {
    typedef typename Constraint::Scalar Scalar;
    typedef typename ConstraintForceOp<Constraint,ForceDerived>::ReturnType OriginalReturnType;
    
    typedef typename ScalarMatrixProduct<Scalar,OriginalReturnType>::type IdealReturnType;
    typedef Eigen::Matrix<Scalar,IdealReturnType::RowsAtCompileTime,IdealReturnType::ColsAtCompileTime,Constraint::Options> ReturnType;
  };
  
  template<class Constraint, typename ForceSet>
  struct ConstraintForceSetOp< ScaledConstraint<Constraint>, ForceSet>
  {
    typedef typename Constraint::Scalar Scalar;
    typedef typename ConstraintForceSetOp<Constraint,ForceSet>::ReturnType OriginalReturnType;
    typedef typename ScalarMatrixProduct<Scalar,OriginalReturnType>::type IdealReturnType;
    typedef Eigen::Matrix<Scalar,Constraint::NV,ForceSet::ColsAtCompileTime,Constraint::Options | Eigen::RowMajor> ReturnType;
  };
    
  template<class Constraint>
  struct ScaledConstraint
  : ConstraintBase< ScaledConstraint<Constraint> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    PINOCCHIO_CONSTRAINT_TYPEDEF_TPL(ScaledConstraint)
    enum { NV = Constraint::NV };
    typedef ConstraintBase<ScaledConstraint> Base;
    using Base::nv;
    
    typedef typename SE3GroupAction<Constraint>::ReturnType SE3ActionReturnType;
    
    ScaledConstraint() {}
    
    explicit ScaledConstraint(const Scalar & scaling_factor)
    : m_scaling_factor(scaling_factor)
    {}
    
    ScaledConstraint(const Constraint & constraint,
                     const Scalar & scaling_factor)
    : m_constraint(constraint)
    , m_scaling_factor(scaling_factor)
    {}

    ScaledConstraint(const ScaledConstraint & other)
    : m_constraint(other.m_constraint)
    , m_scaling_factor(other.m_scaling_factor)
    {}

    ScaledConstraint & operator=(const ScaledConstraint & other)
    {
      m_constraint = other.m_constraint;
      m_scaling_factor = other.m_scaling_factor;
      return *this;
    }
    
    template<typename VectorLike>
    JointMotion __mult__(const Eigen::MatrixBase<VectorLike> & v) const
    {
      assert(v.size() == nv());
      JointMotion jm = m_constraint * v;
      return jm * m_scaling_factor;
    }
    
    template<typename S1, int O1>
    SE3ActionReturnType
    se3Action(const SE3Tpl<S1,O1> & m) const
    {
      SE3ActionReturnType res = m_constraint.se3Action(m);
      return m_scaling_factor * res;
    }
    
    template<typename S1, int O1>
    SE3ActionReturnType
    se3ActionInverse(const SE3Tpl<S1,O1> & m) const
    {
      SE3ActionReturnType res = m_constraint.se3ActionInverse(m);
      return m_scaling_factor * res;
    }
    
    int nv_impl() const { return m_constraint.nv(); }
    
    struct TransposeConst
    {
      const ScaledConstraint & ref;
      TransposeConst(const ScaledConstraint & ref) : ref(ref) {}
      
      template<typename Derived>
      typename ConstraintForceOp<ScaledConstraint,Derived>::ReturnType
      operator*(const ForceDense<Derived> & f) const
      {
        // TODO: I don't know why, but we should a dense a return type, otherwise it failes at the evaluation level;
        typedef typename ConstraintForceOp<ScaledConstraint,Derived>::ReturnType ReturnType;
        return ReturnType(ref.m_scaling_factor * (ref.m_constraint.transpose() * f));
      }
      
      /// [CRBA]  MatrixBase operator* (Constraint::Transpose S, ForceSet::Block)
      template<typename Derived>
      typename ConstraintForceSetOp<ScaledConstraint,Derived>::ReturnType
      operator*(const Eigen::MatrixBase<Derived> & F) const
      {
        typedef typename ConstraintForceSetOp<ScaledConstraint,Derived>::ReturnType ReturnType;
        return ReturnType(ref.m_scaling_factor * (ref.m_constraint.transpose() * F));
      }
      
    }; // struct TransposeConst
    
    TransposeConst transpose() const { return TransposeConst(*this); }
    
    DenseBase matrix_impl() const
    {
      DenseBase S = m_scaling_factor * m_constraint.matrix();
      return S;
    }
    
    template<typename MotionDerived>
    typename MotionAlgebraAction<ScaledConstraint,MotionDerived>::ReturnType
    motionAction(const MotionDense<MotionDerived> & m) const
    {
      typedef typename MotionAlgebraAction<ScaledConstraint,MotionDerived>::ReturnType ReturnType;
      ReturnType res = m_scaling_factor * m_constraint.motionAction(m);
      return res;
    }
    
    inline const Scalar & scaling() const { return m_scaling_factor; }
    inline Scalar & scaling() { return m_scaling_factor; }
    
    inline const Constraint & constraint() const { return m_constraint; }
    inline Constraint & constraint() { return m_constraint; }
    
    bool isEqual(const ScaledConstraint & other) const
    {
      return m_constraint == other.m_constraint
      && m_scaling_factor == other.m_scaling_factor;
    }
    
  protected:
    
    Constraint m_constraint;
    Scalar m_scaling_factor;
  }; // struct ScaledConstraint
  
  template<typename S1, int O1, typename _Constraint>
  struct MultiplicationOp<InertiaTpl<S1,O1>, ScaledConstraint<_Constraint> >
  {
    typedef InertiaTpl<S1,O1> Inertia;
    typedef ScaledConstraint<_Constraint> Constraint;
    typedef typename Constraint::Scalar Scalar;
    
    typedef typename MultiplicationOp<Inertia,_Constraint>::ReturnType OriginalReturnType;
//    typedef typename ScalarMatrixProduct<Scalar,OriginalReturnType>::type ReturnType;
    typedef OriginalReturnType ReturnType;
  };
  
  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
  namespace impl
  {
    template<typename S1, int O1, typename _Constraint>
    struct LhsMultiplicationOp<InertiaTpl<S1,O1>, ScaledConstraint<_Constraint> >
    {
      typedef InertiaTpl<S1,O1> Inertia;
      typedef ScaledConstraint<_Constraint> Constraint;
      typedef typename MultiplicationOp<Inertia,Constraint>::ReturnType ReturnType;
      
      static inline ReturnType run(const Inertia & Y,
                                   const Constraint & scaled_constraint)
      {
        return scaled_constraint.scaling() * (Y * scaled_constraint.constraint());
      }
    };
  } // namespace impl
  
  template<typename M6Like, typename _Constraint>
  struct MultiplicationOp<Eigen::MatrixBase<M6Like>, ScaledConstraint<_Constraint> >
  {
    typedef typename MultiplicationOp<Inertia,_Constraint>::ReturnType OriginalReturnType;
    typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(OriginalReturnType) ReturnType;
  };
  
  /* [ABA] operator* (Inertia Y,Constraint S) */
  namespace impl
  {
    template<typename M6Like, typename _Constraint>
    struct LhsMultiplicationOp<Eigen::MatrixBase<M6Like>, ScaledConstraint<_Constraint> >
    {
      typedef ScaledConstraint<_Constraint> Constraint;
      typedef typename MultiplicationOp<Eigen::MatrixBase<M6Like>,Constraint>::ReturnType ReturnType;
      
      static inline ReturnType run(const Eigen::MatrixBase<M6Like> & Y,
                                   const Constraint & scaled_constraint)
      {
        return scaled_constraint.scaling() * (Y.derived() * scaled_constraint.constraint());
      }
    };
  } // namespace impl
  
  template<class Joint> struct JointMimic;
  template<class JointModel> struct JointModelMimic;
  template<class JointData> struct JointDataMimic;
  
  template<class Joint>
  struct traits< JointMimic<Joint> >
  {
    enum
    {
      NQ = traits<Joint>::NV,
      NV = traits<Joint>::NQ
    };
    typedef typename traits<Joint>::Scalar Scalar;
    enum { Options = traits<Joint>::Options };
    
    typedef typename traits<Joint>::JointDataDerived JointDataBase;
    typedef typename traits<Joint>::JointModelDerived JointModelBase;
    
    typedef JointDataMimic<JointDataBase> JointDataDerived;
    typedef JointModelMimic<JointModelBase> JointModelDerived;
    
    typedef ScaledConstraint<typename traits<Joint>::Constraint_t> Constraint_t;
    typedef typename traits<Joint>::Transformation_t Transformation_t;
    typedef typename traits<Joint>::Motion_t Motion_t;
    typedef typename traits<Joint>::Bias_t Bias_t;

    // [ABA]
    typedef typename traits<Joint>::U_t U_t;
    typedef typename traits<Joint>::D_t D_t;
    typedef typename traits<Joint>::UD_t UD_t;
    
    PINOCCHIO_JOINT_DATA_BASE_ACCESSOR_DEFAULT_RETURN_TYPE
    
    typedef typename traits<Joint>::ConfigVector_t ConfigVector_t;
    typedef typename traits<Joint>::TangentVector_t TangentVector_t;
  };
  
  template<class Joint>
  struct traits< JointDataMimic<Joint> >
  { typedef JointMimic<typename traits<Joint>::JointDerived> JointDerived; };
  
  template<class Joint>
  struct traits< JointModelMimic<Joint> >
  { typedef JointMimic<typename traits<Joint>::JointDerived> JointDerived; };
  
  template<class JointData>
  struct JointDataMimic
  : public JointDataBase< JointDataMimic<JointData> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef typename traits<JointDataMimic>::JointDerived JointDerived;
    typedef JointDataBase< JointDataMimic<JointData> > Base;
    
    PINOCCHIO_JOINT_DATA_TYPEDEF_TEMPLATE(JointDerived);
    
    JointDataMimic()
    : m_scaling((Scalar)0)
    , m_q_transform(ConfigVector_t::Zero())
    , m_v_transform(TangentVector_t::Zero())
    , S((Scalar)0)
    {}
    
    JointDataMimic(const JointDataBase<JointData> & jdata,
                   const Scalar & scaling)
    : m_jdata_ref(jdata.derived())
    , m_scaling(scaling)
    , S(m_jdata_ref.S,scaling)
    {}
    
    JointDataMimic & operator=(const JointDataMimic & other)
    {
      m_jdata_ref = other.m_jdata_ref;
      m_scaling = other.m_scaling;
      m_q_transform = other.m_q_transform;
      m_v_transform = other.m_v_transform;
      S = Constraint_t(m_jdata_ref.S,other.m_scaling);
      return *this;
    }
    
    bool isEqual(const JointDataMimic & other) const
    {
      return Base::isEqual(other)
      && m_jdata_ref == other.m_jdata_ref
      && m_scaling == other.m_scaling
      && m_q_transform == other.m_q_transform
      && m_v_transform == other.m_v_transform
      ;
    }
    
    static std::string classname()
    {
      return std::string("JointDataMimic<") + JointData::classname() + std::string(">");
    }
    
    std::string shortname() const
    {
      return std::string("JointDataMimic<") + m_jdata_ref.shortname() + std::string(">");
    }
    
    // Accessors
    ConstraintTypeConstRef S_accessor() const { return S; }
    ConstraintTypeRef S_accessor() { return S; }
    
    TansformTypeConstRef M_accessor() const { return m_jdata_ref.M; }
    TansformTypeRef M_accessor() { return m_jdata_ref.M; }
    
    MotionTypeConstRef v_accessor() const { return m_jdata_ref.v; }
    MotionTypeRef v_accessor() { return m_jdata_ref.v; }
    
    BiasTypeConstRef c_accessor() const { return m_jdata_ref.c; }
    BiasTypeRef c_accessor() { return m_jdata_ref.c; }
    
    UTypeConstRef U_accessor() const { return m_jdata_ref.U; }
    UTypeRef U_accessor() { return m_jdata_ref.U; }
    
    DTypeConstRef Dinv_accessor() const { return m_jdata_ref.Dinv; }
    DTypeRef Dinv_accessor() { return m_jdata_ref.Dinv; }
    
    UDTypeConstRef UDinv_accessor() const { return m_jdata_ref.UDinv; }
    UDTypeRef UDinv_accessor() { return m_jdata_ref.UDinv; }
    
    template<class JointModel>
    friend struct JointModelMimic;
    
    const JointData & jdata() const { return m_jdata_ref; }
    JointData & jdata() { return m_jdata_ref; }
    
    const Scalar & scaling() const { return m_scaling; }
    Scalar & scaling() { return m_scaling; }
    
    ConfigVector_t & jointConfiguration() { return m_q_transform; }
    const ConfigVector_t & jointConfiguration() const { return m_q_transform; }
    
    TangentVector_t & jointVelocity() { return m_v_transform; }
    const TangentVector_t & jointVelocity() const { return m_v_transform; }
    
  protected:
    
    JointData m_jdata_ref;
    Scalar m_scaling;
    
    /// \brief Transform configuration vector
    ConfigVector_t m_q_transform;
    /// \brief Transform velocity vector.
    TangentVector_t m_v_transform;
    
  public:
    
    // data
    Constraint_t S;
    
  }; // struct JointDataMimic
  
  template<typename NewScalar, typename JointModel>
  struct CastType< NewScalar, JointModelMimic<JointModel> >
  {
    typedef typename CastType<NewScalar,JointModel>::type JointModelNewType;
    typedef JointModelMimic<JointModelNewType> type;
  };
  
  template<class JointModel>
  struct JointModelMimic
  : public JointModelBase< JointModelMimic<JointModel> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef typename traits<JointModelMimic>::JointDerived JointDerived;
    
    PINOCCHIO_JOINT_TYPEDEF_TEMPLATE(JointDerived);
    
    typedef JointModelBase<JointModelMimic> Base;
    using Base::id;
    using Base::idx_q;
    using Base::idx_v;
    using Base::nq;
    using Base::nv;
    using Base::setIndexes;
    
    JointModelMimic()
    {}
    
    JointModelMimic(const JointModelBase<JointModel> & jmodel,
                    const Scalar & scaling,
                    const Scalar & offset)
    : m_jmodel_ref(jmodel.derived())
    , m_scaling(scaling)
    , m_offset(offset)
    {}
    
    Base & base() { return *static_cast<Base*>(this); }
    const Base & base() const { return *static_cast<const Base*>(this); }
    
    inline int nq_impl() const { return 0; }
    inline int nv_impl() const { return 0; }
    
    inline int idx_q_impl() const { return m_jmodel_ref.idx_q(); }
    inline int idx_v_impl() const { return m_jmodel_ref.idx_v(); }
    
    void setIndexes_impl(JointIndex id, int /*q*/, int /*v*/)
    {
      Base::i_id = id; // Only the id of the joint in the model is different.
      Base::i_q = m_jmodel_ref.idx_q();
      Base::i_v = m_jmodel_ref.idx_v();
    }
    
    JointDataDerived createData() const
    {
      return JointDataDerived(m_jmodel_ref.createData(),scaling());
    }
    
    template<typename ConfigVector>
    EIGEN_DONT_INLINE
    void calc(JointDataDerived & jdata,
              const typename Eigen::MatrixBase<ConfigVector> & qs) const
    {
      typedef typename ConfigVectorAffineTransform<JointDerived>::Type AffineTransform;
      
      AffineTransform::run(qs.head(m_jmodel_ref.nq()),
                           m_scaling,m_offset,jdata.m_q_transform);
      m_jmodel_ref.calc(jdata.m_jdata_ref,jdata.m_q_transform);
    }
    
    template<typename ConfigVector, typename TangentVector>
    EIGEN_DONT_INLINE
    void calc(JointDataDerived & jdata,
              const typename Eigen::MatrixBase<ConfigVector> & qs,
              const typename Eigen::MatrixBase<TangentVector> & vs) const
    {
      typedef typename ConfigVectorAffineTransform<JointDerived>::Type AffineTransform;
      
      AffineTransform::run(qs.head(m_jmodel_ref.nq()),
                           m_scaling,m_offset,jdata.m_q_transform);
      jdata.m_v_transform = m_scaling * vs.head(m_jmodel_ref.nv());
      m_jmodel_ref.calc(jdata.m_jdata_ref,
                        jdata.m_q_transform,
                        jdata.m_v_transform);
    }
    
    template<typename Matrix6Like>
    void calc_aba(JointDataDerived & data,
                  const Eigen::MatrixBase<Matrix6Like> & I,
                  const bool update_I) const
    {
      // TODO: fixme
      m_jmodel_ref.calc_aba(data.m_jdata_ref,
                            PINOCCHIO_EIGEN_CONST_CAST(Matrix6Like,I),
                            update_I);
    }
    
    static std::string classname()
    {
      return std::string("JointModelMimic<") + JointModel::classname() + std::string(">");;
    }
    
    std::string shortname() const
    {
      return std::string("JointModelMimic<") + m_jmodel_ref.shortname() + std::string(">");
    }
    
    /// \returns An expression of *this with the Scalar type casted to NewScalar.
    template<typename NewScalar>
    typename CastType<NewScalar,JointModelMimic>::type cast() const
    {
      typedef typename CastType<NewScalar,JointModelMimic>::type ReturnType;
      
      ReturnType res(m_jmodel_ref.template cast<NewScalar>(),
                     (NewScalar)m_scaling,
                     (NewScalar)m_offset);
      res.setIndexes(id(),idx_q(),idx_v());
      return res;
    }
    
    const JointModel & jmodel() const { return m_jmodel_ref; }
    JointModel & jmodel() { return m_jmodel_ref; }
    
    const Scalar & scaling() const { return m_scaling; }
    Scalar & scaling() { return m_scaling; }
    
    const Scalar & offset() const { return m_offset; }
    Scalar & offset() { return m_offset; }
    
  protected:
    
    // data
    JointModel m_jmodel_ref;
    Scalar m_scaling, m_offset;
    
  public:
    
    /* Acces to dedicated segment in robot config space.  */
    // Const access
    template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::ConstType
    jointConfigSelector_impl(const Eigen::MatrixBase<D> & a) const
    {
      return SizeDepType<NQ>::segment(a.derived(),
                                      m_jmodel_ref.idx_q(),
                                      m_jmodel_ref.nq());
    }
    
    // Non-const access
    template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::Type
    jointConfigSelector_impl(Eigen::MatrixBase<D> & a) const
    {
      return SizeDepType<NQ>::segment(a.derived(),
                                      m_jmodel_ref.idx_q(),
                                      m_jmodel_ref.nq());
    }
    
    /* Acces to dedicated segment in robot config velocity space.  */
    // Const access
    template<typename D>
    typename SizeDepType<NV>::template SegmentReturn<D>::ConstType
    jointVelocitySelector_impl(const Eigen::MatrixBase<D> & a) const
    {
      return SizeDepType<NV>::segment(a.derived(),
                                      m_jmodel_ref.idx_v(),
                                      m_jmodel_ref.nv());
    }
    
    // Non-const access
    template<typename D>
    typename SizeDepType<NV>::template SegmentReturn<D>::Type
    jointVelocitySelector_impl(Eigen::MatrixBase<D> & a) const
    {
      return SizeDepType<NV>::segment(a.derived(),
                                      m_jmodel_ref.idx_v(),
                                      m_jmodel_ref.nv());
    }
    
    /* Acces to dedicated columns in a ForceSet or MotionSet matrix.*/
    // Const access
    template<typename D>
    typename SizeDepType<NV>::template ColsReturn<D>::ConstType
    jointCols_impl(const Eigen::MatrixBase<D> & A) const
    {
      return SizeDepType<NV>::middleCols(A.derived(),
                                         m_jmodel_ref.idx_v(),
                                         m_jmodel_ref.nv());
    }
    
    // Non-const access
    template<typename D>
    typename SizeDepType<NV>::template ColsReturn<D>::Type
    jointCols_impl(Eigen::MatrixBase<D> & A) const
    {
      return SizeDepType<NV>::middleCols(A.derived(),
                                         m_jmodel_ref.idx_v(),
                                         m_jmodel_ref.nv());
    }
    
    /* Acces to dedicated rows in a matrix.*/
    // Const access
    template<typename D>
    typename SizeDepType<NV>::template RowsReturn<D>::ConstType
    jointRows_impl(const Eigen::MatrixBase<D> & A) const
    {
      return SizeDepType<NV>::middleRows(A.derived(),
                                         m_jmodel_ref.idx_v(),
                                         m_jmodel_ref.nv());
    }
    
    // Non-const access
    template<typename D>
    typename SizeDepType<NV>::template RowsReturn<D>::Type
    jointRows_impl(Eigen::MatrixBase<D> & A) const
    {
      return SizeDepType<NV>::middleRows(A.derived(),
                                         m_jmodel_ref.idx_v(),
                                         m_jmodel_ref.nv());
    }
    
    /// \brief Returns a block of dimension nv()xnv() located at position idx_v(),idx_v() in the matrix Mat
    // Const access
    template<typename D>
    typename SizeDepType<NV>::template BlockReturn<D>::ConstType
    jointBlock_impl(const Eigen::MatrixBase<D> & Mat) const
    {
      return SizeDepType<NV>::block(Mat.derived(),
                                    m_jmodel_ref.idx_v(),m_jmodel_ref.idx_v(),
                                    m_jmodel_ref.nv(),m_jmodel_ref.nv());
    }
    
    // Non-const access
    template<typename D>
    typename SizeDepType<NV>::template BlockReturn<D>::Type
    jointBlock_impl(Eigen::MatrixBase<D> & Mat) const
    {
      return SizeDepType<NV>::block(Mat.derived(),
                                    m_jmodel_ref.idx_v(),m_jmodel_ref.idx_v(),
                                    m_jmodel_ref.nv(),m_jmodel_ref.nv());
    }

  }; // struct JointModelMimic
  
} // namespace pinocchio

#endif // ifndef __pinocchio_joint_mimic_hpp__
