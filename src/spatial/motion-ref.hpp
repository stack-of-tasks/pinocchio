//
// Copyright (c) 2017-2019 CNRS INRIA
//

#ifndef __pinocchio_motion_ref_hpp__
#define __pinocchio_motion_ref_hpp__

namespace pinocchio
{
  
  template<typename Vector6ArgType>
  struct traits< MotionRef<Vector6ArgType> >
  {
    typedef typename Vector6ArgType::Scalar Scalar;
    typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(Vector6ArgType) Vector6;
    enum {
      LINEAR = 0,
      ANGULAR = 3,
      Options = Vector6::Options
    };
    typedef Eigen::Matrix<Scalar,3,1,Options> Vector3;
    typedef Eigen::Matrix<Scalar,6,6,Options> Matrix6;
    typedef Matrix6 ActionMatrixType;
    typedef typename Vector6ArgType::template FixedSegmentReturnType<3>::Type LinearType;
    typedef typename Vector6ArgType::template FixedSegmentReturnType<3>::Type AngularType;
    typedef typename Vector6ArgType::template ConstFixedSegmentReturnType<3>::Type ConstLinearType;
    typedef typename Vector6ArgType::template ConstFixedSegmentReturnType<3>::Type ConstAngularType;
    typedef MotionTpl<Scalar,Options> MotionPlain;
    typedef MotionPlain PlainReturnType;
    typedef typename PINOCCHIO_EIGEN_REF_TYPE(Vector6ArgType) DataRefType;
    typedef DataRefType ToVectorReturnType;
    typedef typename PINOCCHIO_EIGEN_REF_CONST_TYPE(Vector6ArgType) ConstDataRefType;
    typedef ConstDataRefType ToVectorConstReturnType;
    typedef MotionRef<Vector6ArgType> MotionRefType;

  }; // traits MotionRef
  
  template<typename Vector6ArgType>
  struct SE3GroupAction< MotionRef<Vector6ArgType> >
  {
    typedef typename traits< MotionRef<Vector6ArgType> >::MotionPlain ReturnType;
  };

  template<typename Vector6ArgType, typename MotionDerived>
  struct MotionAlgebraAction< MotionRef<Vector6ArgType>, MotionDerived >
  {
    typedef typename traits< MotionRef<Vector6ArgType> >::MotionPlain ReturnType;
  };
  
  namespace internal
  {
    template<typename Vector6ArgType, typename Scalar>
    struct RHSScalarMultiplication< MotionRef<Vector6ArgType>, Scalar >
    {
      typedef typename pinocchio::traits< MotionRef<Vector6ArgType> >::MotionPlain ReturnType;
    };
    
    template<typename Vector6ArgType, typename Scalar>
    struct LHSScalarMultiplication< MotionRef<Vector6ArgType>, Scalar >
    {
      typedef typename traits< MotionRef<Vector6ArgType> >::MotionPlain ReturnType;
    };
  }
  
  template<typename Vector6ArgType>
  class MotionRef : public MotionDense< MotionRef<Vector6ArgType> >
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef MotionDense<MotionRef> Base;
    typedef typename traits<MotionRef>::DataRefType DataRefType;
    MOTION_TYPEDEF_TPL(MotionRef);

    using Base::operator=;
    using Base::linear;
    using Base::angular;
    
    using Base::__plus__;
    using Base::__opposite__;
    using Base::__minus__;
    using Base::__pequ__;
    using Base::__mequ__;
    using Base::__mult__;
    
    MotionRef(typename PINOCCHIO_EIGEN_REF_TYPE(Vector6ArgType) v_like)
    : m_ref(v_like)
    {
      EIGEN_STATIC_ASSERT(Vector6ArgType::ColsAtCompileTime == 1,
                          YOU_TRIED_CALLING_A_VECTOR_METHOD_ON_A_MATRIX);
      assert(v_like.size() == 6);
    }
    
    ToVectorConstReturnType toVector_impl() const { return m_ref; }
    ToVectorReturnType toVector_impl() { return m_ref; }
    
    // Getters
    ConstAngularType angular_impl() const { return ConstAngularType(m_ref.derived(),ANGULAR); }
    ConstLinearType linear_impl()  const { return ConstLinearType(m_ref.derived(),LINEAR); }
    AngularType angular_impl() { return m_ref.template segment<3> (ANGULAR); }
    LinearType linear_impl()  { return m_ref.template segment<3> (LINEAR); }
    
    template<typename V3>
    void angular_impl(const Eigen::MatrixBase<V3> & w)
    {
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(V3,3);
      angular_impl()=w;
    }
    
    template<typename V3>
    void linear_impl(const Eigen::MatrixBase<V3> & v)
    {
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(V3,3);
      linear_impl()=v;
    }
    
    // Specific operators for MotionTpl and MotionRef
    template<typename S1, int O1>
    MotionPlain __plus__(const MotionTpl<S1,O1> & v) const
    { return MotionPlain(m_ref+v.toVector()); }
    
    template<typename Vector6Like>
    MotionPlain __plus__(const MotionRef<Vector6ArgType> & v) const
    { return MotionPlain(m_ref+v.toVector()); }
    
    template<typename S1, int O1>
    MotionPlain __minus__(const MotionTpl<S1,O1> & v) const
    { return MotionPlain(m_ref-v.toVector()); }
    
    template<typename Vector6Like>
    MotionPlain __minus__(const MotionRef<Vector6ArgType> & v) const
    { return MotionPlain(m_ref-v.toVector()); }
    
    template<typename S1, int O1>
    MotionRef & __pequ__(const MotionTpl<S1,O1> & v)
    { m_ref += v.toVector(); return *this; }
    
    template<typename Vector6Like>
    MotionRef & __pequ__(const MotionRef<Vector6ArgType> & v)
    { m_ref += v.toVector(); return *this; }
    
    template<typename S1, int O1>
    MotionRef & __mequ__(const MotionTpl<S1,O1> & v)
    { m_ref -= v.toVector(); return *this; }
    
    template<typename Vector6Like>
    MotionRef & __mequ__(const MotionRef<Vector6ArgType> & v)
    { m_ref -= v.toVector(); return *this; }
    
    template<typename OtherScalar>
    MotionPlain __mult__(const OtherScalar & alpha) const
    { return MotionPlain(alpha*m_ref); }
    
    MotionRef & ref() { return *this; }
    
    inline PlainReturnType plain() const { return PlainReturnType(m_ref); }

  protected:
    DataRefType m_ref;

  }; // class MotionRef<Vector6Like>

  template<typename Vector6ArgType>
  struct traits< MotionRef<const Vector6ArgType> >
  {
    typedef typename Vector6ArgType::Scalar Scalar;
    typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(Vector6ArgType) Vector6;
    enum {
      LINEAR = 0,
      ANGULAR = 3,
      Options = Vector6::Options
    };
    typedef Eigen::Matrix<Scalar,3,1,Options> Vector3;
    typedef Eigen::Matrix<Scalar,6,6,Options> Matrix6;
    typedef Matrix6 ActionMatrixType;
    typedef typename Vector6ArgType::template ConstFixedSegmentReturnType<3>::Type ConstLinearType;
    typedef typename Vector6ArgType::template ConstFixedSegmentReturnType<3>::Type ConstAngularType;
    typedef ConstLinearType LinearType;
    typedef ConstAngularType AngularType;
    typedef MotionTpl<Scalar,Options> MotionPlain;
    typedef MotionPlain PlainReturnType;
    typedef typename PINOCCHIO_EIGEN_REF_CONST_TYPE(Vector6ArgType) ConstDataRefType;
    typedef ConstDataRefType ToVectorConstReturnType;
    typedef ConstDataRefType DataRefType;
    typedef DataRefType ToVectorReturnType;
    typedef MotionRef<const Vector6ArgType> MotionRefType;
  
  }; // traits MotionRef<const Vector6ArgType>
  
  template<typename Vector6ArgType>
  class MotionRef<const Vector6ArgType>
  : public MotionDense< MotionRef<const Vector6ArgType> >
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef MotionDense<MotionRef> Base;
    typedef typename traits<MotionRef>::DataRefType DataRefType;
    MOTION_TYPEDEF_TPL(MotionRef);
    
    using Base::operator=;
    using Base::linear;
    using Base::angular;
    
    using Base::__plus__;
    using Base::__opposite__;
    using Base::__minus__;
    using Base::__mult__;
    
    MotionRef(typename PINOCCHIO_EIGEN_REF_CONST_TYPE(Vector6ArgType) v_like)
    : m_ref(v_like)
    {
      EIGEN_STATIC_ASSERT(Vector6ArgType::ColsAtCompileTime == 1,
                          YOU_TRIED_CALLING_A_VECTOR_METHOD_ON_A_MATRIX);
      assert(v_like.size() == 6);
    }
    
    ToVectorConstReturnType toVector_impl() const { return m_ref; }
    
    // Getters
    ConstAngularType angular_impl() const { return ConstAngularType(m_ref.derived(),ANGULAR); }
    ConstLinearType linear_impl()  const { return ConstLinearType(m_ref.derived(),LINEAR); }
    
    // Specific operators for MotionTpl and MotionRef
    template<typename S1, int O1>
    MotionPlain __plus__(const MotionTpl<S1,O1> & v) const
    { return MotionPlain(m_ref+v.toVector()); }
    
    template<typename Vector6Like>
    MotionPlain __plus__(const MotionRef<Vector6ArgType> & v) const
    { return MotionPlain(m_ref+v.toVector()); }
    
    template<typename S1, int O1>
    MotionPlain __minus__(const MotionTpl<S1,O1> & v) const
    { return MotionPlain(m_ref-v.toVector()); }
    
    template<typename Vector6Like>
    MotionPlain __minus__(const MotionRef<Vector6ArgType> & v) const
    { return MotionPlain(m_ref-v.toVector()); }
    
    template<typename OtherScalar>
    MotionPlain __mult__(const OtherScalar & alpha) const
    { return MotionPlain(alpha*m_ref); }
    
    const MotionRef & ref() const { return *this; }

    inline PlainReturnType plain() const { return PlainReturnType(m_ref); }
    
  protected:
    DataRefType m_ref;
    
  }; // class MotionRef<const Vector6Like>
  
} // namespace pinocchio

#endif // ifndef __pinocchio_motion_ref_hpp__
