//
// Copyright (c) 2017-2019 CNRS INRIA
//

#ifndef __pinocchio_force_ref_hpp__
#define __pinocchio_force_ref_hpp__

namespace pinocchio
{
  
  template<typename Vector6ArgType>
  struct traits< ForceRef<Vector6ArgType> >
  {
    typedef typename Vector6ArgType::Scalar Scalar;
    typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(Vector6ArgType) Vector6;
    enum {
      LINEAR = 0,
      ANGULAR = 3,
      Options = Vector6::Options
    };
    typedef Eigen::Matrix<Scalar,3,1,Options> Vector3;
    typedef Eigen::Matrix<Scalar,4,4,Options> Matrix4;
    typedef Eigen::Matrix<Scalar,6,6,Options> Matrix6;
    typedef Matrix6 ActionMatrixType;
    typedef Matrix4 HomogeneousMatrixType;
    typedef typename Vector6ArgType::template FixedSegmentReturnType<3>::Type LinearType;
    typedef typename Vector6ArgType::template FixedSegmentReturnType<3>::Type AngularType;
    typedef typename Vector6ArgType::template ConstFixedSegmentReturnType<3>::Type ConstLinearType;
    typedef typename Vector6ArgType::template ConstFixedSegmentReturnType<3>::Type ConstAngularType;
    typedef ForceTpl<Scalar,Options> ForcePlain;
    typedef typename PINOCCHIO_EIGEN_REF_TYPE(Vector6ArgType) DataRefType;
    typedef DataRefType ToVectorReturnType;
    typedef typename PINOCCHIO_EIGEN_REF_CONST_TYPE(Vector6ArgType) ConstDataRefType;
    typedef ConstDataRefType ToVectorConstReturnType;
    typedef ForceRef<Vector6ArgType> ForceRefType;
    
  }; // traits ForceRef
  
    template<typename Vector6ArgType>
    struct SE3GroupAction< ForceRef<Vector6ArgType> >
    {
      typedef typename traits< ForceRef<Vector6ArgType> >::ForcePlain ReturnType;
    };
    
    template<typename Vector6ArgType, typename MotionDerived>
    struct MotionAlgebraAction< ForceRef<Vector6ArgType>, MotionDerived >
    {
      typedef typename traits< ForceRef<Vector6ArgType> >::ForcePlain ReturnType;
    };

  template<typename Vector6ArgType>
  class ForceRef : public ForceDense< ForceRef<Vector6ArgType> >
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef ForceDense<ForceRef> Base;
    typedef typename traits<ForceRef>::DataRefType DataRefType;
    FORCE_TYPEDEF_TPL(ForceRef);
    
    using Base::operator=;
    using Base::operator==;
    using Base::operator!=;
    
    /// \brief Default constructor from a 6 dimensional vector.
    ForceRef(typename PINOCCHIO_EIGEN_REF_TYPE(Vector6ArgType) f_like)
    : m_ref(f_like)
    {
      EIGEN_STATIC_ASSERT(Vector6ArgType::ColsAtCompileTime == 1,
                          YOU_TRIED_CALLING_A_VECTOR_METHOD_ON_A_MATRIX);
      assert(f_like.size() == 6);
    }
    
    /// \brief Copy constructor from another ForceRef.
    ForceRef(const ForceRef & other)
    : m_ref(other.m_ref)
    {}
    
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
    
    ForceRef & ref() { return *this; }
    
  protected:
    DataRefType m_ref;
    
  }; // class ForceRef<Vector6Like>

  template<typename Vector6ArgType>
  struct traits< ForceRef<const Vector6ArgType> >
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
    typedef ForceTpl<Scalar,Options> ForcePlain;
    typedef ForcePlain PlainReturnType;
    typedef typename PINOCCHIO_EIGEN_REF_CONST_TYPE(Vector6ArgType) ConstDataRefType;
    typedef ConstDataRefType ToVectorConstReturnType;
    typedef ConstDataRefType DataRefType;
    typedef DataRefType ToVectorReturnType;
    typedef ForceRef<const Vector6ArgType> ForceRefType;
    
  }; // traits ForceRef<const Vector6ArgType>
  
  template<typename Vector6ArgType>
  class ForceRef<const Vector6ArgType>
  : public ForceDense< ForceRef<const Vector6ArgType> >
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef ForceDense<ForceRef> Base;
    typedef typename traits<ForceRef>::DataRefType DataRefType;
    FORCE_TYPEDEF_TPL(ForceRef);
    
    ForceRef(typename PINOCCHIO_EIGEN_REF_CONST_TYPE(Vector6ArgType) f_like)
    : m_ref(f_like)
    {
      EIGEN_STATIC_ASSERT(Vector6ArgType::ColsAtCompileTime == 1,
                          YOU_TRIED_CALLING_A_VECTOR_METHOD_ON_A_MATRIX);
      assert(f_like.size() == 6);
    }
    
    ToVectorConstReturnType toVector_impl() const { return m_ref; }
    
    // Getters
    ConstAngularType angular_impl() const { return ConstAngularType(m_ref.derived(),ANGULAR); }
    ConstLinearType linear_impl()  const { return ConstLinearType(m_ref.derived(),LINEAR); }
    
    const ForceRef & ref() const { return *this; }
    
  protected:
    DataRefType m_ref;
    
  }; // class ForceRef<Vector6Like>
  
} // namespace pinocchio

#endif // ifndef __pinocchio_force_ref_hpp__
