//
// Copyright (c) 2015-2019 CNRS INRIA
// Copyright (c) 2015-2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_force_tpl_hpp__
#define __pinocchio_force_tpl_hpp__

namespace pinocchio
{
  template<typename _Scalar, int _Options>
  struct traits< ForceTpl<_Scalar, _Options> >
  {
    typedef _Scalar Scalar;
    typedef Eigen::Matrix<Scalar,3,1,_Options> Vector3;
    typedef Eigen::Matrix<Scalar,6,1,_Options> Vector6;
    typedef Eigen::Matrix<Scalar,6,6,_Options> Matrix6;
    typedef typename PINOCCHIO_EIGEN_REF_CONST_TYPE(Vector6) ToVectorConstReturnType;
    typedef typename PINOCCHIO_EIGEN_REF_TYPE(Vector6) ToVectorReturnType;
    typedef typename Vector6::template FixedSegmentReturnType<3>::Type LinearType;
    typedef typename Vector6::template FixedSegmentReturnType<3>::Type AngularType;
    typedef typename Vector6::template ConstFixedSegmentReturnType<3>::Type ConstLinearType;
    typedef typename Vector6::template ConstFixedSegmentReturnType<3>::Type ConstAngularType;
    typedef ForceTpl<Scalar,_Options> ForcePlain;
    enum {
      LINEAR = 0,
      ANGULAR = 3,
      Options = _Options
    };
    
    typedef ForceRef<Vector6> ForceRefType;
  }; // traits ForceTpl
  
  template<typename _Scalar, int _Options>
  class ForceTpl : public ForceDense< ForceTpl<_Scalar, _Options> >
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef ForceDense<ForceTpl> Base;
    FORCE_TYPEDEF_TPL(ForceTpl);
    enum { Options = _Options };
    
    using Base::operator=;
    using Base::operator!=;
    using Base::linear;
    using Base::angular;
    
    // Constructors
    ForceTpl() : m_data() {}
    
    template<typename V1,typename V2>
    ForceTpl(const Eigen::MatrixBase<V1> & v, const Eigen::MatrixBase<V2> & w)
    {
      assert(v.size() == 3);
      assert(w.size() == 3);
      linear() = v; angular() = w;
    }
    
    template<typename V6>
    explicit ForceTpl(const Eigen::MatrixBase<V6> & v)
    : m_data(v)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(V6);
      assert(v.size() == 6);
    }

    ForceTpl(const ForceTpl & clone) // Copy constructor
    : m_data(clone.toVector())
    {}

    ForceTpl& operator=(const ForceTpl & clone)  // Copy assignment operator
    {
      m_data = clone.toVector();
      return *this;
    }

    template<int O2>
    explicit ForceTpl(const ForceTpl<Scalar,O2> & clone)
    : m_data(clone.toVector())
    {}
    
    template<typename M2>
    explicit ForceTpl(const ForceDense<M2> & clone)
    { linear() = clone.linear(); angular() = clone.angular(); }
    
    // initializers
    static ForceTpl Zero()   { return ForceTpl(Vector6::Zero());   }
    static ForceTpl Random() { return ForceTpl(Vector6::Random()); }
    
    ToVectorConstReturnType toVector_impl() const { return m_data; }
    ToVectorReturnType toVector_impl() { return m_data; }
    
    // Getters
    ConstAngularType angular_impl() const { return m_data.template segment<3> (ANGULAR); }
    ConstLinearType linear_impl()  const { return m_data.template segment<3> (LINEAR); }
    AngularType angular_impl() { return m_data.template segment<3> (ANGULAR); }
    LinearType linear_impl()  { return m_data.template segment<3> (LINEAR); }
    
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
    
    ForceRef<Vector6> ref() { return ForceRef<Vector6>(m_data); }
    
    /// \returns An expression of *this with the Scalar type casted to NewScalar.
    template<typename NewScalar>
    ForceTpl<NewScalar,Options> cast() const
    {
      typedef ForceTpl<NewScalar,Options> ReturnType;
      ReturnType res(linear().template cast<NewScalar>(),
                     angular().template cast<NewScalar>());
      return res;
    }
    
  protected:
    Vector6 m_data;
    
  }; // class ForceTpl
  
} // namespace pinocchio

#endif // ifndef __pinocchio_force_tpl_hpp__
