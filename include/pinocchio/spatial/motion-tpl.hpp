//
// Copyright (c) 2015-2018 CNRS
// Copyright (c) 2015-2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_spatial_motion_tpl_hpp__
#define __pinocchio_spatial_motion_tpl_hpp__

namespace pinocchio
{
  template<typename _Scalar, int _Options>
  struct traits<MotionTpl<_Scalar, _Options>>
  {
    typedef _Scalar Scalar;
    typedef Eigen::Matrix<Scalar, 3, 1, _Options> Vector3;
    typedef Eigen::Matrix<Scalar, 6, 1, _Options> Vector6;
    typedef Eigen::Matrix<Scalar, 4, 4, _Options> Matrix4;
    typedef Eigen::Matrix<Scalar, 6, 6, _Options> Matrix6;
    typedef Matrix6 ActionMatrixType;
    typedef Matrix4 HomogeneousMatrixType;
    typedef typename PINOCCHIO_EIGEN_REF_CONST_TYPE(Vector6) ToVectorConstReturnType;
    typedef typename PINOCCHIO_EIGEN_REF_TYPE(Vector6) ToVectorReturnType;
    typedef typename Vector6::template FixedSegmentReturnType<3>::Type LinearType;
    typedef typename Vector6::template FixedSegmentReturnType<3>::Type AngularType;
    typedef typename Vector6::template ConstFixedSegmentReturnType<3>::Type ConstLinearType;
    typedef typename Vector6::template ConstFixedSegmentReturnType<3>::Type ConstAngularType;
    typedef MotionTpl<Scalar, _Options> MotionPlain;
    typedef const MotionPlain & PlainReturnType;
    enum
    {
      LINEAR = 0,
      ANGULAR = 3,
      Options = _Options
    };

    typedef MotionRef<Vector6> MotionRefType;
  }; // traits MotionTpl

  template<typename _Scalar, int _Options>
  class MotionTpl : public MotionDense<MotionTpl<_Scalar, _Options>>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef MotionDense<MotionTpl> Base;
    MOTION_TYPEDEF_TPL(MotionTpl);
    enum
    {
      Options = _Options
    };

    using Base::operator=;
    using Base::angular;
    using Base::linear;

    using Base::__mequ__;
    using Base::__minus__;
    using Base::__mult__;
    using Base::__opposite__;
    using Base::__pequ__;
    using Base::__plus__;

    // Constructors
    MotionTpl()
    {
    }

    template<typename V1, typename V2>
    MotionTpl(const Eigen::MatrixBase<V1> & v, const Eigen::MatrixBase<V2> & w)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(V1);
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(V2);
      linear() = v;
      angular() = w;
    }

    template<typename V6>
    explicit MotionTpl(const Eigen::MatrixBase<V6> & v)
    : m_data(v)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(V6);
    }

    MotionTpl(const MotionTpl & other)
    {
      *this = other;
    }

    template<typename S2, int O2>
    explicit MotionTpl(const MotionTpl<S2, O2> & other)
    {
      *this = other.template cast<Scalar>();
    }

    template<int O2>
    explicit MotionTpl(const MotionTpl<Scalar, O2> & clone)
    : m_data(clone.toVector())
    {
    }

    // Same explanation as converting constructor from MotionBase
    template<
      typename M2,
      typename std::enable_if<!std::is_convertible<MotionDense<M2>, MotionTpl>::value, bool>::type =
        true>
    explicit MotionTpl(const MotionDense<M2> & clone)
    {
      linear() = clone.linear();
      angular() = clone.angular();
    }

    // MotionBase implement a conversion function to PlainReturnType.
    // Usually, PlainReturnType is defined as MotionTpl.
    // In this case, this converting constructor is redundant and
    // create a warning with -Wconversion
    template<
      typename M2,
      typename std::enable_if<!std::is_convertible<MotionBase<M2>, MotionTpl>::value, bool>::type =
        true>
    explicit MotionTpl(const MotionBase<M2> & clone)
    {
      *this = clone;
    }

    ///
    /// \brief Copy assignment operator.
    ///
    /// \param[in] other MotionTpl to copy
    ///
    MotionTpl & operator=(const MotionTpl & clone) // Copy assignment operator
    {
      m_data = clone.toVector();
      return *this;
    }

    // initializers
    static MotionTpl Zero()
    {
      return MotionTpl(Vector6::Zero());
    }
    static MotionTpl Random()
    {
      return MotionTpl(Vector6::Random());
    }

    inline PlainReturnType plain() const
    {
      return *this;
    }

    ToVectorConstReturnType toVector_impl() const
    {
      return m_data;
    }
    ToVectorReturnType toVector_impl()
    {
      return m_data;
    }

    // Getters
    ConstAngularType angular_impl() const
    {
      return m_data.template segment<3>(ANGULAR);
    }
    ConstLinearType linear_impl() const
    {
      return m_data.template segment<3>(LINEAR);
    }
    AngularType angular_impl()
    {
      return m_data.template segment<3>(ANGULAR);
    }
    LinearType linear_impl()
    {
      return m_data.template segment<3>(LINEAR);
    }

    template<typename V3>
    void angular_impl(const Eigen::MatrixBase<V3> & w)
    {
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(V3, 3);
      angular_impl() = w;
    }
    template<typename V3>
    void linear_impl(const Eigen::MatrixBase<V3> & v)
    {
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(V3, 3);
      linear_impl() = v;
    }

    // Specific operators for MotionTpl and MotionRef
    template<int O2>
    MotionPlain __plus__(const MotionTpl<Scalar, O2> & v) const
    {
      return MotionPlain(m_data + v.toVector());
    }

    template<typename Vector6ArgType>
    MotionPlain __plus__(const MotionRef<Vector6ArgType> & v) const
    {
      return MotionPlain(m_data + v.toVector());
    }

    template<int O2>
    MotionPlain __minus__(const MotionTpl<Scalar, O2> & v) const
    {
      return MotionPlain(m_data - v.toVector());
    }

    template<typename Vector6ArgType>
    MotionPlain __minus__(const MotionRef<Vector6ArgType> & v) const
    {
      return MotionPlain(m_data - v.toVector());
    }

    template<int O2>
    MotionTpl & __pequ__(const MotionTpl<Scalar, O2> & v)
    {
      m_data += v.toVector();
      return *this;
    }

    template<typename Vector6ArgType>
    MotionTpl & __pequ__(const MotionRef<Vector6ArgType> & v)
    {
      m_data += v.toVector();
      return *this;
    }

    template<int O2>
    MotionTpl & __mequ__(const MotionTpl<Scalar, O2> & v)
    {
      m_data -= v.toVector();
      return *this;
    }

    template<typename Vector6ArgType>
    MotionTpl & __mequ__(const MotionRef<Vector6ArgType> & v)
    {
      m_data -= v.toVector();
      return *this;
    }

    template<typename OtherScalar>
    MotionPlain __mult__(const OtherScalar & alpha) const
    {
      return MotionPlain(alpha * m_data);
    }

    MotionRef<Vector6> ref()
    {
      return MotionRef<Vector6>(m_data);
    }

    /// \returns An expression of *this with the Scalar type casted to NewScalar.
    template<typename NewScalar>
    MotionTpl<NewScalar, Options> cast() const
    {
      typedef MotionTpl<NewScalar, Options> ReturnType;
      ReturnType res(linear().template cast<NewScalar>(), angular().template cast<NewScalar>());
      return res;
    }

  protected:
    Vector6 m_data;

  }; // class MotionTpl

} // namespace pinocchio

#endif // ifndef __pinocchio_spatial_motion_tpl_hpp__
