
//
// Copyright (c) 2017-2018 CNRS
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

#ifndef __se3_motion_ref_hpp__
#define __se3_motion_ref_hpp__

namespace se3
{
  
  template<typename Vector6ArgType>
  struct traits< MotionRef<Vector6ArgType> >
  {
    typedef typename Vector6ArgType::Scalar Scalar;
    typedef typename EIGEN_PLAIN_TYPE(Vector6ArgType) Vector6;
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
    typedef typename EIGEN_REF_TYPE(Vector6ArgType) DataRefType;
    typedef DataRefType ToVectorReturnType;
    typedef typename EIGEN_REF_CONSTTYPE(Vector6ArgType) ConstDataRefType;
    typedef ConstDataRefType ToVectorConstReturnType;
    typedef MotionRef<Vector6ArgType> MotionRefType;

  }; // traits MotionRef
  
  namespace internal
  {
    template<typename Vector6ArgType>
    struct SE3GroupAction< MotionRef<Vector6ArgType> >
    {
      typedef typename traits< MotionRef<Vector6ArgType> >::MotionPlain ReturnType;
    };
    
    template<typename Vector6ArgType, typename MotionDerived>
    struct MotionAlgebraAction< MotionRef<Vector6ArgType>, MotionDerived >
    { typedef typename traits< MotionRef<Vector6ArgType> >::MotionPlain ReturnType; };
  
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
    
    MotionRef(const Eigen::MatrixBase<Vector6ArgType> & v_like)
    : m_ref(const_cast<Vector6ArgType &>(v_like.derived()))
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

  protected:
    DataRefType m_ref;

  }; // class MotionTpl
  
} // namespace se3

#endif // ifndef __se3_motion_ref_hpp__
