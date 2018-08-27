//
// Copyright (c) 2015-2018 CNRS
// Copyright (c) 2015-2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
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

#ifndef __se3_motion_tpl_hpp__
#define __se3_motion_tpl_hpp__

namespace se3
{
  template<typename _Scalar, int _Options>
  struct traits< MotionTpl<_Scalar, _Options> >
  {
    typedef _Scalar Scalar;
    typedef Eigen::Matrix<Scalar,3,1,_Options> Vector3;
    typedef Eigen::Matrix<Scalar,6,1,_Options> Vector6;
    typedef Eigen::Matrix<Scalar,6,6,_Options> Matrix6;
    typedef Matrix6 ActionMatrixType;
    typedef typename EIGEN_REF_CONSTTYPE(Vector6) ToVectorConstReturnType;
    typedef typename EIGEN_REF_TYPE(Vector6) ToVectorReturnType;
    typedef typename Vector6::template FixedSegmentReturnType<3>::Type LinearType;
    typedef typename Vector6::template FixedSegmentReturnType<3>::Type AngularType;
    typedef typename Vector6::template ConstFixedSegmentReturnType<3>::Type ConstLinearType;
    typedef typename Vector6::template ConstFixedSegmentReturnType<3>::Type ConstAngularType;
    typedef MotionTpl<Scalar,_Options> MotionPlain;
    enum {
      LINEAR = 0,
      ANGULAR = 3,
      Options = _Options
    };
    
    typedef MotionRef<Vector6> MotionRefType;
  }; // traits MotionTpl
  
  template<typename _Scalar, int _Options>
  class MotionTpl : public MotionDense< MotionTpl< _Scalar, _Options > >
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef MotionDense<MotionTpl> Base;
    MOTION_TYPEDEF_TPL(MotionTpl);

    using Base::operator=;
    using Base::linear;
    using Base::angular;
    
    using Base::__plus__;
    using Base::__opposite__;
    using Base::__minus__;
    using Base::__pequ__;
    using Base::__mequ__;
    using Base::__mult__;

    // Constructors
    MotionTpl() : data() {}
    
    template<typename V1,typename V2>
    MotionTpl(const Eigen::MatrixBase<V1> & v, const Eigen::MatrixBase<V2> & w)
    {
      assert(v.size() == 3);
      assert(w.size() == 3);
      data << v, w;
    }
    
    template<typename V6>
    explicit MotionTpl(const Eigen::MatrixBase<V6> & v)
    : data(v)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(V6);
      assert(v.size() == 6);
    }
    
    template<typename S2,int O2>
    explicit MotionTpl(const MotionTpl<S2,O2> & clone)
    : data(clone.toVector())
    {}
    
    template<typename M2>
    explicit MotionTpl(const MotionDense<M2> & clone)
    { linear() = clone.linear(); angular() = clone.angular(); }
    
    // initializers
    static MotionTpl Zero()   { return MotionTpl(Vector6::Zero());   }
    static MotionTpl Random() { return MotionTpl(Vector6::Random()); }
    
    ToVectorConstReturnType toVector_impl() const { return data; }
    ToVectorReturnType toVector_impl() { return data; }
    
    // Getters
    ConstAngularType angular_impl() const { return data.template segment<3> (ANGULAR); }
    ConstLinearType linear_impl()  const { return data.template segment<3> (LINEAR); }
    AngularType angular_impl() { return data.template segment<3> (ANGULAR); }
    LinearType linear_impl()  { return data.template segment<3> (LINEAR); }
    
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
    template<typename S2, int O2>
    MotionPlain __plus__(const MotionTpl<S2,O2> & v) const
    { return MotionPlain(data+v.toVector()); }
    
    template<typename Vector6ArgType>
    MotionPlain __plus__(const MotionRef<Vector6ArgType> & v) const
    { return MotionPlain(data+v.toVector()); }
    
    template<typename S2, int O2>
    MotionPlain __minus__(const MotionTpl<S2,O2> & v) const
    { return MotionPlain(data-v.toVector()); }
    
    template<typename Vector6ArgType>
    MotionPlain __minus__(const MotionRef<Vector6ArgType> & v) const
    { return MotionPlain(data-v.toVector()); }
    
    template<typename S2, int O2>
    MotionTpl & __pequ__(const MotionTpl<S2,O2> & v)
    { data += v.toVector(); return *this; }
    
    template<typename Vector6ArgType>
    MotionTpl & __pequ__(const MotionRef<Vector6ArgType> & v)
    { data += v.toVector(); return *this; }
    
    template<typename S2, int O2>
    MotionTpl & __mequ__(const MotionTpl<S2,O2> & v)
    { data -= v.toVector(); return *this; }
    
    template<typename Vector6ArgType>
    MotionTpl & __mequ__(const MotionRef<Vector6ArgType> & v)
    { data -= v.toVector(); return *this; }
    
    template<typename OtherScalar>
    MotionPlain __mult__(const OtherScalar & alpha) const
    { return MotionPlain(alpha*data); }
    
    MotionRef<Vector6> ref() { return MotionRef<Vector6>(data); }

  protected:
    Vector6 data;
    
  }; // class MotionTpl
  
} // namespace se3

#endif // ifndef __se3_motion_tpl_hpp__
