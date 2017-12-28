
//
// Copyright (c) 2017 CNRS
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
    typedef typename EIGEN_REF_CONSTTYPE(Vector6ArgType) ConstDataRefType;
    
  }; // traits MotionRef
  
  namespace internal
  {
    template<typename Vector6ArgType>
    struct SE3GroupAction< MotionRef<Vector6ArgType> >
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
    
    MotionRef(Vector6ArgType & v_like)
    : ref(v_like)
    {
      EIGEN_STATIC_ASSERT(Vector6ArgType::ColsAtCompileTime == 1,
                          YOU_TRIED_CALLING_A_VECTOR_METHOD_ON_A_MATRIX);
      assert(v_like.size() == 6);
    }
    
    const Vector6 & toVector_impl() const { return Vector6(ref); }
    Vector6 & toVector_impl() { return Vector6(ref); }
    
    // Getters
    ConstAngularType angular_impl() const { return ConstAngularType(ref.derived(),ANGULAR); }
    ConstLinearType linear_impl()  const { return ConstLinearType(ref.derived(),LINEAR); }
    AngularType angular_impl() { return ref.template segment<3> (ANGULAR); }
    LinearType linear_impl()  { return ref.template segment<3> (LINEAR); }
    
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
    
  protected:
    DataRefType ref;

  }; // class MotionTpl
  
} // namespace se3

#endif // ifndef __se3_motion_ref_hpp__
