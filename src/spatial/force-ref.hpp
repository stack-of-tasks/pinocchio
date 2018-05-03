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

#ifndef __se3_force_ref_hpp__
#define __se3_force_ref_hpp__

namespace se3
{
  
  template<typename Vector6ArgType>
  struct traits< ForceRef<Vector6ArgType> >
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
    typedef ForceTpl<Scalar,Options> ForcePlain;
    typedef typename EIGEN_REF_TYPE(Vector6ArgType) DataRefType;
    typedef DataRefType ToVectorReturnType;
    typedef typename EIGEN_REF_CONSTTYPE(Vector6ArgType) ConstDataRefType;
    typedef ConstDataRefType ToVectorConstReturnType;
    typedef MotionRef<Vector6ArgType> ForceRefType;
    
  }; // traits ForceRef
  
  namespace internal
  {
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
  }
  
  template<typename Vector6ArgType>
  class ForceRef : public ForceDense< ForceRef<Vector6ArgType> >
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef ForceDense<ForceRef> Base;
    typedef typename traits<ForceRef>::DataRefType DataRefType;
    FORCE_TYPEDEF_TPL(ForceRef);
    
    using Base::operator=;
    
    ForceRef(const Eigen::MatrixBase<Vector6ArgType> & f_like)
    : m_ref(const_cast<Vector6ArgType &>(f_like.derived()))
    {
      EIGEN_STATIC_ASSERT(Vector6ArgType::ColsAtCompileTime == 1,
                          YOU_TRIED_CALLING_A_VECTOR_METHOD_ON_A_MATRIX);
      assert(f_like.size() == 6);
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
    
    ForceRef & ref() { return *this; }
    
  protected:
    DataRefType m_ref;
    
  }; // class MotionTpl
  
} // namespace se3

#endif // ifndef __se3_force_ref_hpp__
