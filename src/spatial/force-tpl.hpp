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

#ifndef __se3_force_tpl_hpp__
#define __se3_force_tpl_hpp__

namespace se3
{
  template<typename _Scalar, int _Options>
  struct traits< ForceTpl<_Scalar, _Options> >
  {
    typedef _Scalar Scalar;
    typedef Eigen::Matrix<Scalar,3,1,_Options> Vector3;
    typedef Eigen::Matrix<Scalar,6,1,_Options> Vector6;
    typedef Eigen::Matrix<Scalar,6,6,_Options> Matrix6;
    typedef typename EIGEN_REF_CONSTTYPE(Vector6) ToVectorConstReturnType;
    typedef typename EIGEN_REF_TYPE(Vector6) ToVectorReturnType;
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
    
    using Base::operator=;
    using Base::linear;
    using Base::angular;
    
    // Constructors
    ForceTpl() : data() {}
    
    template<typename V1,typename V2>
    ForceTpl(const Eigen::MatrixBase<V1> & v, const Eigen::MatrixBase<V2> & w)
    {
      assert(v.size() == 3);
      assert(w.size() == 3);
      data << v, w;
    }
    
    template<typename V6>
    explicit ForceTpl(const Eigen::MatrixBase<V6> & v)
    : data(v)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(V6);
      assert(v.size() == 6);
    }
    
    template<typename S2,int O2>
    explicit ForceTpl(const ForceTpl<S2,O2> & clone)
    : data(clone.toVector())
    {}
    
    template<typename M2>
    explicit ForceTpl(const ForceDense<M2> & clone)
    { linear() = clone.linear(); angular() = clone.angular(); }
    
    // initializers
    static ForceTpl Zero()   { return ForceTpl(Vector6::Zero());   }
    static ForceTpl Random() { return ForceTpl(Vector6::Random()); }
    
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
    
    ForceRef<Vector6> ref() { return ForceRef<Vector6>(data); }
    
  protected:
    Vector6 data;
    
  }; // class ForceTpl
  
} // namespace se3

#endif // ifndef __se3_force_tpl_hpp__
