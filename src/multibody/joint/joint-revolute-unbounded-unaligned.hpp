//
// Copyright (c) 2019 INRIA
//

#ifndef __pinocchio_joint_revolute_unbounded_unaligned_hpp__
#define __pinocchio_joint_revolute_unbounded_unaligned_hpp__

#include "pinocchio/fwd.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/math/rotation.hpp"
#include "pinocchio/math/matrix.hpp"

#include "pinocchio/multibody/joint/joint-revolute-unaligned.hpp"

namespace pinocchio
{

  template<typename Scalar, int Options = 0> struct JointRevoluteUnboundedUnalignedTpl;
  
  template<typename _Scalar, int _Options>
  struct traits< JointRevoluteUnboundedUnalignedTpl<_Scalar,_Options> >
  {
    enum {
      NQ = 2,
      NV = 1
    };
    typedef _Scalar Scalar;
    enum { Options = _Options };
    
    typedef Eigen::Matrix<Scalar,NQ,1,Options> ConfigVector_t;
    typedef Eigen::Matrix<Scalar,NV,1,Options> TangentVector_t;
    
    typedef JointDataRevoluteUnboundedUnalignedTpl<Scalar,Options> JointDataDerived;
    typedef JointModelRevoluteUnboundedUnalignedTpl<Scalar,Options> JointModelDerived;
    typedef ConstraintRevoluteUnalignedTpl<Scalar,Options> Constraint_t;
    typedef SE3Tpl<Scalar,Options> Transformation_t;
    typedef MotionRevoluteUnalignedTpl<Scalar,Options> Motion_t;
    typedef MotionZeroTpl<Scalar,Options> Bias_t;
    typedef Eigen::Matrix<Scalar,6,NV,Options> F_t;
    
    // [ABA]
    typedef Eigen::Matrix<Scalar,6,NV,Options> U_t;
    typedef Eigen::Matrix<Scalar,NV,NV,Options> D_t;
    typedef Eigen::Matrix<Scalar,6,NV,Options> UD_t;
    
    PINOCCHIO_JOINT_DATA_BASE_ACCESSOR_DEFAULT_RETURN_TYPE
  };

  template<typename Scalar, int Options>
  struct traits< JointDataRevoluteUnboundedUnalignedTpl<Scalar,Options> >
  { typedef JointRevoluteUnboundedUnalignedTpl<Scalar,Options> JointDerived; };
  
  template<typename Scalar, int Options>
  struct traits <JointModelRevoluteUnboundedUnalignedTpl<Scalar,Options> >
  { typedef JointRevoluteUnboundedUnalignedTpl<Scalar,Options> JointDerived; };

  template<typename _Scalar, int _Options>
  struct JointDataRevoluteUnboundedUnalignedTpl
  : public JointDataBase< JointDataRevoluteUnboundedUnalignedTpl<_Scalar,_Options> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef JointRevoluteUnboundedUnalignedTpl<_Scalar,_Options> JointDerived;
    PINOCCHIO_JOINT_DATA_TYPEDEF_TEMPLATE(JointDerived);
    PINOCCHIO_JOINT_DATA_BASE_DEFAULT_ACCESSOR
    
    Transformation_t M;
    Constraint_t S;
    Motion_t v;
    Bias_t c;
    
    // [ABA] specific data
    U_t U;
    D_t Dinv;
    UD_t UDinv;
    
    JointDataRevoluteUnboundedUnalignedTpl()
    : M(Transformation_t::Identity())
    , S(Constraint_t::Vector3::Zero())
    , v(Constraint_t::Vector3::Zero(),(Scalar)0)
    , U(U_t::Zero())
    , Dinv(D_t::Zero())
    , UDinv(UD_t::Zero())
    {}
    
    template<typename Vector3Like>
    JointDataRevoluteUnboundedUnalignedTpl(const Eigen::MatrixBase<Vector3Like> & axis)
    : M(Transformation_t::Identity())
    , S(axis)
    , v(axis,(Scalar)NAN)
    , U(U_t::Zero())
    , Dinv(D_t::Zero())
    , UDinv(UD_t::Zero())
    {}
    
    static std::string classname() { return std::string("JointDataRevoluteUnboundedUnalignedTpl"); }
    std::string shortname() const { return classname(); }
    
  }; // struct JointDataRevoluteUnboundedUnalignedTpl

  PINOCCHIO_JOINT_CAST_TYPE_SPECIALIZATION(JointModelRevoluteUnboundedUnalignedTpl);
  
  template<typename _Scalar, int _Options>
  struct JointModelRevoluteUnboundedUnalignedTpl
  : public JointModelBase< JointModelRevoluteUnboundedUnalignedTpl<_Scalar,_Options> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef JointRevoluteUnboundedUnalignedTpl<_Scalar,_Options> JointDerived;
    PINOCCHIO_JOINT_TYPEDEF_TEMPLATE(JointDerived);
    typedef Eigen::Matrix<Scalar,3,1,Options> Vector3;
    
    typedef JointModelBase<JointModelRevoluteUnboundedUnalignedTpl> Base;
    using Base::id;
    using Base::idx_q;
    using Base::idx_v;
    using Base::setIndexes;
    
    JointModelRevoluteUnboundedUnalignedTpl() {}
    
    JointModelRevoluteUnboundedUnalignedTpl(const Scalar & x,
                                            const Scalar & y,
                                            const Scalar & z)
    : axis(x,y,z)
    {
      axis.normalize();
      assert(isUnitary(axis) && "Rotation axis is not unitary");
    }
    
    template<typename Vector3Like>
    JointModelRevoluteUnboundedUnalignedTpl(const Eigen::MatrixBase<Vector3Like> & axis)
    : axis(axis)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(Vector3Like);
      assert(isUnitary(axis) && "Rotation axis is not unitary");
    }

    JointDataDerived createData() const { return JointDataDerived(axis); }
    
    using Base::isEqual;
    bool isEqual(const JointModelRevoluteUnboundedUnalignedTpl & other) const
    {
      return Base::isEqual(other) && axis == other.axis;
    }
    
    template<typename ConfigVector>
    void calc(JointDataDerived & data,
              const typename Eigen::MatrixBase<ConfigVector> & qs) const
    {
      typedef typename ConfigVector::Scalar OtherScalar;
      typename ConfigVector::template ConstFixedSegmentReturnType<NQ>::Type
      & q = qs.template segment<NQ>(idx_q());
      
      const OtherScalar & ca = q(0);
      const OtherScalar & sa = q(1);
      
      toRotationMatrix(axis,ca,sa,data.M.rotation());
    }

    template<typename ConfigVector, typename TangentVector>
    void calc(JointDataDerived & data,
              const typename Eigen::MatrixBase<ConfigVector> & qs,
              const typename Eigen::MatrixBase<TangentVector> & vs) const
    {
      calc(data,qs.derived());
      data.v.angularRate() = static_cast<Scalar>(vs[idx_v()]);
    }
    
    template<typename Matrix6Like>
    void calc_aba(JointDataDerived & data,
                  const Eigen::MatrixBase<Matrix6Like> & I,
                  const bool update_I) const
    {
      data.U.noalias() = I.template middleCols<3>(Motion::ANGULAR) * axis;
      data.Dinv[0] = (Scalar)(1)/axis.dot(data.U.template segment<3>(Motion::ANGULAR));
      data.UDinv.noalias() = data.U * data.Dinv;
      
      if (update_I)
        PINOCCHIO_EIGEN_CONST_CAST(Matrix6Like,I) -= data.UDinv * data.U.transpose();
    }
    
    static std::string classname() { return std::string("JointModelRevoluteUnboundedUnaligned"); }
    std::string shortname() const { return classname(); }
    
    /// \returns An expression of *this with the Scalar type casted to NewScalar.
    template<typename NewScalar>
    JointModelRevoluteUnboundedUnalignedTpl<NewScalar,Options> cast() const
    {
      typedef JointModelRevoluteUnboundedUnalignedTpl<NewScalar,Options> ReturnType;
      ReturnType res(axis.template cast<NewScalar>());
      res.setIndexes(id(),idx_q(),idx_v());
      return res;
    }

    // data
    ///
    /// \brief 3d main axis of the joint.
    ///
    Vector3 axis;
  }; // struct JointModelRevoluteUnboundedUnalignedTpl

} //namespace pinocchio

#include <boost/type_traits.hpp>

namespace boost
{
  template<typename Scalar, int Options>
  struct has_nothrow_constructor< ::pinocchio::JointModelRevoluteUnboundedUnalignedTpl<Scalar,Options> >
  : public integral_constant<bool,true> {};
  
  template<typename Scalar, int Options>
  struct has_nothrow_copy< ::pinocchio::JointModelRevoluteUnboundedUnalignedTpl<Scalar,Options> >
  : public integral_constant<bool,true> {};
  
  template<typename Scalar, int Options>
  struct has_nothrow_constructor< ::pinocchio::JointDataRevoluteUnboundedUnalignedTpl<Scalar,Options> >
  : public integral_constant<bool,true> {};
  
  template<typename Scalar, int Options>
  struct has_nothrow_copy< ::pinocchio::JointDataRevoluteUnboundedUnalignedTpl<Scalar,Options> >
  : public integral_constant<bool,true> {};
}


#endif // ifndef __pinocchio_joint_revolute_unbounded_unaligned_hpp__
