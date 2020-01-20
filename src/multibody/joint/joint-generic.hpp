//
// Copyright (c) 2016-2020 CNRS INRIA
//

#ifndef __pinocchio_joint_generic_hpp__
#define __pinocchio_joint_generic_hpp__

#include "pinocchio/multibody/joint/joint-collection.hpp"
#include "pinocchio/multibody/joint/joint-composite.hpp"
#include "pinocchio/multibody/joint/joint-basic-visitors.hxx"
#include "pinocchio/container/aligned-vector.hpp"

#include <boost/mpl/contains.hpp>

namespace pinocchio
{

  template<typename Scalar, int Options = 0, template<typename S, int O> class JointCollectionTpl = JointCollectionDefaultTpl>
  struct JointTpl;
  typedef JointTpl<double> Joint;

  template<typename _Scalar, int _Options, template<typename S, int O> class JointCollectionTpl>
  struct traits< JointTpl<_Scalar,_Options,JointCollectionTpl> >
  {
    enum {
      Options = _Options,
      NQ = Eigen::Dynamic, // Dynamic because unknown at compile time
      NV = Eigen::Dynamic
    };
    
    typedef _Scalar Scalar;
    typedef JointCollectionTpl<Scalar,Options> JointCollection;
    typedef JointDataTpl<Scalar,Options,JointCollectionTpl> JointDataDerived;
    typedef JointModelTpl<Scalar,Options,JointCollectionTpl> JointModelDerived;
    
    typedef ConstraintTpl<Eigen::Dynamic,Scalar,Options> Constraint_t;
    typedef SE3Tpl<Scalar,Options> Transformation_t;
    typedef MotionTpl<Scalar,Options>  Motion_t;
    typedef MotionTpl<Scalar,Options>  Bias_t;

    // [ABA]
    typedef Eigen::Matrix<Scalar,6,Eigen::Dynamic,Options> U_t;
    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic,Options> D_t;
    typedef Eigen::Matrix<Scalar,6,Eigen::Dynamic,Options> UD_t;
    
    typedef Constraint_t ConstraintTypeConstRef;
    typedef Constraint_t ConstraintTypeRef;
    typedef Transformation_t TansformTypeConstRef;
    typedef Transformation_t TansformTypeRef;
    typedef Motion_t MotionTypeConstRef;
    typedef Motion_t MotionTypeRef;
    typedef Bias_t BiasTypeConstRef;
    typedef Bias_t BiasTypeRef;
    typedef U_t UTypeConstRef;
    typedef U_t UTypeRef;
    typedef D_t DTypeConstRef;
    typedef D_t DTypeRef;
    typedef UD_t UDTypeConstRef;
    typedef UD_t UDTypeRef;

    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1,Options> ConfigVector_t;
    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1,Options> TangentVector_t;
  };
  
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct traits< JointDataTpl<Scalar,Options,JointCollectionTpl> >
  { typedef JointTpl<Scalar,Options,JointCollectionTpl> JointDerived; };
  
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct traits< JointModelTpl<Scalar,Options,JointCollectionTpl> >
  { typedef JointTpl<Scalar,Options,JointCollectionTpl> JointDerived; };

  template<typename _Scalar, int _Options, template<typename S, int O> class JointCollectionTpl>
  struct JointDataTpl
  : public JointDataBase< JointDataTpl<_Scalar,_Options,JointCollectionTpl> >
  , JointCollectionTpl<_Scalar,_Options>::JointDataVariant
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef JointTpl<_Scalar,_Options,JointCollectionTpl> JointDerived;
    typedef JointDataBase<JointDataTpl> Base;
    
    PINOCCHIO_JOINT_DATA_TYPEDEF_TEMPLATE(JointDerived);

    typedef JointCollectionTpl<_Scalar,_Options> JointCollection;
    typedef typename JointCollection::JointDataVariant JointDataVariant;
    
    using Base::operator==;
    using Base::operator!=;
    
    JointDataVariant & toVariant() { return *static_cast<JointDataVariant*>(this); }
    const JointDataVariant & toVariant() const { return *static_cast<const JointDataVariant*>(this); }

    Constraint_t      S() const  { return constraint_xd(*this); }
    Transformation_t  M() const  { return joint_transform(*this); }
    Motion_t          v() const  { return motion(*this); }
    Bias_t            c() const  { return bias(*this); }
    
    // [ABA CCRBA]
    U_t               U()     const { return u_inertia(*this); }
    D_t               Dinv()  const { return dinv_inertia(*this); }
    UD_t              UDinv() const { return udinv_inertia(*this); }

    JointDataTpl()
    : JointDataVariant()
    {}
    
    JointDataTpl(const JointDataVariant & jdata_variant)
    : JointDataVariant(jdata_variant)
    {}
    
    template<typename JointDataDerived>
    JointDataTpl(const JointDataBase<JointDataDerived> & jdata)
    : JointCollection::JointDataVariant((JointDataVariant)jdata.derived())
    {
      BOOST_MPL_ASSERT((boost::mpl::contains<typename JointDataVariant::types,JointDataDerived>));
    }
    
    // Define all the standard accessors
    Constraint_t S_accessor() const { return S(); }
    Transformation_t M_accessor() const { return M(); }
    Motion_t v_accessor() const { return v(); }
    Bias_t c_accessor() const { return c(); }
    U_t U_accessor() const { return U(); }
    D_t Dinv_accessor() const { return Dinv(); }
    UD_t UDinv_accessor() const { return UDinv(); }

    static std::string classname() { return "JointData"; }
    std::string shortname() const { return ::pinocchio::shortname(*this); }
    
    bool isEqual(const JointDataTpl & other) const
    {
      return Base::isEqual(other)
      && toVariant() == other.toVariant();
    }

  };
  
  template<typename NewScalar, typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct CastType< NewScalar, JointModelTpl<Scalar,Options,JointCollectionTpl> >
  {
    typedef JointModelTpl<NewScalar,Options,JointCollectionTpl> type;
  };

  template<typename _Scalar, int _Options, template<typename S, int O> class JointCollectionTpl>
  struct JointModelTpl
  : JointModelBase< JointModelTpl<_Scalar,_Options,JointCollectionTpl> >
  , JointCollectionTpl<_Scalar,_Options>::JointModelVariant
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef JointTpl<_Scalar,_Options,JointCollectionTpl> JointDerived;

    PINOCCHIO_JOINT_TYPEDEF_TEMPLATE(JointDerived);
    PINOCCHIO_JOINT_USE_INDEXES(JointModelTpl);
    
    typedef JointCollectionTpl<Scalar,Options> JointCollection;
    typedef typename JointCollection::JointDataVariant JointDataVariant;
    typedef typename JointCollection::JointModelVariant JointModelVariant;
    
    using Base::id;
    using Base::setIndexes;
    using Base::operator==;
    using Base::operator!=;

    JointModelTpl() : JointModelVariant() {}
    
    JointModelTpl(const JointModelVariant & jmodel_variant)
    : JointCollection::JointModelVariant(jmodel_variant)
    {}
    
    template<typename JointModelDerived>
    JointModelTpl(const JointModelBase<JointModelDerived> & jmodel)
    : JointModelVariant((JointModelVariant)jmodel.derived())
    {
      BOOST_MPL_ASSERT((boost::mpl::contains<typename JointModelVariant::types,JointModelDerived>));
    }
    
    JointModelVariant & toVariant()
    { return *static_cast<JointModelVariant*>(this); }
    
    const JointModelVariant & toVariant() const
    { return *static_cast<const JointModelVariant*>(this); }

    JointDataDerived createData() const
    { return ::pinocchio::createData<Scalar,Options,JointCollectionTpl>(*this); }

    using Base::isEqual;
    bool isEqual(const JointModelTpl & other) const
    {
      return Base::isEqual(other)
      && toVariant() == other.toVariant();
    }

    template<typename ConfigVector>
    void calc(JointDataDerived & data,
              const Eigen::MatrixBase<ConfigVector> & q) const
    { calc_zero_order(*this,data,q); }

    template<typename ConfigVector, typename TangentVector>
    void calc(JointDataDerived & data,
              const Eigen::MatrixBase<ConfigVector> & q,
              const Eigen::MatrixBase<TangentVector> & v) const
    { calc_first_order(*this,data,q,v); }
    
    template<typename Matrix6Like>
    void calc_aba(JointDataDerived & data, const Eigen::MatrixBase<Matrix6Like> & I, const bool update_I) const
    { ::pinocchio::calc_aba(*this,data,PINOCCHIO_EIGEN_CONST_CAST(Matrix6Like,I),update_I); }
    
    std::string shortname() const { return ::pinocchio::shortname(*this); }
    static std::string classname() { return "JointModel"; }

    int     nq_impl() const { return ::pinocchio::nq(*this); }
    int     nv_impl() const { return ::pinocchio::nv(*this); }

    int     idx_q_impl()   const { return ::pinocchio::idx_q(*this); }
    int     idx_v_impl()   const { return ::pinocchio::idx_v(*this); }

    JointIndex     id_impl()      const { return ::pinocchio::id(*this); }

    void setIndexes(JointIndex id, int nq, int nv)
    { ::pinocchio::setIndexes(*this,id, nq, nv); }
    
    /// \returns An expression of *this with the Scalar type casted to NewScalar.
    template<typename NewScalar>
    JointModelTpl<NewScalar,Options,JointCollectionTpl> cast() const
    {
      return cast_joint<NewScalar,Scalar,Options,JointCollectionTpl>(*this);
    }
  };
  
  typedef PINOCCHIO_ALIGNED_STD_VECTOR(JointData) JointDataVector;
  typedef PINOCCHIO_ALIGNED_STD_VECTOR(JointModel) JointModelVector;

} // namespace pinocchio

#endif // ifndef __pinocchio_joint_generic_hpp__
