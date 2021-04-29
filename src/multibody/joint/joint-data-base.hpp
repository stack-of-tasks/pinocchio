//
// Copyright (c) 2015-2019 CNRS INRIA
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_multibody_joint_data_base_hpp__
#define __pinocchio_multibody_joint_data_base_hpp__

#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/joint/joint-model-base.hpp"
  
#define PINOCCHIO_JOINT_DATA_TYPEDEF_GENERIC(Joint,TYPENAME)              \
  PINOCCHIO_JOINT_MODEL_TYPEDEF_GENERIC(Joint,TYPENAME); \
  typedef TYPENAME traits<Joint>::ConstraintTypeConstRef ConstraintTypeConstRef;      \
  typedef TYPENAME traits<Joint>::ConstraintTypeRef ConstraintTypeRef;      \
  typedef TYPENAME traits<Joint>::TansformTypeConstRef TansformTypeConstRef;      \
  typedef TYPENAME traits<Joint>::TansformTypeRef TansformTypeRef;      \
  typedef TYPENAME traits<Joint>::MotionTypeConstRef MotionTypeConstRef;      \
  typedef TYPENAME traits<Joint>::MotionTypeRef MotionTypeRef;      \
  typedef TYPENAME traits<Joint>::BiasTypeConstRef BiasTypeConstRef;      \
  typedef TYPENAME traits<Joint>::BiasTypeRef BiasTypeRef;      \
  typedef TYPENAME traits<Joint>::UTypeConstRef UTypeConstRef;      \
  typedef TYPENAME traits<Joint>::UTypeRef UTypeRef;      \
  typedef TYPENAME traits<Joint>::DTypeConstRef DTypeConstRef;      \
  typedef TYPENAME traits<Joint>::DTypeRef DTypeRef;      \
  typedef TYPENAME traits<Joint>::UDTypeConstRef UDTypeConstRef; \
  typedef TYPENAME traits<Joint>::UDTypeRef UDTypeRef
  
#ifdef __clang__

  #define PINOCCHIO_JOINT_DATA_TYPEDEF(Joint) PINOCCHIO_JOINT_DATA_TYPEDEF_GENERIC(Joint,PINOCCHIO_EMPTY_ARG)
  #define PINOCCHIO_JOINT_DATA_TYPEDEF_TEMPLATE(Joint) PINOCCHIO_JOINT_DATA_TYPEDEF_GENERIC(Joint,typename)

#elif (__GNUC__ == 4) && (__GNUC_MINOR__ == 4) && (__GNUC_PATCHLEVEL__ == 2)

  #define PINOCCHIO_JOINT_DATA_TYPEDEF(Joint) PINOCCHIO_JOINT_DATA_TYPEDEF_GENERIC(Joint,PINOCCHIO_EMPTY_ARG)
  #define PINOCCHIO_JOINT_DATA_TYPEDEF_TEMPLATE(Joint) PINOCCHIO_JOINT_DATA_TYPEDEF_GENERIC(Joint,typename)

#else

  #define PINOCCHIO_JOINT_DATA_TYPEDEF(Joint) PINOCCHIO_JOINT_DATA_TYPEDEF_GENERIC(Joint,typename)
  #define PINOCCHIO_JOINT_DATA_TYPEDEF_TEMPLATE(Joint) PINOCCHIO_JOINT_DATA_TYPEDEF_GENERIC(Joint,typename)

#endif
  
#define PINOCCHIO_JOINT_DATA_BASE_DEFAULT_ACCESSOR \
  ConstraintTypeConstRef S_accessor() const { return S; } \
  ConstraintTypeRef S_accessor() { return S; } \
  TansformTypeConstRef M_accessor() const { return M; } \
  TansformTypeRef M_accessor() { return M; } \
  MotionTypeConstRef v_accessor() const { return v; } \
  MotionTypeRef v_accessor() { return v; } \
  BiasTypeConstRef c_accessor() const { return c; } \
  BiasTypeRef c_accessor() { return c; } \
  UTypeConstRef U_accessor() const { return U; } \
  UTypeRef U_accessor() { return U; } \
  DTypeConstRef Dinv_accessor() const { return Dinv; } \
  DTypeRef Dinv_accessor() { return Dinv; } \
  UDTypeConstRef UDinv_accessor() const { return UDinv; } \
  UDTypeRef UDinv_accessor() { return UDinv; }
  
#define PINOCCHIO_JOINT_DATA_BASE_ACCESSOR_DEFAULT_RETURN_TYPE \
  typedef const Constraint_t & ConstraintTypeConstRef; \
  typedef Constraint_t & ConstraintTypeRef; \
  typedef const Transformation_t & TansformTypeConstRef; \
  typedef Transformation_t & TansformTypeRef; \
  typedef const Motion_t & MotionTypeConstRef; \
  typedef Motion_t & MotionTypeRef; \
  typedef const Bias_t & BiasTypeConstRef; \
  typedef Bias_t & BiasTypeRef; \
  typedef const U_t & UTypeConstRef; \
  typedef U_t & UTypeRef; \
  typedef const D_t & DTypeConstRef; \
  typedef D_t & DTypeRef; \
  typedef const UD_t & UDTypeConstRef; \
  typedef UD_t & UDTypeRef;

namespace pinocchio
{

  template<typename Derived>
  struct JointDataBase
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef typename traits<Derived>::JointDerived JointDerived;
    PINOCCHIO_JOINT_DATA_TYPEDEF_TEMPLATE(JointDerived);

    Derived & derived() { return *static_cast<Derived*>(this); }
    const Derived & derived() const { return *static_cast<const Derived*>(this); }

    ConstraintTypeConstRef S() const     { return derived().S_accessor(); }
    ConstraintTypeRef S()     { return derived().S_accessor(); }
    TansformTypeConstRef M() const     { return derived().M_accessor(); }
    TansformTypeRef M() { return derived().M_accessor(); }
    MotionTypeConstRef v() const     { return derived().v_accessor(); }
    MotionTypeRef v()     { return derived().v_accessor(); }
    BiasTypeConstRef c() const     { return derived().c_accessor(); }
    BiasTypeRef c()     { return derived().c_accessor(); }

    UTypeConstRef U() const     { return derived().U_accessor(); }
    UTypeRef U()           { return derived().U_accessor(); }
    DTypeConstRef Dinv() const  { return derived().Dinv_accessor(); }
    DTypeRef Dinv()  { return derived().Dinv_accessor(); }
    UDTypeConstRef UDinv() const { return derived().UDinv_accessor(); }
    UDTypeRef UDinv() { return derived().UDinv_accessor(); }

    std::string shortname() const { return derived().shortname(); }
    static std::string classname() { return Derived::classname(); }

    void disp(std::ostream & os) const
    {
      using namespace std;
      os << shortname() << endl;
    }
    
    friend std::ostream & operator << (std::ostream & os, const JointDataBase<Derived> & joint)
    {
      joint.disp(os);
      return os;
    }
    
    template<typename OtherDerived>
    bool operator==(const JointDataBase<OtherDerived> & other) const
    {
      return derived().isEqual(other.derived());
    }
    
    /// \brief Default operator== implementation
    bool isEqual(const JointDataBase & other) const
    {
      return S() == other.S()
          && M() == other.M()
          && v() == other.v()
          && c() == other.c()
          && U() == other.U()
          && Dinv() == other.Dinv()
          && UDinv() == other.UDinv()
      ;
    }
    
    /// \brief Default operator== implementation
    template<typename OtherDerived>
    bool isEqual(const JointDataBase<OtherDerived> & /*other*/) const
    {
      return false;
      ;
    }
    
    bool operator!=(const JointDataBase<Derived> & other) const
    {
      return derived().isNotEqual(other.derived());
    }
    
    /// \brief Default operator!= implementation
    bool isNotEqual(const JointDataBase<Derived> & other) const
    {
      return !(derived() == other.derived());
    }
    
  protected:
    
    /// \brief Default constructor: protected.
    inline JointDataBase() {}

  }; // struct JointDataBase

} // namespace pinocchio

#endif // ifndef __pinocchio_multibody_joint_data_base_hpp__
