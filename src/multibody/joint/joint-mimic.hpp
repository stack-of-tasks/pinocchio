//
// Copyright (c) 2019 INRIA
//

#ifndef __pinocchio_joint_mimic_hpp__
#define __pinocchio_joint_mimic_hpp__

#include "pinocchio/macros.hpp"
#include "pinocchio/multibody/joint/joint-base.hpp"

namespace pinocchio
{
  
  template<class Constraint> struct ScaledConstraint;
  
  template<class Constraint>
  struct traits< ScaledConstraint<Constraint> >
  {
    typedef typename traits<Constraint>::Scalar Scalar;
    enum { Options = traits<Constraint>::Options };
    enum {
      LINEAR = traits<Constraint>::LINEAR,
      ANGULAR = traits<Constraint>::ANGULAR
    };
    typedef typename traits<Constraint>::JointMotion JointMotion;
    typedef typename traits<Constraint>::JointForce JointForce;
    typedef typename traits<Constraint>::DenseBase DenseBase;
    typedef typename traits<Constraint>::MatrixReturnType MatrixReturnType;
    typedef typename traits<Constraint>::ConstMatrixReturnType ConstMatrixReturnType;
  }; // traits ScaledConstraint
  
  namespace internal
  {
    template<class Constraint>
    struct SE3GroupAction< ScaledConstraint<Constraint> >
    { typedef typename SE3GroupAction<Constraint>::ReturnType ReturnType; };
    
    template<class Constraint, typename MotionDerived>
    struct MotionAlgebraAction< ScaledConstraint<Constraint>, MotionDerived >
    { typedef typename MotionAlgebraAction<Constraint,MotionDerived>::ReturnType ReturnType; };
    
    template<class Constraint, typename ForceDerived>
    struct ConstraintForceOp< ScaledConstraint<Constraint>, ForceDerived>
    {
      typedef typename Constraint::Scalar Scalar;
      typedef typename ConstraintForceOp<Constraint,ForceDerived>::ReturnType OriginalReturnType;
      
      typedef typename ScalarMatrixProduct<Scalar,OriginalReturnType>::type IdealReturnType;
      typedef Eigen::Matrix<Scalar,IdealReturnType::RowsAtCompileTime,IdealReturnType::ColsAtCompileTime,Constraint::Options> ReturnType;
    };
    
    template<class Constraint, typename ForceSet>
    struct ConstraintForceSetOp< ScaledConstraint<Constraint>, ForceSet>
    {
      typedef typename Constraint::Scalar Scalar;
      typedef typename ConstraintForceSetOp<Constraint,ForceSet>::ReturnType OriginalReturnType;
      typedef typename ScalarMatrixProduct<Scalar,OriginalReturnType>::type ReturnType;
    };
    
  }
  
  template<class Constraint>
  struct ScaledConstraint
  : ConstraintBase< ScaledConstraint<Constraint> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    PINOCCHIO_CONSTRAINT_TYPEDEF_TPL(ScaledConstraint)
    enum { NV = Constraint::NV };
    typedef ConstraintBase<ScaledConstraint> Base;
    using Base::nv;
    
    typedef typename internal::SE3GroupAction<Constraint>::ReturnType SE3ActionReturnType;
    
    ScaledConstraint(Constraint & constraint,
                     const Scalar & scaling_factor)
    : constraint(constraint)
    , scaling_factor(scaling_factor)
    {}
    
    template<typename VectorLike>
    JointMotion __mult__(const Eigen::MatrixBase<VectorLike> & v) const
    {
      assert(v.size() == nv());
      JointMotion jm = constraint * v;
      return jm * scaling_factor;
    }
    
    template<typename S1, int O1>
    SE3ActionReturnType
    se3Action(const SE3Tpl<S1,O1> & m) const
    {
      SE3ActionReturnType res = constraint.se3Action(m);
      return scaling_factor * res;
    }
    
    int nv_impl() const { return constraint.nv(); }
    
    struct TransposeConst
    {
      const ScaledConstraint & ref;
      TransposeConst(const ScaledConstraint & ref) : ref(ref) {}
      
      template<typename Derived>
      typename internal::ConstraintForceOp<ScaledConstraint,Derived>::ReturnType
      operator*(const ForceDense<Derived> & f) const
      {
        // TODO: I don't know why, but we should a dense a return type, otherwise it failes at the evaluation level;
        typedef typename internal::ConstraintForceOp<ScaledConstraint,Derived>::ReturnType ReturnType;
        return ReturnType(ref.scaling_factor * (ref.constraint.transpose() * f));
      }
      
      /// [CRBA]  MatrixBase operator* (Constraint::Transpose S, ForceSet::Block)
      template<typename Derived>
      typename internal::ConstraintForceSetOp<ScaledConstraint,Derived>::ReturnType
      operator*(const Eigen::MatrixBase<Derived> & F) const
      {
        return ref.scaling_factor * (ref.constraint.transpose() * F);
      }
      
    }; // struct TransposeConst
    
    TransposeConst transpose() const { return TransposeConst(*this); }
    
    DenseBase matrix_impl() const
    {
      DenseBase S = scaling_factor * constraint.matrix();
      return S;
    }
    
    template<typename MotionDerived>
    typename internal::MotionAlgebraAction<ScaledConstraint,MotionDerived>::ReturnType
    motionAction(const MotionDense<MotionDerived> & m) const
    {
      typedef typename internal::MotionAlgebraAction<ScaledConstraint,MotionDerived>::ReturnType ReturnType;
      ReturnType res = scaling_factor * constraint.motionAction(m);
      return res;
    }
    
  protected:
    
    Constraint & constraint;
    Scalar scaling_factor;
  }; // struct ConstraintRevoluteTpl
  
  template<class Joint> struct JointMimic;
  template<class JointModel> struct JointModelMimic;
  template<class JointData> struct JointDataMimic;
  
  template<class Joint>
  struct traits< JointMimic<Joint> >
  {
    enum {
      NQ = traits<Joint>::NV,
      NV = traits<Joint>::NQ
    };
    typedef typename traits<Joint>::Scalar Scalar;
    enum { Options = traits<Joint>::Options };
    
    typedef JointDataMimic<Joint> JointDataDerived;
    typedef JointModelMimic<Joint> JointModelDerived;
    
    typedef typename traits<Joint>::Constraint_t Constraint_t;
    typedef typename traits<Joint>::Transformation_t Transformation_t;
    typedef typename traits<Joint>::Motion_t Motion_t;
    typedef typename traits<Joint>::Bias_t Bias_t;

    // [ABA]
    typedef typename traits<Joint>::U_t U_t;
    typedef typename traits<Joint>::D_t D_t;
    typedef typename traits<Joint>::UD_t UD_t;
    
    PINOCCHIO_JOINT_DATA_BASE_ACCESSOR_DEFAULT_RETURN_TYPE
    
    typedef typename traits<Joint>::ConfigVector_t ConfigVector_t;
    typedef typename traits<Joint>::TangentVector_t TangentVector_t;
  };
  
  template<class Joint>
  struct traits< JointDataMimic<Joint> >
  { typedef JointMimic<Joint> JointDerived; };
  
  template<class Joint>
  struct traits< JointModelMimic<Joint> >
  { typedef JointMimic<Joint> JointDerived; };
  
  template<class JointData>
  struct JointDataMimic
  : public traits<JointData>::JointDataDerived
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef JointMimic<JointData> JointDerived;
    typedef typename traits<JointData>::JointDataDerived Base;
    
    PINOCCHIO_JOINT_DATA_TYPEDEF_TEMPLATE;
    
    Base & base() { return *static_cast<Base*>(this); }
    const Base & base() const { return *static_cast<const Base*>(this); }
    
    static std::string classname()
    {
      return std::string("JointDataMimic<") + Base::classname() + std::string(">");
    }
    
    std::string shortname() const
    {
      return std::string("JointDataMimic<") + base().shortname() + std::string(">");
    }
    
    // Accessors
    ConstraintTypeConstRef S_accessor() const { return jdata_ref.S; }
    TansformTypeConstRef M_accessor() const { return jdata_ref.M; }
    MotionTypeConstRef v_accessor() const { return jdata_ref.v; }
    BiasTypeConstRef c_accessor() const { return jdata_ref.c; }
    UTypeConstRef U_accessor() const { return jdata_ref.U; }
    UTypeRef U_accessor() { return jdata_ref.U; }
    DTypeConstRef Dinv_accessor() const { return jdata_ref.Dinv; }
    UDTypeConstRef UDinv_accessor() const { return jdata_ref.UDinv; }
    
    /// \brief Transform configuration vector
    ConfigVector_t q_transform;
    /// \brief Transform velocity vector.
    TangentVector_t v_transform;
    
  protected:
    
    const JointData & jdata_ref;
    const Scalar scaling;
    
  }; // struct JointDataMimic
  
  template<class JointModel>
  struct JointModelMimic
  : public JointModelBase< JointModelMimic<JointModel> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef JointModelMimic<JointModel> JointDerived;
    
    PINOCCHIO_JOINT_TYPEDEF_TEMPLATE;
    
    typedef typename traits<JointModel>::JointModelDerived Base;
    using Base::id;
    using Base::idx_q;
    using Base::idx_v;
    using Base::setIndexes;
    
    JointModelMimic(const JointModel & jmodel,
                    const Scalar & multiplier,
                    const Scalar & offset)
    : jmodel_ref(jmodel)
    , multiplier(multiplier)
    , offset(offset)
    {}
    
    Base & base() { return *static_cast<Base*>(this); }
    const Base & base() const { return *static_cast<const Base*>(this); }
    
    static int nq_impl() { return 0; }
    static int nv_impl() { return 0; }
    
    void setIndexes_impl(JointIndex id, int /*q*/, int /*v*/)
    {
      Base::i_id = id; // Only the id of the joint in the model is different.
      Base::i_q = jmodel_ref.idx_q();
      Base::i_v = jmodel_ref.idx_v();
    }
    
    JointDataDerived createData() const
    { return jmodel_ref.createData(); }
    
    template<typename ConfigVector>
    EIGEN_DONT_INLINE
    void calc(JointDataDerived & data,
              const typename Eigen::MatrixBase<ConfigVector> & qs) const
    {
      
    }
    
    template<typename ConfigVector, typename TangentVector>
    EIGEN_DONT_INLINE
    void calc(JointDataDerived & data,
              const typename Eigen::MatrixBase<ConfigVector> & qs,
              const typename Eigen::MatrixBase<TangentVector> & vs) const
    {
      calc(data,qs.derived());
      
      data.v.w = (Scalar)vs[idx_v()];
    }
    
    template<typename Matrix6Like>
    void calc_aba(JointDataDerived & data,
                  const Eigen::MatrixBase<Matrix6Like> & I,
                  const bool update_I) const
    {
      jmodel_ref.calc_aba(data.base(),I.derived(),update_I);
    }
    
    static std::string classname()
    {
      return std::string("JointModelMimic<") + JointModel::classname() + std::string(">");;
    }
    
    std::string shortname() const
    {
      return std::string("JointModelMimic<") + jmodel_ref.shortname() + std::string(">");
    }
    
    /// \returns An expression of *this with the Scalar type casted to NewScalar.
    template<typename NewScalar>
    typename CastType<NewScalar,JointModelMimic>::type cast() const
    {
      typedef typename CastType<NewScalar,JointModelMimic>::type ReturnType;
      ReturnType res;
      res.setIndexes(id(),idx_q(),idx_v());
      return res;
    }
    
  protected:
    
    // data
    Scalar multiplier, offset;
    const JointModel & jmodel_ref;
    
  }; // struct JointModelMimic
  
} // namespace pinocchio

#endif // ifndef __pinocchio_joint_mimic_hpp__
