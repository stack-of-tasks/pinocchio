//
// Copyright (c) 2019-2021 INRIA
//

#ifndef __pinocchio_multibody_joint_mimic_hpp__
#define __pinocchio_multibody_joint_mimic_hpp__

#include "pinocchio/multibody/joint/fwd.hpp"
#include "pinocchio/multibody/joint/joint-collection.hpp"
#include "pinocchio/macros.hpp"
#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/joint/joint-basic-visitors.hpp"

namespace pinocchio
{
  template<typename _Scalar, int _Options, int MaxDim>
  struct ScaledJointMotionSubspaceTpl;

  template<typename _Scalar, int _Options, int _MaxDim>
  struct traits<ScaledJointMotionSubspaceTpl<_Scalar, _Options, _MaxDim>>
  {
    enum
    {
      MaxDim = _MaxDim
    };
    typedef JointMotionSubspaceTpl<Eigen::Dynamic, _Scalar, _Options, _MaxDim>
      RefJointMotionSubspace;
    typedef typename traits<RefJointMotionSubspace>::Scalar Scalar;
    enum
    {
      Options = traits<RefJointMotionSubspace>::Options
    };
    enum
    {
      LINEAR = traits<RefJointMotionSubspace>::LINEAR,
      ANGULAR = traits<RefJointMotionSubspace>::ANGULAR
    };
    typedef typename traits<RefJointMotionSubspace>::JointMotion JointMotion;
    typedef typename traits<RefJointMotionSubspace>::JointForce JointForce;
    typedef typename traits<RefJointMotionSubspace>::DenseBase DenseBase;
    typedef typename traits<RefJointMotionSubspace>::MatrixReturnType MatrixReturnType;
    typedef typename traits<RefJointMotionSubspace>::ConstMatrixReturnType ConstMatrixReturnType;
  }; // traits ScaledJointMotionSubspaceTpl

  template<typename _Scalar, int _Options, int _MaxDim>
  struct SE3GroupAction<ScaledJointMotionSubspaceTpl<_Scalar, _Options, _MaxDim>>
  {
    typedef typename SE3GroupAction<typename traits<
      ScaledJointMotionSubspaceTpl<_Scalar, _Options, _MaxDim>>::RefJointMotionSubspace>::ReturnType
      ReturnType;
  };

  template<typename _Scalar, int _Options, int _MaxDim, typename MotionDerived>
  struct MotionAlgebraAction<
    ScaledJointMotionSubspaceTpl<_Scalar, _Options, _MaxDim>,
    MotionDerived>
  {
    typedef typename MotionAlgebraAction<
      typename traits<
        ScaledJointMotionSubspaceTpl<_Scalar, _Options, _MaxDim>>::RefJointMotionSubspace,
      MotionDerived>::ReturnType ReturnType;
  };

  template<typename _Scalar, int _Options, int _MaxDim, typename ForceDerived>
  struct ConstraintForceOp<ScaledJointMotionSubspaceTpl<_Scalar, _Options, _MaxDim>, ForceDerived>
  {
    typedef
      typename ScaledJointMotionSubspaceTpl<_Scalar, _Options, _MaxDim>::RefJointMotionSubspace
        RefJointMotionSubspace;
    typedef
      typename ConstraintForceOp<RefJointMotionSubspace, ForceDerived>::ReturnType RefReturnType;
    typedef typename ScalarMatrixProduct<_Scalar, RefReturnType>::type ReturnType;
  };

  template<typename _Scalar, int _Options, int _MaxDim, typename ForceSet>
  struct ConstraintForceSetOp<ScaledJointMotionSubspaceTpl<_Scalar, _Options, _MaxDim>, ForceSet>
  {
    typedef
      typename ScaledJointMotionSubspaceTpl<_Scalar, _Options, _MaxDim>::RefJointMotionSubspace
        RefJointMotionSubspace;
    typedef
      typename ConstraintForceSetOp<RefJointMotionSubspace, ForceSet>::ReturnType RefReturnType;
    typedef typename ScalarMatrixProduct<_Scalar, RefReturnType>::type ReturnType;
  };

  template<typename _Scalar, int _Options, int _MaxDim>
  struct ScaledJointMotionSubspaceTpl
  : JointMotionSubspaceBase<ScaledJointMotionSubspaceTpl<_Scalar, _Options, _MaxDim>>
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PINOCCHIO_CONSTRAINT_TYPEDEF_TPL(ScaledJointMotionSubspaceTpl)
    enum
    {
      NV = Eigen::Dynamic,
      MaxDim = _MaxDim
    };
    typedef JointMotionSubspaceBase<ScaledJointMotionSubspaceTpl> Base;
    using Base::nv;

    typedef typename traits<ScaledJointMotionSubspaceTpl<_Scalar, _Options, _MaxDim>>::
      RefJointMotionSubspace RefJointMotionSubspace;
    typedef typename SE3GroupAction<RefJointMotionSubspace>::ReturnType SE3ActionReturnType;

    ScaledJointMotionSubspaceTpl()
    : m_constraint(0)
    , m_scaling_factor(Scalar(1))
    {
    }

    explicit ScaledJointMotionSubspaceTpl(const Scalar & scaling_factor)
    : m_constraint(0)
    , m_scaling_factor(scaling_factor)
    {
    }

    template<typename ConstraintTpl>
    ScaledJointMotionSubspaceTpl(const ConstraintTpl & constraint, const Scalar & scaling_factor)
    : m_constraint(constraint)
    , m_scaling_factor(scaling_factor)
    {
    }

    ScaledJointMotionSubspaceTpl(const ScaledJointMotionSubspaceTpl & other)
    : m_constraint(other.m_constraint)
    , m_scaling_factor(other.m_scaling_factor)
    {
    }

    ScaledJointMotionSubspaceTpl & operator=(const ScaledJointMotionSubspaceTpl & other)
    {
      m_constraint = other.m_constraint;
      m_scaling_factor = other.m_scaling_factor;
      return *this;
    }

    template<typename VectorLike>
    JointMotion __mult__(const Eigen::MatrixBase<VectorLike> & v) const
    {

      assert(v.size() == nv());
      JointMotion jm = m_constraint * v;
      return m_scaling_factor * jm;
    }

    template<typename S1, int O1>
    SE3ActionReturnType se3Action(const SE3Tpl<S1, O1> & m) const
    {
      return m_scaling_factor * m_constraint.se3Action(m);
    }

    template<typename S1, int O1>
    SE3ActionReturnType se3ActionInverse(const SE3Tpl<S1, O1> & m) const
    {
      return m_scaling_factor * m_constraint.se3ActionInverse(m);
    }

    int nv_impl() const
    {
      return m_constraint.nv();
    }

    struct TransposeConst
    {
      const ScaledJointMotionSubspaceTpl & ref;
      explicit TransposeConst(const ScaledJointMotionSubspaceTpl & ref)
      : ref(ref)
      {
      }

      template<typename Derived>
      // typename ConstraintForceOp<ScaledJointMotionSubspaceTpl, Derived>::ReturnType
      JointForce operator*(const ForceDense<Derived> & f) const
      {
        return ref.m_scaling_factor * (ref.m_constraint.transpose() * f);
      }

      /// [CRBA]  MatrixBase operator* (RefConstraint::Transpose S, ForceSet::Block)
      template<typename Derived>
      typename ConstraintForceSetOp<ScaledJointMotionSubspaceTpl, Derived>::ReturnType
      operator*(const Eigen::MatrixBase<Derived> & F) const
      {
        return ref.m_scaling_factor * (ref.m_constraint.transpose() * F);
      }

    }; // struct TransposeConst

    TransposeConst transpose() const
    {
      return TransposeConst(*this);
    }

    const DenseBase & matrix_impl() const
    {
      S = m_scaling_factor * m_constraint.matrix_impl();
      return S;
    }

    DenseBase & matrix_impl()
    {
      S = m_scaling_factor * m_constraint.matrix_impl();
      return S;
    }

    template<typename MotionDerived>
    typename MotionAlgebraAction<ScaledJointMotionSubspaceTpl, MotionDerived>::ReturnType
    motionAction(const MotionDense<MotionDerived> & m) const
    {
      return m_scaling_factor * m_constraint.motionAction(m);
    }

    inline const Scalar & scaling() const
    {
      return m_scaling_factor;
    }
    inline Scalar & scaling()
    {
      return m_scaling_factor;
    }

    inline const RefJointMotionSubspace & constraint() const
    {
      return m_constraint.derived();
    }
    inline RefJointMotionSubspace & constraint()
    {
      return m_constraint.derived();
    }

    bool isEqual(const ScaledJointMotionSubspaceTpl & other) const
    {
      return m_constraint == other.m_constraint && m_scaling_factor == other.m_scaling_factor;
    }

  protected:
    RefJointMotionSubspace m_constraint;
    Scalar m_scaling_factor;
    mutable DenseBase S;
  }; // struct ScaledJointMotionSubspaceTpl

  template<typename S1, int O1, typename S2, int O2, int MD2>
  struct MultiplicationOp<InertiaTpl<S1, O1>, ScaledJointMotionSubspaceTpl<S2, O2, MD2>>
  {
    typedef InertiaTpl<S1, O1> Inertia;
    typedef ScaledJointMotionSubspaceTpl<S2, O2, MD2> Constraint;
    typedef typename Constraint::Scalar Scalar;

    typedef Eigen::Matrix<S2, 6, Eigen::Dynamic, O2, 6, MD2> ReturnType;
  };

  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
  namespace impl
  {
    template<typename S1, int O1, typename S2, int O2, int MD2>
    struct LhsMultiplicationOp<InertiaTpl<S1, O1>, ScaledJointMotionSubspaceTpl<S2, O2, MD2>>
    {
      typedef InertiaTpl<S1, O1> Inertia;
      typedef ScaledJointMotionSubspaceTpl<S2, O2, MD2> Constraint;
      typedef typename MultiplicationOp<Inertia, Constraint>::ReturnType ReturnType;

      static inline ReturnType run(const Inertia & Y, const Constraint & scaled_constraint)
      {
        return scaled_constraint.scaling() * (Y * scaled_constraint.constraint());
      }
    };
  } // namespace impl

  template<typename M6Like, typename S2, int O2, int MD2>
  struct MultiplicationOp<Eigen::MatrixBase<M6Like>, ScaledJointMotionSubspaceTpl<S2, O2, MD2>>
  {
    typedef ScaledJointMotionSubspaceTpl<S2, O2, MD2> MotionSubspace;
    typedef Eigen::Matrix<S2, 6, Eigen::Dynamic, O2, 6, MD2> ReturnType;
  };

  /* [ABA] operator* (Inertia Y,Constraint S) */
  namespace impl
  {
    template<typename M6Like, typename S2, int O2, int MD2>
    struct LhsMultiplicationOp<Eigen::MatrixBase<M6Like>, ScaledJointMotionSubspaceTpl<S2, O2, MD2>>
    {
      typedef ScaledJointMotionSubspaceTpl<S2, O2, MD2> Constraint;
      typedef
        typename MultiplicationOp<Eigen::MatrixBase<M6Like>, Constraint>::ReturnType ReturnType;

      static inline ReturnType
      run(const Eigen::MatrixBase<M6Like> & Y, const Constraint & scaled_constraint)
      {
        return scaled_constraint.scaling() * (Y.derived() * scaled_constraint.constraint());
      }
    };
  } // namespace impl

  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct JointMimicTpl;
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct JointModelMimicTpl;
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct JointDataMimicTpl;

  template<typename _Scalar, int _Options, template<typename S, int O> class JointCollectionTpl>
  struct traits<JointMimicTpl<_Scalar, _Options, JointCollectionTpl>>
  {
    typedef _Scalar Scalar;

    enum
    {
      Options = _Options,
      NQ = Eigen::Dynamic,
      NV = Eigen::Dynamic,
      NVExtended = Eigen::Dynamic,
      MaxNVMimicked = 6
    };

    typedef JointCollectionTpl<Scalar, Options> JointCollection;
    typedef JointDataMimicTpl<Scalar, Options, JointCollectionTpl> JointDataDerived;
    typedef JointModelMimicTpl<Scalar, Options, JointCollectionTpl> JointModelDerived;

    typedef ScaledJointMotionSubspaceTpl<Scalar, Options, MaxNVMimicked> Constraint_t;
    typedef SE3Tpl<Scalar, Options> Transformation_t;
    typedef MotionTpl<Scalar, Options> Motion_t;
    typedef MotionTpl<Scalar, Options> Bias_t;

    // [ABA]
    typedef Eigen::Matrix<Scalar, 6, Eigen::Dynamic, Options> U_t;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Options> D_t;
    typedef Eigen::Matrix<Scalar, 6, Eigen::Dynamic, Options> UD_t;

    typedef const Constraint_t & ConstraintTypeConstRef;
    typedef Constraint_t & ConstraintTypeRef;
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

    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1, Options> ConfigVector_t;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1, Options> TangentVector_t;

    typedef const ConfigVector_t & ConfigVectorTypeConstRef;
    typedef ConfigVector_t & ConfigVectorTypeRef;
    typedef const TangentVector_t TangentVectorTypeConstRef;
    typedef TangentVector_t & TangentVectorTypeRef;

    typedef boost::mpl::false_ is_mimicable_t;
  };

  template<typename _Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct traits<JointDataMimicTpl<_Scalar, Options, JointCollectionTpl>>
  {
    typedef JointMimicTpl<_Scalar, Options, JointCollectionTpl> JointDerived;
    typedef _Scalar Scalar;
  };

  template<typename _Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct traits<JointModelMimicTpl<_Scalar, Options, JointCollectionTpl>>
  {
    typedef JointMimicTpl<_Scalar, Options, JointCollectionTpl> JointDerived;
    typedef _Scalar Scalar;
  };

  template<typename _Scalar, int _Options, template<typename S, int O> class JointCollectionTpl>
  struct JointDataMimicTpl
  : public JointDataBase<JointDataMimicTpl<_Scalar, _Options, JointCollectionTpl>>
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef JointDataBase<JointDataMimicTpl> Base;
    typedef JointMimicTpl<_Scalar, _Options, JointCollectionTpl> JointDerived;
    PINOCCHIO_JOINT_DATA_TYPEDEF_TEMPLATE(JointDerived);

    typedef JointDataTpl<_Scalar, _Options, JointCollectionTpl> RefJointData;
    typedef typename RefJointData::JointDataVariant RefJointDataVariant;

    JointDataMimicTpl()
    : S((Scalar)0)
    {
      joint_q.resize(0, 1);
      joint_q_transformed.resize(0, 1);
      joint_v.resize(0, 1);
      joint_v_transformed.resize(0, 1);
    }

    JointDataMimicTpl(
      const RefJointData & jdata, const Scalar & scaling, const int & nq, const int & nv)
    : m_jdata_mimicking(checkMimic(jdata.derived()))
    , S(m_jdata_mimicking.S(), scaling)
    {
      joint_q.resize(nq, 1);
      joint_q_transformed.resize(nq, 1);
      joint_v.resize(nv, 1);
      joint_v_transformed.resize(nv, 1);
    }

    JointDataMimicTpl(const JointDataMimicTpl & other)
    {
      *this = other;
    }

    JointDataMimicTpl & operator=(const JointDataMimicTpl & other)
    {
      m_jdata_mimicking = other.m_jdata_mimicking;
      joint_q = other.joint_q;
      joint_q_transformed = other.joint_q_transformed;
      joint_v = other.joint_v;
      joint_v_transformed = other.joint_v_transformed;
      S = Constraint_t(other.S);
      return *this;
    }

    using Base::isEqual;
    bool isEqual(const JointDataMimicTpl & other) const
    {
      return Base::isEqual(other) && m_jdata_mimicking == other.m_jdata_mimicking
             && joint_q == other.joint_q && joint_q_transformed == other.joint_q_transformed
             && joint_v == other.joint_v && joint_v_transformed == other.joint_v_transformed;
    }

    static std::string classname()
    {
      return std::string("JointDataMimic");
    }

    std::string shortname() const
    {
      return classname();
    }

    // // Accessors
    ConstraintTypeConstRef S_accessor() const
    {
      return S;
    }
    ConstraintTypeRef S_accessor()
    {
      return S;
    }

    Transformation_t M_accessor() const
    {
      return m_jdata_mimicking.M();
    }

    Motion_t v_accessor() const
    {
      return m_jdata_mimicking.v();
    }

    Bias_t c_accessor() const
    {
      return m_jdata_mimicking.c();
    }

    U_t U_accessor() const
    {
      return m_jdata_mimicking.U();
    }

    D_t Dinv_accessor() const
    {
      return m_jdata_mimicking.Dinv();
    }

    UD_t UDinv_accessor() const
    {
      return m_jdata_mimicking.UDinv();
    }

    D_t StU_accessor() const
    {
      return m_jdata_mimicking.StU();
    }

    friend struct JointModelMimicTpl<_Scalar, _Options, JointCollectionTpl>;

    const RefJointData & jdata() const
    {
      return m_jdata_mimicking;
    }
    RefJointData & jdata()
    {
      return m_jdata_mimicking;
    }

    ConfigVectorTypeRef joint_q_accessor()
    {
      return joint_q;
    }
    ConfigVectorTypeConstRef joint_q_accessor() const
    {
      return joint_q;
    }

    ConfigVector_t & q_transformed()
    {
      return joint_q_transformed;
    }
    const ConfigVector_t & q_transformed() const
    {
      return joint_q_transformed;
    }
    TangentVectorTypeRef joint_v_accessor()
    {
      return joint_v;
    }
    TangentVectorTypeConstRef joint_v_accessor() const
    {
      return joint_v;
    }

    TangentVector_t & v_transformed()
    {
      return joint_v_transformed;
    }
    const TangentVector_t & v_transformed() const
    {
      return joint_v_transformed;
    }

    void disp(std::ostream & os) const
    {
      Base::disp(os);
      os << "  Mimicking joint data: " << m_jdata_mimicking.shortname() << std::endl;
    }

    RefJointData m_jdata_mimicking;

    /// \brief original configuration vector
    ConfigVector_t joint_q;
    /// \brief Transformed configuration vector
    ConfigVector_t joint_q_transformed;
    /// \brief original velocity vector
    TangentVector_t joint_v;
    /// \brief Transform velocity vector.
    TangentVector_t joint_v_transformed;
    // data
    Constraint_t S;
  }; // struct JointDataMimicTpl

  template<
    typename NewScalar,
    typename Scalar,
    int Options,
    template<typename S, int O> class JointCollectionTpl>
  struct CastType<NewScalar, JointModelMimicTpl<Scalar, Options, JointCollectionTpl>>
  {
    typedef JointModelMimicTpl<NewScalar, Options, JointCollectionTpl> type;
  };

  template<typename _Scalar, int _Options, template<typename S, int O> class JointCollectionTpl>
  struct JointModelMimicTpl
  : public JointModelBase<JointModelMimicTpl<_Scalar, _Options, JointCollectionTpl>>
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef JointModelBase<JointModelMimicTpl> Base;
    typedef JointMimicTpl<_Scalar, _Options, JointCollectionTpl> JointDerived;
    PINOCCHIO_JOINT_TYPEDEF_TEMPLATE(JointDerived);
    enum
    {
      MaxNVMimicked = traits<JointDerived>::MaxNVMimicked
    };

    typedef JointCollectionTpl<Scalar, Options> JointCollection;
    typedef JointModelTpl<Scalar, Options, JointCollectionTpl> JointModel;

    typedef SE3Tpl<Scalar, Options> SE3;
    typedef MotionTpl<Scalar, Options> Motion;
    typedef InertiaTpl<Scalar, Options> Inertia;

    using Base::id;
    using Base::idx_q;
    using Base::idx_v;
    using Base::idx_vExtended;
    using Base::nq;
    using Base::nv;
    using Base::nvExtended;
    using Base::setIndexes;

    JointModelMimicTpl()
    {
    }

    template<typename JointModel>
    JointModelMimicTpl(
      const JointModelBase<JointModel> & jmodel, const Scalar & scaling, const Scalar & offset)
    : JointModelMimicTpl(jmodel, jmodel, scaling, offset)
    {
    }

    template<typename JointModelMimicking, typename JointModelMimicked>
    JointModelMimicTpl(
      const JointModelBase<JointModelMimicking> & jmodel_mimicking,
      const JointModelBase<JointModelMimicked> & jmodel_mimicked,
      const Scalar & scaling,
      const Scalar & offset)
    : m_jmodel_mimicking(checkMimic((JointModel)jmodel_mimicking.derived()))
    , m_scaling(scaling)
    , m_offset(offset)
    , m_nqExtended(jmodel_mimicking.nq())
    , m_nvExtended(jmodel_mimicking.nvExtended())
    {
      assert(jmodel_mimicking.nq() == jmodel_mimicked.nq());
      assert(jmodel_mimicking.nv() == jmodel_mimicked.nv());
      assert(jmodel_mimicking.nvExtended() == jmodel_mimicked.nvExtended());

      setMimicIndexes(
        jmodel_mimicked.id(), jmodel_mimicked.idx_q(), jmodel_mimicked.idx_v(),
        jmodel_mimicked.idx_vExtended());
    }

    Base & base()
    {
      return *static_cast<Base *>(this);
    }
    const Base & base() const
    {
      return *static_cast<const Base *>(this);
    }

    inline int nq_impl() const
    {
      return 0;
    }
    inline int nv_impl() const
    {
      return 0;
    }
    inline int nvExtended_impl() const
    {
      return m_nvExtended;
    }

    /**
     * @note q and v are ignored in the _impl for mimic joint because most algorithms will pass
     * indexes of their current position in the tree, while in this case idx_q and idx_v should
     * remain pointing to the mimicked joint. (See setMimicIndexes)
     */
    void setIndexes_impl(JointIndex id, int /*q*/, int /*v*/, int vExtended)
    {
      PINOCCHIO_THROW(
        (id > m_jmodel_mimicking.id()), std::invalid_argument,
        "Mimic joint index is lower than its directing joint. Should never happen");
      Base::i_id = id;
      // Base::i_q = q;
      // Base::i_v = v;
      Base::i_vExtended = vExtended;
    }

    /**
     * @brief Specific way for mimic joints to set the mimicked q,v indexes.
     * Used for manipulating tree (e.g. appendModel)
     *
     * @param id Set the mimicking joint id
     * @param q Set the mimic joint idx_q (should point to the mimicked joint)
     * @param v Set the mimic joint idx_v (should point to the mimicked joint)
     * @param vExtended Set the mimicking idx_vExtended
     */
    void setMimicIndexes(JointIndex id, int q, int v, int vExtended)
    {
      // Set idx_q, idx_v to zero because only the corresponding subsegment of q,v are passed to the
      // m_jmodel_mimicking, thus, its indexes starts at 0
      m_jmodel_mimicking.setIndexes(id, 0, 0, vExtended);

      // idx_q, idx_v are kept separately to extract the subsegment
      Base::i_q = q;
      Base::i_v = v;
    }

    JointDataDerived createData() const
    {
      return JointDataDerived(
        m_jmodel_mimicking.createData(), scaling(), m_nqExtended, m_nvExtended);
    }

    const std::vector<bool> hasConfigurationLimit() const
    {
      return m_jmodel_mimicking.hasConfigurationLimit();
    }

    const std::vector<bool> hasConfigurationLimitInTangent() const
    {
      return m_jmodel_mimicking.hasConfigurationLimitInTangent();
    }

    template<typename ConfigVector>
    PINOCCHIO_DONT_INLINE void
    calc(JointDataDerived & jdata, const typename Eigen::MatrixBase<ConfigVector> & qs) const
    {
      jdata.joint_q = qs.segment(Base::i_q, m_nqExtended);
      configVectorAffineTransform(
        m_jmodel_mimicking, jdata.joint_q, m_scaling, m_offset, jdata.joint_q_transformed);
      m_jmodel_mimicking.calc(jdata.m_jdata_mimicking, jdata.joint_q_transformed);
    }

    template<typename ConfigVector, typename TangentVector>
    PINOCCHIO_DONT_INLINE void calc(
      JointDataDerived & jdata,
      const typename Eigen::MatrixBase<ConfigVector> & qs,
      const typename Eigen::MatrixBase<TangentVector> & vs) const
    {
      jdata.joint_q = qs.segment(Base::i_q, m_nqExtended);
      jdata.joint_v = vs.segment(Base::i_v, m_nvExtended);
      configVectorAffineTransform(
        m_jmodel_mimicking, jdata.joint_q, m_scaling, m_offset, jdata.joint_q_transformed);
      jdata.joint_v_transformed = m_scaling * jdata.joint_v;

      m_jmodel_mimicking.calc(
        jdata.m_jdata_mimicking, jdata.joint_q_transformed, jdata.joint_v_transformed);
    }

    template<typename VectorLike, typename Matrix6Like>
    void calc_aba(
      JointDataDerived &,
      const Eigen::MatrixBase<VectorLike> &,
      const Eigen::MatrixBase<Matrix6Like> &,
      const bool) const
    {
      assert(
        false
        && "Joint Mimic is not supported for aba yet. Remove it from your model if you want to use "
           "this function");
    }

    static std::string classname()
    {
      return std::string("JointModelMimic");
    }

    std::string shortname() const
    {
      return classname();
    }

    /// \returns An expression of *this with the Scalar type casted to NewScalar.
    template<typename NewScalar>
    typename CastType<NewScalar, JointModelMimicTpl>::type cast() const
    {
      typedef typename CastType<NewScalar, JointModelMimicTpl>::type ReturnType;
      ReturnType res(
        m_jmodel_mimicking.template cast<NewScalar>(),
        ScalarCast<NewScalar, Scalar>::cast(m_scaling),
        ScalarCast<NewScalar, Scalar>::cast(m_offset));
      res.setIndexes(id(), Base::i_q, Base::i_v, Base::i_vExtended);
      res.setMimicIndexes(m_jmodel_mimicking.id(), Base::i_q, Base::i_v, Base::i_vExtended);
      return res;
    }

    const JointModel & jmodel() const
    {
      return m_jmodel_mimicking;
    }
    JointModel & jmodel()
    {
      return m_jmodel_mimicking;
    }

    const Scalar & scaling() const
    {
      return m_scaling;
    }
    Scalar & scaling()
    {
      return m_scaling;
    }

    const Scalar & offset() const
    {
      return m_offset;
    }
    Scalar & offset()
    {
      return m_offset;
    }

  protected:
    // data
    JointModel m_jmodel_mimicking;
    Scalar m_scaling, m_offset;
    int m_nqExtended, m_nvExtended;

  public:
    /* Acces to dedicated segment in robot config space.  */
    // Const access
    template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::ConstType
    JointMappedConfigSelector_impl(const Eigen::MatrixBase<D> & a) const
    {
      return SizeDepType<NQ>::segment(a.derived(), Base::i_q, m_nqExtended);
    }

    // Non-const access
    template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::Type
    JointMappedConfigSelector_impl(Eigen::MatrixBase<D> & a) const
    {
      return SizeDepType<NQ>::segment(a.derived(), Base::i_q, m_nqExtended);
    }
    /* Acces to dedicated segment in robot tangent space.  */
    // Const access
    template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::ConstType
    JointMappedVelocitySelector_impl(const Eigen::MatrixBase<D> & a) const
    {
      return SizeDepType<NQ>::segment(a.derived(), Base::i_v, m_nvExtended);
    }

    // Non-const access
    template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::Type
    JointMappedVelocitySelector_impl(Eigen::MatrixBase<D> & a) const
    {
      return SizeDepType<NQ>::segment(a.derived(), Base::i_v, m_nvExtended);
    }

    /* Acces to dedicated columns in a ForceSet or MotionSet matrix.*/
    // Const access
    template<typename D>
    typename SizeDepType<NV>::template ColsReturn<D>::ConstType
    jointCols_impl(const Eigen::MatrixBase<D> & A) const
    {
      return SizeDepType<NV>::middleCols(A.derived(), Base::i_v, m_nvExtended);
    }

    // Non-const access
    template<typename D>
    typename SizeDepType<NV>::template ColsReturn<D>::Type
    jointCols_impl(Eigen::MatrixBase<D> & A) const
    {
      return SizeDepType<NV>::middleCols(A.derived(), Base::i_v, m_nvExtended);
    }

    /* Acces to dedicated rows in a matrix.*/
    // Const access
    template<typename D>
    typename SizeDepType<NV>::template RowsReturn<D>::ConstType
    jointRows_impl(const Eigen::MatrixBase<D> & A) const
    {
      return SizeDepType<NV>::middleRows(A.derived(), Base::i_v, m_nvExtended);
    }

    // Non-const access
    template<typename D>
    typename SizeDepType<NV>::template RowsReturn<D>::Type
    jointRows_impl(Eigen::MatrixBase<D> & A) const
    {
      return SizeDepType<NV>::middleRows(A.derived(), Base::i_v, m_nvExtended);
    }

    // /// \brief Returns a block of dimension nv()xnv() located at position idx_v(),idx_v() in the
    // matrix Mat
    // // Const access
    template<typename D>
    typename SizeDepType<NV>::template BlockReturn<D>::ConstType
    jointBlock_impl(const Eigen::MatrixBase<D> & Mat) const
    {
      return SizeDepType<NV>::block(
        Mat.derived(), Base::i_v, Base::i_v, m_nvExtended, m_nvExtended);
    }

    // Non-const access
    template<typename D>
    typename SizeDepType<NV>::template BlockReturn<D>::Type
    jointBlock_impl(Eigen::MatrixBase<D> & Mat) const
    {
      return SizeDepType<NV>::block(
        Mat.derived(), Base::i_v, Base::i_v, m_nvExtended, m_nvExtended);
    }

    void disp(std::ostream & os) const
    {
      Base::disp(os);
      os << "  Mimicking joint type: " << m_jmodel_mimicking.shortname() << std::endl;
      os << "  Mimicked joint id: " << m_jmodel_mimicking.id() << std::endl;
      os << "  Mimic scaling: " << m_scaling << std::endl;
      os << "  Mimic offset: " << m_offset << std::endl;
    }

  }; // struct JointModelMimicTpl

} // namespace pinocchio

#include <boost/type_traits.hpp>

namespace boost
{
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct has_nothrow_constructor<
    ::pinocchio::JointModelMimicTpl<Scalar, Options, JointCollectionTpl>>
  : public integral_constant<bool, true>
  {
  };

  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct has_nothrow_copy<::pinocchio::JointModelMimicTpl<Scalar, Options, JointCollectionTpl>>
  : public integral_constant<bool, true>
  {
  };

  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct has_nothrow_constructor<
    ::pinocchio::JointDataMimicTpl<Scalar, Options, JointCollectionTpl>>
  : public integral_constant<bool, true>
  {
  };

  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct has_nothrow_copy<::pinocchio::JointDataMimicTpl<Scalar, Options, JointCollectionTpl>>
  : public integral_constant<bool, true>
  {
  };
} // namespace boost

#endif // ifndef __pinocchio_multibody_joint_mimic_hpp__
