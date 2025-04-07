//
// Copyright (c) 2015-2020 CNRS INRIA
// Copyright (c) 2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_multibody_constraint_generic_hpp__
#define __pinocchio_multibody_constraint_generic_hpp__

namespace pinocchio
{

  template<int _Dim, typename _Scalar, int _Options, int _MaxDim>
  struct traits<JointMotionSubspaceTpl<_Dim, _Scalar, _Options, _MaxDim>>
  {
    typedef _Scalar Scalar;
    enum
    {
      LINEAR = 0,
      ANGULAR = 3,
      Options = _Options,
      Dim = _Dim
    };

    typedef MotionTpl<Scalar, Options> JointMotion;
    typedef Eigen::Matrix<Scalar, Dim, 1, Options, _MaxDim, 1> JointForce;
    typedef Eigen::Matrix<Scalar, 6, Dim, Options, 6, _MaxDim> DenseBase;
    typedef Eigen::Matrix<Scalar, Dim, Dim, Options, _MaxDim, _MaxDim> ReducedSquaredMatrix;

    typedef typename PINOCCHIO_EIGEN_REF_CONST_TYPE(DenseBase) ConstMatrixReturnType;
    typedef typename PINOCCHIO_EIGEN_REF_TYPE(DenseBase) MatrixReturnType;

    typedef ReducedSquaredMatrix StDiagonalMatrixSOperationReturnType;
  }; // traits JointMotionSubspaceTpl

  template<int Dim, typename Scalar, int Options, int _MaxDim>
  struct SE3GroupAction<JointMotionSubspaceTpl<Dim, Scalar, Options, _MaxDim>>
  {
    typedef Eigen::Matrix<Scalar, 6, Dim, Options, 6, _MaxDim> ReturnType;
  };

  template<int Dim, typename Scalar, int Options, int MaxDim, typename MotionDerived>
  struct MotionAlgebraAction<JointMotionSubspaceTpl<Dim, Scalar, Options, MaxDim>, MotionDerived>
  {
    typedef Eigen::Matrix<Scalar, 6, Dim, Options, 6, MaxDim> ReturnType;
  };

  template<int Dim, typename Scalar, int Options, int MaxDim, typename ForceDerived>
  struct ConstraintForceOp<JointMotionSubspaceTpl<Dim, Scalar, Options, MaxDim>, ForceDerived>
  {
    typedef
      typename traits<JointMotionSubspaceTpl<Dim, Scalar, Options, MaxDim>>::DenseBase DenseBase;
    typedef Eigen::Matrix<Scalar, Dim, Dim, Options, MaxDim, MaxDim> ReturnType;
  };

  template<int Dim, typename Scalar, int Options, int MaxDim, typename ForceSet>
  struct ConstraintForceSetOp<JointMotionSubspaceTpl<Dim, Scalar, Options, MaxDim>, ForceSet>
  {
    typedef
      typename traits<JointMotionSubspaceTpl<Dim, Scalar, Options, MaxDim>>::DenseBase DenseBase;
    typedef
      typename MatrixMatrixProduct<Eigen::Transpose<const DenseBase>, ForceSet>::type ReturnType;
  };

  template<int _Dim, typename _Scalar, int _Options, int _MaxDim>
  struct JointMotionSubspaceTpl
  : public JointMotionSubspaceBase<JointMotionSubspaceTpl<_Dim, _Scalar, _Options, _MaxDim>>
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef JointMotionSubspaceBase<JointMotionSubspaceTpl> Base;

    friend class JointMotionSubspaceBase<JointMotionSubspaceTpl>;
    PINOCCHIO_CONSTRAINT_TYPEDEF_TPL(JointMotionSubspaceTpl)

    enum
    {
      NV = _Dim
    };

    constexpr static int MaxNV = NV < 0 ? _MaxDim : NV;

    using Base::nv;

    template<typename D>
    explicit JointMotionSubspaceTpl(const Eigen::MatrixBase<D> & _S)
    : S(_S)
    {
      // There is currently a bug in Eigen/Core/util/StaticAssert.h in the use of the full namespace
      // path
      // TODO
      EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(DenseBase, D);
    }

    JointMotionSubspaceTpl()
    : S()
    {
      EIGEN_STATIC_ASSERT(
        _Dim != Eigen::Dynamic, YOU_CALLED_A_DYNAMIC_SIZE_METHOD_ON_A_FIXED_SIZE_MATRIX_OR_VECTOR)
    }

    // It is only valid for dynamics size
    explicit JointMotionSubspaceTpl(const int dim)
    : S(6, dim)
    {
      EIGEN_STATIC_ASSERT(
        _Dim == Eigen::Dynamic, YOU_CALLED_A_FIXED_SIZE_METHOD_ON_A_DYNAMIC_SIZE_MATRIX_OR_VECTOR);
      assert(_MaxDim < 0 || dim <= _MaxDim);
    }

    // It is only valid for dynamics size
    template<int D, int MD>
    explicit JointMotionSubspaceTpl(const JointMotionSubspaceTpl<D, _Scalar, _Options, MD> subspace)
    : S(subspace.matrix())
    {
      EIGEN_STATIC_ASSERT(
        _Dim == Eigen::Dynamic, YOU_CALLED_A_FIXED_SIZE_METHOD_ON_A_DYNAMIC_SIZE_MATRIX_OR_VECTOR);
      assert(_MaxDim < 0 || subspace.matrix().cols() <= _MaxDim);
    }

    static JointMotionSubspaceTpl Zero(const int dim)
    {
      return JointMotionSubspaceTpl(dim);
    }

    template<typename VectorLike>
    JointMotion __mult__(const Eigen::MatrixBase<VectorLike> & vj) const
    {
      return JointMotion(S * vj);
    }

    struct Transpose : JointMotionSubspaceTransposeBase<JointMotionSubspaceTpl>
    {
      const JointMotionSubspaceTpl & ref;
      Transpose(const JointMotionSubspaceTpl & ref)
      : ref(ref)
      {
      }

      template<typename ForceDerived>
      typename ConstraintForceOp<JointMotionSubspaceTpl, ForceDerived>::ReturnType
      operator*(const ForceDense<ForceDerived> & f) const
      {
        return (ref.S.transpose() * f.toVector()).eval();
      }

      template<typename ForceSet>
      typename ConstraintForceSetOp<JointMotionSubspaceTpl, ForceSet>::ReturnType
      operator*(const Eigen::MatrixBase<ForceSet> & F)
      {
        return ref.S.transpose() * F.derived();
      }
    };

    Transpose transpose() const
    {
      return Transpose(*this);
    }

    MatrixReturnType matrix_impl()
    {
      return S;
    }
    ConstMatrixReturnType matrix_impl() const
    {
      return S;
    }

    int nv_impl() const
    {
      return (int)S.cols();
    }

    template<typename S2, int O2>
    friend typename JointMotionSubspaceTpl<_Dim, _Scalar, _Options, _MaxDim>::DenseBase
    operator*(const InertiaTpl<S2, O2> & Y, const JointMotionSubspaceTpl & S)
    {
      typedef typename JointMotionSubspaceTpl::DenseBase ReturnType;
      ReturnType res(6, S.nv());
      motionSet::inertiaAction(Y, S.S, res);
      return res;
    }

    template<typename S2, int O2>
    friend Eigen::Matrix<_Scalar, 6, _Dim, _Options, 6, _MaxDim>
    operator*(const Eigen::Matrix<S2, 6, 6, O2> & Ymatrix, const JointMotionSubspaceTpl & S)
    {
      typedef Eigen::Matrix<_Scalar, 6, _Dim, _Options, 6, _MaxDim> ReturnType;
      return ReturnType(Ymatrix * S.matrix());
    }

    DenseBase se3Action(const SE3Tpl<Scalar, Options> & m) const
    {
      DenseBase res(6, nv());
      motionSet::se3Action(m, S, res);
      return res;
    }

    DenseBase se3ActionInverse(const SE3Tpl<Scalar, Options> & m) const
    {
      DenseBase res(6, nv());
      motionSet::se3ActionInverse(m, S, res);
      return res;
    }

    template<typename MotionDerived>
    DenseBase motionAction(const MotionDense<MotionDerived> & v) const
    {
      DenseBase res(6, nv());
      motionSet::motionAction(v, S, res);
      return res;
    }

    void disp_impl(std::ostream & os) const
    {
      os << "S =\n" << S << std::endl;
    }

    bool isEqual(const JointMotionSubspaceTpl & other) const
    {
      return S == other.S;
    }

  protected:
    DenseBase S;
  }; // class JointMotionSubspaceTpl

  namespace details
  {
    template<int Dim, typename Scalar, int Options, int MaxDim>
    struct StDiagonalMatrixSOperation<JointMotionSubspaceTpl<Dim, Scalar, Options, MaxDim>>
    {
      typedef JointMotionSubspaceTpl<Dim, Scalar, Options, MaxDim> Constraint;
      typedef typename traits<Constraint>::StDiagonalMatrixSOperationReturnType ReturnType;

      static ReturnType run(const JointMotionSubspaceBase<Constraint> & constraint)
      {
        return constraint.matrix().transpose() * constraint.matrix();
      }
    };
  } // namespace details

} // namespace pinocchio

#endif // ifndef __pinocchio_multibody_constraint_generic_hpp__
