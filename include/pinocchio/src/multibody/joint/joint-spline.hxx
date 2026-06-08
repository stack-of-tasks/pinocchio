//
// Copyright (c) 2025 INRIA
// Copyright (c) 2025-2026 ISIR
//

#pragma once

// IWYU pragma: private, include "pinocchio/multibody/joint.hpp"

#ifdef PINOCCHIO_LSP
  #undef PINOCCHIO_LSP
  #include "pinocchio/multibody/joint.hpp"
#endif // PINOCCHIO_LSP

namespace pinocchio
{
  template<typename Scalar, int Options>
  struct JointSplineTpl;

  template<typename _Scalar, int _Options>
  struct traits<JointSplineTpl<_Scalar, _Options>>
  {
    enum
    {
      NQ = 1,
      NV = 1,
      NVExtended = 1
    };
    typedef _Scalar Scalar;
    enum
    {
      Options = _Options
    };
    typedef JointDataSplineTpl<Scalar, Options> JointDataDerived;
    typedef JointModelSplineTpl<Scalar, Options> JointModelDerived;

    typedef JointMotionSubspaceTpl<1, Scalar, Options, 1> Constraint_t;
    typedef SE3Tpl<Scalar, Options> Transformation_t;
    typedef MotionTpl<Scalar, Options> Motion_t;
    typedef MotionTpl<Scalar, Options> Bias_t;

    // [ABA]
    typedef Eigen::Matrix<Scalar, 6, NV, Options> U_t;
    typedef Eigen::Matrix<Scalar, NV, NV, Options> D_t;
    typedef Eigen::Matrix<Scalar, 6, NV, Options> UD_t;

    typedef Eigen::Matrix<Scalar, NQ, 1, Options> ConfigVector_t;
    typedef Eigen::Matrix<Scalar, NV, 1, Options> TangentVector_t;

    typedef boost::mpl::false_ is_mimicable_t;

    PINOCCHIO_JOINT_DATA_BASE_ACCESSOR_DEFAULT_RETURN_TYPE
  };

  template<typename _Scalar, int _Options>
  struct traits<JointDataSplineTpl<_Scalar, _Options>>
  {
    typedef JointSplineTpl<_Scalar, _Options> JointDerived;
    typedef _Scalar Scalar;
  };

  template<typename _Scalar, int _Options>
  struct traits<JointModelSplineTpl<_Scalar, _Options>>
  {
    typedef JointSplineTpl<_Scalar, _Options> JointDerived;
    typedef _Scalar Scalar;
  };

  template<typename _Scalar, int _Options>
  struct JointDataSplineTpl : public JointDataBase<JointDataSplineTpl<_Scalar, _Options>>
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef JointSplineTpl<_Scalar, _Options> JointDerived;
    typedef Eigen::Vector<_Scalar, Eigen::Dynamic> Vector;

    PINOCCHIO_JOINT_DATA_TYPEDEF_TEMPLATE(JointDerived);
    PINOCCHIO_JOINT_DATA_BASE_DEFAULT_ACCESSOR

    ConfigVector_t joint_q;
    TangentVector_t joint_v;

    Constraint_t S;
    Transformation_t M;
    Motion_t v;
    Bias_t c;

    // [ABA] specific data
    U_t U;
    D_t Dinv;
    UD_t UDinv;
    D_t StU;

    // Bspline values
    Vector N;
    Vector N_der;
    Vector N_der2;

    JointDataSplineTpl()
    : joint_q(ConfigVector_t::Zero())
    , joint_v(TangentVector_t::Zero())
    , M(Transformation_t::Identity())
    , v(Motion_t::Zero())
    , c(Bias_t::Zero())
    , U(U_t::Zero())
    , Dinv(D_t::Zero())
    , UDinv(UD_t::Identity())
    , StU(D_t::Zero())
    , N(Vector::Zero(1))
    , N_der(Vector::Zero(1))
    , N_der2(Vector::Zero(1))
    {
    }

    JointDataSplineTpl(const size_t nbCtrlFrames)
    : joint_q(ConfigVector_t::Zero())
    , joint_v(TangentVector_t::Zero())
    , M(Transformation_t::Identity())
    , v(Motion_t::Zero())
    , c(Bias_t::Zero())
    , U(U_t::Zero())
    , Dinv(D_t::Zero())
    , UDinv(UD_t::Identity())
    , StU(D_t::Zero())
    , N(Vector::Zero(nbCtrlFrames))
    , N_der(Vector::Zero(nbCtrlFrames))
    , N_der2(Vector::Zero(nbCtrlFrames))
    {
    }

    static std::string classname()
    {
      return std::string("JointDataSpline");
    }
    std::string shortname() const
    {
      return classname();
    }

  }; // struct JointDataSplinerTpl

  PINOCCHIO_JOINT_CAST_TYPE_SPECIALIZATION(JointModelSplineTpl);

  /// @brief Spline joint in \f$SE(3)\f$.
  ///
  /// A spline joint constrains the movement of the child frame to follow the spline defined by the
  /// controlFrames and the degree of the spline. Implementation of the joint is based on the paper
  /// from Lee et al. Spline Joints for Multibody Dynamics
  /// (https://web.cs.ucla.edu/~dt/papers/siggraph08/siggraph08.pdf)
  ///
  template<typename _Scalar, int _Options>
  struct JointModelSplineTpl : public JointModelBase<JointModelSplineTpl<_Scalar, _Options>>
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef JointSplineTpl<_Scalar, _Options> JointDerived;
    typedef Eigen::Vector<_Scalar, Eigen::Dynamic> Vector;
    typedef internal::SpanIndexes SpanIndexes;
    PINOCCHIO_JOINT_TYPEDEF_TEMPLATE(JointDerived);

    typedef JointModelBase<JointModelSplineTpl> Base;
    using Base::id;
    using Base::idx_q;
    using Base::idx_v;
    using Base::idx_vExtended;
    using Base::setIndexes;

    JointModelSplineTpl()
    : degree(3)
    , nbCtrlFrames(0)
    , min_q(Scalar(0))
    , max_q(Scalar(1))
    {
    }

    JointModelSplineTpl(
      const std::vector<Transformation_t> & controlFrames,
      const Vector & knotVector,
      const size_t degree)
    : degree(degree)
    , ctrlFrames(controlFrames)
    , knots(knotVector)
    {
      if (controlFrames.size() <= degree)
        PINOCCHIO_THROW_PRETTY(
          std::invalid_argument,
          "JointSpline - Number of control frames must be greater than degree of spline.");

      nbCtrlFrames = controlFrames.size();
      if (knotVector.size() != static_cast<Eigen::Index>(nbCtrlFrames + degree + 1))
        PINOCCHIO_THROW_PRETTY(
          std::invalid_argument,
          "JointSpline - Size of knot vector should be nbControlFrames + degree + 1.");

      for (Eigen::Index i = 1; i < knotVector.size(); ++i)
      {
        if (!check_expression_if_real<Scalar>(knotVector[i] >= knotVector[i - 1]))
          PINOCCHIO_THROW_PRETTY(
            std::invalid_argument, "JointSpline - Knot vector must be non-decreasing (knots must "
                                   "satisfy knots[i] <= knots[i+1]).");
      }

      min_q = knotVector[0];
      max_q = knotVector[knots.size() - 1];

      computeRelativeMotions();
    }

    JointDataDerived createData() const
    {
      return JointDataDerived(nbCtrlFrames);
    }

    const std::vector<bool> hasConfigurationLimit() const
    {
      return {true};
    }

    const std::vector<bool> hasConfigurationLimitInTangent() const
    {
      return {true};
    }

    using Base::isEqual;
    bool isEqual(const JointModelSplineTpl & other) const
    {
      return Base::isEqual(other) && other.degree == degree && other.nbCtrlFrames == nbCtrlFrames
             && other.ctrlFrames == ctrlFrames && other.knots == knots
             && other.relativeMotions == relativeMotions;
    }

    template<typename ConfigVector>
    void calc(JointDataDerived & data, const Eigen::MatrixBase<ConfigVector> & qs) const
    {
      assert(
        check_expression_if_real<Scalar>(qs[0] >= min_q && qs[0] <= max_q)
        && "Spline joint configuration (q) must be between min_q and max_q. ");

      data.joint_q = qs.template segment<NQ>(idx_q());

      SpanIndexes indexes =
        internal::FindSpan<Scalar, Options>::run(qs, degree, nbCtrlFrames, knots);

      computeBasisFunctions(data, data.joint_q[0], indexes, false);
      computeTransformations(data, indexes, false);
    }

    template<typename ConfigVector, typename TangentVector>
    void calc(
      JointDataDerived & data,
      const Eigen::MatrixBase<ConfigVector> & qs,
      const Eigen::MatrixBase<TangentVector> & vs) const
    {
      assert(
        check_expression_if_real<Scalar>(qs[0] >= min_q && qs[0] <= max_q)
        && "Spline joint configuration (q) must be between min_q and max_q. ");

      data.joint_q = qs.template segment<NQ>(idx_q());
      data.joint_v = vs.template segment<NV>(idx_v());

      SpanIndexes indexes =
        internal::FindSpan<Scalar, Options>::run(qs, degree, nbCtrlFrames, knots);

      computeBasisFunctions(data, data.joint_q[0], indexes, true);
      computeTransformations(data, indexes, true);
    }

    template<typename TangentVector>
    void
    calc(JointDataDerived & data, const Blank, const Eigen::MatrixBase<TangentVector> & vs) const
    {
      data.joint_v = vs.template segment<NV>(idx_v());

      SpanIndexes indexes =
        internal::FindSpan<Scalar, Options>::run(data.joint_q, degree, nbCtrlFrames, knots);

      computeBasisFunctions(data, data.joint_q[0], indexes, true);
      computeTransformations(data, indexes, true);
    }

    template<typename VectorLike, typename Matrix6Like>
    void calc_aba(
      JointDataDerived & data,
      const Eigen::MatrixBase<VectorLike> & armature,
      const Eigen::MatrixBase<Matrix6Like> & I,
      const bool update_I) const
    {
      data.U.noalias() = I * data.S.matrix();
      data.StU.noalias() = data.S.transpose() * data.U;
      data.StU.diagonal() += armature;
      internal::matrix_inversion(data.StU, data.Dinv);

      data.UDinv.noalias() = data.U * data.Dinv;

      if (update_I)
        PINOCCHIO_EIGEN_CONST_CAST(Matrix6Like, I).noalias() -= data.UDinv * data.U.transpose();
    }

    static std::string classname()
    {
      return std::string("JointModelSpline");
    }
    std::string shortname() const
    {
      return classname();
    }

    template<typename NewScalar>
    JointModelSplineTpl<NewScalar, Options> cast() const
    {
      typedef JointModelSplineTpl<NewScalar, Options> ReturnType;
      ReturnType res;
      res.degree = degree;
      res.nbCtrlFrames = nbCtrlFrames;
      res.ctrlFrames.reserve(ctrlFrames.size());
      for (size_t k = 0; k < ctrlFrames.size(); k++)
        res.ctrlFrames.push_back(ctrlFrames[k].template cast<NewScalar>());

      res.min_q = static_cast<NewScalar>(min_q);
      res.max_q = static_cast<NewScalar>(max_q);
      res.knots = knots.template cast<NewScalar>();

      res.setIndexes(id(), idx_q(), idx_v(), idx_vExtended());
      return res;
    }

    // attributes
    size_t degree;
    size_t nbCtrlFrames;
    Vector knots;
    Scalar min_q;
    Scalar max_q;

    std::vector<Transformation_t> ctrlFrames;
    std::vector<Motion_t> relativeMotions;

  private:
    void computeRelativeMotions()
    {
      for (size_t i = 0; i < nbCtrlFrames - 1; i++)
        relativeMotions.push_back(log6(ctrlFrames[i].inverse() * ctrlFrames[i + 1]));
    }

    void computeBasisFunctions(
      JointDataDerived & data,
      const Scalar joint_q_val,
      const SpanIndexes & indexes,
      bool computeSecondDerivative = false) const
    {
      data.N.setZero();
      data.N_der.setZero();
      if (computeSecondDerivative)
        data.N_der2.setZero();

      for (size_t i = indexes.start_idx; i < indexes.end_idx; i++)
      {
        data.N[i] = internal::bsplineBasis(i, degree, joint_q_val, knots);
        data.N_der[i] = internal::bsplineBasisDerivative(i, degree, joint_q_val, knots);
        if (computeSecondDerivative)
          data.N_der2[i] = internal::bsplineBasisDerivative2(i, degree, joint_q_val, knots);
      }
    }

    void computeTransformations(
      JointDataDerived & data, const SpanIndexes & indexes, bool computeVelocity = false) const
    {
      data.M = ctrlFrames[indexes.start_idx];
      data.S.matrix().setZero();
      if (computeVelocity)
      {
        data.c.setZero();
        data.v.setZero();
      }

      for (size_t i = indexes.start_idx + 1; i < indexes.end_idx; i++)
      {
        const Scalar phi_i = data.N.segment(i, indexes.end_idx - i).sum();
        const Scalar phi_dot_i = data.N_der.segment(i, indexes.end_idx - i).sum();

        const Transformation_t transformation_temp(exp6(relativeMotions[i - 1] * phi_i));
        data.M = data.M * transformation_temp;

        if (computeVelocity)
        {
          const Scalar phi_ddot_i = data.N_der2.segment(i, indexes.end_idx - i).sum();
          data.c = relativeMotions[i - 1] * phi_ddot_i
                   + transformation_temp.actInv(
                     data.c + Motion_t(data.S.matrix()).cross(relativeMotions[i - 1]) * phi_dot_i);
        }

        data.S.matrix() =
          transformation_temp.actInv(data.S) + relativeMotions[i - 1].toVector() * phi_dot_i;
      }
      if (computeVelocity)
      {
        // C = Sdot * qdot = (dS/dq * qdot) * dot
        data.c = data.c * data.joint_v[0] * data.joint_v[0];
        data.v = data.S * data.joint_v;
      }
    }
  }; // struct JointModelSplineTpl

  /// @brief Helper structure to specify attributes of a spline joint.
  template<typename _Scalar, int _Options>
  struct JointModelSplineBuilderTpl
  {
    using Scalar = _Scalar;
    static constexpr int Options = _Options;

    using JointModel_t = JointModelSplineTpl<Scalar, Options>;
    using Transformation_t = typename JointModel_t::Transformation_t;
    using Vector = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

    enum class KnotPolicy
    {
      OpenUniform,
      Uniform,
      Custom
    };

    JointModelSplineBuilderTpl()
    : degree_(3)
    , min_q_(Scalar(0))
    , max_q_(Scalar(1))
    , knot_policy_(KnotPolicy::OpenUniform)
    {
    }

    JointModelSplineBuilderTpl & addControlFrame(const Transformation_t & frame)
    {
      ctrlFrames_.push_back(frame);
      return *this;
    }

    JointModelSplineBuilderTpl &
    withControlFrameVector(const std::vector<Transformation_t> & frames)
    {
      ctrlFrames_ = frames;
      return *this;
    }

    JointModelSplineBuilderTpl & withDegree(size_t degree)
    {
      degree_ = degree;
      return *this;
    }

    JointModelSplineBuilderTpl & withKnotVector(const Vector & knots)
    {
      knots_ = knots;
      knot_policy_ = KnotPolicy::Custom;

      return *this;
    }

    JointModelSplineBuilderTpl & withKnotVector(const std::vector<Scalar> & knots)
    {
      knots_.resize(knots.size());
      for (size_t i = 0; i < knots.size(); ++i)
        knots_[i] = knots[i];
      knot_policy_ = KnotPolicy::Custom;

      return *this;
    }

    JointModelSplineBuilderTpl & withOpenUniformKnots(const Scalar min_q, const Scalar max_q)
    {
      knot_policy_ = KnotPolicy::OpenUniform;
      min_q_ = min_q;
      max_q_ = max_q;

      return *this;
    }

    JointModelSplineBuilderTpl & withUniformKnots(const Scalar min_q, const Scalar max_q)
    {
      knot_policy_ = KnotPolicy::Uniform;
      min_q_ = min_q;
      max_q_ = max_q;

      return *this;
    }

    JointModel_t build() const
    {
      Vector knots;

      const size_t nCtrl = ctrlFrames_.size();
      switch (knot_policy_)
      {
      case KnotPolicy::OpenUniform:
        knots = internal::generateOpenUniformKnots(min_q_, max_q_, nCtrl, degree_);
        break;

      case KnotPolicy::Uniform:
        knots = internal::generateUniformKnots(min_q_, max_q_, nCtrl, degree_);
        break;

      case KnotPolicy::Custom:
        knots = knots_;
        break;
      default:
        break;
      }
      return JointModel_t(ctrlFrames_, knots, degree_);
    }

  private:
    std::vector<Transformation_t> ctrlFrames_;

    size_t degree_;

    Scalar min_q_;
    Scalar max_q_;

    Vector knots_;
    KnotPolicy knot_policy_;
  }; // struct JointModelSplineBuilder

} // namespace pinocchio

#include <boost/type_traits.hpp>

namespace boost
{
  template<typename Scalar, int Options>
  struct has_nothrow_constructor<::pinocchio::JointModelSplineTpl<Scalar, Options>>
  : public integral_constant<bool, true>
  {
  };

  template<typename Scalar, int Options>
  struct has_nothrow_copy<::pinocchio::JointModelSplineTpl<Scalar, Options>>
  : public integral_constant<bool, true>
  {
  };

  template<typename Scalar, int Options>
  struct has_nothrow_constructor<::pinocchio::JointDataSplineTpl<Scalar, Options>>
  : public integral_constant<bool, true>
  {
  };

  template<typename Scalar, int Options>
  struct has_nothrow_copy<::pinocchio::JointDataSplineTpl<Scalar, Options>>
  : public integral_constant<bool, true>
  {
  };
} // namespace boost
