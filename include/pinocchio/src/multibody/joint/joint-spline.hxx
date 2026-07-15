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
    typedef Eigen::Matrix<_Scalar, Eigen::Dynamic, Eigen::Dynamic> Matrix;

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
    Matrix N;

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
    , N(Matrix::Zero(1, 1))
    {
    }

    // TODO use a better parameter
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
    , N(Matrix::Zero(
        static_cast<Eigen::Index>(nbCtrlFrames), static_cast<Eigen::Index>(nbCtrlFrames)))
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
    , knots(knotVector)
    , ctrlFrames(controlFrames)
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

      size_t knot_multiplicity = 1;
      for (Eigen::Index i = 1; i < knotVector.size(); ++i)
      {
        if (check_expression_if_real<Scalar>(knotVector[i] < knotVector[i - 1]))
        {
          PINOCCHIO_THROW_PRETTY(
            std::invalid_argument, "JointSpline - Knot vector must be non-decreasing (knots must "
                                   "satisfy knots[i] <= knots[i+1]).");
        }

        if (check_expression_if_real<Scalar>(knotVector[i] == knotVector[i - 1]))
        {
          knot_multiplicity++;
          if (knot_multiplicity > degree + 1)
          {
            PINOCCHIO_THROW_PRETTY(
              std::invalid_argument,
              "JointSpline - Knot vector values cannot be repeated more than degree + 1 times.");
          }
        }
        else
        {
          knot_multiplicity = 1;
        }
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

      SpanIndexes indexes = internal::FindSpan<Scalar>::run(qs[0], degree, knots);

      computeBasisFunctions(data, data.joint_q[0], indexes);
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

      SpanIndexes indexes = internal::FindSpan<Scalar>::run(qs[0], degree, knots);

      computeBasisFunctions(data, data.joint_q[0], indexes);
      computeTransformations(data, indexes, true);
    }

    template<typename TangentVector>
    void
    calc(JointDataDerived & data, const Blank, const Eigen::MatrixBase<TangentVector> & vs) const
    {
      data.joint_v = vs.template segment<NV>(idx_v());

      SpanIndexes indexes = internal::FindSpan<Scalar>::run(data.joint_q[0], degree, knots);

      computeBasisFunctions(data, data.joint_q[0], indexes);
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
      res.relativeMotions.reserve(relativeMotions.size());
      for (const auto & cf : ctrlFrames)
      {
        res.ctrlFrames.push_back(cf.template cast<NewScalar>());
      }
      for (const auto & rm : relativeMotions)
      {
        res.relativeMotions.push_back(rm.template cast<NewScalar>());
      }

      res.min_q = ScalarCast<NewScalar, Scalar>::cast(min_q);
      res.max_q = ScalarCast<NewScalar, Scalar>::cast(max_q);
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
      JointDataDerived & data, const Scalar joint_q_val, const SpanIndexes & indexes) const
    {
      // TODO support stop_idx for casadi support
      internal::deBoorBasis(degree, knots, indexes.start_idx, joint_q_val, data.N);
    }

    void computeTransformations(
      JointDataDerived & data, const SpanIndexes & indexes, bool computeVelocity = false) const
    {
      const int start_basis = indexes.start_idx - degree;
      data.M = ctrlFrames[start_basis];
      data.S.matrix().setZero();
      if (computeVelocity)
      {
        data.c.setZero();
        data.v.setZero();
      }

      for (int i = 1; i < degree + 1; i++)
      {
        const int current_basis = start_basis + i;

        const Scalar phi_i = data.N.row(degree).segment(i, degree + 1 - i).sum();

        const Scalar alpha = degree / (knots[current_basis + degree] - knots[current_basis]);
        const Scalar phi_dot_i = alpha * data.N.row(degree - 1)[i - 1];

        const Transformation_t transformation_temp(
          exp6(relativeMotions[current_basis - 1] * phi_i));
        data.M = data.M * transformation_temp;

        if (computeVelocity)
        {
          Scalar phi_ddot_i_sum = Scalar(0);
          if (i > 1)
          {
            const Scalar left_side_den = knots[current_basis + degree] - knots[current_basis];
            phi_ddot_i_sum = data.N.row(degree - 2)[i - 2] / left_side_den;
          }
          if (i < degree)
          {
            const Scalar right_side_den =
              knots[current_basis + degree + 1] - knots[current_basis + 1];
            phi_ddot_i_sum -= data.N.row(degree - 2)[i - 1] / right_side_den;
          }
          const Scalar phi_ddot_i = alpha * degree * phi_ddot_i_sum;

          data.c =
            relativeMotions[current_basis - 1] * phi_ddot_i
            + transformation_temp.actInv(
              data.c
              + Motion_t(data.S.matrix()).cross(relativeMotions[current_basis - 1]) * phi_dot_i);
        }

        data.S.matrix() = transformation_temp.actInv(data.S)
                          + relativeMotions[current_basis - 1].toVector() * phi_dot_i;
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
      for (std::size_t i = 0; i < knots.size(); ++i)
      {
        knots_[i] = knots[i];
      }
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
