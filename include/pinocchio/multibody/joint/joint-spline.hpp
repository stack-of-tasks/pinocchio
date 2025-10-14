//
// Copyright (c) 2025 INRIA
//

#ifndef __pinocchio_multibody_joint_spline_hpp__
#define __pinocchio_multibody_joint_spline_hpp__

#include "pinocchio/macros.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/joint-motion-subspace.hpp"
#include "pinocchio/math/fwd.hpp"
#include "pinocchio/math/quaternion.hpp"

#include <iostream>

namespace pinocchio
{
  struct SpanIndexes
  {
    int start_idx;
    int end_idx;
  };

  template<typename Scalar, int Options>
  struct FindSpan
  {
    static SpanIndexes run(
      const Eigen::Vector<Scalar, Eigen::Dynamic> & /*q*/,
      const int /*degree*/,
      const int nbCtrlFrames,
      const Eigen::Vector<Scalar, Eigen::Dynamic> & /*knots*/)
    {
      return {0, nbCtrlFrames};
    }
  };

  template<int Options>
  struct FindSpan<double, Options>
  {
    static SpanIndexes run(
      const Eigen::Vector<double, Eigen::Dynamic> & q,
      const int degree,
      const int nbCtrlFrames,
      const Eigen::Vector<double, Eigen::Dynamic> & knots)
    {
      // Edge case: if q is at or beyond the end of the spline parameterization
      if (q[0] >= 1.0)
        return {nbCtrlFrames - (degree + 1), nbCtrlFrames};

      int low = degree;
      int high = nbCtrlFrames;
      int mid;

      while (low < high)
      {
        mid = low + (high - low) / 2;
        if (q[0] < knots[mid])
          high = mid;
        else
          low = mid + 1;
      }

      return {low - 1 - degree, low + 1};
    }
  };

  // template<typename Scalar, int Options>
  // struct FindSpan
  // {
  //   static SpanIndexes run(
  //     const Eigen::Vector<Scalar, Options> & /*q*/,
  //     const int degree,
  //     const int nbCtrlFrames,
  //     const Eigen::Vector<Scalar, Options> & knots)
  //   {
  //     return {0, nbCtrlFrames};
  //   }

  //   static SpanIndexes run(
  //     const Eigen::Vector<double, Options> & q,
  //     const int degree,
  //     const int nbCtrlFrames,
  //     const Eigen::Vector<double, Options> & knots)
  //   {
  //     // Edge case

  //     if (q >= 1.0)
  //       return {0, nbCtrlFrames - 1};

  //     int low = degree;
  //     int high = nbCtrlFrames;
  //     int mid = low;

  //     while (low < high)
  //     {
  //       mid = low + (high - low) / 2;
  //       if (q < knots[mid])
  //         high = mid;
  //       else
  //         low = mid + 1;
  //     }

  //     return {low - (degree+ 1), low + 1};
  //   }
  // };

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
    // typedef JointMotionSubspace1d Constraint_t;
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
    , c(Motion_t::Zero())
    , U(U_t::Zero())
    , Dinv(D_t::Zero())
    , UDinv(UD_t::Identity())
    , StU(D_t::Zero())
    , N(Vector::Zero(1))
    , N_der(Vector::Zero(1))
    , N_der2(Vector::Zero(1))
    {
    }

    JointDataSplineTpl(const int nbCtrlFrames)
    : joint_q(ConfigVector_t::Zero())
    , joint_v(TangentVector_t::Zero())
    , M(Transformation_t::Identity())
    , v(Motion_t::Zero())
    , c(Motion_t::Zero())
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
  template<typename _Scalar, int _Options>
  struct JointModelSplineTpl : public JointModelBase<JointModelSplineTpl<_Scalar, _Options>>
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef JointSplineTpl<_Scalar, _Options> JointDerived;
    typedef SE3Tpl<_Scalar, _Options> SE3;
    typedef MotionTpl<_Scalar, _Options> Motion;
    typedef Eigen::Vector<_Scalar, Eigen::Dynamic> Vector;
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
    {
    }

    JointModelSplineTpl(
      const PINOCCHIO_ALIGNED_STD_VECTOR(SE3) & controlFrames, const int degree = 3)
    : degree(degree)
    , nbCtrlFrames(controlFrames.size())
    , ctrlFrames(controlFrames)
    {
      makeKnots();
      computeRelativeMotions();
    }

    void addControlFrame(const SE3 & frame)
    {
      ctrlFrames.push_back(frame);
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
             && other.ctrlFrames == ctrlFrames;
    }

    template<typename ConfigVector>
    void calc(JointDataDerived & data, const typename Eigen::MatrixBase<ConfigVector> & qs) const
    {
      data.joint_q = qs.template segment<NQ>(idx_q());
      SpanIndexes indexes = FindSpan<Scalar, Options>::run(qs, degree, nbCtrlFrames, knots);

      // Basis functions and their derivatives
      data.N.setZero();
      data.N_der.setZero();
      for (size_t i = indexes.start_idx; i < indexes.end_idx; i++)
      {
        data.N[i] = bsplineBasis(i, degree, data.joint_q[0]);
        data.N_der[i] = bsplineBasisDerivative(i, degree, data.joint_q[0]);
      }
      // Compute joint transform M
      data.M = ctrlFrames[indexes.start_idx];
      // joint subspace S
      data.S.matrix().setZero();
      for (size_t i = indexes.start_idx + 1; i < indexes.end_idx; i++)
      {
        const Scalar phi_i = data.N.segment(i, indexes.end_idx - i).sum();
        const Scalar phi_dot_i = data.N_der.segment(i, indexes.end_idx - i).sum();

        data.M = data.M * exp6(relativeMotions[i - 1] * phi_i);
        data.S.matrix() = exp6(relativeMotions[i - 1] * phi_i).actInv(data.S)
                          + relativeMotions[i - 1].toVector() * phi_dot_i;
      }
    }

    template<typename ConfigVector, typename TangentVector>
    void calc(
      JointDataDerived & data,
      const typename Eigen::MatrixBase<ConfigVector> & qs,
      const typename Eigen::MatrixBase<TangentVector> & vs) const
    {
      data.joint_q = qs.template segment<NQ>(idx_q());
      data.joint_v = vs.template segment<NV>(idx_v());
      SpanIndexes indexes = FindSpan<Scalar, Options>::run(qs, degree, nbCtrlFrames, knots);

      // Basis functions and their derivatives
      data.N.setZero();
      data.N_der.setZero();
      data.N_der2.setZero();
      for (size_t i = indexes.start_idx; i < indexes.end_idx; i++)
      {
        data.N[i] = bsplineBasis(i, degree, data.joint_q[0]);
        data.N_der[i] = bsplineBasisDerivative(i, degree, data.joint_q[0]);
        data.N_der2[i] = bsplineBasisDerivative2(i, degree, data.joint_q[0]);
      }

      // Compute time derivative of S (for bias acceleration c)
      data.S.matrix().setZero();
      data.c.setZero();
      data.M = ctrlFrames[indexes.start_idx];
      for (size_t i = indexes.start_idx + 1; i < indexes.end_idx; i++)
      {
        const Scalar phi_i = data.N.segment(i, indexes.end_idx - i).sum();
        const Scalar phi_dot_i = data.N_der.segment(i, indexes.end_idx - i).sum();
        const Scalar phi_ddot_i = data.N_der2.segment(i, indexes.end_idx - i).sum();

        data.M = data.M * exp6(relativeMotions[i - 1] * phi_i);
        data.c =
          relativeMotions[i - 1] * phi_ddot_i
          + exp6(relativeMotions[i - 1] * phi_i)
              .actInv(data.c + Motion(data.S.matrix()).cross(relativeMotions[i - 1]) * phi_dot_i);
        data.S.matrix() = exp6(relativeMotions[i - 1] * phi_i).actInv(data.S)
                          + relativeMotions[i - 1].toVector() * phi_dot_i;
      }

      data.c = data.c * data.joint_v[0];
      data.v = data.S * data.joint_v;
    }

    template<typename TangentVector>
    void calc(
      JointDataDerived & data,
      const Blank not_used,
      const typename Eigen::MatrixBase<TangentVector> & vs) const
    {
      data.joint_v = vs.template segment<NV>(idx_v());

      // Basis functions and their derivatives
      data.N.setZero();
      data.N_der.setZero();
      data.N_der2.setZero();
      for (int i = 0; i < nbCtrlFrames; ++i)
      {
        data.N[i] = bsplineBasis(i, degree, data.joint_q[0]);
        data.N_der[i] = bsplineBasisDerivative(i, degree, data.joint_q[0]);
        data.N_der2[i] = bsplineBasisDerivative2(i, degree, data.joint_q[0]);
      }

      // Compute time derivative of S (for bias acceleration c)
      data.S.matrix().setZero();
      data.c.setZero();
      for (int i = 0; i < nbCtrlFrames - 1; ++i)
      {
        const Scalar phi_i = data.N.tail(nbCtrlFrames - (i + 1)).sum();
        const Scalar phi_dot_i = data.N_der.tail(nbCtrlFrames - (i + 1)).sum();
        const Scalar phi_ddot_i = data.N_der2.tail(nbCtrlFrames - (i + 1)).sum();

        data.c =
          relativeMotions[i] * phi_ddot_i
          + exp6(relativeMotions[i] * phi_i)
              .actInv(data.c + Motion(data.S.matrix()).cross(relativeMotions[i]) * phi_dot_i);
        data.S.matrix() = exp6(relativeMotions[i] * phi_i).actInv(data.S)
                          + relativeMotions[i].toVector() * phi_dot_i;
      }

      data.c = data.c * data.joint_v[0];
      data.v = data.S * data.joint_v;
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
      internal::PerformStYSInversion<Scalar>::run(data.StU, data.Dinv);

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
      for (size_t k = 0; k < ctrlFrames.size(); k++)
      {
        res.ctrlFrames.push_back(ctrlFrames[k].template cast<NewScalar>());
      }
      res.makeKnots();
      res.computeRelativeMotions();
      res.setIndexes(id(), idx_q(), idx_v(), idx_vExtended());
      return res;
    }

    void makeKnots()
    {
      const int n_knots = nbCtrlFrames + degree + 1;
      knots.resize(n_knots);
      knots.head(degree + 1).setZero();
      const Scalar denominator = static_cast<Scalar>(nbCtrlFrames - degree + 1);

      for (size_t i = degree + 1; i < nbCtrlFrames; i++)
        knots[i] = static_cast<Scalar>(i - degree) / denominator;

      knots.tail(degree + 1).setOnes();
    }

    void computeRelativeMotions()
    {
      for (size_t i = 0; i < nbCtrlFrames - 1; i++)
        relativeMotions.push_back(log6(ctrlFrames[i].inverse() * ctrlFrames[i + 1]));
    }

    // attributes
    int degree;
    int nbCtrlFrames;
    Vector knots;
    PINOCCHIO_ALIGNED_STD_VECTOR(SE3) ctrlFrames;
    PINOCCHIO_ALIGNED_STD_VECTOR(Motion) relativeMotions;

  private:
    Scalar bsplineBasis(int i, int k, const Scalar x) const
    {
      // Base case of the recursion (structural control flow, remains an `if`)
      if (k == 0)
      {
        // Replicate (knots[i] <= x && x <= knots[i+1]) using nested calls
        return pinocchio::internal::if_then_else(
          pinocchio::internal::LE, knots[i], x, // if (knots[i] <= x)
          pinocchio::internal::if_then_else(
            pinocchio::internal::LE, x, knots[i + 1], //   if (x <= knots[i + 1])
            Scalar(1),                                //     return 1;
            Scalar(0)),                               //   else return 0;
          Scalar(0));                                 // else return 0;
      }

      // Calculate the left term
      const Scalar den1 = knots[i + k] - knots[i];
      const Scalar left = pinocchio::internal::if_then_else(
        pinocchio::internal::GT,                           // Operator: Greater Than
        den1,                                              // LHS: The denominator
        Eigen::NumTraits<Scalar>::dummy_precision(),       // RHS: A small positive value for safe
                                                           // comparison
        (x - knots[i]) / den1 * bsplineBasis(i, k - 1, x), // "Then" value (if den1 > 0)
        Scalar(0)                                          // "Else" value (if den1 <= 0)
      );

      // Calculate the right term
      const Scalar den2 = knots[i + k + 1] - knots[i + 1];
      const Scalar right = pinocchio::internal::if_then_else(
        pinocchio::internal::GT,                                       // Operator: Greater Than
        den2,                                                          // LHS: The denominator
        Eigen::NumTraits<Scalar>::dummy_precision(),                   // RHS
        (knots[i + k + 1] - x) / den2 * bsplineBasis(i + 1, k - 1, x), // "Then" value
        Scalar(0)                                                      // "Else" value
      );

      return left + right;
    }

    // if (k == 0)
    //   return (knots[i] <= x && x <= knots[i + 1]) ? Scalar(1) : Scalar(0);

    // Scalar left = 0, right = 0;
    // Scalar den1 = knots[i + k] - knots[i];
    // if (knots[i + k] > knots[i])
    //   left = (x - knots[i]) / den1 * bsplineBasis(i, k - 1, x);

    // Scalar den2 = knots[i + k + 1] - knots[i + 1];
    // if (knots[i + k + 1] > knots[i + 1])
    //   right = (knots[i + k + 1] - x) / den2 * bsplineBasis(i + 1, k - 1, x);

    // return left + right;

    Scalar bsplineBasisDerivative(int i, int k, const Scalar x) const
    {
      // Base case (structural, remains an `if`)
      if (k == 0)
      {
        return Scalar(0);
      }

      // --- Refactored Part ---
      const Scalar k_scalar = static_cast<Scalar>(k);

      // Calculate the first term of the derivative
      const Scalar den1 = knots[i + k] - knots[i];
      const Scalar term1 = pinocchio::internal::if_then_else(
        pinocchio::internal::GT, den1, Eigen::NumTraits<Scalar>::dummy_precision(),
        (k_scalar / den1) * bsplineBasis(i, k - 1, x), Scalar(0));

      // Calculate the second term of the derivative
      const Scalar den2 = knots[i + k + 1] - knots[i + 1];
      const Scalar term2 = pinocchio::internal::if_then_else(
        pinocchio::internal::GT, den2, Eigen::NumTraits<Scalar>::dummy_precision(),
        (k_scalar / den2) * bsplineBasis(i + 1, k - 1, x), Scalar(0));

      return term1 - term2;
    }
    // if (k == 0)
    //   return Scalar(0);

    // Scalar term1 = 0, term2 = 0;
    // Scalar den1 = knots[i + k] - knots[i];
    // if (den1 > Eigen::NumTraits<Scalar>::dummy_precision())
    // {
    //   term1 = (static_cast<Scalar>(k) / den1) * bsplineBasis(i, k - 1, x);
    // }
    // Scalar den2 = knots[i + k + 1] - knots[i + 1];
    // if (den2 > Eigen::NumTraits<Scalar>::dummy_precision())
    // {
    //   term2 = (static_cast<Scalar>(k) / den2) * bsplineBasis(i + 1, k - 1, x);
    // }
    // return term1 - term2;

    Scalar bsplineBasisDerivative2(int i, int k, const Scalar x) const
    {
      // Base case (structural, remains an `if`)
      if (k < 2)
      {
        return Scalar(0);
      }

      // --- Refactored Part ---
      const Scalar k_scalar = static_cast<Scalar>(k);

      // Calculate the first term
      const Scalar den1 = knots[i + k] - knots[i];
      const Scalar term1 = pinocchio::internal::if_then_else(
        pinocchio::internal::GT, den1, Eigen::NumTraits<Scalar>::dummy_precision(),
        (k_scalar / den1) * bsplineBasisDerivative(i, k - 1, x), Scalar(0));

      // Calculate the second term
      const Scalar den2 = knots[i + k + 1] - knots[i + 1];
      const Scalar term2 = pinocchio::internal::if_then_else(
        pinocchio::internal::GT, den2, Eigen::NumTraits<Scalar>::dummy_precision(),
        (k_scalar / den2) * bsplineBasisDerivative(i + 1, k - 1, x), Scalar(0));

      return term1 - term2;
    }
    //   if (k < 2)
    //     return Scalar(0);

    //   Scalar term1 = 0, term2 = 0;
    //   Scalar den1 = knots[i + k] - knots[i];
    //   if (den1 > Eigen::NumTraits<Scalar>::dummy_precision())
    //   {
    //     term1 = (static_cast<Scalar>(k) / den1) * bsplineBasisDerivative(i, k - 1, x);
    //   }
    //   Scalar den2 = knots[i + k + 1] - knots[i + 1];
    //   if (den2 > Eigen::NumTraits<Scalar>::dummy_precision())
    //   {
    //     term2 = (static_cast<Scalar>(k) / den2) * bsplineBasisDerivative(i + 1, k - 1, x);
    //   }
    //   return term1 - term2;
    // }
  }; // struct JointModelSplineTpl

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

#endif // ifndef __pinocchio_multibody_joint_spline_hpp__
