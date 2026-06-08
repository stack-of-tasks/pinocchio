//
// Copyright (c) 2025 INRIA
//

#pragma once

// IWYU pragma: private, include "pinocchio/parsers/graph.hpp"

#ifdef PINOCCHIO_LSP
  #undef PINOCCHIO_LSP
  #include "pinocchio/parsers/graph.hpp"
#endif // PINOCCHIO_LSP

namespace pinocchio
{
  namespace graph
  {
    struct PINOCCHIO_PARSERS_DLLAPI JointLimits
    {
      // Max effort
      Eigen::VectorXd maxEffort;
      // Max velocity
      Eigen::VectorXd maxVel;
      // Max position
      Eigen::VectorXd maxConfig;
      // Min position
      Eigen::VectorXd minConfig;

      // friction applied in this joint
      Eigen::VectorXd friction;
      // Damping applied by this joint.
      Eigen::VectorXd damping;

      // Armature inertia created by this joint
      Eigen::VectorXd armature;
      // Dry friction.
      double frictionLoss = 0.;

      JointLimits() = default;

      template<int Nq, int Nv>
      void setDimensions();

      void append(const JointLimits & jlimit, const int nq, const int nv);
    };

    struct JointFixed
    {
      SE3 joint_offset = SE3::Identity();
      static constexpr int nq = 0;
      static constexpr int nv = 0;

      JointFixed() = default;
      explicit JointFixed(const SE3 & pose)
      : joint_offset(pose)
      {
      }

      bool operator==(const JointFixed & other) const
      {
        return joint_offset == other.joint_offset;
      }
    };

    struct JointRevolute
    {
      // rotation axis
      Eigen::Vector3d axis = Eigen::Vector3d::UnitX();
      static constexpr int nq = 1;
      static constexpr int nv = 1;

      JointRevolute() = default;
      explicit JointRevolute(const Eigen::Vector3d & ax)
      : axis(ax)
      {
      }

      bool operator==(const JointRevolute & other) const
      {
        return axis == other.axis;
      }
    };

    struct JointRevoluteUnbounded
    {
      Eigen::Vector3d axis = Eigen::Vector3d::UnitX();
      static constexpr int nq = 2;
      static constexpr int nv = 1;

      JointRevoluteUnbounded() = default;
      explicit JointRevoluteUnbounded(const Eigen::Vector3d & ax)
      : axis(ax)
      {
      }

      bool operator==(const JointRevoluteUnbounded & other) const
      {
        return axis == other.axis;
      }
    };

    struct JointPrismatic
    {
      Eigen::Vector3d axis = Eigen::Vector3d::UnitX();
      static constexpr int nq = 1;
      static constexpr int nv = 1;

      JointPrismatic() = default;
      explicit JointPrismatic(const Eigen::Vector3d & ax)
      : axis(ax)
      {
      }

      bool operator==(const JointPrismatic & other) const
      {
        return axis == other.axis;
      }
    };

    struct JointFreeFlyer
    {
      static constexpr int nq = 7;
      static constexpr int nv = 6;

      JointFreeFlyer() = default;

      bool operator==(const JointFreeFlyer &) const
      {
        return true;
      }
    };

    struct JointSpherical
    {
      static constexpr int nq = 4;
      static constexpr int nv = 3;

      JointSpherical() = default;

      bool operator==(const JointSpherical &) const
      {
        return true;
      }
    };

    struct JointSphericalZYX
    {
      static constexpr int nq = 3;
      static constexpr int nv = 3;

      JointSphericalZYX() = default;

      bool operator==(const JointSphericalZYX &) const
      {
        return true;
      }
    };

    struct JointEllipsoid
    {
      double radius_x = 0;
      double radius_y = 0;
      double radius_z = 0;

      static constexpr int nq = 3;
      static constexpr int nv = 3;

      JointEllipsoid() = default;
      JointEllipsoid(const double r_a, const double r_b, const double r_c)
      : radius_x(r_a)
      , radius_y(r_b)
      , radius_z(r_c)
      {
      }

      bool operator==(const JointEllipsoid & other) const
      {
        return radius_x == other.radius_x && radius_y == other.radius_y
               && radius_z == other.radius_z;
      }
    };

    struct JointTranslation
    {
      static constexpr int nq = 3;
      static constexpr int nv = 3;

      JointTranslation() = default;

      bool operator==(const JointTranslation &) const
      {
        return true;
      }
    };

    struct JointPlanar
    {
      static constexpr int nq = 4;
      static constexpr int nv = 3;

      JointPlanar() = default;

      bool operator==(const JointPlanar &) const
      {
        return true;
      }
    };

    struct JointHelical
    {
      Eigen::Vector3d axis = Eigen::Vector3d::UnitX();
      double pitch = 0.;

      static constexpr int nq = 1;
      static constexpr int nv = 1;

      JointHelical() = default;
      JointHelical(const Eigen::Vector3d & ax, const double p)
      : axis(ax)
      , pitch(p)
      {
      }

      bool operator==(const JointHelical & other) const
      {
        return axis == other.axis && pitch == other.pitch;
      }
    };

    struct JointUniversal
    {
      Eigen::Vector3d axis1 = Eigen::Vector3d::UnitX();
      Eigen::Vector3d axis2 = Eigen::Vector3d::UnitY();

      static constexpr int nq = 2;
      static constexpr int nv = 2;

      JointUniversal() = default;
      JointUniversal(const Eigen::Vector3d & ax1, const Eigen::Vector3d & ax2)
      : axis1(ax1)
      , axis2(ax2)
      {
      }

      bool operator==(const JointUniversal & other) const
      {
        return axis1 == other.axis1 && axis2 == other.axis2;
      }
    };

    struct JointSpline
    {
      std::vector<SE3> ctrlFrames;
      Eigen::VectorXd knots;
      std::size_t degree = 3;

      static constexpr int nq = 1;
      static constexpr int nv = 1;

      JointSpline() = default;
      JointSpline(const int degree)
      : degree(degree)
      {
      }

      JointSpline(
        const std::vector<SE3> & ctrlFrames, const Eigen::VectorXd & knots, std::size_t degree)
      : ctrlFrames(ctrlFrames)
      , knots(knots)
      , degree(degree)
      {
      }

      bool operator==(const JointSpline & other) const
      {
        return ctrlFrames == other.ctrlFrames && knots == other.knots && degree == other.degree;
      }
    };

    struct JointSplineBuilder
    {
      using JointModelSplineBuilder = JointModelSplineBuilderTpl<double>;
      using KnotPolicy = typename JointModelSplineBuilder::KnotPolicy;

      JointSplineBuilder()
      : degree(3)
      , min_q(0)
      , max_q(1)
      , knot_policy(KnotPolicy::OpenUniform)
      {
      }

      JointSplineBuilder & addControlFrame(const SE3 & frame)
      {
        ctrlFrames.push_back(frame);
        return *this;
      }

      JointSplineBuilder & withControlFrameVector(const std::vector<SE3> & frames)
      {
        ctrlFrames = frames;
        return *this;
      }

      JointSplineBuilder & withDegree(size_t p_degree)
      {
        degree = p_degree;
        return *this;
      }

      JointSplineBuilder & withKnotVector(const Eigen::VectorXd & p_knots)
      {
        knots = p_knots;
        knot_policy = KnotPolicy::Custom;

        return *this;
      }

      JointSplineBuilder & withKnotVector(const std::vector<double> & p_knots)
      {
        knots.resize(p_knots.size());
        for (std::size_t i = 0; i < p_knots.size(); ++i)
        {
          knots[i] = p_knots[i];
        }
        knot_policy = KnotPolicy::Custom;

        return *this;
      }

      JointSplineBuilder & withOpenUniformKnots(double p_min_q, double p_max_q)
      {
        knot_policy = KnotPolicy::OpenUniform;
        min_q = p_min_q;
        max_q = p_max_q;

        return *this;
      }

      JointSplineBuilder & withUniformKnots(double p_min_q, double p_max_q)
      {
        knot_policy = KnotPolicy::Uniform;
        min_q = p_min_q;
        max_q = p_max_q;

        return *this;
      }

      JointSpline build() const
      {
        Eigen::VectorXd joint_knots;

        const size_t nCtrl = ctrlFrames.size();
        switch (knot_policy)
        {
        case KnotPolicy::OpenUniform:
          joint_knots = pinocchio::internal::generateOpenUniformKnots(min_q, max_q, nCtrl, degree);
          break;

        case KnotPolicy::Uniform:
          joint_knots = pinocchio::internal::generateUniformKnots(min_q, max_q, nCtrl, degree);
          break;

        case KnotPolicy::Custom:
          joint_knots = knots;
          break;
        default:
          break;
        }

        return JointSpline(ctrlFrames, joint_knots, degree);
      }

    private:
      std::vector<SE3> ctrlFrames;

      size_t degree;

      double min_q;
      double max_q;

      Eigen::VectorXd knots;
      KnotPolicy knot_policy;
    };

    // Forward declare
    struct JointComposite;
    struct JointMimic;

    typedef boost::variant<
      JointFixed,
      JointRevolute,
      JointRevoluteUnbounded,
      JointPrismatic,
      JointFreeFlyer,
      JointSpherical,
      JointSphericalZYX,
      JointEllipsoid,
      JointTranslation,
      JointPlanar,
      JointHelical,
      JointUniversal,
      JointSpline,
      boost::recursive_wrapper<JointComposite>,
      boost::recursive_wrapper<JointMimic>>
      JointVariant;

    struct JointMimic
    {
      std::string primary_name;

      JointVariant secondary_joint;
      double scaling = 1.;
      double offset = 0.;

      static constexpr int nq = 0;
      static constexpr int nv = 0;

      JointMimic() = default;

      JointMimic(
        const JointVariant & jmodel_secondary,
        const std::string & name_primary,
        const double scaling_,
        const double offset_)
      : primary_name(name_primary)
      , secondary_joint(jmodel_secondary)
      , scaling(scaling_)
      , offset(offset_)
      {
      }

      bool operator==(const JointMimic & other) const
      {
        return primary_name == other.primary_name && scaling == other.scaling
               && offset == other.offset && secondary_joint == other.secondary_joint;
      }
    };

    struct JointComposite
    {
      std::vector<JointVariant> joints;
      std::vector<SE3> jointsPlacements;

      int nq = 0;
      int nv = 0;

      JointComposite() = default;

      JointComposite(const JointVariant & j, const SE3 & jPose)
      {
        joints.push_back(j);
        jointsPlacements.push_back(jPose);
        nq += boost::apply_visitor([](const auto & j_) { return j_.nq; }, j);
        nv += boost::apply_visitor([](const auto & j_) { return j_.nv; }, j);
      }

      JointComposite(const std::vector<JointVariant> & js, const std::vector<SE3> & jPoses)
      : joints(js)
      , jointsPlacements(jPoses)
      {
        for (const auto & j : js)
        {
          nq += boost::apply_visitor([](const auto & j_) { return j_.nq; }, j);
          nv += boost::apply_visitor([](const auto & j_) { return j_.nv; }, j);
        }
      }

      void addJoint(const JointVariant & jm, const SE3 & pose = SE3::Identity())
      {
        joints.push_back(jm);
        jointsPlacements.push_back(pose);
        nq += boost::apply_visitor([](const auto & j) { return j.nq; }, jm);
        nv += boost::apply_visitor([](const auto & j) { return j.nv; }, jm);
      }

      bool operator==(const JointComposite & other) const
      {
        return joints == other.joints && jointsPlacements == other.jointsPlacements
               && nq == other.nq && nv == other.nv;
      }
    };
  } // namespace graph
} // namespace pinocchio
