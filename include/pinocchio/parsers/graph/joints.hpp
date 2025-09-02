//
// Copyright (c) 2025 INRIA
//

#ifndef __pinocchio_parsers_graph_joints_hpp__
#define __pinocchio_parsers_graph_joints_hpp__

#include "pinocchio/parsers/graph/fwd.hpp"

#include "pinocchio/parsers/config.hpp"

#include <boost/variant/apply_visitor.hpp>
#include <boost/variant/variant.hpp>

#include <Eigen/Core>

#include <string>

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
      JointTranslation,
      JointPlanar,
      JointHelical,
      JointUniversal,
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

#endif // ifndef __pinocchio_parsers_graph_joints_hpp__
