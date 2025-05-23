//
// Copyright (c) 2025 INRIA
//

#ifndef __pinocchio_joint_graph_hpp__
#define __pinocchio_joint_graph_hpp__

#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/multibody/joint/joint-generic.hpp"
#include "pinocchio/spatial/se3.hpp"

namespace pinocchio
{
  struct JointFixedGraph
  {
    pinocchio::SE3 joint_offset = pinocchio::SE3::Identity();

    JointFixedGraph() = default;
    JointFixedGraph(const pinocchio::SE3 & pose)
    : joint_offset(pose)
    {
    }
  };

  struct JointRevoluteGraph
  {
    // rotation axis
    Eigen::Vector3d axis;
    double q_ref;

    explicit JointRevoluteGraph(const Eigen::Vector3d & ax, const double q_ = 0)
    : axis(ax)
    , q_ref(q_)
    {
    }
  };

  struct JointRevoluteUnboundedGraph
  {
    Eigen::Vector3d axis;
    Eigen::Vector2d q_ref = Eigen::Vector2d::Zero();

    explicit JointRevoluteUnboundedGraph(const Eigen::Vector3d & ax)
    : axis(ax)
    {
      q_ref << 1, 0; // cos(0), sin(0)
    }

    explicit JointRevoluteUnboundedGraph(const Eigen::Vector3d & ax, const Eigen::Vector2d & q_)
    : axis(ax)
    , q_ref(q_)
    {
    }
  };

  struct JointPrismaticGraph
  {
    Eigen::Vector3d axis;
    double q_ref;

    explicit JointPrismaticGraph(const Eigen::Vector3d & ax, const double q_ = 0)
    : axis(ax)
    , q_ref(q_)
    {
    }
  };

  struct JointFreeFlyerGraph
  {
    Eigen::VectorXd q_ref = Eigen::VectorXd::Zero(7);

    JointFreeFlyerGraph()
    {
      q_ref << 0, 0, 0, 0, 0, 0, 1;
    }

    JointFreeFlyerGraph(const Eigen::VectorXd & q_)
    : q_ref(q_)
    {
    }
  };

  struct JointSphericalGraph
  {
    Eigen::VectorXd q_ref = Eigen::VectorXd::Zero(4); // quaternion - x, y, z, w

    JointSphericalGraph()
    {
      q_ref << 0, 0, 0, 1;
    }

    JointSphericalGraph(const Eigen::VectorXd & q_)
    : q_ref(q_)
    {
    }
  };

  // Flipped whne model is reversed ?
  struct JointSphericalZYXGraph
  {
    Eigen::Vector3d q_ref = Eigen::Vector3d::Zero();

    JointSphericalZYXGraph() = default;
    JointSphericalZYXGraph(const Eigen::Vector3d & q_)
    : q_ref(q_)
    {
    }
  };

  struct JointTranslationGraph
  {
    Eigen::Vector3d q_ref = Eigen::Vector3d::Zero(); // T_x, T_y, T_z

    JointTranslationGraph() = default;
    JointTranslationGraph(const Eigen::Vector3d & q_)
    : q_ref(q_)
    {
    }
  };

  struct JointPlanarGraph
  {
    Eigen::VectorXd q_ref = Eigen::VectorXd::Zero(4); // T_x, T_y, cos(a), sin(a)

    JointPlanarGraph()
    {
      q_ref << 0, 0, 1, 0;
    }

    JointPlanarGraph(const Eigen::VectorXd & q_)
    : q_ref(q_)
    {
    }
  };

  struct JointHelicalGraph
  {
    Eigen::Vector3d axis;
    double pitch;

    double q_ref;

    JointHelicalGraph(const Eigen::Vector3d & ax, const double p, const double q_ = 0)
    : axis(ax)
    , pitch(p)
    , q_ref(q_)
    {
    }
  };

  struct JointUniversalGraph
  {
    Eigen::Vector3d axis1;
    Eigen::Vector3d axis2;

    Eigen::Vector2d q_ref;

    JointUniversalGraph(
      const Eigen::Vector3d & ax1,
      const Eigen::Vector3d & ax2,
      const Eigen::Vector2d & q_ = Eigen::Vector2d::Zero())
    : axis1(ax1)
    , axis2(ax2)
    , q_ref(q_)
    {
    }
  };

  // Forward declare
  struct JointCompositeGraph;
  // Forward declare
  struct JointMimicGraph;

  using JointGraphVariant = boost::variant<
    JointFixedGraph,
    JointRevoluteGraph,
    JointRevoluteUnboundedGraph,
    JointPrismaticGraph,
    JointFreeFlyerGraph,
    JointSphericalGraph,
    JointSphericalZYXGraph,
    JointTranslationGraph,
    JointPlanarGraph,
    JointHelicalGraph,
    JointUniversalGraph,
    boost::recursive_wrapper<JointCompositeGraph>,
    boost::recursive_wrapper<JointMimicGraph>>;

  struct JointCompositeGraph
  {
    std::vector<JointGraphVariant> joints;
    std::vector<SE3> jointsPlacements;

    JointCompositeGraph() = default;

    JointCompositeGraph(const JointGraphVariant & j, const SE3 & jPose)
    {
      joints.push_back(j);
      jointsPlacements.push_back(jPose);
    }

    JointCompositeGraph(const std::vector<JointGraphVariant> & js, const std::vector<SE3> & jPoses)
    : joints(js)
    , jointsPlacements(jPoses)
    {
    }

    void addJoint(const JointGraphVariant & jm, const SE3 & pose = SE3::Identity())
    {
      joints.push_back(jm);
      jointsPlacements.push_back(pose);
    }
  };

  struct JointMimicGraph
  {
    std::string primary_name;

    JointGraphVariant secondary_joint;
    double scaling;
    double offset;

    JointMimicGraph() = default;

    JointMimicGraph(
      const JointGraphVariant & jmodel_secondary,
      const std::string & name_primary,
      const double scaling_,
      const double offset_)
    : primary_name(name_primary)
    , secondary_joint(jmodel_secondary)
    , scaling(scaling_)
    , offset(offset_)
    {
    }
  };
} // namespace pinocchio

#endif
