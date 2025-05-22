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
    JointFixedGraph() = default;
  };

  struct JointRevoluteGraph
  {
    // rotation axis
    Eigen::Vector3d axis;

    explicit JointRevoluteGraph(const Eigen::Vector3d & ax)
    : axis(ax)
    {
    }
  };

  struct JointRevoluteUnboundedGraph
  {
    Eigen::Vector3d axis;

    explicit JointRevoluteUnboundedGraph(const Eigen::Vector3d & ax)
    : axis(ax)
    {
    }
  };

  struct JointPrismaticGraph
  {
    Eigen::Vector3d axis;

    explicit JointPrismaticGraph(const Eigen::Vector3d & ax)
    : axis(ax)
    {
    }
  };

  struct JointFreeFlyerGraph
  {
    JointFreeFlyerGraph() = default;
  };

  struct JointSphericalGraph
  {
    JointSphericalGraph() = default;
  };

  // Flipped whne model is reversed ?
  struct JointSphericalZYXGraph
  {
    JointSphericalZYXGraph() = default;
  };

  struct JointTranslationGraph
  {
    JointTranslationGraph() = default;
  };

  struct JointPlanarGraph
  {
    JointPlanarGraph() = default;
  };

  struct JointHelicalGraph
  {
    Eigen::Vector3d axis;
    double pitch;

    JointHelicalGraph(const Eigen::Vector3d & ax, const double p)
    : axis(ax)
    , pitch(p)
    {
    }
  };

  struct JointUniversalGraph
  {
    Eigen::Vector3d axis1;
    Eigen::Vector3d axis2;

    JointUniversalGraph(const Eigen::Vector3d & ax1, const Eigen::Vector3d & ax2)
    : axis1(ax1)
    , axis2(ax2)
    {
    }
  };

  // Mimic : joint name, ratio et offset

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