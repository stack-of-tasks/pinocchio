//
// Copyright (c) 2025 INRIA
//

#ifndef __pinocchio_parsers_graph_model_graph_hpp__
#define __pinocchio_parsers_graph_model_graph_hpp__

#include "pinocchio/parsers/graph/fwd.hpp"

#include "pinocchio/parsers/config.hpp"

#include "pinocchio/parsers/graph/joints.hpp"
#include "pinocchio/parsers/graph/frames.hpp"
#include "pinocchio/parsers/graph/geometries.hpp"

#include <Eigen/Core>

#include <boost/optional.hpp>
#include <boost/graph/adjacency_list.hpp>

#include <vector>
#include <string>
#include <unordered_map>
#include <stdexcept>

namespace pinocchio
{
  namespace graph
  {
    struct EdgeBuilder;
    struct EdgeParameters;
    struct GeometryBuilder;

    /// @brief Represents a vertex (body, sensor, operational frame) in the model graph.
    struct ModelGraphVertex
    {
      /// @brief Unique name of the body.
      std::string name;

      FrameVariant frame;

      std::vector<Geometry> geometries;

      void addGeometry(const Geometry & geo)
      {
        geometries.push_back(geo);
      }
    };

    /// @brief Represents an edge (joint) in the model graph.
    struct ModelGraphEdge
    {
      /// @brief Unique name of the joint
      std::string name;

      /// @brief What is the type of the joint
      JointVariant joint;

      /// @brief All the limits of the joint
      JointLimits jlimit;

      /// @brief Transformation from the previous vertex to edge
      ///
      /// Correspond to the transformation from body supporting joint to said joint
      SE3 source_to_joint;

      /// @brief Transformation from edge to next vertex
      ///
      /// Correspond to the transformation from the current joint to its supported body.
      SE3 joint_to_target;

      /// @brief boolean to know if we are in a forward or backward edge
      bool forward = true;
    };

    /// @brief Contains information about how \ref buildModel walked the \ref ModelGraph to
    /// construct a \ref Model.
    /// All members are considered internal.
    struct ModelGraphBuildInfo
    {
      /// Map joint name to joint direction.
      std::unordered_map<std::string, bool> _joint_forward;
      /// True if the root joint is fixed.
      bool _is_fixed;
    };

    /// @brief Represents multibody model as a bidirectional graph.
    ///
    /// This is an intermediate step before creating a model, that
    /// allows more flexibility as to which body will be the root...
    struct PINOCCHIO_PARSERS_DLLAPI ModelGraph
    {
      typedef boost::
        adjacency_list<boost::vecS, boost::vecS, boost::directedS, ModelGraphVertex, ModelGraphEdge>
          Graph;
      typedef typename boost::graph_traits<Graph>::vertex_descriptor VertexDesc;
      typedef typename boost::graph_traits<Graph>::edge_descriptor EdgeDesc;

      ModelGraph() = default;
      /// \brief Add a new vertex to the graph
      ///
      /// \param[in] vertex_name Name of the vertex
      /// \param[in] frame which type of frame will be added to the model (op_frame, sensor, body)
      void addFrame(const std::string & vertex_name, const FrameVariant & frame);

      /// \brief Add a new body to the graph
      ///
      /// \param[in] vertex_name Name of the vertex
      /// \param[in] inert inertia of the body
      void addBody(const std::string & vertex_name, const Inertia & inert);

      GeometryBuilder geometryBuilder();
      void addGeometry(const std::string & vertex_name, const Geometry & geom);
      void addGeometries(const std::string & vertex_name, const std::vector<Geometry> & geoms);

      /// \brief Add edges (joint) to the graph. Since it's a bidirectional graph,
      /// edge and its reverse are added to the graph.
      ///
      /// \param[in] joint_name Name of the edge
      /// \param[in] joint Type of the joint
      /// \param[in] source_body Vertex that is supporting the edge
      /// \param[in] source_to_joint Transformation from supporting vertex to edge
      /// \param[in] target_body Vertex that is supported by edge
      /// \param[in] joint_to_target Transformation from edge to supported vertex
      ///
      /// \note Since it's a bidirectional graph, two edges are added to the graph.
      /// Joints and transformation are inverted, to create reverse edge.
      void addJoint(
        const std::string & joint_name,
        const JointVariant & joint,
        const std::string & source_body,
        const SE3 & source_to_joint,
        const std::string & target_body,
        const SE3 & joint_to_target);

      /// \brief Add edges (joint) to the graph. Since it's a bidirectional graph,
      /// edge and its reverse are added to the graph.
      ///
      /// \param[in] params Structure that holds all of the joint parameters
      ///
      /// \note Since it's a bidirectional graph, two edges are added to the graph.
      /// Joints and transformation are inverted, to create reverse edge.
      void addJoint(const EdgeParameters & params);

      /// \brief Create an EdgeBuilde. This will allow to use EdgeBuilder interface to have a more
      /// flexible edge configuration.
      EdgeBuilder edgeBuilder();

      /// @brief  add all the vertex and edges from a graph to this one.
      /// Attention : it does not add an edge between the two, so it will be like having two graph
      /// coexisting in this structure.
      ///
      /// @param g graph that will be added
      ///
      void appendGraph(const ModelGraph & g);

      /// @brief Boost graph structure that holds the graph structure
      Graph graph;
      /// @brief Name of the vertexes in the graph. Useful for graph parcours.
      std::unordered_map<std::string, VertexDesc> name_to_vertex;
    };

    /// @brief Structure that holds all the parameters useful to create an edge.
    struct PINOCCHIO_PARSERS_DLLAPI EdgeParameters
    {
      /// @brief Edge name
      std::string name;

      /// @brief Source name
      std::string source_vertex;
      /// @brief Placement of Edge wrt source vertex
      SE3 source_to_joint = SE3::Identity();
      /// @brief Target name
      std::string target_vertex;
      /// @brief Placement of target wrt edge
      SE3 joint_to_target = SE3::Identity();

      /// @brief Type of joint for edge
      JointVariant joint = JointFixed();

      /// @brief Bias for the joint
      boost::optional<Eigen::VectorXd> q_ref = boost::none;

      JointLimits jlimit;

      /// @brief Default Constructor
      EdgeParameters() = default;

      /// @brief Constructor with all parameters
      EdgeParameters(
        const std::string & jname,
        const std::string & source_name,
        const SE3 & source_to_joint,
        const std::string & target_name,
        const SE3 & joint_to_target,
        const JointVariant & joint,
        const boost::optional<Eigen::VectorXd> q_ref = boost::none);
    };

    /// @brief Builder interface to add an edge to the graph.
    /// Allows for an easy customization of the edge.
    struct PINOCCHIO_PARSERS_DLLAPI EdgeBuilder
    {
      /// @brief ModelGraph to which the edge will be added
      ModelGraph & g;

      /// @brief Parameters of the edge
      EdgeParameters param;

      boost::optional<Eigen::VectorXd> minConfig;
      boost::optional<Eigen::VectorXd> maxConfig;
      boost::optional<Eigen::VectorXd> maxVel;
      boost::optional<Eigen::VectorXd> maxEffort;
      boost::optional<Eigen::VectorXd> armature;
      boost::optional<Eigen::VectorXd> friction;
      boost::optional<Eigen::VectorXd> damping;

      double frictionLoss = 0;

      /// @brief Constructor
      explicit EdgeBuilder(ModelGraph & graph)
      : g(graph)
      {
      }

      /// @brief Specify the type of joint for the edge. Default : Fixed
      EdgeBuilder & withJointType(const JointVariant & jtype)
      {
        param.joint = jtype;
        return *this;
      }

      /// @brief Specify the name of the edge
      EdgeBuilder & withName(const std::string & name)
      {
        param.name = name;
        return *this;
      }
      /// @brief Specify the name of the target vertex
      EdgeBuilder & withTargetVertex(const std::string & target_name)
      {
        param.target_vertex = target_name;
        return *this;
      }
      /// @brief Specify the name of the source vertex
      EdgeBuilder & withSourceVertex(const std::string & source_name)
      {
        param.source_vertex = source_name;
        return *this;
      }
      /// @brief Specify the pose of target vertex wrt edge. Default : Identity
      EdgeBuilder & withTargetPose(const SE3 & target_pose)
      {
        param.joint_to_target = target_pose;
        return *this;
      }
      /// @brief Specify the pose of the joint wrt the source vertex. Default : Identity
      EdgeBuilder & withSourcePose(const SE3 & source_pose)
      {
        param.source_to_joint = source_pose;
        return *this;
      }

      /// @brief Specify a bias for the joint configuration
      EdgeBuilder & withQref(const Eigen::VectorXd & qref)
      {
        param.q_ref = qref;
        return *this;
      }

      /// @brief Specify limit minConfig
      EdgeBuilder & withMinConfig(const Eigen::VectorXd & minConfig_)
      {
        minConfig = minConfig_;
        return *this;
      }

      /// @brief Specify limit maxConfig
      EdgeBuilder & withMaxConfig(const Eigen::VectorXd & maxConfig_)
      {
        maxConfig = maxConfig_;
        return *this;
      }

      /// @brief Specify limit maxVel
      EdgeBuilder & withMaxVel(const Eigen::VectorXd & maxVel_)
      {
        maxVel = maxVel_;
        return *this;
      }

      /// @brief Specify limit maxEffort
      EdgeBuilder & withMaxEffort(const Eigen::VectorXd & maxEffort_)
      {
        maxEffort = maxEffort_;
        return *this;
      }

      /// @brief Specify friction
      EdgeBuilder & withFriction(const Eigen::VectorXd & friction_)
      {
        friction = friction_;
        return *this;
      }

      /// @brief Specify damping
      EdgeBuilder & withDamping(const Eigen::VectorXd & damping_)
      {
        damping = damping_;
        return *this;
      }

      /// @brief Specify armature
      EdgeBuilder & withArmature(const Eigen::VectorXd & armature_)
      {
        armature = armature_;
        return *this;
      }

      /// @brief Specify friction loss
      EdgeBuilder & withFrictionLoss(const double frictionLoss_)
      {
        frictionLoss = frictionLoss_;
        return *this;
      }

      /// @brief Add the edge to the ModelGraph
      void build();
    };

    struct GeometryBuilder
    {
      Geometry geometry;
      std::string name_body;

      ModelGraph & g;

      GeometryBuilder(ModelGraph & g)
      : g(g)
      {
      }

      GeometryBuilder & withName(const std::string & n)
      {
        geometry.name = n;
        return *this;
      }

      GeometryBuilder & withBody(const std::string & n)
      {
        name_body = n;
        return *this;
      }

      GeometryBuilder & withPlacement(const SE3 & p)
      {
        geometry.placement = p;
        return *this;
      }

      GeometryBuilder & withScale(const Eigen::Vector3d & s)
      {
        geometry.scale = s;
        return *this;
      }

      GeometryBuilder & withColor(const Eigen::Vector4d & c)
      {
        geometry.color = c;
        return *this;
      }

      GeometryBuilder & withGeomType(const GeomType type)
      {
        geometry.type = type;
        return *this;
      }

      GeometryBuilder & withGeom(const GeomVariant & g)
      {
        geometry.geometry = g;
        return *this;
      }

      void build()
      {
        if (geometry.name.empty())
          PINOCCHIO_THROW_PRETTY(std::invalid_argument, "Graph - geometry should have a name.");

        return g.addGeometry(name_body, geometry);
      }
    };
  } // namespace graph
} // namespace pinocchio

#endif // ifndef __pinocchio_parsers_graph_model_graph_hpp__
