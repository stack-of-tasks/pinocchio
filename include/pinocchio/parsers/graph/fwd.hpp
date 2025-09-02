//
// Copyright (c) 2025 INRIA
//

#ifndef __pinocchio_parsers_graph_fwd_hpp__
#define __pinocchio_parsers_graph_fwd_hpp__

#include "pinocchio/fwd.hpp"

#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/inertia.hpp"

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/frame.hpp"

namespace pinocchio
{
  namespace graph
  {
    typedef SE3Tpl<double, PINOCCHIO_OPTIONS_DEFAULT> SE3;
    typedef InertiaTpl<double, PINOCCHIO_OPTIONS_DEFAULT> Inertia;
    typedef FrameTpl<double, PINOCCHIO_OPTIONS_DEFAULT> Frame;
    typedef ModelTpl<double, PINOCCHIO_OPTIONS_DEFAULT, JointCollectionDefaultTpl> Model;

    // joints.hpp
    struct JointLimits;
    struct JointFixed;
    struct JointRevolute;
    struct JointRevoluteUnbounded;
    struct JointPrismatic;
    struct JointFreeFlyer;
    struct JointSpherical;
    struct JointSphericalZYX;
    struct JointTranslation;
    struct JointPlanar;
    struct JointHelical;
    struct JointUniversal;
    struct JointComposite;
    struct JointMimic;

    // frames.hpp
    struct BodyFrame;
    struct SensorFrame;
    struct OpFrame;

    // geometries.hpp
    enum struct GeomType;
    struct Mesh;
    struct Box;
    struct Cylinder;
    struct Capsule;
    struct Sphere;
    struct Geometry;

    // model-graph.hpp
    struct EdgeBuilder;
    struct EdgeParameters;
    struct GeometryBuilder;
    struct ModelGraphVertex;
    struct ModelGraphEdge;
    struct ModelGraphBuildInfo;
    struct ModelGraph;

    // model-graph-algo.hpp
    struct BuildModelWithBuildInfoReturn;

  } // namespace graph
} // namespace pinocchio

#endif // ifndef __pinocchio_parsers_graph_fwd_hpp__
