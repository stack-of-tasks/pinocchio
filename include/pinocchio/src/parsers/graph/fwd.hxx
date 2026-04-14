//
// Copyright (c) 2025 INRIA
//

#pragma once

// IWYU pragma: private, include "pinocchio/parsers/graph/fwd.hpp"

#ifdef PINOCCHIO_LSP
  #undef PINOCCHIO_LSP
  #include "pinocchio/parsers/graph/fwd.hpp"
#endif // PINOCCHIO_LSP

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
    struct JointSpline;

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
