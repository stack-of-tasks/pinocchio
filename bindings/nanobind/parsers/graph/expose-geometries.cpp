// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"
#include "pinocchio/bindings/python-nb/utils/boost-variant.hpp"

#include "pinocchio/parsers/graph.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/bind_vector.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

using namespace nb::literals;

void exposeGeometriesVariant(nb::module_ m)
{
  using namespace pinocchio::graph;
  nb::class_<Mesh>(m, "Mesh", "Represents a mesh geometry.")
    .def(nb::init<>(), "Default constructor.")
    .def(nb::init<const std::string &>(), "path"_a, "Constructor from mesh file path.")
    .def_rw("path", &Mesh::path, "Path to the mesh file.");

  nb::class_<Box>(m, "Box", "Represents a box.")
    .def(nb::init<>(), "Default constructor.")
    .def(nb::init<const Eigen::Vector3d &>(), "size"_a, "Constructor from 3D size vector.")
    .def_rw("size", &Box::size, "Size of the box.");

  nb::class_<Cylinder>(m, "Cylinder", "Represents a cylinder.")
    .def(nb::init<>(), "Default constructor.")
    .def(
      nb::init<const Eigen::Vector2d &>(), "size"_a,
      "Constructor from 2D size vector (radius, height).")
    .def_rw("size", &Cylinder::size, "Size of the cylinder (radius, height).");

  nb::class_<Capsule>(m, "Capsule", "Represents a capsule.")
    .def(nb::init<>(), "Default constructor.")
    .def(
      nb::init<const Eigen::Vector2d &>(), "size"_a,
      "Constructor from 2D size vector (radius, length).")
    .def_rw("size", &Capsule::size, "Size of the capsule (radius, length).");

  nb::class_<Sphere>(m, "Sphere", "Represents a sphere.")
    .def(nb::init<>(), "Default constructor.")
    .def(nb::init<const double>(), "radius"_a, "Constructor from radius.")
    .def_rw("radius", &Sphere::radius, "Radius of the sphere.");
}

void exposeGeometryGraph(nb::module_ m)
{
  using namespace pinocchio::graph;

  nb::enum_<GeomType>(m, "GeomType")
    .value("VISUAL", GeomType::VISUAL)
    .value("COLLISION", GeomType::COLLISION)
    .value("BOTH", GeomType::BOTH)
    .export_values();

  nb::class_<Geometry>(
    m, "Geometry", "Main geometry object containing all properties for the Model Graph.")
    .def(nb::init<>(), "Default constructor.")
    .def(
      nb::init<
        const std::string &, const pinocchio::SE3 &, const GeomType &, const Eigen::Vector3d &,
        const Eigen::Vector4d &, const GeomVariant &>(),
      "name"_a, "placement"_a, "type"_a, "scale"_a, "color"_a, "geometry"_a,
      "Full constructor for Geometry.")
    .def_rw("name", &Geometry::name, "Name of the geometry object.")
    .def_rw("type", &Geometry::type, "Type of geometry (VISUAL, COLLISION, BOTH).")
    .def_rw("scale", &Geometry::scale, "Scaling factors.")
    .def_rw("color", &Geometry::color, "RGBA color.")
    .def_rw(
      "placement", &Geometry::placement, "SE3 placement (pose) of the geometry wrt to the body.")
    .def_rw("geometry", &Geometry::geometry, "The actual geometric primitive.");

  nb::bind_vector<std::vector<Geometry>>(m, "StdVec_Geometry");
}

void exposeGeometryBuilder(nb::module_ m)
{
  using namespace pinocchio::graph;

  nb::class_<GeometryBuilder>(m, "GeometryBuilder", "A builder for Geometries in Model Graph.")
    .def(nb::init<ModelGraph &>(), "model_graph"_a)
    .def("withName", &GeometryBuilder::withName, "name"_a, "Sets the name of the geometry.")
    .def(
      "withBody", &GeometryBuilder::withBody, "body_name"_a,
      "Sets the name of the body this geometry is attached to.")
    .def(
      "withPlacement", &GeometryBuilder::withPlacement, "placement"_a,
      "Sets the SE3 placement of the geometry.")
    .def("withScale", &GeometryBuilder::withScale, "scale"_a, "Sets the scale factors.")
    .def("withColor", &GeometryBuilder::withColor, "color"_a, "Sets the RGBA color.")
    .def(
      "withGeomType", &GeometryBuilder::withGeomType, "geom_type"_a,
      "Sets the type of geometry (VISUAL, COLLISION, BOTH).")
    .def(
      "withGeom", &GeometryBuilder::withGeom, "geometry_primitive"_a,
      "Sets the geometry primitive .")
    .def(
      "build", &GeometryBuilder::build,
      "Builds the Geometry object and adds it to the right vertex in the ModelGraph "
      "Throws if geometry name is empty.");
}

PINOCCHIO_PYTHON_NAMESPACE_END