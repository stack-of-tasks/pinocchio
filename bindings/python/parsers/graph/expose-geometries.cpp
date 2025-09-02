//
// Copyright (c) 2025 INRIA
//

#include <boost/python.hpp>

#include "pinocchio/bindings/python/parsers/model-graph.hpp"
#include "pinocchio/parsers/graph/model-graph.hpp"
#include "pinocchio/bindings/python/utils/std-aligned-vector.hpp"

#include <eigenpy/variant.hpp>
namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    void exposeGeometriesVariant()
    {
      using namespace pinocchio::graph;

      bp::class_<Mesh>("Mesh", "Represents a mesh geometry.", bp::init<>())
        .def(bp::init<const std::string &>((bp::arg("path")), "Constructor from mesh file path."))
        .def_readwrite("path", &Mesh::path, "Path to the mesh file.");

      bp::class_<Box>("Box", "Represents a box.", bp::init<>())
        .def(
          bp::init<const Eigen::Vector3d &>((bp::arg("size")), "Constructor from 3D size vector."))
        .def_readwrite("size", &Box::size, "Size of the box.");

      bp::class_<Cylinder>("Cylinder", "Represents a cylinder.", bp::init<>())
        .def(bp::init<const Eigen::Vector2d &>(
          (bp::arg("size")), "Constructor from 2D size vector (radius, height)."))
        .def_readwrite("size", &Cylinder::size, "Size of the cylinder (radius, height).");

      bp::class_<Capsule>("Capsule", "Represents a capsule.", bp::init<>())
        .def(bp::init<const Eigen::Vector2d &>(
          (bp::arg("size")), "Constructor from 2D size vector (radius, length)."))
        .def_readwrite("size", &Capsule::size, "Size of the capsule (radius, length).");

      bp::class_<Sphere>("Sphere", "Represents a sphere.", bp::init<>())
        .def(bp::init<const double>((bp::arg("radius")), "Constructor from radius."))
        .def_readwrite("radius", &Sphere::radius, "Radius of the sphere.");

      typedef eigenpy::VariantConverter<GeomVariant> Converter;
      Converter::registration();
    }

    void exposeGeometryGraph()
    {
      using namespace pinocchio::graph;

      bp::enum_<GeomType>("GeomType")
        .value("VISUAL", GeomType::VISUAL)
        .value("COLLISION", GeomType::COLLISION)
        .value("BOTH", GeomType::BOTH)
        .export_values();

      bp::class_<Geometry>(
        "Geometry", "Main geometry object containing all properties for the Model Graph.",
        bp::init<>())
        .def(bp::init<
             const std::string &, const pinocchio::SE3 &, const GeomType &, const Eigen::Vector3d &,
             const Eigen::Vector4d &, const GeomVariant &>(
          (bp::arg("name"), bp::arg("placement"), bp::arg("type"), bp::arg("scale"),
           bp::arg("color"), bp::arg("geometry")),
          "Full constructor for Geometry."))
        .def_readwrite("name", &Geometry::name, "Name of the geometry object.")
        .def_readwrite("type", &Geometry::type, "Type of geometry (VISUAL, COLLISION, BOTH).")
        .def_readwrite("scale", &Geometry::scale, "Scaling factors.")
        .def_readwrite("color", &Geometry::color, "RGBA color.")
        .def_readwrite(
          "placement", &Geometry::placement,
          "SE3 placement (pose) of the geometry wrt to the body.")
        .def_readwrite("geometry", &Geometry::geometry, "The actual geometric primitive.");

      StdAlignedVectorPythonVisitor<Geometry>::expose("StdVec_Geometry");
    }

    void exposeGeometryBuilder()
    {
      using namespace pinocchio::graph;

      bp::class_<GeometryBuilder>(
        "GeometryBuilder", "A builder for Geometries in Model Graph.",
        bp::init<ModelGraph &>((bp::arg("model_graph"))))
        .def(
          "withName", &GeometryBuilder::withName, bp::return_self<>(), (bp::arg("name")),
          "Sets the name of the geometry.")
        .def(
          "withBody", &GeometryBuilder::withBody, bp::return_self<>(), (bp::arg("body_name")),
          "Sets the name of the body this geometry is attached to.")
        .def(
          "withPlacement", &GeometryBuilder::withPlacement, bp::return_self<>(),
          (bp::arg("placement")), "Sets the SE3 placement of the geometry.")
        .def(
          "withScale", &GeometryBuilder::withScale, bp::return_self<>(), (bp::arg("scale")),
          "Sets the scale factors.")
        .def(
          "withColor", &GeometryBuilder::withColor, bp::return_self<>(), (bp::arg("color")),
          "Sets the RGBA color.")
        .def(
          "withGeomType", &GeometryBuilder::withGeomType, bp::return_self<>(),
          (bp::arg("geom_type")), "Sets the type of geometry (VISUAL, COLLISION, BOTH).")
        .def(
          "withGeom", &GeometryBuilder::withGeom, bp::return_self<>(),
          (bp::arg("geometry_primitive")), "Sets the geometry primitive .")
        .def(
          "build", &GeometryBuilder::build,
          "Builds the Geometry object and adds it to the right vertex in the ModelGraph "
          "Throws if geometry name is empty.");
    }
  } // namespace python
} // namespace pinocchio
