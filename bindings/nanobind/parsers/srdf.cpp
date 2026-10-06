// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"

#include "pinocchio/parsers/srdf.hpp"

#include <sstream>

#include <nanobind/stl/string.h>
#include <nanobind/stl/filesystem.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

using namespace nb::literals;

void exposeSRDF(nb::module_ m)
{
  m.def(
    "removeCollisionPairs",
    [](
      const Model & model, GeometryModel & geom_model, const std::filesystem::path & srdf_filename,
      bool verbose) {
      pinocchio::srdf::removeCollisionPairs(model, geom_model, srdf_filename, verbose);
    },
    "model"_a, "geom_model"_a, "srdf_filename"_a, "verbose"_a = false,
    "Parse an SRDF file in order to remove some collision pairs for a specific GeometryModel.\n"
    "Parameters:\n"
    "\tmodel: model of the robot\n"
    "\tgeom_model: geometry model of the robot\n"
    "\tsrdf_filename: path to the SRDF file containing the collision pairs to remove\n"
    "\tverbose: [optional] display to the current terminal some internal information");

  m.def(
    "removeCollisionPairsFromXML",
    [](
      const Model & model, GeometryModel & geom_model, const std::string & srdf_xml_stream,
      bool verbose) {
      pinocchio::srdf::removeCollisionPairsFromXML(model, geom_model, srdf_xml_stream, verbose);
    },
    "model"_a, "geom_model"_a, "srdf_xml_stream"_a, "verbose"_a = false,
    "Parse an SRDF string in order to remove some collision pairs for a specific GeometryModel.\n"
    "Parameters:\n"
    "\tmodel: model of the robot\n"
    "\tgeom_model: geometry model of the robot\n"
    "\tsrdf_xml_stream: XML stream containing the SRDF information with the collision pairs to "
    "remove\n"
    "\tverbose: [optional] display to the current terminal some internal information");

  m.def(
    "loadReferenceConfigurations",
    [](Model & model, const std::filesystem::path & srdf_filename, bool verbose) {
      pinocchio::srdf::loadReferenceConfigurations(model, srdf_filename, verbose);
    },
    "model"_a, "srdf_filename"_a, "verbose"_a = false,
    "Retrieve all the reference configurations of a given model from an SRDF file.\n"
    "Parameters:\n"
    "\tmodel: model of the robot\n"
    "\tsrdf_filename: path to the SRDF file containing the reference configurations\n"
    "\tverbose: [optional] display to the current terminal some internal information");

  m.def(
    "loadReferenceConfigurationsFromXML",
    [](Model & model, const std::string & srdf_xml_stream, bool verbose) {
      std::istringstream iss(srdf_xml_stream);
      pinocchio::srdf::loadReferenceConfigurationsFromXML(model, iss, verbose);
    },
    "model"_a, "srdf_xml_stream"_a, "verbose"_a = false,
    "Retrieve all the reference configurations of a given model from an SRDF XML string.\n"
    "Parameters:\n"
    "\tmodel: model of the robot\n"
    "\tsrdf_xml_stream: XML stream containing the SRDF information with the reference "
    "configurations\n"
    "\tverbose: [optional] display to the current terminal some internal information");

  m.def(
    "loadRotorParameters",
    [](Model & model, const std::filesystem::path & srdf_filename, bool verbose) {
      return pinocchio::srdf::loadRotorParameters(model, srdf_filename, verbose);
    },
    "model"_a, "srdf_filename"_a, "verbose"_a = false,
    "Load the rotor parameters of a given model from an SRDF file.\n"
    "Results are stored in model.rotorInertia and model.rotorGearRatio.\n"
    "This function also fills the armature of the model.\n"
    "Parameters:\n"
    "\tmodel: model of the robot\n"
    "\tsrdf_filename: path to the SRDF file containing the rotor parameters\n"
    "\tverbose: [optional] display to the current terminal some internal information");
}

PINOCCHIO_PYTHON_NAMESPACE_END
