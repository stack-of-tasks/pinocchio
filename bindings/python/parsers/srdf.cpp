//
// Copyright (c) 2015-2021 CNRS INRIA
//

#include "pinocchio/parsers/srdf.hpp"
#include "pinocchio/bindings/python/parsers/srdf.hpp"

#include <boost/python.hpp>

namespace pinocchio
{
  namespace python
  {

    namespace bp = boost::python;

    void loadReferenceConfigurationsFromXML(
      Model & model, const std::string & xmlStream, const bool verbose = false)
    {
      std::istringstream iss(xmlStream);
      pinocchio::srdf::loadReferenceConfigurationsFromXML(model, iss, verbose);
    }

    void exposeSRDFParser()
    {

      bp::def(
        "removeCollisionPairs",
        static_cast<void (*)(const Model &, GeometryModel &, const std::string &, const bool)>(
          &srdf::removeCollisionPairs),
        (bp::arg("model"), bp::arg("geom_model"), bp::arg("srdf_filename"),
         bp::arg("verbose") = false),
        "Parse an SRDF file in order to remove some collision pairs for a specific GeometryModel.\n"
        "Parameters:\n"
        "Parameters:\n"
        "\tmodel: model of the robot\n"
        "\tgeom_model: geometry model of the robot\n"
        "\tsrdf_filename: path to the SRDF file containing the collision pairs to remove\n"
        "\tverbose: [optional] display to the current terminal some internal information");

      bp::def(
        "removeCollisionPairsFromXML",
        static_cast<void (*)(const Model &, GeometryModel &, const std::string &, const bool)>(
          &srdf::removeCollisionPairsFromXML),
        (bp::arg("model"), bp::arg("geom_model"), bp::arg("srdf_xml_stream"),
         bp::arg("verbose") = false),
        "Parse an SRDF file in order to remove some collision pairs for a specific GeometryModel.\n"
        "Parameters:\n"
        "Parameters:\n"
        "\tmodel: model of the robot\n"
        "\tgeom_model: geometry model of the robot\n"
        "\tsrdf_xml_stream: XML stream containing the SRDF information with the collision pairs to "
        "remove\n"
        "\tverbose: [optional] display to the current terminal some internal information");

      bp::def(
        "loadReferenceConfigurations",
        static_cast<void (*)(Model &, const std::string &, const bool)>(
          &srdf::loadReferenceConfigurations),
        (bp::arg("model"), bp::arg("srdf_filename"), bp::arg("verbose") = false),
        "Retrieve all the reference configurations of a given model from the SRDF file.\n"
        "Parameters:\n"
        "\tmodel: model of the robot\n"
        "\tsrdf_filename: path to the SRDF file containing the reference configurations\n"
        "\tverbose: [optional] display to the current terminal some internal information");

      bp::def(
        "loadReferenceConfigurationsFromXML",
        static_cast<void (*)(Model &, const std::string &, const bool)>(
          &srdf::loadReferenceConfigurations),
        (bp::arg("model"), bp::arg("srdf_xml_stream"), bp::arg("verbose") = false),
        "Retrieve all the reference configurations of a given model from the SRDF file.\n"
        "Parameters:\n"
        "\tmodel: model of the robot\n"
        "\tsrdf_xml_stream: XML stream containing the SRDF information with the reference "
        "configurations\n"
        "\tverbose: [optional] display to the current terminal some internal information");

      bp::def(
        "loadRotorParameters",
        static_cast<bool (*)(Model &, const std::string &, const bool)>(&srdf::loadRotorParameters),
        (bp::arg("model"), bp::arg("srdf_filename"), bp::arg("verbose") = false),
        "Load the rotor parameters of a given model from a SRDF file.\n"
        "Results are stored in model.rotorInertia and model.rotorGearRatio."
        "This function also fills the armature of the model."
        "Parameters:\n"
        "\tmodel: model of the robot\n"
        "\tsrdf_filename: path to the SRDF file containing the rotor parameters\n"
        "\tverbose: [optional] display to the current terminal some internal information");
    }
  } // namespace python
} // namespace pinocchio
