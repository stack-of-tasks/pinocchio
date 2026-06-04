// Copyright (c) 2026 INRIA

#ifdef PINOCCHIO_WITH_URDFDOM
  #include "pinocchio/parsers/urdf.hpp"
#endif

#include "pinocchio/bindings/python-nb/fwd.hpp"

#include <nanobind/stl/string.h>
#include <nanobind/stl/filesystem.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
using namespace nb::literals;

void exposeURDFModel(nb::module_ m)
{
#ifdef PINOCCHIO_WITH_URDFDOM
  m.def(
    "buildModelFromUrdf",
    [](const std::filesystem::path & filename, const bool mimic) {
      Model model;
      pinocchio::urdf::buildModel(filename.string(), model, false, mimic);
      return model;
    },
    "urdf_filename"_a, "mimic"_a = false,
    "Parse the URDF file given in input and return a pinocchio Model.");

  m.def(
    "buildModelFromUrdf",
    [](const std::filesystem::path & filename, Model & model, const bool mimic) -> Model & {
      return pinocchio::urdf::buildModel(filename.string(), model, false, mimic);
    },
    "urdf_filename"_a, "model"_a, "mimic"_a = false,
    "Append to a given model a URDF structure given by its filename.", nb::rv_policy::reference);

  m.def(
    "buildModelFromUrdf",
    [](const std::filesystem::path & filename, const JointModel & root_joint, const bool mimic) {
      Model model;
      pinocchio::urdf::buildModel(filename.string(), root_joint, model, false, mimic);
      return model;
    },
    "urdf_filename"_a, "root_joint"_a, "mimic"_a = false,
    "Parse the URDF file given in input and return a pinocchio Model starting with the given "
    "root joint.");

  m.def(
    "buildModelFromUrdf",
    [](
      const std::filesystem::path & filename, const JointModel & root_joint,
      const std::string & root_joint_name, const bool mimic) {
      Model model;
      pinocchio::urdf::buildModel(
        filename.string(), root_joint, root_joint_name, model, false, mimic);
      return model;
    },
    "urdf_filename"_a, "root_joint"_a, "root_joint_name"_a, "mimic"_a = false,
    "Parse the URDF file given in input and return a pinocchio Model starting with the given "
    "root joint with its specified name.");

  m.def(
    "buildModelFromUrdf",
    [](
      const std::filesystem::path & filename, const JointModel & root_joint, Model & model,
      const bool mimic) -> Model & {
      return pinocchio::urdf::buildModel(filename.string(), root_joint, model, false, mimic);
    },
    "urdf_filename"_a, "root_joint"_a, "model"_a, "mimic"_a = false,
    "Append to a given model a URDF structure given by its filename and the root joint.\n"
    "Remark: In the URDF format, a joint of type fixed can be defined. For efficiency "
    "reasons, it is treated as operational frame and not as a joint of the model.",
    nb::rv_policy::reference);

  m.def(
    "buildModelFromUrdf",
    [](
      const std::filesystem::path & filename, const JointModel & root_joint,
      const std::string & root_joint_name, Model & model, const bool mimic) -> Model & {
      return pinocchio::urdf::buildModel(
        filename.string(), root_joint, root_joint_name, model, false, mimic);
    },
    "urdf_filename"_a, "root_joint"_a, "root_joint_name"_a, "model"_a, "mimic"_a = false,
    "Append to a given model a URDF structure given by its filename and the root joint with "
    "its specified name.\n"
    "Remark: In the URDF format, a joint of type fixed can be defined. For efficiency "
    "reasons, it is treated as operational frame and not as a joint of the model.",
    nb::rv_policy::reference);

  m.def(
    "buildModelFromXML",
    [](const std::string & xml_stream, const bool mimic) {
      Model model;
      pinocchio::urdf::buildModelFromXML(xml_stream, model, false, mimic);
      return model;
    },
    "urdf_xml_stream"_a, "mimic"_a = false,
    "Parse the URDF XML stream given in input and return a pinocchio Model.");

  m.def(
    "buildModelFromXML",
    [](const std::string & xml_stream, Model & model, const bool mimic) -> Model & {
      pinocchio::urdf::buildModelFromXML(xml_stream, model, false, mimic);
      return model;
    },
    "urdf_xml_stream"_a, "model"_a, "mimic"_a = false,
    "Parse the URDF XML stream given in input and append it to the input model.",
    nb::rv_policy::reference);

  m.def(
    "buildModelFromXML",
    [](const std::string & xml_stream, const JointModel & root_joint, const bool mimic) {
      Model model;
      pinocchio::urdf::buildModelFromXML(xml_stream, root_joint, model, false, mimic);
      return model;
    },
    "urdf_xml_stream"_a, "root_joint"_a, "mimic"_a = false,
    "Parse the URDF XML stream given in input and return a pinocchio Model starting with the "
    "given root joint.");

  m.def(
    "buildModelFromXML",
    [](
      const std::string & xml_stream, const JointModel & root_joint,
      const std::string & root_joint_name, const bool mimic) {
      Model model;
      pinocchio::urdf::buildModelFromXML(
        xml_stream, root_joint, root_joint_name, model, false, mimic);
      return model;
    },
    "urdf_xml_stream"_a, "root_joint"_a, "root_joint_name"_a, "mimic"_a = false,
    "Parse the URDF XML stream given in input and return a pinocchio Model starting with the "
    "given root joint with its specified name.");

  m.def(
    "buildModelFromXML",
    [](
      const std::string & xml_stream, const JointModel & root_joint, Model & model,
      const bool mimic) -> Model & {
      pinocchio::urdf::buildModelFromXML(xml_stream, root_joint, model, false, mimic);
      return model;
    },
    "urdf_xml_stream"_a, "root_joint"_a, "model"_a, "mimic"_a = false,
    "Parse the URDF XML stream given in input and append it to the input model with the given "
    "interfacing joint.",
    nb::rv_policy::reference);

  m.def(
    "buildModelFromXML",
    [](
      const std::string & xml_stream, const JointModel & root_joint,
      const std::string & root_joint_name, Model & model, const bool mimic) -> Model & {
      pinocchio::urdf::buildModelFromXML(
        xml_stream, root_joint, root_joint_name, model, false, mimic);
      return model;
    },
    "urdf_xml_stream"_a, "root_joint"_a, "root_joint_name"_a, "model"_a, "mimic"_a = false,
    "Parse the URDF XML stream given in input and append it to the input model with the given "
    "interfacing joint with its specified name.",
    nb::rv_policy::reference);
#endif
  (void)m;
}

PINOCCHIO_PYTHON_NAMESPACE_END
