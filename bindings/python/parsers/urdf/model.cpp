//
// Copyright (c) 2015-2020 CNRS INRIA
//

#ifdef PINOCCHIO_WITH_URDFDOM
  #include "pinocchio/parsers/urdf.hpp"
#endif

#include "pinocchio/bindings/python/parsers/urdf.hpp"
#include "pinocchio/bindings/python/utils/path.hpp"

#include <boost/python.hpp>

namespace pinocchio
{
  namespace python
  {

    namespace bp = boost::python;

#ifdef PINOCCHIO_WITH_URDFDOM

    Model buildModelFromUrdf(const bp::object & filename, const bool mimic = false)
    {
      Model model;
      pinocchio::urdf::buildModel(path(filename), model, mimic);
      return model;
    }

    Model & buildModelFromUrdf(const bp::object & filename, Model & model, const bool mimic = false)
    {
      return pinocchio::urdf::buildModel(path(filename), model, mimic);
    }

    Model buildModelFromUrdf(
      const bp::object & filename, const JointModel & root_joint, const bool mimic = false)
    {
      Model model;
      pinocchio::urdf::buildModel(path(filename), root_joint, model, mimic);
      return model;
    }

    Model buildModelFromUrdf(
      const bp::object & filename,
      const JointModel & root_joint,
      const std::string & root_joint_name,
      const bool mimic = false)
    {
      Model model;
      pinocchio::urdf::buildModel(path(filename), root_joint, root_joint_name, model, mimic);
      return model;
    }

    Model & buildModelFromUrdf(
      const bp::object & filename,
      const JointModel & root_joint,
      Model & model,
      const bool mimic = false)
    {
      return pinocchio::urdf::buildModel(path(filename), root_joint, model, mimic);
    }

    Model & buildModelFromUrdf(
      const bp::object & filename,
      const JointModel & root_joint,
      const std::string & root_joint_name,
      Model & model,
      const bool mimic = false)
    {
      return pinocchio::urdf::buildModel(path(filename), root_joint, root_joint_name, model, mimic);
    }

    Model buildModelFromXML(
      const std::string & xml_stream, const JointModel & root_joint, const bool mimic = false)
    {
      Model model;
      pinocchio::urdf::buildModelFromXML(xml_stream, root_joint, model, mimic);
      return model;
    }

    Model buildModelFromXML(
      const std::string & xml_stream,
      const JointModel & root_joint,
      const std::string & root_joint_name,
      const bool mimic = false)
    {
      Model model;
      pinocchio::urdf::buildModelFromXML(xml_stream, root_joint, root_joint_name, model, mimic);
      return model;
    }

    Model & buildModelFromXML(
      const std::string & xml_stream,
      const JointModel & root_joint,
      Model & model,
      const bool mimic = false)
    {
      pinocchio::urdf::buildModelFromXML(xml_stream, root_joint, model, mimic);
      return model;
    }

    Model & buildModelFromXML(
      const std::string & xml_stream,
      const JointModel & root_joint,
      const std::string & root_joint_name,
      Model & model,
      const bool mimic = false)
    {
      pinocchio::urdf::buildModelFromXML(xml_stream, root_joint, root_joint_name, model, mimic);
      return model;
    }

    Model buildModelFromXML(const std::string & xml_stream, const bool mimic = true)
    {
      Model model;
      pinocchio::urdf::buildModelFromXML(xml_stream, model, mimic);
      return model;
    }

    Model &
    buildModelFromXML(const std::string & xml_stream, Model & model, const bool mimic = true)
    {
      pinocchio::urdf::buildModelFromXML(xml_stream, model, mimic);
      return model;
    }

#endif

    void exposeURDFModel()
    {

#ifdef PINOCCHIO_WITH_URDFDOM

      bp::def(
        "buildModelFromUrdf",
        static_cast<Model (*)(const bp::object &, const JointModel &, const bool)>(
          pinocchio::python::buildModelFromUrdf),
        bp::args("urdf_filename", "root_joint", "mimic"),
        "Parse the URDF file given in input and return a pinocchio Model starting with the "
        "given root joint.");

      bp::def(
        "buildModelFromUrdf",
        static_cast<Model (*)(
          const bp::object &, const JointModel &, const std::string &, const bool)>(
          pinocchio::python::buildModelFromUrdf),
        bp::args("urdf_filename", "root_joint", "root_joint_name", "mimic"),
        "Parse the URDF file given in input and return a pinocchio Model starting with the "
        "given root joint with its specified name.");

      bp::def(
        "buildModelFromUrdf",
        static_cast<Model (*)(const bp::object &, const bool)>(
          pinocchio::python::buildModelFromUrdf),
        bp::args("urdf_filename", "mimic"),
        "Parse the URDF file given in input and return a pinocchio Model.");

      bp::def(
        "buildModelFromUrdf",
        static_cast<Model & (*)(const bp::object &, Model &, const bool)>(
          pinocchio::python::buildModelFromUrdf),
        bp::args("urdf_filename", "model", "mimic"),
        "Append to a given model a URDF structure given by its filename.",
        bp::return_internal_reference<2>());

      bp::def(
        "buildModelFromUrdf",
        static_cast<Model & (*)(const bp::object &, const JointModel &, Model &, const bool)>(
          pinocchio::python::buildModelFromUrdf),
        bp::args("urdf_filename", "root_joint", "model", "mimic"),
        "Append to a given model a URDF structure given by its filename and the root joint.\n"
        "Remark: In the URDF format, a joint of type fixed can be defined. For efficiency reasons,"
        "it is treated as operational frame and not as a joint of the model.",
        bp::return_internal_reference<3>());

      bp::def(
        "buildModelFromUrdf",
        static_cast<Model & (*)(const bp::object &, const JointModel &, const std::string &,
                                Model &, const bool)>(pinocchio::python::buildModelFromUrdf),
        bp::args("urdf_filename", "root_joint", "root_joint_name", "model", "mimic"),
        "Append to a given model a URDF structure given by its filename and the root joint with "
        "its specified name.\n"
        "Remark: In the URDF format, a joint of type fixed can be defined. For efficiency reasons,"
        "it is treated as operational frame and not as a joint of the model.",
        bp::return_internal_reference<3>());

      bp::def(
        "buildModelFromXML",
        static_cast<Model (*)(const std::string &, const JointModel &, const bool)>(
          pinocchio::python::buildModelFromXML),
        bp::args("urdf_xml_stream", "root_joint", "mimic"),
        "Parse the URDF XML stream given in input and return a pinocchio Model starting with "
        "the given root joint.");

      bp::def(
        "buildModelFromXML",
        static_cast<Model (*)(
          const std::string &, const JointModel &, const std::string &, const bool)>(
          pinocchio::python::buildModelFromXML),
        bp::args(
          "urdf_xml_stream", "root_joint",
          "root_joint_name"
          "mimic"),
        "Parse the URDF XML stream given in input and return a pinocchio Model starting with "
        "the given root joint with its specified name.");

      bp::def(
        "buildModelFromXML",
        static_cast<Model (*)(
          const std::string &, const JointModel &, const std::string &, const bool)>(
          pinocchio::python::buildModelFromXML),
        bp::args("urdf_xml_stream", "root_joint", "root_joint_name", "mimic"),
        "Parse the URDF XML stream given in input and return a pinocchio Model starting with "
        "the given root joint with its specified name.");

      bp::def(
        "buildModelFromXML",
        static_cast<Model & (*)(const std::string &, const JointModel &, Model &, const bool)>(
          pinocchio::python::buildModelFromXML),
        bp::args("urdf_xml_stream", "root_joint", "model", "mimic"),
        "Parse the URDF XML stream given in input and append it to the input model with the "
        "given interfacing joint.",
        bp::return_internal_reference<3>());

      bp::def(
        "buildModelFromXML",
        static_cast<Model & (*)(const std::string &, const JointModel &, const std::string &,
                                Model &, const bool)>(pinocchio::python::buildModelFromXML),
        bp::args("urdf_xml_stream", "root_joint", "root_joint_name", "model", "mimic"),
        "Parse the URDF XML stream given in input and append it to the input model with the "
        "given interfacing joint with its specified name.",
        bp::return_internal_reference<3>());

      bp::def(
        "buildModelFromXML",
        static_cast<Model & (*)(const std::string &, const JointModel &, const std::string &,
                                Model &, const bool)>(pinocchio::python::buildModelFromXML),
        bp::args("urdf_xml_stream", "root_joint", "root_joint_name", "model", "mimic"),
        "Parse the URDF XML stream given in input and append it to the input model with the "
        "given interfacing joint with its specified name.",
        bp::return_internal_reference<3>());

      bp::def(
        "buildModelFromXML",
        static_cast<Model (*)(const std::string &, const bool)>(
          pinocchio::python::buildModelFromXML),
        bp::args("urdf_xml_stream", "mimic"),
        "Parse the URDF XML stream given in input and return a pinocchio Model.");

      bp::def(
        "buildModelFromXML",
        static_cast<Model & (*)(const std::string &, Model &, const bool)>(
          pinocchio::python::buildModelFromXML),
        bp::args("urdf_xml_stream", "model", "mimic"),
        "Parse the URDF XML stream given in input and append it to the input model.",
        bp::return_internal_reference<2>());
#endif
    }
  } // namespace python
} // namespace pinocchio
