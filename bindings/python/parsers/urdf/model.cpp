//
// Copyright (c) 2015-2020 CNRS INRIA
//

#ifdef PINOCCHIO_WITH_URDFDOM
  #include "pinocchio/parsers/urdf.hpp"
#endif

#include "pinocchio/bindings/python/parsers/urdf.hpp"

#include <boost/python.hpp>

namespace pinocchio
{
  namespace python
  {

    namespace bp = boost::python;

#ifdef PINOCCHIO_WITH_URDFDOM

    Model buildModelFromUrdf(const std::string & filename)
    {
      Model model;
      pinocchio::urdf::buildModel(filename, model);
      return model;
    }

    Model & buildModelFromUrdf(const std::string & filename, Model & model)
    {
      return pinocchio::urdf::buildModel(filename, model);
    }

    Model buildModelFromUrdf(const std::string & filename, const JointModel & root_joint)
    {
      Model model;
      pinocchio::urdf::buildModel(filename, root_joint, model);
      return model;
    }

    Model buildModelFromUrdf(
      const std::string & filename,
      const JointModel & root_joint,
      const std::string & root_joint_name)
    {
      Model model;
      pinocchio::urdf::buildModel(filename, root_joint, root_joint_name, model);
      return model;
    }

    Model &
    buildModelFromUrdf(const std::string & filename, const JointModel & root_joint, Model & model)
    {
      return pinocchio::urdf::buildModel(filename, root_joint, model);
    }

    Model & buildModelFromUrdf(
      const std::string & filename,
      const JointModel & root_joint,
      const std::string & root_joint_name,
      Model & model)
    {
      return pinocchio::urdf::buildModel(filename, root_joint, root_joint_name, model);
    }

    Model buildModelFromXML(const std::string & XMLstream, const JointModel & root_joint)
    {
      Model model;
      pinocchio::urdf::buildModelFromXML(XMLstream, root_joint, model);
      return model;
    }

    Model buildModelFromXML(
      const std::string & XMLstream,
      const JointModel & root_joint,
      const std::string & root_joint_name)
    {
      Model model;
      pinocchio::urdf::buildModelFromXML(XMLstream, root_joint, root_joint_name, model);
      return model;
    }

    Model &
    buildModelFromXML(const std::string & XMLstream, const JointModel & root_joint, Model & model)
    {
      pinocchio::urdf::buildModelFromXML(XMLstream, root_joint, model);
      return model;
    }

    Model & buildModelFromXML(
      const std::string & XMLstream,
      const JointModel & root_joint,
      const std::string & root_joint_name,
      Model & model)
    {
      pinocchio::urdf::buildModelFromXML(XMLstream, root_joint, root_joint_name, model);
      return model;
    }

    Model buildModelFromXML(const std::string & XMLstream)
    {
      Model model;
      pinocchio::urdf::buildModelFromXML(XMLstream, model);
      return model;
    }

    Model & buildModelFromXML(const std::string & XMLstream, Model & model)
    {
      pinocchio::urdf::buildModelFromXML(XMLstream, model);
      return model;
    }

#endif

    void exposeURDFModel()
    {

#ifdef PINOCCHIO_WITH_URDFDOM

      bp::def(
        "buildModelFromUrdf",
        static_cast<Model (*)(const std::string &, const JointModel &)>(
          pinocchio::python::buildModelFromUrdf),
        bp::args("urdf_filename", "root_joint"),
        "Parse the URDF file given in input and return a pinocchio Model starting with the "
        "given root joint.");

      bp::def(
        "buildModelFromUrdf",
        static_cast<Model (*)(const std::string &, const JointModel &, const std::string &)>(
          pinocchio::python::buildModelFromUrdf),
        bp::args("urdf_filename", "root_joint", "root_joint_name"),
        "Parse the URDF file given in input and return a pinocchio Model starting with the "
        "given root joint.");

      bp::def(
        "buildModelFromUrdf",
        static_cast<Model (*)(const std::string &)>(pinocchio::python::buildModelFromUrdf),
        bp::args("urdf_filename"),
        "Parse the URDF file given in input and return a pinocchio Model.");

      bp::def(
        "buildModelFromUrdf",
        static_cast<Model & (*)(const std::string &, Model &)>(
          pinocchio::python::buildModelFromUrdf),
        bp::args("urdf_filename", "model"),
        "Append to a given model a URDF structure given by its filename.",
        bp::return_internal_reference<2>());

      bp::def(
        "buildModelFromUrdf",
        static_cast<Model & (*)(const std::string &, const JointModel &, Model &)>(
          pinocchio::python::buildModelFromUrdf),
        bp::args("urdf_filename", "root_joint", "model"),
        "Append to a given model a URDF structure given by its filename and the root joint.\n"
        "Remark: In the URDF format, a joint of type fixed can be defined. For efficiency reasons,"
        "it is treated as operational frame and not as a joint of the model.",
        bp::return_internal_reference<3>());

      bp::def(
        "buildModelFromUrdf",
        static_cast<Model & (*)(const std::string &, const JointModel &, const std::string &,
                                Model &)>(pinocchio::python::buildModelFromUrdf),
        bp::args("urdf_filename", "root_joint", "root_joint_name", "model"),
        "Append to a given model a URDF structure given by its filename and the root joint.\n"
        "Remark: In the URDF format, a joint of type fixed can be defined. For efficiency reasons,"
        "it is treated as operational frame and not as a joint of the model.",
        bp::return_internal_reference<3>());

      bp::def(
        "buildModelFromXML",
        static_cast<Model (*)(const std::string &, const JointModel &)>(
          pinocchio::python::buildModelFromXML),
        bp::args("urdf_xml_stream", "root_joint"),
        "Parse the URDF XML stream given in input and return a pinocchio Model starting with "
        "the given root joint.");

      bp::def(
        "buildModelFromXML",
        static_cast<Model (*)(const std::string &, const JointModel &, const std::string &)>(
          pinocchio::python::buildModelFromXML),
        bp::args("urdf_xml_stream", "root_joint", "root_joint_name"),
        "Parse the URDF XML stream given in input and return a pinocchio Model starting with "
        "the given root joint.");

      bp::def(
        "buildModelFromXML",
        static_cast<Model & (*)(const std::string &, const JointModel &, Model &)>(
          pinocchio::python::buildModelFromXML),
        bp::args("urdf_xml_stream", "root_joint", "model"),
        "Parse the URDF XML stream given in input and append it to the input model with the "
        "given interfacing joint.",
        bp::return_internal_reference<3>());

      bp::def(
        "buildModelFromXML",
        static_cast<Model & (*)(const std::string &, const JointModel &, const std::string &,
                                Model &)>(pinocchio::python::buildModelFromXML),
        bp::args("urdf_xml_stream", "root_joint", "root_joint_name", "model"),
        "Parse the URDF XML stream given in input and append it to the input model with the "
        "given interfacing joint.",
        bp::return_internal_reference<3>());

      bp::def(
        "buildModelFromXML",
        static_cast<Model (*)(const std::string &)>(pinocchio::python::buildModelFromXML),
        bp::args("urdf_xml_stream"),
        "Parse the URDF XML stream given in input and return a pinocchio Model.");

      bp::def(
        "buildModelFromXML",
        static_cast<Model & (*)(const std::string &, Model &)>(
          pinocchio::python::buildModelFromXML),
        bp::args("urdf_xml_stream", "model"),
        "Parse the URDF XML stream given in input and append it to the input model.",
        bp::return_internal_reference<2>());
#endif
    }
  } // namespace python
} // namespace pinocchio
