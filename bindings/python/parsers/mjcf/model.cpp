//
// Copyright (c) 2024 INRIA
//

#include "pinocchio/parsers/mjcf.hpp"
#include "pinocchio/bindings/python/parsers/mjcf.hpp"

#include <boost/python.hpp>

namespace pinocchio
{
  namespace python
  {

    namespace bp = boost::python;

    Model buildModelFromMJCF(const std::string & filename)
    {
      Model model;
      ::pinocchio::mjcf::buildModel(filename, model);
      return model;
    }

    Model buildModelFromMJCF(const std::string & filename, const JointModel & root_joint)
    {
      Model model;
      ::pinocchio::mjcf::buildModel(filename, root_joint, "root_joint", model);
      return model;
    }

    Model buildModelFromMJCF(
      const std::string & filename,
      const JointModel & root_joint,
      const std::string & root_joint_name)
    {
      Model model;
      ::pinocchio::mjcf::buildModel(filename, root_joint, root_joint_name, model);
      return model;
    }

    void exposeMJCFModel()
    {
      bp::def(
        "buildModelFromMJCF",
        static_cast<Model (*)(const std::string &)>(pinocchio::python::buildModelFromMJCF),
        bp::args("mjcf_filename"),
        "Parse the MJCF file given in input and return a pinocchio Model.");

      bp::def(
        "buildModelFromMJCF",
        static_cast<Model (*)(const std::string &, const JointModel &)>(
          pinocchio::python::buildModelFromMJCF),
        bp::args("mjcf_filename", "root_joint"),
        "Parse the MJCF file and return a pinocchio Model with the given root Joint and its name.");

      bp::def(
        "buildModelFromMJCF",
        static_cast<Model (*)(const std::string &, const JointModel &, const std::string &)>(
          pinocchio::python::buildModelFromMJCF),
        bp::args("mjcf_filename", "root_joint", "root_joint_name"),
        "Parse the MJCF file and return a pinocchio Model with the given root Joint and its name.");
    }
  } // namespace python
} // namespace pinocchio
