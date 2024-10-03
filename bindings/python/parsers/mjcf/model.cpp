//
// Copyright (c) 2024 INRIA
//

#include "pinocchio/parsers/mjcf.hpp"
#include "pinocchio/bindings/python/parsers/mjcf.hpp"
#include "pinocchio/bindings/python/utils/path.hpp"

#include <boost/python.hpp>

namespace pinocchio
{
  namespace python
  {

    namespace bp = boost::python;

    Model buildModelFromMJCF(const bp::object & filename)
    {
      Model model;
      ::pinocchio::mjcf::buildModel(path(filename), model);
      return model;
    }

    Model buildModelFromMJCF(const bp::object & filename, const JointModel & root_joint)
    {
      Model model;
      ::pinocchio::mjcf::buildModel(path(filename), root_joint, model);
      return model;
    }

    bp::tuple buildModelFromMJCF(
      const bp::object & filename,
      const JointModel & root_joint,
      const std::string & root_joint_name)
    {
      Model model;
      PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
      ::pinocchio::mjcf::buildModel(
        path(filename), root_joint, root_joint_name, model, contact_models);
      return bp::make_tuple(model, contact_models);
    }

    void exposeMJCFModel()
    {
      bp::def(
        "buildModelFromMJCF",
        static_cast<Model (*)(const bp::object &)>(pinocchio::python::buildModelFromMJCF),
        bp::args("mjcf_filename"),
        "Parse the MJCF file given in input and return a pinocchio Model.");

      bp::def(
        "buildModelFromMJCF",
        static_cast<Model (*)(const bp::object &, const JointModel &)>(
          pinocchio::python::buildModelFromMJCF),
        bp::args("mjcf_filename", "root_joint"),
        "Parse the MJCF file and return a pinocchio Model with the given root Joint.");

      bp::def(
        "buildModelFromMJCF",
        static_cast<bp::tuple (*)(const bp::object &, const JointModel &, const std::string &)>(
          pinocchio::python::buildModelFromMJCF),
        bp::args("mjcf_filename", "root_joint", "root_joint_name"),
        "Parse the MJCF file and return a pinocchio Model with the given root Joint and its "
        "specified name as well as a constraint list if some are present in the MJCF file.");
    }
  } // namespace python
} // namespace pinocchio
