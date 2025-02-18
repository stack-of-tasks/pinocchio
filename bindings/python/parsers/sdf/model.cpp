//
// Copyright (c) 2021 CNRS INRIA
//

#ifdef PINOCCHIO_WITH_SDFORMAT
  #include "pinocchio/parsers/sdf.hpp"
#endif
#include "pinocchio/bindings/python/parsers/sdf.hpp"
#include "pinocchio/bindings/python/utils/path.hpp"

#include <boost/python.hpp>
#include <boost/python/tuple.hpp>

namespace pinocchio
{
  namespace python
  {

    namespace bp = boost::python;

#ifdef PINOCCHIO_WITH_SDFORMAT
    bp::tuple buildModelFromSdf(
      const bp::object & filename,
      const std::string & root_link_name,
      const std::vector<std::string> & parent_guidance)
    {
      Model model;
      PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
      ::pinocchio::sdf::buildModel(
        path(filename), model, contact_models, root_link_name, parent_guidance);
      return bp::make_tuple(model, contact_models);
    }

    bp::tuple buildModelFromSdf(
      const bp::object & filename,
      const JointModel & root_joint,
      const std::string & root_link_name,
      const std::vector<std::string> & parent_guidance)
    {
      Model model;
      PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
      pinocchio::sdf::buildModel(
        path(filename), root_joint, model, contact_models, root_link_name, parent_guidance);
      return bp::make_tuple(model, contact_models);
    }

    bp::tuple buildModelFromSdf(
      const bp::object & filename,
      const JointModel & root_joint,
      const std::string & root_link_name,
      const std::string & root_joint_name,
      const std::vector<std::string> & parent_guidance)
    {
      Model model;
      PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
      pinocchio::sdf::buildModel(
        path(filename), root_joint, root_joint_name, model, contact_models, root_link_name,
        parent_guidance);
      return bp::make_tuple(model, contact_models);
    }
#endif

    void exposeSDFModel()
    {
#ifdef PINOCCHIO_WITH_SDFORMAT
      bp::def(
        "buildModelFromSdf",
        static_cast<bp::tuple (*)(
          const bp::object &, const std::string &, const std::vector<std::string> &)>(
          pinocchio::python::buildModelFromSdf),
        (bp::arg("sdf_filename"), bp::arg("root_link_name"),
         bp::arg("parent_guidance") = bp::list()),
        "Parse the SDF file given in input and return a pinocchio Model and constraint models.");

      bp::def(
        "buildModelFromSdf",
        static_cast<bp::tuple (*)(
          const bp::object &, const JointModel &, const std::string &,
          const std::vector<std::string> &)>(pinocchio::python::buildModelFromSdf),
        (bp::arg("sdf_filename"), bp::arg("root_joint"), bp::arg("root_link_name"),
         bp::arg("parent_guidance") = bp::list()),
        "Parse the SDF file given in input and return a pinocchio Model and constraint "
        "models starting with the given root joint.");

      bp::def(
        "buildModelFromSdf",
        static_cast<bp::tuple (*)(
          const bp::object &, const JointModel &, const std::string &, const std::string &,
          const std::vector<std::string> &)>(pinocchio::python::buildModelFromSdf),
        (bp::arg("sdf_filename"), bp::arg("root_joint"), bp::arg("root_link_name"),
         bp::arg("root_joint_name"), bp::arg("parent_guidance") = bp::list()),
        "Parse the SDF file given in input and return a pinocchio Model and constraint "
        "models starting with the given root joint and its specified name.");
#endif
    }
  } // namespace python
} // namespace pinocchio
