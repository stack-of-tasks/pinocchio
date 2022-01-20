//
// Copyright (c) 2021 CNRS INRIA
//

#ifdef PINOCCHIO_PINOCCHIO_WITH_SDFORMAT
  #include "pinocchio/parsers/sdf.hpp"
#endif
#include "pinocchio/bindings/python/parsers/sdf.hpp"

#include <boost/python.hpp>
#include <boost/python/tuple.hpp>

namespace pinocchio
{
  namespace python
  {

    namespace bp = boost::python;

#ifdef PINOCCHIO_PINOCCHIO_WITH_SDFORMAT
    bp::tuple buildModelFromSdf(const std::string & filename)
    {
      Model model;
      PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
      ::pinocchio::sdf::buildModel(filename, model, contact_models);
      return bp::make_tuple(model,contact_models);
    }

    bp::tuple buildModelFromSdf(const std::string & filename,
                                const bp::object & root_joint_object)
    {
      JointModelVariant root_joint = bp::extract<JointModelVariant>(root_joint_object)();
      Model model;
      PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
      pinocchio::sdf::buildModel(filename, root_joint, model, contact_models);
      return bp::make_tuple(model,contact_models);
    }
#endif
    
    void exposeSDFModel()
    {
#ifdef PINOCCHIO_PINOCCHIO_WITH_SDFORMAT
      bp::def("buildModelFromSdf",
              static_cast <bp::tuple (*) (const std::string &)> (pinocchio::python::buildModelFromSdf),
              bp::args("sdf_filename"),
              "Parse the SDF file given in input and return a pinocchio Model."
              );

      bp::def("buildModelFromSdf",
              static_cast <bp::tuple (*) (const std::string &, const bp::object &)> (pinocchio::python::buildModelFromSdf),
              bp::args("sdf_filename","root_joint"),
              "Append to a given model a SDF structure given by its filename and the root joint."
              );
#endif
    }
  }
}
