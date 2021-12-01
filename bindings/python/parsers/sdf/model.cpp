//
// Copyright (c) 2021 CNRS INRIA
//

#ifdef PINOCCHIO_WITH_SDFORMAT
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

#ifdef PINOCCHIO_WITH_SDFORMAT
    bp::tuple buildModelFromSdf(const std::string & filename,
                                const std::string & root_link_name)
    {
      Model model;
      PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
      ::pinocchio::sdf::buildModel(filename, model, contact_models, root_link_name);
      return bp::make_tuple(model,contact_models);
    }

    bp::tuple buildModelFromSdf(const std::string & filename,
				const JointModel & root_joint,
                                const std::string & root_link_name)
    {
      Model model;
      PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
      pinocchio::sdf::buildModel(filename, root_joint, model, contact_models, root_link_name);
      return bp::make_tuple(model,contact_models);
    }
#endif
    
    void exposeSDFModel()
    {
#ifdef PINOCCHIO_WITH_SDFORMAT
      bp::def("buildModelFromSdf",
              static_cast <bp::tuple (*) (const std::string &, const std::string &)> (pinocchio::python::buildModelFromSdf),
              bp::args("sdf_filename", "root_link_name"),
              "Parse the SDF file given in input and return a pinocchio Model and constraint models."
              );

      bp::def("buildModelFromSdf",
              static_cast <bp::tuple (*) (const std::string &, const JointModel &, const std::string &)> (pinocchio::python::buildModelFromSdf),
              bp::args("sdf_filename","root_joint", "root_link_name"),
              "Parse the SDF file given in input and return a pinocchio Model and constraint models starting with the given root joint."
              );
#endif
    }
  }
}
