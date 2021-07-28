//
// Copyright (c) 2021 CNRS
//

#ifdef PINOCCHIO_WITH_SDF
#ifdef PINOCCHIO_WITH_HPP_FCL
#include "pinocchio/parsers/sdf.hpp"
#include "pinocchio/bindings/python/parsers/sdf.hpp"
#endif // #ifdef PINOCCHIO_WITH_HPP_FCL
#endif

#include <boost/python.hpp>
#include <boost/python/tuple.hpp>

namespace pinocchio
{
  namespace python
  {

    namespace bp = boost::python;
#ifdef PINOCCHIO_WITH_SDF
#ifdef PINOCCHIO_WITH_HPP_FCL
    bp::tuple buildModelFromSdf(const std::string & filename)
    {
      Model model;
      PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
      ::pinocchio::sdf::buildModel(filename, model, contact_models);
      return bp::make_tuple(model,contact_models);
    }

    bp::tuple buildModelFromSdf(const std::string & filename,
				const JointModel & root_joint)
    {
      Model model;
      PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
      pinocchio::sdf::buildModel(filename, root_joint, model, contact_models);
      return bp::make_tuple(model,contact_models);
    }
#endif // #ifdef PINOCCHIO_WITH_HPP_FCL
#endif
    
    void exposeSDFModel()
    {

#ifdef PINOCCHIO_WITH_SDF
#ifdef PINOCCHIO_WITH_HPP_FCL      
      bp::def("buildModelFromSdf",
              static_cast <bp::tuple (*) (const std::string &)> (pinocchio::python::buildModelFromSdf),
              bp::args("sdf_filename"),
              "Parse the SDF file given in input and return a pinocchio Model and constraint models."
              );

      bp::def("buildModelFromSdf",
              static_cast <bp::tuple (*) (const std::string &, const JointModel &)> (pinocchio::python::buildModelFromSdf),
              bp::args("sdf_filename","root_joint"),
              "Parse the SDF file given in input and return a pinocchio Model and constraint models starting with the given root joint."
              );
#endif // #ifdef PINOCCHIO_WITH_HPP_FCL
#endif
      
    }
  }
}
