//
// Copyright (c) 2021 CNRS
//

#include "pinocchio/parsers/sdf.hpp"
#include "pinocchio/bindings/python/parsers/sdf.hpp"

#include <boost/python.hpp>
#include <boost/python/tuple.hpp>

namespace pinocchio
{
  namespace python
  {

    namespace bp = boost::python;

    bp::tuple buildModelFromSdf(const std::string & filename)
    {
      Model model;
      PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_models;
      ::pinocchio::sdf::buildModel(filename, model, contact_models);
      return bp::make_tuple(model,contact_models);
    }
  
    void exposeSDFModel()
    {
      bp::def("buildModelFromSdf",
              static_cast <bp::tuple (*) (const std::string &)> (pinocchio::python::buildModelFromSdf),
              bp::args("sdf_filename"),
              "Parse the SDF file given in input and return a pinocchio Model."
              );
    }
  }
}
