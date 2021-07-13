//
// Copyright (c) 2015-2021 CNRS INRIA
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/utils/version.hpp"
#include "pinocchio/bindings/python/utils/version.hpp"
#include "pinocchio/bindings/python/utils/dependencies.hpp"
#include "pinocchio/bindings/python/utils/conversions.hpp"
#include "pinocchio/bindings/python/utils/registration.hpp"

#include "pinocchio/bindings/python/utils/std-vector.hpp"
#include "pinocchio/spatial/cartesian-axis.hpp"
#include "pinocchio/bindings/python/serialization/serialization.hpp"

#include <eigenpy/eigenpy.hpp>

#include <Eigen/Geometry>
#include <eigenpy/geometry.hpp>

namespace bp = boost::python;
using namespace pinocchio::python;

BOOST_PYTHON_MODULE(PINOCCHIO_PYTHON_MODULE_NAME)
{
  bp::docstring_options module_docstring_options(true,true,false);
  
  bp::scope().attr("__version__") = pinocchio::printVersion();
  bp::scope().attr("__raw_version__") = bp::str(PINOCCHIO_VERSION);
  eigenpy::enableEigenPy();
  
  // Enable warnings
  bp::import("warnings");
  
  eigenpy::enableEigenPy();
  exposeEigenTypes();
  exposeSpecificTypeFeatures();
  
  bp::scope().attr("ScalarType") = getScalarType();

  bp::scope().attr("XAxis")
  = bp::object(bp::handle<>(eigenpy::EigenToPy<const context::Vector3s &>::convert(pinocchio::XAxis::vector<context::Scalar>())));
  bp::scope().attr("YAxis")
  = bp::object(bp::handle<>(eigenpy::EigenToPy<const context::Vector3s &>::convert(pinocchio::YAxis::vector<context::Scalar>())));
  bp::scope().attr("ZAxis")
  = bp::object(bp::handle<>(eigenpy::EigenToPy<const context::Vector3s &>::convert(pinocchio::ZAxis::vector<context::Scalar>())));

  exposeSE3();
  exposeForce();
  exposeMotion();
  exposeInertia();
  exposeJoints();
  exposeExplog();
  exposeRpy();
  exposeSkew();
  exposeLieGroups();

  if(!register_symbolic_link_to_registered_type<::pinocchio::ReferenceFrame>())
  {
    bp::enum_< ::pinocchio::ReferenceFrame >("ReferenceFrame")
    .value("WORLD",::pinocchio::WORLD)
    .value("LOCAL",::pinocchio::LOCAL)
    .value("LOCAL_WORLD_ALIGNED",::pinocchio::LOCAL_WORLD_ALIGNED)
    .export_values()
    ;
  }

  if(!register_symbolic_link_to_registered_type<::pinocchio::KinematicLevel>())
  {
    bp::enum_< ::pinocchio::KinematicLevel >("KinematicLevel")
    .value("POSITION",::pinocchio::POSITION)
    .value("VELOCITY",::pinocchio::VELOCITY)
    .value("ACCELERATION",::pinocchio::ACCELERATION)
    .export_values()
    ;
  }
    
  if(!register_symbolic_link_to_registered_type<::pinocchio::ArgumentPosition>())
  {
    bp::enum_< ::pinocchio::ArgumentPosition>("ArgumentPosition")
    .value("ARG0",::pinocchio::ARG0)
    .value("ARG1",::pinocchio::ARG1)
    .value("ARG2",::pinocchio::ARG2)
    .value("ARG3",::pinocchio::ARG3)
    .value("ARG4",::pinocchio::ARG4)
    .export_values()
    ;
  }

  exposeModel();
  exposeFrame();
  exposeData();
  exposeGeometry();

  exposeAlgorithms();
  exposeParsers();
  exposeSerialization();
  
#ifdef PINOCCHIO_PYTHON_INTERFACE_WITH_HPP_FCL_PYTHON_BINDINGS
  exposeFCL();
#endif // PINOCCHIO_PYTHON_INTERFACE_WITH_HPP_FCL_PYTHON_BINDINGS
  
#ifdef PINOCCHIO_PYTHON_INTERFACE_WITH_OPENMP
  exposePool();
  exposeParallelAlgorithms();
#endif
  
  exposeVersion();
  exposeDependencies();
  exposeConversions();
  
}
 
