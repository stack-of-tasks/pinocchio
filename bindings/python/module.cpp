//
// Copyright (c) 2015-2022 CNRS INRIA
//

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/utils/version.hpp"
#include "pinocchio/bindings/python/utils/version.hpp"
#include "pinocchio/bindings/python/utils/dependencies.hpp"
#include "pinocchio/bindings/python/utils/registration.hpp"

#include "pinocchio/bindings/python/utils/std-vector.hpp"
#include "pinocchio/spatial/cartesian-axis.hpp"
#include "pinocchio/bindings/python/serialization/serialization.hpp"

#include <eigenpy/eigenpy.hpp>

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
  
#if defined(PINOCCHIO_PYTHON_INTERFACE_MAIN_MODULE) && defined(PINOCCHIO_PYTHON_INTERFACE_WITH_HPP_FCL_PYTHON_BINDINGS)
  bp::import("hppfcl");
#endif
  
  exposeEigenTypes();
  exposeSpecificTypeFeatures();
  
#if defined(PINOCCHIO_PYTHON_INTERFACE_MAIN_MODULE)
  StdContainerFromPythonList< std::vector<std::string> >::register_converter();
#endif

  bp::scope().attr("ScalarType") = getScalarType();

  bp::scope().attr("XAxis")
  = bp::object(bp::handle<>(eigenpy::EigenToPy<context::Vector3s>::convert(pinocchio::XAxis::vector<context::Scalar>())));
  bp::scope().attr("YAxis")
  = bp::object(bp::handle<>(eigenpy::EigenToPy<context::Vector3s>::convert(pinocchio::YAxis::vector<context::Scalar>())));
  bp::scope().attr("ZAxis")
  = bp::object(bp::handle<>(eigenpy::EigenToPy<context::Vector3s>::convert(pinocchio::ZAxis::vector<context::Scalar>())));

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

  exposeSE3();
  exposeForce();
  exposeMotion();
  exposeInertia();
  exposeSymmetric3();
  exposeJoints();
  exposeExplog();
  exposeRpy();
  exposeLinalg();
  exposeSkew();
  exposeLieGroups();
  
  exposeFrame();
  exposeModel();
  exposeData();
#if defined(PINOCCHIO_PYTHON_INTERFACE_MAIN_MODULE)
  exposeGeometry();
  exposeParsers();
#endif // defined(PINOCCHIO_PYTHON_INTERFACE_MAIN_MODULE)

  exposeAlgorithms();
  exposeSerialization();
  
#if defined(PINOCCHIO_PYTHON_INTERFACE_WITH_HPP_FCL_PYTHON_BINDINGS) && defined(PINOCCHIO_PYTHON_INTERFACE_MAIN_MODULE)
  exposeFCL();
#endif // defined(PINOCCHIO_PYTHON_INTERFACE_WITH_HPP_FCL_PYTHON_BINDINGS) && defined(PINOCCHIO_PYTHON_INTERFACE_MAIN_MODULE)
  
#if defined(PINOCCHIO_PYTHON_INTERFACE_WITH_OPENMP) && defined(PINOCCHIO_PYTHON_INTERFACE_MAIN_MODULE)
  exposePool();
  exposeParallelAlgorithms();
#endif
  
  exposeVersion();
  exposeDependencies();
  exposeConversions();
  
}
 
