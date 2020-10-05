//
// Copyright (c) 2015-2020 CNRS INRIA
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

#include <eigenpy/eigenpy.hpp>

#include <Eigen/Geometry>
#include <eigenpy/geometry.hpp>

namespace bp = boost::python;
using namespace pinocchio::python;

BOOST_PYTHON_MODULE(pinocchio_pywrap)
{
  bp::docstring_options module_docstring_options(true,true,false);
  
  bp::scope().attr("__version__") = pinocchio::printVersion();
  bp::scope().attr("__raw_version__") = bp::str(PINOCCHIO_VERSION);
  eigenpy::enableEigenPy();
  
  // Enable warning
#ifndef Py_LIMITED_API
  _PyWarnings_Init();
#endif
  
  if(! register_symbolic_link_to_registered_type<Eigen::Quaterniond>())
    eigenpy::exposeQuaternion();
  if(! register_symbolic_link_to_registered_type<Eigen::AngleAxisd>())
    eigenpy::exposeAngleAxis();
  
  StdContainerFromPythonList< std::vector<std::string> >::register_converter();

  typedef Eigen::Matrix<double,6,6> Matrix6d;
  typedef Eigen::Matrix<double,6,1> Vector6d;
  typedef Eigen::Matrix<double,6,Eigen::Dynamic> Matrix6x;
  typedef Eigen::Matrix<double,3,Eigen::Dynamic> Matrix3x;
  
  eigenpy::enableEigenPySpecific<Matrix6d>();
  eigenpy::enableEigenPySpecific<Vector6d>();
  eigenpy::enableEigenPySpecific<Matrix6x>();
  eigenpy::enableEigenPySpecific<Matrix3x>();

  exposeSE3();
  exposeForce();
  exposeMotion();
  exposeInertia();
  exposeJoints();
  exposeExplog();
  exposeRpy();
  exposeSkew();
  exposeLieGroups();

  bp::enum_< ::pinocchio::ReferenceFrame >("ReferenceFrame")
  .value("WORLD",::pinocchio::WORLD)
  .value("LOCAL",::pinocchio::LOCAL)
  .value("LOCAL_WORLD_ALIGNED",::pinocchio::LOCAL_WORLD_ALIGNED)
  .export_values()
  ;

  bp::enum_< ::pinocchio::KinematicLevel >("KinematicLevel")
  .value("POSITION",::pinocchio::POSITION)
  .value("VELOCITY",::pinocchio::VELOCITY)
  .value("ACCELERATION",::pinocchio::ACCELERATION)
  .export_values()
  ;
  
  bp::enum_< ::pinocchio::ArgumentPosition>("ArgumentPosition")
  .value("ARG0",::pinocchio::ARG0)
  .value("ARG1",::pinocchio::ARG1)
  .value("ARG2",::pinocchio::ARG2)
  .value("ARG3",::pinocchio::ARG3)
  .value("ARG4",::pinocchio::ARG4)
  .export_values()
  ;

  exposeModel();
  exposeFrame();
  exposeData();
  exposeGeometry();
  
  exposeAlgorithms();
  exposeParsers();
  
#ifdef PINOCCHIO_WITH_HPP_FCL_PYTHON_BINDINGS
  exposeFCL();
#endif // PINOCCHIO_WITH_HPP_FCL_PYTHON_BINDINGS
  
  exposeVersion();
  exposeDependencies();
  exposeConversions();
  
}
 
