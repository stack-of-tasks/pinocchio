//
// Copyright (c) 2015-2018 CNRS
// Copyright (c) 2018-2025 INRIA
//

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/constraints/fwd.hpp"
#include "pinocchio/utils/version.hpp"
#include "pinocchio/bindings/python/utils/version.hpp"
#include "pinocchio/bindings/python/utils/dependencies.hpp"

#include "pinocchio/bindings/python/utils/std-vector.hpp"
#include "pinocchio/spatial.hpp"
#include "pinocchio/bindings/python/serialization/serialization.hpp"

#include <eigenpy/eigenpy.hpp>
#include <eigenpy/optional.hpp>

namespace bp = boost::python;
using namespace pinocchio::python;

BOOST_PYTHON_MODULE(PINOCCHIO_PYTHON_MODULE_NAME)
{
  bp::docstring_options module_docstring_options(true, true, false);

  bp::scope().attr("__bindings_framework__") = bp::str("Boost.Python");
  bp::scope().attr("__version__") = pinocchio::printVersion();
  bp::scope().attr("__raw_version__") = bp::str(PINOCCHIO_VERSION);
  eigenpy::enableEigenPy();

  // Enable warnings
  bp::import("warnings");

#if defined(PINOCCHIO_PYTHON_INTERFACE_MAIN_MODULE)                                                \
  && defined(PINOCCHIO_PYTHON_INTERFACE_WITH_COLLISION_PYTHON_BINDINGS)
  bp::import("coal");
#endif

  exposeEigenTypes();
  exposeSpecificTypeFeatures();

  eigenpy::OptionalConverter<context::VectorXs, boost::optional>::registration();
  eigenpy::OptionalConverter<context::RefVectorXs, boost::optional>::registration();
  eigenpy::OptionalConverter<context::RefConstVectorXs, boost::optional>::registration();
  eigenpy::OptionalConverter<const context::RefConstVectorXs, boost::optional>::registration();
  eigenpy::OptionalConverter<context::Scalar, boost::optional>::registration();

#if defined(PINOCCHIO_PYTHON_INTERFACE_MAIN_MODULE)
  eigenpy::StdContainerFromPythonList<std::vector<std::string>>::register_converter();
#endif

  bp::scope().attr("ScalarType") = getScalarType();

  if (!register_symbolic_link_to_registered_type<::pinocchio::ReferenceFrame>())
  {
    bp::enum_<::pinocchio::ReferenceFrame>("ReferenceFrame")
      .value("WORLD", ::pinocchio::WORLD)
      .value("LOCAL", ::pinocchio::LOCAL)
      .value("LOCAL_WORLD_ALIGNED", ::pinocchio::LOCAL_WORLD_ALIGNED)
      .export_values();
  }

  if (!register_symbolic_link_to_registered_type<::pinocchio::KinematicLevel>())
  {
    bp::enum_<::pinocchio::KinematicLevel>("KinematicLevel")
      .value("POSITION", ::pinocchio::POSITION)
      .value("VELOCITY", ::pinocchio::VELOCITY)
      .value("ACCELERATION", ::pinocchio::ACCELERATION)
      .export_values();
  }

  if (!register_symbolic_link_to_registered_type<::pinocchio::Convention>())
  {
    bp::enum_<::pinocchio::Convention>("Convention")
      .value("WORLD", ::pinocchio::Convention::WORLD)
      .value("LOCAL", ::pinocchio::Convention::LOCAL);
  }

  if (!register_symbolic_link_to_registered_type<::pinocchio::ArgumentPosition>())
  {
    bp::enum_<::pinocchio::ArgumentPosition>("ArgumentPosition")
      .value("ARG0", ::pinocchio::ARG0)
      .value("ARG1", ::pinocchio::ARG1)
      .value("ARG2", ::pinocchio::ARG2)
      .value("ARG3", ::pinocchio::ARG3)
      .value("ARG4", ::pinocchio::ARG4)
      .export_values();
  }

  if (!register_symbolic_link_to_registered_type<::pinocchio::ConstraintSelectionType>())
  {
    bp::enum_<::pinocchio::ConstraintSelectionType>("ConstraintSelectionType")
      .value("CURRENT", ::pinocchio::ConstraintSelectionType::CURRENT)
      .value("MAXIMAL", ::pinocchio::ConstraintSelectionType::MAXIMAL);
  }

  exposeSE3();
  exposeForce();
  exposeMotion();
  exposeInertia();
  exposeSymmetric3();
  exposeJoints();
  exposeConstraints();
  exposeExplog();
  exposeRpy();
  exposeLinalg();
  exposeTridiagonalMatrix();
  exposeLanczosDecomposition();
  exposeGramSchmidtOrthonormalisation();
  exposeSkew();
  exposeLieGroups();

  exposeFrame();
  exposeModel();
  exposeData();
  exposeSampleModels();
#if defined(PINOCCHIO_PYTHON_INTERFACE_MAIN_MODULE)
  exposeGeometry();
#endif // defined(PINOCCHIO_PYTHON_INTERFACE_MAIN_MODULE)
  exposeParsers();

  exposeAlgorithms();
  exposeExtras();
  exposeSerialization();

#if defined(PINOCCHIO_PYTHON_INTERFACE_WITH_COLLISION_PYTHON_BINDINGS)                             \
  && defined(PINOCCHIO_PYTHON_INTERFACE_MAIN_MODULE)
  exposeCoal();
  exposeCollision();
#endif // defined(PINOCCHIO_PYTHON_INTERFACE_WITH_COLLISION_PYTHON_BINDINGS) &&
       // defined(PINOCCHIO_PYTHON_INTERFACE_MAIN_MODULE)

#if defined(PINOCCHIO_PYTHON_INTERFACE_WITH_OPENMP)                                                \
  && defined(PINOCCHIO_PYTHON_INTERFACE_MAIN_MODULE)
  exposePool();
  exposeParallelAlgorithms();
#endif

#if defined(PINOCCHIO_PYTHON_INTERFACE_WITH_COLLISION_PYTHON_BINDINGS)                             \
  && defined(PINOCCHIO_PYTHON_INTERFACE_WITH_OPENMP)                                               \
  && defined(PINOCCHIO_PYTHON_INTERFACE_MAIN_MODULE)
  exposePoolCollision();
  exposeParallelCollision();
#endif

  exposeVersion();
  exposeDependencies();
  exposeConversions();

  typedef std::vector<::pinocchio::VectorXb> StdVec_VectorXb;
  typedef std::vector<::Eigen::Index> StdVec_Index;
  typedef std::vector<context::MatrixXs> StdVec_MatrixXs;

  StdVectorPythonVisitor<StdVec_VectorXb, false>::expose(
    "StdVec_VectorXb", eigenpy::details::overload_base_get_item_for_std_vector<StdVec_VectorXb>());
  StdVectorPythonVisitor<StdVec_Index, true>::expose("StdVec_long");
  StdVectorPythonVisitor<StdVec_MatrixXs, false>::expose(
    "StdVec_MatrixXs", eigenpy::details::overload_base_get_item_for_std_vector<StdVec_MatrixXs>());
}
