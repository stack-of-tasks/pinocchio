#include "pinocchio/bindings/python-nb/fwd.hpp"
#include "pinocchio/bindings/python-nb/multibody/model.hpp"
#include "pinocchio/bindings/python-nb/multibody/data.hpp"
#include "pinocchio/bindings/python-nb/multibody/frame.hpp"
#include "pinocchio/bindings/python-nb/multibody/geometry-data.hpp"
#include "pinocchio/bindings/python-nb/multibody/geometry-model.hpp"
#include "pinocchio/bindings/python-nb/multibody/geometry-object.hpp"

#include "pinocchio/bindings/python-nb/multibody/joint-generic.hpp"
#include "pinocchio/bindings/python-nb/multibody/joint-collection.hpp"

constexpr pinocchio::FrameType kAllFrameTypes = static_cast<pinocchio::FrameType>(
  pinocchio::JOINT | pinocchio::FIXED_JOINT | pinocchio::BODY | pinocchio::OP_FRAME
  | pinocchio::SENSOR);

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
namespace nb = nanobind;

void exposeSampleModels(nb::module_ m);
void exposeLieGroups(nb::module_ m);
#ifdef PINOCCHIO_PYTHON_INTERFACE_WITH_OPENMP
void exposePool(nb::module_ m);
#endif

void exposeJointModelSplineBuilder(nb::module_ m)
{
  using namespace nb::literals;

  nb::class_<JointModelSplineBuilder>(m, "JointModelSplineBuilder", "JointSpline builder helper")
    .def(nb::init<>(), "Default constructor")
    .def(
      "addControlFrame", &JointModelSplineBuilder::addControlFrame, nb::rv_policy::reference,
      "frame"_a, "Add a control frame")
    .def(
      "withControlFrameVector", &JointModelSplineBuilder::withControlFrameVector,
      nb::rv_policy::reference, "frames"_a, "Set control frame")
    .def(
      "withDegree", &JointModelSplineBuilder::withDegree, nb::rv_policy::reference, "degree"_a,
      "Set spline's degree")
    .def(
      "withKnotVector",
      static_cast<JointModelSplineBuilder & (
        JointModelSplineBuilder::*)(const std::vector<Scalar> &)>(
        &JointModelSplineBuilder::withKnotVector),
      nb::rv_policy::reference, "knots"_a, "Set knot vector")
    .def(
      "withOpenUniformKnots", &JointModelSplineBuilder::withOpenUniformKnots,
      nb::rv_policy::reference, "min"_a, "max"_a, "Set spline knot vector as open uniform")
    .def(
      "withUniformKnots", &JointModelSplineBuilder::withUniformKnots, nb::rv_policy::reference,
      "min"_a, "max"_a, "Set spline knot vector as uniform")
    .def("build", &JointModelSplineBuilder::build, "Build a JointSpline from provided parameters");
}

void exposeMultibody(nb::module_ m)
{
  using pinocchio::FrameType;

  nb::enum_<FrameType>(m, "FrameType")
    .value("OP_FRAME", pinocchio::OP_FRAME)
    .value("JOINT", pinocchio::JOINT)
    .value("FIXED_JOINT", pinocchio::FIXED_JOINT)
    .value("BODY", pinocchio::BODY)
    .value("SENSOR", pinocchio::SENSOR)
    .value("_FRAMETYPE_ALL_TYPES", kAllFrameTypes) // useful for default arguments
    .export_values();

  exposeModel<pinocchio::Model>(m);
  exposeData<pinocchio::Data>(m);
  exposeFrame<pinocchio::Frame>(m);
  exposeSampleModels(m);

  // Lie groups
  // before joints to ensure JointModel.lieGroup() works
  exposeLieGroups(m);

  // Joints
  exposeJointModel<pinocchio::JointModel>(m);
  exposeJointData<pinocchio::JointData>(m);
  exposeJointCollection<pinocchio::JointCollectionDefault>(m);
  exposeJointModelSplineBuilder(m);

  // Pool
#ifdef PINOCCHIO_PYTHON_INTERFACE_WITH_OPENMP
  exposePool(m);
#endif

  // Geometry
  exposeGeometryData(m);
  exposeGeometryModel(m);
  exposeGeometryObject(m);
}
PINOCCHIO_PYTHON_NAMESPACE_END
