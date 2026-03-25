#include "pinocchio/bindings/python-nb/fwd.hpp"
#include "pinocchio/bindings/python-nb/multibody/model.hpp"
#include "pinocchio/bindings/python-nb/multibody/data.hpp"
#include "pinocchio/bindings/python-nb/multibody/joint-model.hpp"
#include "pinocchio/bindings/python-nb/multibody/frame.hpp"

constexpr pinocchio::FrameType kAllFrameTypes = static_cast<pinocchio::FrameType>(
  pinocchio::JOINT | pinocchio::FIXED_JOINT | pinocchio::BODY | pinocchio::OP_FRAME
  | pinocchio::SENSOR);

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
namespace nb = nanobind;

void exposeSampleModels(nb::module_ m);

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
  exposeJointModel<pinocchio::JointModel>(m);
  exposeFrame<pinocchio::Frame>(m);
}
PINOCCHIO_PYTHON_NAMESPACE_END
