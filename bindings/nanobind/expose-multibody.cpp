#include "pinocchio/bindings/python-nb/fwd.hpp"
#include "pinocchio/bindings/python-nb/multibody/model.hpp"
#include "pinocchio/bindings/python-nb/multibody/joint-model.hpp"

void exposeMultibody(nanobind::module_ m)
{
  using namespace pinocchio::python_nb;

  exposeModel<pinocchio::Model>(m);
  exposeJointModel<pinocchio::JointModel>(m);
}
