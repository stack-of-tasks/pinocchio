#pragma once

#include "./context.hpp"
#include <nanobind/nanobind.h>
#include <vector>
#include <string>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
namespace nb = nanobind;
PINOCCHIO_PYTHON_NAMESPACE_END

/// On the model of NB_MAKE_OPAQUE, ensure std::vector of a given Pinocchio template class is opaque
/// e.g. excluded from the type_caster.
#define PINOCCHIO_PYTHON_STD_VEC_OPAQUE_TPL(TplClass)                                              \
  namespace nanobind::detail                                                                       \
  {                                                                                                \
    template<typename... Ts>                                                                       \
    class type_caster<TplClass<Ts...>> : public type_caster_base<TplClass<Ts...>>                  \
    {                                                                                              \
    };                                                                                             \
  }

// DISABLE SPECIFIC TYPE_CASTERS FOR std::vector
NB_MAKE_OPAQUE(std::vector<std::string>);
PINOCCHIO_PYTHON_STD_VEC_OPAQUE_TPL(pinocchio::SE3Tpl);
PINOCCHIO_PYTHON_STD_VEC_OPAQUE_TPL(pinocchio::MotionTpl);
PINOCCHIO_PYTHON_STD_VEC_OPAQUE_TPL(pinocchio::ForceTpl);
PINOCCHIO_PYTHON_STD_VEC_OPAQUE_TPL(pinocchio::InertiaTpl);
PINOCCHIO_PYTHON_STD_VEC_OPAQUE_TPL(pinocchio::JointModelTpl);
