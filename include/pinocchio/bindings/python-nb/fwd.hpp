// Copyright (c) 2026 INRIA

#pragma once

#include "./context.hpp"
#include <nanobind/nanobind.h>
#include <vector>
#include <string>
#include <map>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
namespace nb = nanobind;

template<template<class> class BaseTpl, typename T>
static constexpr bool is_tpl_base_of_v = std::is_base_of_v<BaseTpl<T>, T>;
PINOCCHIO_PYTHON_NAMESPACE_END

/// On the model of NB_MAKE_OPAQUE, ensure std::vector of a given Pinocchio template class is opaque
/// i.e. excluded from the type_caster.
#define PINOCCHIO_PYTHON_STD_VEC_OPAQUE_TPL(TplClass)                                              \
  namespace nanobind::detail                                                                       \
  {                                                                                                \
    template<typename... Ts>                                                                       \
    class type_caster<std::vector<TplClass<Ts...>>>                                                \
    : public type_caster_base<std::vector<TplClass<Ts...>>>                                        \
    {                                                                                              \
    };                                                                                             \
  }

// DISABLE SPECIFIC TYPE_CASTERS FOR std::vector
NB_MAKE_OPAQUE(std::vector<std::string>);
NB_MAKE_OPAQUE(std::vector<pinocchio::Index>);
// multibody map types
NB_MAKE_OPAQUE(std::map<std::string, pinocchio::python_nb::VectorXs>);
// spatial
PINOCCHIO_PYTHON_STD_VEC_OPAQUE_TPL(pinocchio::SE3Tpl);
PINOCCHIO_PYTHON_STD_VEC_OPAQUE_TPL(pinocchio::MotionTpl);
PINOCCHIO_PYTHON_STD_VEC_OPAQUE_TPL(pinocchio::ForceTpl);
PINOCCHIO_PYTHON_STD_VEC_OPAQUE_TPL(pinocchio::InertiaTpl);
// multibody
PINOCCHIO_PYTHON_STD_VEC_OPAQUE_TPL(pinocchio::JointModelTpl);
PINOCCHIO_PYTHON_STD_VEC_OPAQUE_TPL(pinocchio::JointDataTpl);
PINOCCHIO_PYTHON_STD_VEC_OPAQUE_TPL(pinocchio::FrameTpl);
