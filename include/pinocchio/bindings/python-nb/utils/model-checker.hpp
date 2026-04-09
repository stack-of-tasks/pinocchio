// Copyright (c) 2026 INRIA

#pragma once

#include "pinocchio/bindings/python-nb/fwd.hpp"

#include "pinocchio/algorithm/check-model.hpp"

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

/// A nanobind call policy which checks that if given a Model object, it does not containt a mimic
/// joint (JointModelMimic). This assumes
template<size_t model_arg_idx>
struct mimic_not_supported_policy
{

  template<size_t N>
  static void
  precall(PyObject ** args, std::integral_constant<size_t, N>, nb::detail::cleanup_list *)
  {
    static_assert(
      model_arg_idx < N, "Provided argument for Model object exceeds arity of function!");
    nb::handle model_obj = args[model_arg_idx];
    if (!nb::isinstance<Model>(model_obj))
    {
      static char msg[68];
      snprintf(
        msg, sizeof(msg), "Argument in the specified location (%lu) is not of type Model.",
        model_arg_idx);
      throw std::runtime_error(msg);
    }
    const Model & model = nb::cast<const Model &>(model_obj);

    if (!model.check(MimicChecker()))
    {
      throw std::runtime_error(error_message);
    }
  }

  static void postcall(PyObject **, size_t, nb::handle)
  {
  }

  static constexpr char error_message[] = "The algorithm does not support joints of type "
                                          "JointMimic being present in the Pinocchio model.";
};

PINOCCHIO_PYTHON_NAMESPACE_END
