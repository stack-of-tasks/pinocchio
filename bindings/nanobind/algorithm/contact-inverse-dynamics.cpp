// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"

#include "pinocchio/algorithm/contact-inverse-dynamics.hpp"

#include <nanobind/stl/optional.h>
#include <nanobind/eigen/dense.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

using namespace nb::literals;

using PointContactConstraintModelVector = std::vector<PointContactConstraintModel>;
using PointContactConstraintDataVector = std::vector<PointContactConstraintData>;

void exposeContactInverseDynamics(nb::module_ m)
{
#ifndef PINOCCHIO_PYTHON_SKIP_ALGORITHM_CONSTRAINED_DYNAMICS
  m.def(
    "computeInverseDynamicsConstraintForces",
    [](
      ConstVectorRef c_ref, const PointContactConstraintModelVector & contact_models,
      PointContactConstraintDataVector & contact_datas, std::optional<VectorXs> lambda_guess,
      ProximalSettings & settings, bool solve_ncp) {
      const Eigen::Index problem_size = getTotalConstraintResidualSize(contact_models);
      VectorXs lambda_sol = lambda_guess.value_or(VectorXs::Zero(problem_size));
      const bool has_converged = computeInverseDynamicsConstraintForces(
        contact_models, contact_datas, c_ref, lambda_sol, settings, solve_ncp);
      return nb::make_tuple(has_converged, lambda_sol);
    },
    "c_ref"_a, "contact_models"_a, "contact_datas"_a, "lambda_guess"_a = nb::none(), "settings"_a,
    "solve_ncp"_a = true,
    "Computes the inverse dynamics with point contacts. Returns a tuple containing "
    "(has_converged, lambda_sol).\n\n"
    "Parameters:\n"
    "\tcontact_models: list of contact models\n"
    "\tc_ref: the reference velocity of contact points\n"
    "\tlambda_guess: optional initial guess for contact forces\n"
    "\tsettings: the settings of the proximal algorithm\n"
    "\tsolve_ncp: whether to solve the NCP (true) or CCP (false)");

  m.def(
    "contactInverseDynamics",
    [](
      const Model & model, Data & data, ConstVectorRef q, ConstVectorRef v, ConstVectorRef a,
      Scalar dt, const PointContactConstraintModelVector & contact_models,
      PointContactConstraintDataVector & contact_datas, Eigen::Ref<VectorXs> constraint_correction,
      ProximalSettings & settings, std::optional<VectorXs> lambda_guess, bool solve_ncp) {
      const Eigen::Index problem_size = getTotalConstraintResidualSize(contact_models);
      VectorXs lambda_sol = lambda_guess.value_or(VectorXs::Zero(problem_size));
      const bool has_converged = contactInverseDynamics(
        model, data, q, v, a, dt, contact_models, contact_datas, constraint_correction, lambda_sol,
        settings, solve_ncp);
      return nb::make_tuple(has_converged, data.tau, lambda_sol);
    },
    "model"_a, "data"_a, "q"_a, "v"_a, "a"_a, "dt"_a, "contact_models"_a, "contact_datas"_a,
    "constraint_correction"_a, "settings"_a, "lambda_guess"_a = nb::none(), "solve_ncp"_a = true,
    "Compute the inverse dynamics with point contacts, store the result in data and return it.\n\n"
    "Parameters:\n"
    "\tmodel: model of the kinematic tree\n"
    "\tdata: data related to the model\n"
    "\tq: the joint configuration vector (size model.nq)\n"
    "\tv: the joint velocity vector (size model.nv)\n"
    "\ta: the joint acceleration vector (size model.nv)\n"
    "\tdt: the time step\n"
    "\tcontact_models: list of contact models\n"
    "\tcontact_datas: list of contact datas\n"
    "\tconstraint_correction: vector representing the constraint correction\n"
    "\tsettings: the settings of the proximal algorithm\n"
    "\tlambda_guess: initial guess for contact forces\n"
    "\tsolve_ncp: whether to solve the NCP (true) or CCP (false)");
#endif // PINOCCHIO_PYTHON_SKIP_ALGORITHM_CONSTRAINED_DYNAMICS
}

PINOCCHIO_PYTHON_NAMESPACE_END
