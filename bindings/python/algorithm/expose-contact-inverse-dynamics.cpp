//
// Copyright (c) 2024 INRIA
//

#define BOOST_PYTHON_MAX_ARITY 24

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/bindings/python/utils/std-vector.hpp"
#include "pinocchio/algorithm/contact-inverse-dynamics.hpp"

namespace pinocchio
{
  namespace python
  {

#ifndef PINOCCHIO_PYTHON_SKIP_ALGORITHM_CONSTRAINED_DYNAMICS
    typedef context::Scalar Scalar;
    typedef context::VectorXs VectorXs;
    typedef const Eigen::Ref<const VectorXs> ConstRefVectorXs;
    enum
    {
      Options = context::Options
    };

    static ConstRefVectorXs computeContactImpulses_wrapper(
      const ModelTpl<Scalar, Options, JointCollectionDefaultTpl> & model,
      DataTpl<Scalar, Options, JointCollectionDefaultTpl> & data,
      const ConstRefVectorXs & c_ref,
      const context::RigidConstraintModelVector & contact_models,
      context::RigidConstraintDataVector & contact_datas,
      const context::CoulombFrictionConeVector & cones,
      const ConstRefVectorXs & R,
      const ConstRefVectorXs & constraint_correction,
      ProximalSettingsTpl<Scalar> & settings,
      const boost::optional<ConstRefVectorXs> & lambda_guess = boost::none)
    {
      return computeContactImpulses(
        model, data, c_ref, contact_models, contact_datas, cones, R, constraint_correction,
        settings, lambda_guess);
    }

    static ConstRefVectorXs contactInverseDynamics_wrapper(
      const ModelTpl<Scalar, Options, JointCollectionDefaultTpl> & model,
      DataTpl<Scalar, Options, JointCollectionDefaultTpl> & data,
      ConstRefVectorXs & q,
      ConstRefVectorXs & v,
      ConstRefVectorXs & a,
      Scalar dt,
      const context::RigidConstraintModelVector & contact_models,
      context::RigidConstraintDataVector & contact_datas,
      const context::CoulombFrictionConeVector & cones,
      ConstRefVectorXs & R,
      ConstRefVectorXs & constraint_correction,
      ProximalSettingsTpl<Scalar> & settings,
      const boost::optional<ConstRefVectorXs> & lambda_guess = boost::none)
    {
      return contactInverseDynamics(
        model, data, q, v, a, dt, contact_models, contact_datas, cones, R, constraint_correction,
        settings, lambda_guess);
    }
#endif // PINOCCHIO_PYTHON_SKIP_ALGORITHM_CONSTRAINED_DYNAMICS

    void exposeContactInverseDynamics()
    {
#ifndef PINOCCHIO_PYTHON_SKIP_ALGORITHM_CONSTRAINED_DYNAMICS
      bp::def(
        "computeContactForces", computeContactImpulses_wrapper,
        (bp::arg("model"), "data", "c_ref", "contact_models", "contact_datas", "cones", "R",
         "constraint_correction", bp::arg("settings"), bp::arg("lambda_guess") = boost::none),
        "Compute the inverse dynamics with frictional contacts, store the result in Data and "
        "return it.\n\n"
        "Parameters:\n"
        "\tmodel: model of the kinematic tree\n"
        "\tdata: data related to the model\n"
        "\tc_ref: the reference velocity of contact points\n"
        "\tcontact_models: list of contact models\n"
        "\tcontact_datas: list of contact datas\n"
        "\tcones: list of friction cones\n"
        "\tR: vector representing the diagonal of the compliance matrix\n"
        "\tconstraint_correction: vector representing the constraint correction\n"
        "\tsettings: the settings of the proximal algorithm\n"
        "\tlambda_guess: initial guess for contact forces\n");

      bp::def(
        "contactInverseDynamics", contactInverseDynamics_wrapper,
        (bp::arg("model"), "data", "q", "v", "a", "dt", "contact_models", "contact_datas", "cones",
         "R", "constraint_correction", bp::arg("settings"), bp::arg("lambda_guess") = boost::none),
        "Compute the inverse dynamics with frictional contacts, store the result in Data and "
        "return it.\n\n"
        "Parameters:\n"
        "\tmodel: model of the kinematic tree\n"
        "\tdata: data related to the model\n"
        "\tq: the joint configuration vector (size model.nq)\n"
        "\tv: the joint velocity vector (size model.nv)\n"
        "\ta: the joint acceleration vector (size model.nv)\n"
        "\tdt: the time step\n"
        "\tcontact_models: list of contact models\n"
        "\tcontact_datas: list of contact datas\n"
        "\tcones: list of friction cones\n"
        "\tR: vector representing the diagonal of the compliance matrix\n"
        "\tconstraint_correction: vector representing the constraint correction\n"
        "\tsettings: the settings of the proximal algorithm\n"
        "\tlambda_guess: initial guess for contact forces\n");
#endif // PINOCCHIO_PYTHON_SKIP_ALGORITHM_CONSTRAINED_DYNAMICS
    }
  } // namespace python
} // namespace pinocchio
