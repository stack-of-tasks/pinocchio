//
// Copyright (c) 2015-2024 CNRS INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/rnea.hpp"

namespace pinocchio
{
  namespace python
  {

    void exposeRNEA()
    {
      typedef context::Scalar Scalar;
      typedef context::VectorXs VectorXs;
      enum
      {
        Options = context::Options
      };

      bp::def(
        "rnea", &rnea<Scalar, Options, JointCollectionDefaultTpl, VectorXs, VectorXs, VectorXs>,
        bp::args("model", "data", "q", "v", "a"),
        "Compute the RNEA, store the result in Data and return it.\n\n"
        "Parameters:\n"
        "\tmodel: model of the kinematic tree\n"
        "\tdata: data related to the model\n"
        "\tq: the joint configuration vector (size model.nq)\n"
        "\tv: the joint velocity vector (size model.nv)\n"
        "\ta: the joint acceleration vector (size model.nv)\n",
        bp::return_value_policy<bp::return_by_value>());

      bp::def(
        "rnea",
        &rnea<
          Scalar, Options, JointCollectionDefaultTpl, VectorXs, VectorXs, VectorXs, context::Force>,
        bp::args("model", "data", "q", "v", "a", "fext"),
        "Compute the RNEA with external forces, store the result in Data and return it.\n\n"
        "Parameters:\n"
        "\tmodel: model of the kinematic tree\n"
        "\tdata: data related to the model\n"
        "\tq: the joint configuration vector (size model.nq)\n"
        "\tv: the joint velocity vector (size model.nv)\n"
        "\ta: the joint acceleration vector (size model.nv)\n"
        "\tfext: list of external forces expressed in the local frame of the joints (size "
        "model.njoints)\n",
        bp::return_value_policy<bp::return_by_value>());

      bp::def(
        "nonLinearEffects",
        &nonLinearEffects<Scalar, Options, JointCollectionDefaultTpl, VectorXs, VectorXs>,
        bp::args("model", "data", "q", "v"),
        "Compute the Non Linear Effects (coriolis, centrifugal and gravitational effects), "
        "store the result in Data and return it.\n\n"
        "Parameters:\n"
        "\tmodel: model of the kinematic tree\n"
        "\tdata: data related to the model\n"
        "\tq: the joint configuration vector (size model.nq)\n"
        "\tv: the joint velocity vector (size model.nv)\n",
        bp::return_value_policy<bp::return_by_value>());

      bp::def(
        "computeGeneralizedGravity",
        &computeGeneralizedGravity<Scalar, Options, JointCollectionDefaultTpl, VectorXs>,
        bp::args("model", "data", "q"),
        "Compute the generalized gravity contribution g(q) of the Lagrangian dynamics, store "
        "the result in data.g and return it.\n\n"
        "Parameters:\n"
        "\tmodel: model of the kinematic tree\n"
        "\tdata: data related to the model\n"
        "\tq: the joint configuration vector (size model.nq)\n",
        bp::return_value_policy<bp::return_by_value>());

      bp::def(
        "computeStaticTorque",
        &computeStaticTorque<Scalar, Options, JointCollectionDefaultTpl, VectorXs>,
        bp::args("model", "data", "q", "fext"),
        "Computes the generalized static torque contribution g(q) - J.T f_ext of the "
        "Lagrangian dynamics, store the result in data.tau and return it.\n\n"
        "Parameters:\n"
        "\tmodel: model of the kinematic tree\n"
        "\tdata: data related to the model\n"
        "\tq: the joint configuration vector (size model.nq)\n"
        "\tfext: list of external forces expressed in the local frame of the joints (size "
        "model.njoints)\n",
        bp::return_value_policy<bp::return_by_value>());

      bp::def(
        "computeCoriolisMatrix",
        &computeCoriolisMatrix<Scalar, Options, JointCollectionDefaultTpl, VectorXs, VectorXs>,
        bp::args("model", "data", "q", "v"),
        "Compute the Coriolis Matrix C(q,v) of the Lagrangian dynamics, store the result in data.C "
        "and return it.\n\n"
        "Parameters:\n"
        "\tmodel: model of the kinematic tree\n"
        "\tdata: data related to the model\n"
        "\tq: the joint configuration vector (size model.nq)\n"
        "\tv: the joint velocity vector (size model.nv)\n",
        bp::return_value_policy<bp::return_by_value>());

      bp::def(
        "getCoriolisMatrix", &getCoriolisMatrix<Scalar, Options, JointCollectionDefaultTpl>,
        bp::args("model", "data"),
        "Retrives the Coriolis Matrix C(q,v) of the Lagrangian dynamics after calling one of "
        "the derivative algorithms, store the result in data.C and return it.\n\n"
        "Parameters:\n"
        "\tmodel: model of the kinematic tree\n"
        "\tdata: data related to the model\n",
        bp::return_value_policy<bp::return_by_value>());
    }

  } // namespace python
} // namespace pinocchio
