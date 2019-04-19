//
// Copyright (c) 2018 CNRS
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"

namespace pinocchio
{
  namespace python
  {
    void computeABADerivativesDefault(const Model & model, Data & data,
                                      const Eigen::VectorXd & q,
                                      const Eigen::VectorXd & v,
                                      const Eigen::VectorXd & tau)
    {
      computeABADerivatives(model,data,q,v,tau);
    }
    typedef container::aligned_vector<Force> ForceAlignedVector;
    void computeABADerivatives_fext(const Model & model, Data & data,
                                    const Eigen::VectorXd & q,
                                    const Eigen::VectorXd & v,
                                    const Eigen::VectorXd & tau,
                                    const ForceAlignedVector & fext)
    {
      computeABADerivatives(model,data,q,v,tau,fext);
    }

    void exposeABADerivatives()
    {
      using namespace Eigen;

      bp::def("computeABADerivatives",
              computeABADerivativesDefault,
              bp::args("Model","Data",
                       "Configuration q (size Model::nq)",
                       "Velocity v (size Model::nv)",
                       "Torque tau (size Model::nv)"),
              "Computes the ABA derivatives, put the result in data.ddq_dq, data.ddq_dv and data.Minv\n"
              "which correspond to the partial derivatives of the joint acceleration vector output with respect to the joint configuration,\n"
              "velocity and torque vectors.");

      bp::def("computeABADerivatives",
              computeABADerivatives_fext,
              bp::args("Model","Data",
                       "q: configuration vector (size model.nq)",
                       "v: velocity vector (size model.nv)",
                       "a: acceleration vector (size model.nv)",
                       "fext: vector external forces (size model.njoints)"),
              "Computes the ABA derivatives with external contact foces,\n"
              "put the result in data.ddq_dq, data.ddq_dv and data.Minv\n"
              "which correspond to the partial derivatives of the acceleration output with respect to the joint configuration,\n"
              "velocity and torque vectors.\n"
              "The forces are of type StdVec_Force.");
    }
  } // namespace python
} // namespace pinocchio
