//
// Copyright (c) 2018-2020 CNRS INRIA
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
  
    typedef PINOCCHIO_ALIGNED_STD_VECTOR(Force) ForceAlignedVector;
  
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
              &computeABADerivatives<,
              bp::args("model","data","q","v","tau"),
              "Computes the ABA derivatives, store the result in data.ddq_dq, data.ddq_dv and data.Minv\n"
              "which correspond to the partial derivatives of the joint acceleration vector output with respect to the joint configuration,\n"
              "velocity and torque vectors.");

      bp::def("computeABADerivatives",
              computeABADerivatives_fext,
              bp::args("model","data","q","v","tau","fext"),
              "Computes the ABA derivatives with external contact foces,\n"
              "store the result in data.ddq_dq, data.ddq_dv and data.Minv\n"
              "which correspond to the partial derivatives of the acceleration output with respect to the joint configuration,\n"
              "velocity and torque vectors.");
    }
  } // namespace python
} // namespace pinocchio
