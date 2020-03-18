//
// Copyright (c) 2018-2020 CNRS INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/bindings/python/utils/namespace.hpp"
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
              computeABADerivativesDefault,
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
      
      {
        bp::scope current_scope = getOrCreatePythonNamespace("optimized");
        
        bp::def("computeABADerivatives",
                (void (*)(const Model &, Data &))optimized::computeABADerivatives<double,0,JointCollectionDefaultTpl>,
                bp::args("model","data"),
                "Computes the ABA derivatives, store the result in data.ddq_dq, data.ddq_dv and data.Minv\n"
                "which correspond to the partial derivatives of the joint acceleration vector output with respect to the joint configuration,\n"
                "velocity and torque vectors.\n"
                "By calling this function, the user assumes that pinocchio.optimized.aba has been called first, allowing to significantly reduce the computation timings by not recalculating intermediate results.");
        
        bp::def("computeABADerivatives",
                (void (*)(const Model &, Data &, const container::aligned_vector<Force> &))optimized::computeABADerivatives<double,0,JointCollectionDefaultTpl>,
                bp::args("model","data","fext"),
                "Computes the ABA derivatives, store the result in data.ddq_dq, data.ddq_dv and data.Minv\n"
                "which correspond to the partial derivatives of the joint acceleration vector output with respect to the joint configuration,\n"
                "velocity and torque vectors.\n"
                "By calling this function, the user assumes that pinocchio.optimized.aba has been called first, allowing to significantly reduce the computation timings by not recalculating intermediate results.");
      }
      
    }
  } // namespace python
} // namespace pinocchio
