//
// Copyright (c) 2018-2020 CNRS INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/bindings/python/utils/namespace.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/bindings/python/utils/eigen.hpp"

#include <eigenpy/eigen-to-python.hpp>

namespace pinocchio
{
  namespace python
  {
  
    namespace bp = boost::python;
    typedef PINOCCHIO_ALIGNED_STD_VECTOR(Force) ForceAlignedVector;

    bp::tuple computeABADerivatives(const Model & model, Data & data,
                                    const Eigen::VectorXd & q,
                                    const Eigen::VectorXd & v,
                                    const Eigen::VectorXd & tau)
    {
      pinocchio::computeABADerivatives(model,data,q,v,tau);
      make_symmetric(data.Minv);
      return bp::make_tuple(make_ref(data.ddq_dq),
                            make_ref(data.ddq_dv),
                            make_ref(data.Minv));
    }

    bp::tuple computeABADerivatives_fext(const Model & model, Data & data,
                                         const Eigen::VectorXd & q,
                                         const Eigen::VectorXd & v,
                                         const Eigen::VectorXd & tau,
                                         const ForceAlignedVector & fext)
    {
      pinocchio::computeABADerivatives(model,data,q,v,tau,fext);
      make_symmetric(data.Minv);
      return bp::make_tuple(make_ref(data.ddq_dq),
                            make_ref(data.ddq_dv),
                            make_ref(data.Minv));
    }

    namespace optimized
    {
      bp::tuple computeABADerivatives(const Model & model, Data & data)
      {
        pinocchio::optimized::computeABADerivatives(model,data);
        make_symmetric(data.Minv);
        return bp::make_tuple(make_ref(data.ddq_dq),
                              make_ref(data.ddq_dv),
                              make_ref(data.Minv));
      }

      bp::tuple computeABADerivatives_fext(const Model & model, Data & data,
                                           const ForceAlignedVector & fext)
      {
        pinocchio::optimized::computeABADerivatives(model,data,fext);
        make_symmetric(data.Minv);
        return bp::make_tuple(make_ref(data.ddq_dq),
                              make_ref(data.ddq_dv),
                              make_ref(data.Minv));
      }
    }

    void exposeABADerivatives()
    {
      using namespace Eigen;

      bp::def("computeABADerivatives",
              computeABADerivatives,
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
                optimized::computeABADerivatives,
                bp::args("model","data"),
                "Computes the ABA derivatives, store the result in data.ddq_dq, data.ddq_dv and data.Minv\n"
                "which correspond to the partial derivatives of the joint acceleration vector output with respect to the joint configuration,\n"
                "velocity and torque vectors.\n"
                "By calling this function, the user assumes that pinocchio.optimized.aba has been called first, allowing to significantly reduce the computation timings by not recalculating intermediate results.");
        
        bp::def("computeABADerivatives",
                optimized::computeABADerivatives_fext,
                bp::args("model","data","fext"),
                "Computes the ABA derivatives, store the result in data.ddq_dq, data.ddq_dv and data.Minv\n"
                "which correspond to the partial derivatives of the joint acceleration vector output with respect to the joint configuration,\n"
                "velocity and torque vectors.\n"
                "By calling this function, the user assumes that pinocchio.optimized.aba has been called first, allowing to significantly reduce the computation timings by not recalculating intermediate results.");
      }
      
    }
  } // namespace python
} // namespace pinocchio
