//
// Copyright (c) 2018-2020 CNRS INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/bindings/python/utils/eigen.hpp"

#include <eigenpy/eigen-to-python.hpp>

namespace pinocchio
{
  namespace python
  {
  
    namespace bp = boost::python;
    bp::tuple computeABADerivativesDefault(const Model & model, Data & data,
                                           const Eigen::VectorXd & q,
                                           const Eigen::VectorXd & v,
                                           const Eigen::VectorXd & tau)
    {
      computeABADerivatives(model,data,q,v,tau);
      make_symmetric(data.Minv);
      return bp::make_tuple(make_ref(data.ddq_dq),
                            make_ref(data.ddq_dv),
                            make_ref(data.Minv));
    }
  
    typedef PINOCCHIO_ALIGNED_STD_VECTOR(Force) ForceAlignedVector;
  
    bp::tuple computeABADerivatives_fext(const Model & model, Data & data,
                                         const Eigen::VectorXd & q,
                                         const Eigen::VectorXd & v,
                                         const Eigen::VectorXd & tau,
                                         const ForceAlignedVector & fext)
    {
      computeABADerivatives(model,data,q,v,tau,fext);
      make_symmetric(data.Minv);
      return bp::make_tuple(make_ref(data.ddq_dq),
                            make_ref(data.ddq_dv),
                            make_ref(data.Minv));
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
              "Computes the ABA derivatives, store the result in data.ddq_dq, data.ddq_dv and data.Minv\n"
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
              "store the result in data.ddq_dq, data.ddq_dv and data.Minv\n"
              "which correspond to the partial derivatives of the acceleration output with respect to the joint configuration,\n"
              "velocity and torque vectors.\n");
    }
  } // namespace python
} // namespace pinocchio
