//
// Copyright (c) 2015-2020 CNRS INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/bindings/python/utils/eigen.hpp"
#include "pinocchio/algorithm/crba.hpp"

namespace pinocchio
{
  namespace python
  {
    static context::MatrixXs crba_proxy(const context::Model & model,
                                        context::Data & data,
                                        const context::VectorXs & q)
    {
      data.M.fill(0);
      crba(model,data,q);
      make_symmetric(data.M);
      return data.M;
    }
    
    void exposeCRBA()
    {
      bp::def("crba",crba_proxy,
              bp::args("model","data","q"),
              "Computes CRBA, store the result in Data and return it.\n"
              "Parameters:\n"
              "\tq: the joint configuration vector (size model.nq)\n");
    }
    
  } // namespace python
} // namespace pinocchio
