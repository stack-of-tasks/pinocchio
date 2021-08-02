//
// Copyright (c) 2021 INRIA
//

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/utils/namespace.hpp"

#include <Eigen/Dense>

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    template<typename MatrixType>
    typename PINOCCHIO_EIGEN_PLAIN_TYPE(MatrixType) inv(const Eigen::MatrixBase<MatrixType> & mat)
    {
      return mat.inverse();
    }

    void exposeLinalg()
    {
      using namespace Eigen;

      {
        // using the rpy scope
        bp::scope current_scope = getOrCreatePythonNamespace("linalg");

        bp::def("inv",&inv<MatrixXd>,"Computes the inverse of a matrix.");
        bp::def("inv",&inv<Matrix< long double,Dynamic,Dynamic> >,"Computes the inverse of a matrix.");
      }
      
    }
    
  } // namespace python
} // namespace pinocchio
