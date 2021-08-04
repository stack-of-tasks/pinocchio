//
// Copyright (c) 2021 INRIA
//

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/utils/namespace.hpp"

#include "pinocchio/math/matrix.hpp"

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    template<typename MatrixType>
    typename PINOCCHIO_EIGEN_PLAIN_TYPE(MatrixType) inv(const Eigen::MatrixBase<MatrixType> & mat)
    {
      typename PINOCCHIO_EIGEN_PLAIN_TYPE(MatrixType) res(mat.rows(),mat.cols());
      inverse(mat,res);
      return res;
    }

    void exposeLinalg()
    {
      using namespace Eigen;

      {
        // using the rpy scope
        bp::scope current_scope = getOrCreatePythonNamespace("linalg");

        bp::def("inv",&inv<context::MatrixXs>,"Computes the inverse of a matrix.");
#ifdef PINOCCHIO_PYTHON_INTERFACE_MAIN_MODULE
        bp::def("inv",&inv<Matrix< long double,Dynamic,Dynamic> >,"Computes the inverse of a matrix.");
#endif
      }
      
    }
    
  } // namespace python
} // namespace pinocchio
