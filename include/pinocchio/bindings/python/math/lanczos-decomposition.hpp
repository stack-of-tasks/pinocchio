//
// Copyright (c) 2024 INRIA
//

#ifndef __pinocchio_python_math_lanczos_decomposition_hpp__
#define __pinocchio_python_math_lanczos_decomposition_hpp__

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/math/lanczos-decomposition.hpp"

#include <eigenpy/eigenpy.hpp>
#include <eigenpy/memory.hpp>

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    template<typename LanczosDecomposition>
    struct LanczosDecompositionPythonVisitor
    : public boost::python::def_visitor<LanczosDecompositionPythonVisitor<LanczosDecomposition>>
    {
      typedef typename LanczosDecomposition::Scalar Scalar;
      typedef typename LanczosDecomposition::TridiagonalMatrix TridiagonalMatrix;
      typedef typename LanczosDecomposition::PlainMatrix PlainMatrix;

    public:
      template<class PyClass>
      void visit(PyClass & cl) const
      {
        //        static const Scalar dummy_precision = Eigen::NumTraits<Scalar>::dummy_precision();

        cl.def(bp::init<const context::MatrixXs &, const Eigen::DenseIndex>(
                 (bp::arg("self"), bp::arg("mat"), bp::arg("decomposition_size")),
                 "Default constructor from a given matrix and a given decomposition size."))

          .def(
            "compute", &LanczosDecomposition::template compute<context::MatrixXs>,
            bp::args("self", "mat"),
            "Computes the Lanczos decomposition for the given input matrix.")

          .def(
            "Ts", (TridiagonalMatrix & (LanczosDecomposition::*)()) & LanczosDecomposition::Ts,
            bp::arg("self"),
            "Returns the tridiagonal matrix associated with the Lanczos decomposition.",
            bp::return_internal_reference<>())
          .def(
            "Qs", (PlainMatrix & (LanczosDecomposition::*)()) & LanczosDecomposition::Qs,
            bp::arg("self"),
            "Returns the orthogonal basis associated with the Lanczos decomposition.",
            bp::return_internal_reference<>())

          .def(
            "rank", &LanczosDecomposition::rank, bp::arg("self"),
            "Returns the rank of the decomposition.")

          .def(
            "computeDecompositionResidual",
            &LanczosDecomposition::template computeDecompositionResidual<context::MatrixXs>,
            bp::args("self", "mat"),
            "Computes the residual associated with the decomposition, namely, the quantity \f$ "
            "A Q_s - Q_s T_s \f$")

#ifndef PINOCCHIO_PYTHON_SKIP_COMPARISON_OPERATIONS
          .def(bp::self == bp::self)
          .def(bp::self != bp::self)
#endif

          ;
      }

      static void expose()
      {
#if PY_MAJOR_VERSION == 3 && PY_MINOR_VERSION == 6 && EIGENPY_VERSION_AT_LEAST(2, 9, 0)
        typedef PINOCCHIO_SHARED_PTR_HOLDER_TYPE(LanczosDecomposition) HolderType;
#else
        typedef ::boost::python::detail::not_specified HolderType;
#endif
        bp::class_<LanczosDecomposition, HolderType>(
          "LanczosDecomposition", "Lanczos decomposition.", bp::no_init)
          .def(LanczosDecompositionPythonVisitor());
      }
    };

  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_math_lanczos_decomposition_hpp__
