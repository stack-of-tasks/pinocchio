//
// Copyright (c) 2015-2020 CNRS INRIA
//

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/multibody/liegroups.hpp"
#include "pinocchio/bindings/python/utils/namespace.hpp"

#include <eigenpy/memory.hpp>

namespace pinocchio
{
  namespace python
  {

    namespace bp = boost::python;

    template<typename LgType>
    CartesianProductOperationVariantTpl<
      context::Scalar,
      context::Options,
      LieGroupCollectionDefaultTpl>
    makeLieGroup()
    {
      return CartesianProductOperationVariantTpl<
        context::Scalar, context::Options, LieGroupCollectionDefaultTpl>(LgType());
    }

    CartesianProductOperationVariantTpl<
      context::Scalar,
      context::Options,
      LieGroupCollectionDefaultTpl>
    makeRn(int n)
    {
      return CartesianProductOperationVariantTpl<
        context::Scalar, context::Options, LieGroupCollectionDefaultTpl>(
        VectorSpaceOperationTpl<Eigen::Dynamic, context::Scalar, context::Options>(n));
    }

    /* --- Expose --------------------------------------------------------- */
    void exposeLieGroups()
    {
      LieGroupPythonVisitor<CartesianProductOperationVariantTpl<
        context::Scalar, context::Options, LieGroupCollectionDefaultTpl>>::expose("LieGroup");

      {
        // Switch the scope to the submodule, add methods and classes.
        bp::scope submoduleScope = getOrCreatePythonNamespace("liegroups");

        bp::def("R1", makeLieGroup<VectorSpaceOperationTpl<1, context::Scalar, context::Options>>);
        bp::def("R2", makeLieGroup<VectorSpaceOperationTpl<2, context::Scalar, context::Options>>);
        bp::def("R3", makeLieGroup<VectorSpaceOperationTpl<3, context::Scalar, context::Options>>);
        bp::def("Rn", makeRn);
        bp::def(
          "SO2", makeLieGroup<SpecialOrthogonalOperationTpl<2, context::Scalar, context::Options>>);
        bp::def(
          "SO3", makeLieGroup<SpecialOrthogonalOperationTpl<3, context::Scalar, context::Options>>);
        bp::def(
          "SE2", makeLieGroup<SpecialEuclideanOperationTpl<2, context::Scalar, context::Options>>);
        bp::def(
          "SE3", makeLieGroup<SpecialEuclideanOperationTpl<3, context::Scalar, context::Options>>);
      }
    }
  } // namespace python
} // namespace pinocchio
