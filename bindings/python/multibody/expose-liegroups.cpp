//
// Copyright (c) 2015-2020 CNRS INRIA
//

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/multibody/liegroups.hpp"

#include <eigenpy/memory.hpp>

namespace pinocchio
{
namespace python
{

namespace bp = boost::python;

template<typename LgType>
CartesianProductOperationVariantTpl<double,0,LieGroupCollectionDefaultTpl>
makeLieGroup()
{
  return CartesianProductOperationVariantTpl<double,0,LieGroupCollectionDefaultTpl> (LgType());
}

CartesianProductOperationVariantTpl<double,0,LieGroupCollectionDefaultTpl>
makeRn(int n)
{
  return CartesianProductOperationVariantTpl<double,0,LieGroupCollectionDefaultTpl> (
      VectorSpaceOperationTpl<Eigen::Dynamic,double,0>(n));
}


/* --- Expose --------------------------------------------------------- */
void exposeLieGroups()
{
  LieGroupPythonVisitor<
    CartesianProductOperationVariantTpl<double,0,LieGroupCollectionDefaultTpl>
    >::expose("LieGroup");

  bp::scope current;
  std::string submoduleName(bp::extract<const char*>(current.attr("__name__")));
  submoduleName.append(".liegroups");

  // Create the submodule, and attach it to the current scope.
  bp::object submodule(bp::borrowed(PyImport_AddModule(submoduleName.c_str())));
  current.attr("liegroups") = submodule;

  {
    // Switch the scope to the submodule, add methods and classes.
    bp::scope submoduleScope = submodule;

    bp::def("R1", makeLieGroup<VectorSpaceOperationTpl<1,double,0> >);
    bp::def("R2", makeLieGroup<VectorSpaceOperationTpl<2,double,0> >);
    bp::def("R3", makeLieGroup<VectorSpaceOperationTpl<3,double,0> >);
    bp::def("Rn", makeRn);
    bp::def("SO2", makeLieGroup<SpecialOrthogonalOperationTpl<2,double,0> >);
    bp::def("SO3", makeLieGroup<SpecialOrthogonalOperationTpl<3,double,0> >);
    bp::def("SE2", makeLieGroup<SpecialEuclideanOperationTpl<2,double,0> >);
    bp::def("SE3", makeLieGroup<SpecialEuclideanOperationTpl<3,double,0> >);
  }
}
} // namespace python
} // namespace pinocchio
