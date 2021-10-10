//
// Copyright (c) 2015-2021 CNRS INRIA
//

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/multibody/geometry-object.hpp"
#include "pinocchio/bindings/python/multibody/geometry-model.hpp"
#include "pinocchio/bindings/python/multibody/geometry-data.hpp"
#include "pinocchio/bindings/python/utils/std-aligned-vector.hpp"

#ifdef PINOCCHIO_WITH_HPP_FCL
  #include "pinocchio/bindings/python/multibody/geometry-functors.hpp"
#endif

namespace pinocchio
{
  namespace python
  {
    
    void exposeGeometry()
    {
      GeometryObjectPythonVisitor::expose();
      StdAlignedVectorPythonVisitor<GeometryObject>::expose("StdVec_GeometryObject");
      
      CollisionPairPythonVisitor::expose();
      GeometryModelPythonVisitor::expose();
      GeometryDataPythonVisitor::expose();
      
#ifdef PINOCCHIO_WITH_HPP_FCL
      bp::register_ptr_to_python< boost::shared_ptr<hpp::fcl::CollisionGeometry const> >();
      
      bp::class_<ComputeCollision>("ComputeCollision",
                                   "Collision function between two geometry objects.\n\n",
                                   bp::no_init
                                   )
      .def(GeometryFunctorPythonVisitor<ComputeCollision>());
      StdAlignedVectorPythonVisitor<ComputeCollision>::expose("StdVec_ComputeCollision");
      
      bp::class_<ComputeDistance>("ComputeDistance",
                                  "Distance function between two geometry objects.\n\n",
                                  bp::no_init
                                  )
      .def(GeometryFunctorPythonVisitor<ComputeDistance>());
      StdAlignedVectorPythonVisitor<ComputeDistance>::expose("StdVec_ComputeDistance");
#endif
    }
    
  } // namespace python
} // namespace pinocchio
