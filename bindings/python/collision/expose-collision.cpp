//
// Copyright (c) 2024 INRIA
//

#include "pinocchio/fwd.hpp"
#include "pinocchio/bindings/python/utils/std-aligned-vector.hpp"
#include "pinocchio/bindings/python/collision/geometry-functors.hpp"
#include "pinocchio/bindings/python/collision/collision.hpp"

#include "pinocchio/collision/collision.hpp"
#include "pinocchio/collision/distance.hpp"

#include <Eigen/Core>

namespace pinocchio
{
  namespace python
  {

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename ConfigVectorType>
    static std::size_t computeDistances_proxy(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      DataTpl<Scalar, Options, JointCollectionTpl> & data,
      const GeometryModel & geom_model,
      GeometryData & geom_data,
      const Eigen::MatrixBase<ConfigVectorType> & q)
    {
      return computeDistances(model, data, geom_model, geom_data, q);
    }

    void exposeCollision()
    {
      using namespace Eigen;

      bp::register_ptr_to_python<std::shared_ptr<coal::CollisionGeometry const>>();

      bp::class_<ComputeCollision>(
        "ComputeCollision", "Collision function between two geometry objects.\n\n", bp::no_init)
        .def(GeometryFunctorPythonVisitor<ComputeCollision>());
      StdAlignedVectorPythonVisitor<ComputeCollision>::expose("StdVec_ComputeCollision");

      bp::class_<ComputeDistance>(
        "ComputeDistance", "Distance function between two geometry objects.\n\n", bp::no_init)
        .def(GeometryFunctorPythonVisitor<ComputeDistance>());
      StdAlignedVectorPythonVisitor<ComputeDistance>::expose("StdVec_ComputeDistance");

      bp::def(
        "computeCollision",
        static_cast<bool (*)(
          const GeometryModel &, GeometryData &, const PairIndex, fcl::CollisionRequest &)>(
          computeCollision),
        bp::args("geometry_model", "geometry_data", "pair_index", "collision_request"),
        "Check if the collision objects of a collision pair for a given Geometry Model and "
        "Data are in collision.\n"
        "The collision pair is given by the two index of the collision objects.");

      bp::def(
        "computeCollision",
        static_cast<bool (*)(const GeometryModel &, GeometryData &, const PairIndex)>(
          computeCollision),
        bp::args("geometry_model", "geometry_data", "pair_index"),
        "Check if the collision objects of a collision pair for a given Geometry Model and "
        "Data are in collision.\n"
        "The collision pair is given by the two index of the collision objects.");

      bp::def(
        "computeCollisions",
        (bool (*)(const GeometryModel &, GeometryData &, const bool))&computeCollisions,
        (bp::arg("geometry_model"), bp::arg("geometry_data"),
         bp::arg("stop_at_first_collision") = false),
        "Determine if all collision pairs are effectively in collision or not.");

      bp::def(
        "computeCollisions", &computeCollisions<double, 0, JointCollectionDefaultTpl, VectorXd>,
        (bp::arg("model"), bp::arg("data"), bp::arg("geometry_model"), bp::arg("geometry_data"),
         bp::arg("q"), bp::arg("stop_at_first_collision") = false),
        "Update the geometry for a given configuration and "
        "determine if all collision pairs are effectively in collision or not.");

      bp::def(
        "computeDistance", &computeDistance,
        bp::args("geometry_model", "geometry_data", "pair_index"),
        "Compute the distance between the two geometry objects of a given collision pair for "
        "a GeometryModel and associated GeometryData.",
        bp::with_custodian_and_ward_postcall<
          0, 2, bp::return_value_policy<bp::reference_existing_object>>());

      bp::def(
        "computeDistances",
        (std::size_t (*)(const GeometryModel &, GeometryData &))&computeDistances,
        bp::args("geometry_model", "geometry_data"),
        "Compute the distance between each collision pair for a given GeometryModel and "
        "associated GeometryData.");

      bp::def(
        "computeDistances", &computeDistances_proxy<double, 0, JointCollectionDefaultTpl, VectorXd>,
        bp::args("model", "data", "geometry_model", "geometry_data", "q"),
        "Update the geometry for a given configuration and "
        "compute the distance between each collision pair");

      bp::def(
        "computeBodyRadius", &computeBodyRadius<double, 0, JointCollectionDefaultTpl>,
        bp::args("model", "geometry_model", "geometry_data"),
        "Compute the radius of the geometry volumes attached to every joints.");

      exposeBroadphase();
    }

  } // namespace python
} // namespace pinocchio
