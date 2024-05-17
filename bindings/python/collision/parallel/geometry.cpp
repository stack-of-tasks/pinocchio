//
// Copyright (c) 2021-2022 INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/collision/parallel/geometry.hpp"
#include "pinocchio/collision/parallel/broadphase.hpp"

#include <eigenpy/eigen-from-python.hpp>

namespace pinocchio
{
  namespace python
  {

    using namespace Eigen;

    typedef Eigen::Matrix<bool, Eigen::Dynamic, 1> VectorXb;

    static bool computeCollisionsInParallel_proxy(
      const size_t num_threads,
      const GeometryModel & geom_model,
      GeometryData & geom_data,
      const bool stopAtFirstCollision = false)
    {
      return computeCollisionsInParallel(num_threads, geom_model, geom_data, stopAtFirstCollision);
    }

    static bool computeCollisionsInParallel_full_proxy(
      const size_t num_threads,
      const Model & model,
      Data & data,
      const GeometryModel & geom_model,
      GeometryData & geom_data,
      const Eigen::VectorXd & q,
      const bool stopAtFirstCollision = false)
    {
      return computeCollisionsInParallel(
        num_threads, model, data, geom_model, geom_data, q, stopAtFirstCollision);
    }

    static VectorXb computeCollisionsInParallel_pool_proxy(
      const int num_thread,
      GeometryPool & pool,
      const Eigen::MatrixXd & q,
      bool stopAtFirstCollisionInConfiguration = false,
      bool stopAtFirstCollisionInBatch = false)
    {
      VectorXb res(q.cols());
      computeCollisionsInParallel(
        num_thread, pool, q, res, stopAtFirstCollisionInConfiguration, stopAtFirstCollisionInBatch);
      return res;
    }

    void exposeParallelGeometry()
    {
      namespace bp = boost::python;

      using namespace Eigen;

      bp::def(
        "computeCollisionsInParallel", computeCollisionsInParallel_proxy,
        (bp::arg("num_thread"), bp::arg("geometry_model"), bp::arg("geometry_data"),
         bp::arg("stop_at_first_collision") = false),
        "Evaluates in parallel the collisions for a single data and returns the result.\n\n"
        "Parameters:\n"
        "\tnum_thread: number of threads used for the computation\n"
        "\tgeometry_model: the geometry model\n"
        "\tgeometry_data: the geometry data\n"
        "\tstop_at_first_collision: if set to true, stops when encountering the first "
        "collision.\n");

      bp::def(
        "computeCollisionsInParallel", computeCollisionsInParallel_full_proxy,
        (bp::arg("num_thread"), bp::arg("model"), bp::arg("data"), bp::arg("geometry_model"),
         bp::arg("geometry_data"), bp::arg("q"), bp::arg("stop_at_first_collision") = false),
        "Evaluates in parallel the collisions for a single data and returns the result.\n\n"
        "Parameters:\n"
        "\tnum_thread: number of threads used for the computation\n"
        "\tmodel: the kinematic model\n"
        "\tdata: the data associated to the model\n"
        "\tgeometry_model: the geometry model\n"
        "\tgeometry_data: the geometry data associated to the tgeometry_model\n"
        "\tq: the joint configuration vector (size model.nq)\n"
        "\tstop_at_first_collision: if set to true, stops when encountering the first "
        "collision.\n");

      bp::def(
        "computeCollisionsInParallel", computeCollisionsInParallel_pool_proxy,
        (bp::arg("num_thread"), bp::arg("pool"), bp::arg("q"),
         bp::arg("stop_at_first_collision_in_configuration") = false,
         bp::arg("stop_at_first_collision_in_batch") = false),
        "Evaluates in parallel the collisions and returns the result.\n\n"
        "Parameters:\n"
        "\tnum_thread: number of threads used for the computation\n"
        "\tpool: pool of geometry model/ geometry data\n"
        "\tq: the joint configuration vector (size model.nq x batch_size)\n"
        "\tstop_at_first_collision_in_configuration: if set to true, stops when encountering "
        "the first collision in a configuration\n"
        "\tstop_at_first_collision_in_batch: if set to true, stops when encountering the "
        "first collision in a batch.\n");
    }

  } // namespace python
} // namespace pinocchio
