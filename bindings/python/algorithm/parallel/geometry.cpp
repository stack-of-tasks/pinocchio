//
// Copyright (c) 2021 INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/parallel/geometry.hpp"

#include <eigenpy/eigen-from-python.hpp>

namespace pinocchio
{
  namespace python
  {
  
    using namespace Eigen;
    typedef Eigen::Matrix<bool,Eigen::Dynamic,1> VectorXb;
  
    static bool computeCollisions_proxy(const int num_threads,
                                        const GeometryModel & geom_model,
                                        GeometryData & geom_data,
                                        const bool stopAtFirstCollision = false)
    {
      return computeCollisions(num_threads, geom_model, geom_data, stopAtFirstCollision);
    }
  
    static bool computeCollisions_full_proxy(const int num_threads,
                                             const Model & model,
                                             Data & data,
                                             const GeometryModel & geom_model,
                                             GeometryData & geom_data,
                                             const Eigen::VectorXd & q,
                                             const bool stopAtFirstCollision = false)
    {
      return computeCollisions(num_threads, model, data, geom_model, geom_data, q, stopAtFirstCollision);
    }
  
    static void computeCollisions_pool_proxy_res(const int num_thread, GeometryPool & pool,
                                                 const Eigen::MatrixXd & q, Eigen::Ref<VectorXb> res,
                                                 bool stop_at_first_collision = false)
    {
      computeCollisions(num_thread,pool,q,res,stop_at_first_collision);
    }
  
    static VectorXb computeCollisions_pool_proxy(const int num_thread, GeometryPool & pool,
                                                 const Eigen::MatrixXd & q,
                                                 bool stop_at_first_collision = false)
    {
      VectorXb res(q.cols());
      computeCollisions(num_thread,pool,q,res,stop_at_first_collision);
      return res;
    }
  
    BOOST_PYTHON_FUNCTION_OVERLOADS(computeCollisions_pool_proxy_res_overload,computeCollisions_pool_proxy_res,4,5)
    BOOST_PYTHON_FUNCTION_OVERLOADS(computeCollisions_pool_proxy_overload,computeCollisions_pool_proxy,3,4)
    BOOST_PYTHON_FUNCTION_OVERLOADS(computeCollisions_overload,computeCollisions_proxy,3,4)
    BOOST_PYTHON_FUNCTION_OVERLOADS(computeCollisions_full_overload,computeCollisions_full_proxy,6,7)
  
    void exposeParallelGeometry()
    {
      namespace bp = boost::python;
      
      using namespace Eigen;
      
      bp::def("computeCollisions",
              computeCollisions_proxy,
              computeCollisions_overload(bp::args("num_thread","geometry_model","geometry_data","stop_at_first_collision"),
                                         "Evaluates in parallel the collisions for a single data and returns the result.\n\n"
                                         "Parameters:\n"
                                         "\tnum_thread: number of threads used for the computation\n"
                                         "\tgeometry_model: the geometry model\n"
                                         "\tgeometry_data: the geometry data\n"
                                         "\tstop_at_first_collision: if set to true, stops when encountering the first collision.\n"));
      
      bp::def("computeCollisions",
              computeCollisions_full_proxy,
              computeCollisions_full_overload(bp::args("num_thread","model","data","geometry_model","geometry_data","q","stop_at_first_collision"),
                                              "Evaluates in parallel the collisions for a single data and returns the result.\n\n"
                                              "Parameters:\n"
                                              "\tnum_thread: number of threads used for the computation\n"
                                              "\tmodel: the kinematic model\n"
                                              "\tdata: the data associated to the model\n"
                                              "\tgeometry_model: the geometry model\n"
                                              "\tgeometry_data: the geometry data associated to the tgeometry_model\n"
                                              "\tq: the joint configuration vector (size model.nq)\n"
                                              "\tstop_at_first_collision: if set to true, stops when encountering the first collision.\n"));
      
      bp::def("computeCollisions",
              computeCollisions_pool_proxy,
              computeCollisions_pool_proxy_overload(bp::args("num_thread","pool","q","stop_at_first_collision"),
                                                    "Evaluates in parallel the collisions and returns the result.\n\n"
                                                    "Parameters:\n"
                                                    "\tnum_thread: number of threads used for the computation\n"
                                                    "\tpool: pool of geometry model/ geometry data\n"
                                                    "\tq: the joint configuration vector (size model.nq x batch_size)\n"
                                                    "\tstop_at_first_collision: if set to true, stop when encountering the first collision in a batch element.\n"));
      
      bp::def("computeCollisions",
              computeCollisions_pool_proxy_res,
              computeCollisions_pool_proxy_res_overload(bp::args("num_thread","pool","q","res","stop_at_first_collision"),
                                                        "Evaluates in parallel the collisions and stores the result in res.\n\n"
                                                        "Parameters:\n"
                                                        "\tnum_thread: number of threads used for the computation\n"
                                                        "\tpool: pool of geometry model/ geometry data\n"
                                                        "\tq: the joint configuration vector (size model.nq x batch_size)\n"
                                                        "\tres: the resulting collision vector (batch_size)\n"
                                                        "\tstop_at_first_collision: if set to true, stop when encountering the first collision in a batch element.\n"));
      
    }
    
  } // namespace python
} // namespace pinocchio
