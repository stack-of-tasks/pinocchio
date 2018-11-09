//
// Copyright (c) 2015-2016 CNRS
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

namespace pinocchio
{
  namespace python
  {

    static Eigen::VectorXd normalize_proxy(const Model & model,
                                           const Eigen::VectorXd & config)
    {
      Eigen::VectorXd q(config);
      normalize(model,q);
      return q;
    }
    
    static Eigen::VectorXd randomConfiguration_proxy(const Model & model)
    {
      return randomConfiguration(model);
    }

    void exposeJointsAlgo()
    {
      using namespace Eigen;
      
      bp::def("integrate",
              &integrate<double,0,JointCollectionDefaultTpl,VectorXd,VectorXd>,
              bp::args("Model",
                       "Configuration q (size Model::nq)",
                       "Velocity v (size Model::nv)"),
              "Integrate the model for a tangent vector during one unit time .");
      
      bp::def("interpolate",
              &interpolate<double,0,JointCollectionDefaultTpl,VectorXd,VectorXd>,
              bp::args("Model",
                       "Configuration q1 (size Model::nq)",
                       "Configuration q2 (size Model::nq)",
                       "Double u"),
              "Interpolate the model between two configurations.");
      
      bp::def("difference",
              &difference<double,0,JointCollectionDefaultTpl,VectorXd,VectorXd>,
              bp::args("Model",
                       "Configuration q1 (size Model::nq)",
                       "Configuration q2 (size Model::nq)"),
              "Difference between two configurations, ie. the tangent vector that must be integrated during one unit time"
              "to go from q1 to q2");
      
      bp::def("squaredDistance",
              &squaredDistance<double,0,JointCollectionDefaultTpl,VectorXd,VectorXd>,
              bp::args("Model",
                       "Configuration q1 (size Model::nq)",
                       "Configuration q2 (size Model::nq)"),
              "Squared distance vector between two configurations.");
      
      bp::def("distance",
              &distance<double,0,JointCollectionDefaultTpl,VectorXd,VectorXd>,
              bp::args("Model",
                       "Configuration q1 (size Model::nq)",
                       "Configuration q2 (size Model::nq)"),
              "Distance between two configurations.");
      
      bp::def("randomConfiguration",
              &randomConfiguration_proxy,
              bp::arg("Model"),
              "Generate a random configuration in the bounds given by the lower and upper limits contained in model.");
      
      bp::def("randomConfiguration",
              &randomConfiguration<double,0,JointCollectionDefaultTpl,VectorXd,VectorXd>,
              bp::args("Model",
                       "Joint lower limits (size Model::nq)",
                       "Joint upper limits (size Model::nq)"),
              "Generate a random configuration in the bounds given by the Joint lower and upper limits arguments.");
      
      bp::def("normalize",normalize_proxy,
              bp::args("Model",
                       "Configuration q (size Model::nq)"),
              "return the configuration normalized ");
      
      bp::def("isSameConfiguration",
              &isSameConfiguration<double,0,JointCollectionDefaultTpl,VectorXd,VectorXd>,
              bp::args("Model",
                       "Configuration q1 (size Model::nq)",
                       "Configuration q2 (size Model::nq)",
                       "Precision"),
              "Return true if two configurations are equivalent");
    }
    
  } // namespace python
} // namespace pinocchio
