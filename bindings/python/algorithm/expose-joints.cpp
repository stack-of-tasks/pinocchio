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

    bp::tuple dIntegrate_proxy(const Model & model,
                               const Eigen::VectorXd& q,
                               const Eigen::VectorXd& dq)
    {
      Eigen::MatrixXd J0(Eigen::MatrixXd::Zero(model.nv,model.nv));
      Eigen::MatrixXd J1(Eigen::MatrixXd::Zero(model.nv,model.nv));

      dIntegrate(model,q,dq,J0,ARG0);
      dIntegrate(model,q,dq,J1,ARG1);

      return bp::make_tuple(J0,J1);
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
      
      bp::enum_<ArgumentPosition>("ArgumentPosition")
        .value("ARG0",ARG0)
        .value("ARG1",ARG1)
        .value("ARG2",ARG2)
        .value("ARG3",ARG3)
        .value("ARG4",ARG4)
        ;

      bp::def("dIntegrate",
              &dIntegrate_proxy,
              bp::args("Model",
                       "Configuration q (size Model::nq)",
                       "Velocity v (size Model::nv)"),
              "Compute the partial derivatives of integrate function with respect to first "
              "and second argument, and return the two jacobian as a tuple. ");

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
      
      bp::def("neutral",
              &neutral<double,0,JointCollectionDefaultTpl>,
              bp::arg("Model"),
              "Returns the neutral configuration vector associated to the model.");
      
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
