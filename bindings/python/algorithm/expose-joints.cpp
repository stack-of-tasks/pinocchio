//
// Copyright (c) 2015-2020 CNRS INRIA
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
                               const Eigen::VectorXd & q,
                               const Eigen::VectorXd & v)
    {
      Eigen::MatrixXd J0(Eigen::MatrixXd::Zero(model.nv,model.nv));
      Eigen::MatrixXd J1(Eigen::MatrixXd::Zero(model.nv,model.nv));

      dIntegrate(model,q,v,J0,ARG0);
      dIntegrate(model,q,v,J1,ARG1);

      return bp::make_tuple(J0,J1);
    }

    Eigen::MatrixXd dIntegrate_arg_proxy(const Model & model,
                                         const Eigen::VectorXd & q,
                                         const Eigen::VectorXd & v,
                                         const ArgumentPosition arg)
    {
      Eigen::MatrixXd J(Eigen::MatrixXd::Zero(model.nv,model.nv));
      
      dIntegrate(model,q,v,J,arg, SETTO);
      
      return J;
    }

    bp::tuple dDifference_proxy(const Model & model,
                                const Eigen::VectorXd & q1,
                                const Eigen::VectorXd & q2)
    {
      Eigen::MatrixXd J0(Eigen::MatrixXd::Zero(model.nv,model.nv));
      Eigen::MatrixXd J1(Eigen::MatrixXd::Zero(model.nv,model.nv));

      dDifference(model,q1,q2,J0,ARG0);
      dDifference(model,q1,q2,J1,ARG1);

      return bp::make_tuple(J0,J1);
    }

    Eigen::MatrixXd dDifference_arg_proxy(const Model & model,
                                          const Eigen::VectorXd & q1,
                                          const Eigen::VectorXd & q2,
                                          const ArgumentPosition arg)
    {
      Eigen::MatrixXd J(Eigen::MatrixXd::Zero(model.nv,model.nv));
      
      dDifference(model,q1,q2,J,arg);
      
      return J;
    }
  
    void exposeJointsAlgo()
    {
      using namespace Eigen;
      
      bp::def("integrate",
              &integrate<double,0,JointCollectionDefaultTpl,VectorXd,VectorXd>,
              bp::args("model","q","v"),
              "Integrate the joint configuration vector q with a tangent vector v during one unit time.\n"
              "This is the canonical integrator of a Configuration Space composed of Lie groups, such as most robots.\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n"
              "\tq: the joint configuration vector (size model.nq)\n"
              "\tv: the joint velocity vector (size model.nv)\n");
      
      bp::def("dIntegrate",
              &dIntegrate_proxy,
              bp::args("model","q","v"),
              "Computes the partial derivatives of the integrate function with respect to the first "
              "and the second argument, and returns the two Jacobians as a tuple.\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n"
              "\tq: the joint configuration vector (size model.nq)\n"
              "\tv: the joint velocity vector (size model.nv)\n");

      bp::def("dIntegrate",
              &dIntegrate_arg_proxy,
              bp::args("model","q","v","argument_position"),
              "Computes the partial derivatives of the integrate function with respect to the first (arg == ARG0) "
              "or the second argument (arg == ARG1).\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n"
              "\tq: the joint configuration vector (size model.nq)\n"
              "\tv: the joint velocity vector (size model.nv)\n"
              "\targument_position: either pinocchio.ArgumentPosition.ARG0 or pinocchio.ArgumentPosition.ARG1, depending on the desired Jacobian value.\n");

      bp::def("interpolate",
              &interpolate<double,0,JointCollectionDefaultTpl,VectorXd,VectorXd>,
              bp::args("model","q1","q2","alpha"),
              "Interpolate between two given joint configuration vectors q1 and q2.\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n"
              "\tq1: the initial joint configuration vector (size model.nq)\n"
              "\tq2: the terminal joint configuration vector (size model.nq)\n"
              "\talpha: the interpolation coefficient in [0,1]\n");
      
      bp::def("difference",
              &difference<double,0,JointCollectionDefaultTpl,VectorXd,VectorXd>,
              bp::args("model","q1","q2"),
              "Difference between two joint configuration vectors, i.e. the tangent vector that must be integrated during one unit time"
              "to go from q1 to q2.\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n"
              "\tq1: the initial joint configuration vector (size model.nq)\n"
              "\tq2: the terminal joint configuration vector (size model.nq)\n");
      
      bp::def("squaredDistance",
              &squaredDistance<double,0,JointCollectionDefaultTpl,VectorXd,VectorXd>,
              bp::args("model","q1","q2"),
              "Squared distance vector between two joint configuration vectors.\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n"
              "\tq1: the initial joint configuration vector (size model.nq)\n"
              "\tq2: the terminal joint configuration vector (size model.nq)\n");
      
      bp::def("distance",
              &distance<double,0,JointCollectionDefaultTpl,VectorXd,VectorXd>,
              bp::args("model","q1","q2"),
              "Distance between two joint configuration vectors.\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n"
              "\tq1: the initial joint configuration vector (size model.nq)\n"
              "\tq2: the terminal joint configuration vector (size model.nq)\n");

      bp::def("dDifference",
              &dDifference_proxy,
              bp::args("model","q1","q2"),
              "Computes the partial derivatives of the difference function with respect to the first "
              "and the second argument, and returns the two Jacobians as a tuple.\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n"
              "\tq1: the initial joint configuration vector (size model.nq)\n"
              "\tq2: the terminal joint configuration vector (size model.nq)\n");
      
      bp::def("dDifference",
              &dDifference_arg_proxy,
              bp::args("model","q1","q2","argument_position"),
              "Computes the partial derivatives of the difference function with respect to the first (arg == ARG0) "
              "or the second argument (arg == ARG1).\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n"
              "\tq1: the initial joint configuration vector (size model.nq)\n"
              "\tq2: the terminal joint configuration vector (size model.nq)\n"
              "\targument_position: either pinocchio.ArgumentPosition.ARG0 or pinocchio.ArgumentPosition.ARG1, depending on the desired Jacobian value.\n");
      
      bp::def("randomConfiguration",
              &randomConfiguration_proxy,
              bp::arg("model"),
              "Generate a random configuration in the bounds given by the lower and upper limits contained in model.\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n");
      
      bp::def("randomConfiguration",
              &randomConfiguration<double,0,JointCollectionDefaultTpl,VectorXd,VectorXd>,
              bp::args("model","lower_bound","upper_bound"),
              "Generate a random configuration in the bounds given by the Joint lower and upper limits arguments.\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n"
              "\tlower_bound: the lower bound on the joint configuration vectors (size model.nq)\n"
              "\tupper_bound: the upper bound on the joint configuration vectors (size model.nq)\n");
      
      bp::def("neutral",
              &neutral<double,0,JointCollectionDefaultTpl>,
              bp::arg("model"),
              "Returns the neutral configuration vector associated to the model.\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n");
      
      bp::def("normalize",normalize_proxy,
              bp::args("model","q"),
              "Returns the configuration normalized.\n"
              "For instance, when the configuration vectors contains some quaternion values, it must be required to renormalize these components to keep orthonormal rotation values.\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n"
              "\tq: a joint configuration vector to normalize (size model.nq)\n");
      
      bp::def("isSameConfiguration",
              &isSameConfiguration<double,0,JointCollectionDefaultTpl,VectorXd,VectorXd>,
              bp::args("model","q1","q2","prec"),
              "Return true if two configurations are equivalent within the given precision provided by prec.\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n"
              "\tq1: a joint configuration vector (size model.nq)\n"
              "\tq2: a joint configuration vector (size model.nq)\n"
              "\tprec: requested accuracy for the comparison\n");
    }
    
  } // namespace python
} // namespace pinocchio
