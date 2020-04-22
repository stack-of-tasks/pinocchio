//
// Copyright (c) 2020 CNRS INRIA
//

#include "pinocchio/codegen/cppadcg.hpp"
#include "pinocchio/codegen/code-generator-algo.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

using namespace pinocchio;

template<typename D>
void addJointAndBody(Model & model, const JointModelBase<D> & jmodel, const Model::JointIndex parent_id, const SE3 & joint_placement, const std::string & name, const Inertia & Y)
{

  Model::JointIndex idx;
  typedef typename D::TangentVector_t TV;
  typedef typename D::ConfigVector_t CV;
  
  idx = model.addJoint(parent_id,jmodel,joint_placement,
                       name + "_joint",
                       TV::Zero(),
                       1e3 * (TV::Random() + TV::Constant(1)),
                       1e3 * (CV::Random() - CV::Constant(1)),
                       1e3 * (CV::Random() + CV::Constant(1))
                       );
  
  model.appendBodyToJoint(idx,Y,SE3::Identity());
}

void buildModel(Model & model)
{

  addJointAndBody(model,JointModelFreeFlyer(),model.getJointId("universe"),SE3::Identity(),"freeflyer",Inertia::Random());
  addJointAndBody(model,JointModelSpherical(),model.getJointId("freeflyer_joint"),SE3::Identity(),"spherical",Inertia::Random());
  addJointAndBody(model,JointModelPlanar(),model.getJointId("spherical_joint"),SE3::Identity(),"planar",Inertia::Random());
  addJointAndBody(model,JointModelRX(),model.getJointId("planar_joint"),SE3::Identity(),"rx",Inertia::Random());
  addJointAndBody(model,JointModelPX(),model.getJointId("rx_joint"),SE3::Identity(),"px",Inertia::Random());
  addJointAndBody(model,JointModelPrismaticUnaligned(SE3::Vector3(1,0,0)),model.getJointId("px_joint"),SE3::Identity(),"pu",Inertia::Random());
  addJointAndBody(model,JointModelRevoluteUnaligned(SE3::Vector3(0,0,1)),model.getJointId("pu_joint"),SE3::Identity(),"ru",Inertia::Random());
  addJointAndBody(model,JointModelSphericalZYX(),model.getJointId("ru_joint"),SE3::Identity(),"sphericalZYX",Inertia::Random());
  addJointAndBody(model,JointModelTranslation(),model.getJointId("sphericalZYX_joint"),SE3::Identity(),"translation",Inertia::Random());
}

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_joint_configuration_code_generation)
{
  typedef double Scalar;
  typedef CppAD::cg::CG<Scalar> CGScalar;
  typedef CppAD::AD<CGScalar> ADScalar;
  
  typedef pinocchio::ModelTpl<Scalar> Model;   
  typedef pinocchio::ModelTpl<ADScalar> ADModel;
  
  Model model; buildModel(model);
  Eigen::VectorXd qs = Eigen::VectorXd::Random(model.nq);
  Eigen::VectorXd qs2 = Eigen::VectorXd::Random(model.nq);
  normalize(model,qs);
  normalize(model,qs2);
  
  Eigen::VectorXd vs = Eigen::VectorXd::Random(model.nv);
  std::vector<Eigen::VectorXd> results_q(2,Eigen::VectorXd::Zero(model.nq));
  std::vector<Eigen::VectorXd> results_v(2,Eigen::VectorXd::Zero(model.nv));

  //Integrate
  CodeGenIntegrate<double> cg_integrate(model);
  cg_integrate.initLib();
  cg_integrate.loadLib();
  
  cg_integrate.evalFunction(qs,vs, results_q[0]);
  pinocchio::integrate(model, qs,vs,results_q[1]);
  BOOST_CHECK(results_q[1].isApprox(results_q[0]));
  
  //Difference
  CodeGenDifference<double> cg_difference(model);
  cg_difference.initLib();
  cg_difference.loadLib();
  
  cg_difference.evalFunction(qs,qs2, results_v[0]);
  pinocchio::difference(model,qs,qs2,results_v[1]);
  BOOST_CHECK(results_v[1].isApprox(results_v[0]));
}



BOOST_AUTO_TEST_SUITE_END()
