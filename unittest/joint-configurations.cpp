//
// Copyright (c) 2016 CNRS
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/math/quaternion.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

using namespace se3;

bool configurations_are_equals(const Eigen::VectorXd & conf1, const Eigen::VectorXd & conf2)
{
  long size = conf1.size();
  if ( ! conf1.segment<3>(0).isApprox(conf2.segment<3>(0)) )
    return false;
  if( ! defineSameRotation(Eigen::Quaterniond(conf1.segment<4>(3)), Eigen::Quaterniond(conf2.segment<4>(3))))
    return false;
  if( ! defineSameRotation(Eigen::Quaterniond(conf1.segment<4>(7)), Eigen::Quaterniond(conf2.segment<4>(7))))
    return false;
  if ( ! conf1.segment(11, size-11).isApprox(conf2.segment(11, size-11)) )
    return false;
  return true;
}

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
  addJointAndBody(model,JointModelRX(),model.getJointId("spherical_joint"),SE3::Identity(),"rx",Inertia::Random());
  addJointAndBody(model,JointModelPX(),model.getJointId("rx_joint"),SE3::Identity(),"px",Inertia::Random());
  addJointAndBody(model,JointModelPrismaticUnaligned(SE3::Vector3(1,0,0)),model.getJointId("px_joint"),SE3::Identity(),"pu",Inertia::Random());
  addJointAndBody(model,JointModelRevoluteUnaligned(SE3::Vector3(0,0,1)),model.getJointId("pu_joint"),SE3::Identity(),"ru",Inertia::Random());
  addJointAndBody(model,JointModelSphericalZYX(),model.getJointId("ru_joint"),SE3::Identity(),"sphericalZYX",Inertia::Random());
  addJointAndBody(model,JointModelTranslation(),model.getJointId("sphericalZYX_joint"),SE3::Identity(),"translation",Inertia::Random());
  addJointAndBody(model,JointModelPlanar(),model.getJointId("translation_joint"),SE3::Identity(),"planar",Inertia::Random());
}

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

struct TestIntegrationJoint
{
  
  template<typename JointModel>
  static void init (JointModelBase<JointModel> & /*jmodel*/) {}
  
  template<typename JointModel>
  void operator()(JointModelBase<JointModel> & jmodel)
  {
    init(jmodel);
    typedef typename JointModel::ConfigVector_t CV;
    typedef typename JointModel::TangentVector_t TV;
    typedef typename JointModel::Transformation_t SE3;
    
    jmodel.setIndexes(0,0,0);
    typename JointModel::JointDataDerived jdata = jmodel.createData();
    
    CV q0 = jmodel.random();
    TV qdot(TV::Random());
    
    jmodel.calc(jdata,q0,qdot);
    SE3 M0 = jdata.M;
    Motion v0 = jdata.v;
    
    CV q1 = jmodel.integrate(q0,qdot);
    jmodel.calc(jdata,q1);
    SE3 M1 = jdata.M;
    
    SE3 M1_exp = M0*exp6(v0);
    BOOST_CHECK_MESSAGE(M1.isApprox(M1_exp), std::string("Error when integrating1 " + jmodel.shortname()));
    
    qdot *= -1;

    jmodel.calc(jdata,q0,qdot);
    M0 = jdata.M;
    v0 = jdata.v;
    
    q1 = jmodel.integrate(q0,qdot);
    jmodel.calc(jdata,q1);
    M1 = jdata.M;
    
    M1_exp = M0*exp6(v0);
    BOOST_CHECK_MESSAGE(M1.isApprox(M1_exp), std::string("Error when integrating2 " + jmodel.shortname()));
  }
  
};

template<>
void TestIntegrationJoint::operator()< JointModelSphericalZYX >(JointModelBase< JointModelSphericalZYX > & /*jmodel*/) {}

template<>
void TestIntegrationJoint::operator()< JointModelComposite >(JointModelBase< JointModelComposite > & /*jmodel*/)
{
  se3::JointModelComposite jmodel((se3::JointModelRX()), (se3::JointModelRY()));
  jmodel.setIndexes(1,0,0);
  jmodel.updateComponentsIndexes();

  se3::JointModelComposite::JointDataDerived jdata = jmodel.createData();

  typedef typename JointModel::ConfigVector_t CV;
  typedef typename JointModel::TangentVector_t TV;
  typedef typename JointModel::Transformation_t SE3;
  
  CV q0 = jmodel.random();
  TV qdot(Eigen::VectorXd::Random(jmodel.nv()));
  
  jmodel.calc(jdata,q0,qdot);
  SE3 M0 = jdata.M;
  Motion v0 = jdata.v;
  
  CV q1 = jmodel.integrate(q0,qdot);
  jmodel.calc(jdata,q1);
  SE3 M1 = jdata.M;
  
  SE3 M1_exp = M0*exp6(v0);
  // The computations in JointModelComposite::calc() may be wrong, this results cannot be tested yet.
  // BOOST_CHECK_MESSAGE(M1.isApprox(M1_exp), std::string("Error when integrating " + jmodel.shortname()));
}

template<>
void TestIntegrationJoint::init<JointModelRevoluteUnaligned>(JointModelBase<JointModelRevoluteUnaligned> & jmodel)
{
  jmodel.derived().axis.setRandom(); jmodel.derived().axis.normalize();
}

template<>
void TestIntegrationJoint::init<JointModelPrismaticUnaligned>(JointModelBase<JointModelPrismaticUnaligned> & jmodel)
{
  jmodel.derived().axis.setRandom(); jmodel.derived().axis.normalize();
}

BOOST_AUTO_TEST_CASE (intergration_test_joint)
{
  boost::mpl::for_each<JointModelVariant::types>(TestIntegrationJoint());
}

BOOST_AUTO_TEST_CASE ( integration_test )
{
  Model model; buildModel(model);
  Data data(model);

  std::vector<Eigen::VectorXd> qs(2);
  std::vector<Eigen::VectorXd> qdots(2);
  std::vector<Eigen::VectorXd> results(2);

  //
  // Test Case 0 : Integration of a config with zero velocity
  //
  qs[0] = Eigen::VectorXd::Ones(model.nq);
  qs[0].segment<4>(3) /= qs[0].segment<4>(3).norm(); // quaternion of freeflyer
  qs[0].segment<4>(7) /= qs[0].segment<4>(7).norm(); // quaternion of spherical joint
 
  qdots[0] = Eigen::VectorXd::Zero(model.nv);
  results[0] = integrate(model,qs[0],qdots[0]);

  BOOST_CHECK_MESSAGE(results[0].isApprox(qs[0], 1e-12), "integration of full body with zero velocity - wrong results");
}


struct TestDifferentiationJoint
{
  
  template<typename JointModel>
  static void init (JointModelBase<JointModel> & /*jmodel*/) {}
  
  template<typename JointModel>
  void operator()(JointModelBase<JointModel> & jmodel)
  {
    init(jmodel);
    typedef typename JointModel::ConfigVector_t CV;
    typedef typename JointModel::TangentVector_t TV;
    
    jmodel.setIndexes(0,0,0);
    
    CV q0 = jmodel.random();
    CV q1 = jmodel.random();

    TV qdot = jmodel.difference(q0,q1);

    BOOST_CHECK_MESSAGE( jmodel.isSameConfiguration(jmodel.integrate(q0, qdot), q1), std::string("Error in difference for joint " + jmodel.shortname()));

    BOOST_CHECK_MESSAGE( jmodel.isSameConfiguration(jmodel.integrate(q1, -qdot), q0), std::string("Error in difference for joint " + jmodel.shortname()));

  }
  
};

template<>
void TestDifferentiationJoint::operator()< JointModelSphericalZYX >(JointModelBase< JointModelSphericalZYX > & /*jmodel*/) {}

template<>
void TestDifferentiationJoint::init<JointModelRevoluteUnaligned>(JointModelBase<JointModelRevoluteUnaligned> & jmodel)
{
  jmodel.derived().axis.setRandom(); jmodel.derived().axis.normalize();
}

template<>
void TestDifferentiationJoint::init<JointModelPrismaticUnaligned>(JointModelBase<JointModelPrismaticUnaligned> & jmodel)
{
  jmodel.derived().axis.setRandom(); jmodel.derived().axis.normalize();
}

BOOST_AUTO_TEST_CASE (differentiation_test_joint)
{
  boost::mpl::for_each<JointModelVariant::types>(TestDifferentiationJoint());
}

BOOST_AUTO_TEST_CASE ( integrate_difference_test )
{
 Model model; buildModel(model);

 Eigen::VectorXd q0(randomConfiguration(model, -1 * Eigen::VectorXd::Ones(model.nq), Eigen::VectorXd::Ones(model.nq) ));
 Eigen::VectorXd q1(randomConfiguration(model, -1 * Eigen::VectorXd::Ones(model.nq), Eigen::VectorXd::Ones(model.nq) ));
 Eigen::VectorXd qdot(Eigen::VectorXd::Random(model.nv));

 BOOST_CHECK_MESSAGE(isSameConfiguration(model, integrate(model, q0, differentiate(model, q0,q1)), q1), "Integrate (differentiate) - wrong results");

 BOOST_CHECK_MESSAGE(differentiate(model, q0, integrate(model,q0, qdot)).isApprox(qdot),"differentiate (integrate) - wrong results");
}


struct TestInterpolationJoint
{
  
  template<typename JointModel>
  static void init (JointModelBase<JointModel> & /*jmodel*/) {}
  
  template<typename JointModel>
  void operator()(JointModelBase<JointModel> & jmodel)
  {
    init(jmodel);
    typedef typename JointModel::ConfigVector_t CV;
    
    jmodel.setIndexes(0,0,0);
    
    CV q0 = jmodel.random();
    CV q1 = jmodel.random();

    double u = 0;
    
    BOOST_CHECK_MESSAGE(jmodel.isSameConfiguration(jmodel.interpolate(q0, q1,u),q0)
                      , std::string("Error in interpolation with u = 0 for joint " + jmodel.shortname()));

    u = 0.3; 
    
    BOOST_CHECK_MESSAGE(jmodel.isSameConfiguration(jmodel.interpolate(jmodel.interpolate(q0, q1,u), q1, 1),q1)
                      , std::string("Error in double interpolation for joint " + jmodel.shortname()));

    u = 1;
    
    BOOST_CHECK_MESSAGE(jmodel.isSameConfiguration(jmodel.interpolate(q0, q1,u),q1)
                      , std::string("Error in interpolation with u = 1 for joint " + jmodel.shortname()));

  }

  
};

template<>
void TestInterpolationJoint::operator()< JointModelSphericalZYX >(JointModelBase< JointModelSphericalZYX > & /*jmodel*/) {}

template<>
void TestInterpolationJoint::init<JointModelRevoluteUnaligned>(JointModelBase<JointModelRevoluteUnaligned> & jmodel)
{
  jmodel.derived().axis.setRandom(); jmodel.derived().axis.normalize();
}

template<>
void TestInterpolationJoint::init<JointModelPrismaticUnaligned>(JointModelBase<JointModelPrismaticUnaligned> & jmodel)
{
  jmodel.derived().axis.setRandom(); jmodel.derived().axis.normalize();
}

BOOST_AUTO_TEST_CASE (interpolation_test_joint)
{
  boost::mpl::for_each<JointModelVariant::types>(TestInterpolationJoint());
}


BOOST_AUTO_TEST_CASE ( neutral_configuration_test )
{
  Model model; buildModel(model);

  Eigen::VectorXd expected(model.nq);
  expected << 0,0,0,0,0,0,1,
              0,0,0,1,
              0,
              0,
              0,
              0,
              0,0,0,
              0,0,0,
              0,0,0;


  BOOST_CHECK_MESSAGE(model.neutralConfiguration.isApprox(expected, 1e-12), "neutral configuration - wrong results");
}

BOOST_AUTO_TEST_CASE ( uniform_sampling_test )
{
  Model model; buildModel(model);

  Eigen::VectorXd q1(randomConfiguration(model));
  
  for (int i = 0; i < model.nq; ++i)
  {
    BOOST_CHECK_MESSAGE(q1[i] >= model.lowerPositionLimit[i] && q1[i] <= model.upperPositionLimit[i], " UniformlySample : Generated config not in bounds");
  }
}


BOOST_AUTO_TEST_CASE ( normalize_test )
{
  Model model; buildModel(model);

  Eigen::VectorXd q (Eigen::VectorXd::Ones(model.nq));
  se3::normalize(model, q);

  BOOST_CHECK(q.head<3>().isApprox(Eigen::VectorXd::Ones(3)));
  BOOST_CHECK(fabs(q.segment<4>(3).norm() - 1) < Eigen::NumTraits<double>::epsilon()); // quaternion of freeflyer
  BOOST_CHECK(fabs(q.segment<4>(7).norm() - 1) < Eigen::NumTraits<double>::epsilon()); // quaternion of spherical joint
  const int n = model.nq - 7 - 4;
  BOOST_CHECK(q.tail(n).isApprox(Eigen::VectorXd::Ones(n)));
}

BOOST_AUTO_TEST_SUITE_END ()
