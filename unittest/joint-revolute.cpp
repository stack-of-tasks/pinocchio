//
// Copyright (c) 2015-2020 CNRS INRIA
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#include "pinocchio/math/fwd.hpp"
#include "pinocchio/multibody/joint/joints.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"

#include <boost/test/unit_test.hpp>
#include <iostream>

using namespace pinocchio;

template<typename D>
void addJointAndBody(Model & model,
                     const JointModelBase<D> & jmodel,
                     const Model::JointIndex parent_id,
                     const SE3 & joint_placement,
                     const std::string & joint_name,
                     const Inertia & Y)
{
  Model::JointIndex idx;
  
  idx = model.addJoint(parent_id,jmodel,joint_placement,joint_name);
  model.appendBodyToJoint(idx,Y);
}

BOOST_AUTO_TEST_SUITE(JointRevoluteUnaligned)

BOOST_AUTO_TEST_CASE(vsRX)
{
  using namespace pinocchio;
  typedef SE3::Vector3 Vector3;
  typedef SE3::Matrix3 Matrix3;

  Vector3 axis;
  axis << 1.0, 0.0, 0.0;

  Model modelRX, modelRevoluteUnaligned;

  Inertia inertia (1., Vector3 (0.5, 0., 0.0), Matrix3::Identity ());
  SE3 pos(1); pos.translation() = SE3::LinearType(1.,0.,0.);

  JointModelRevoluteUnaligned joint_model_RU(axis);
  
  addJointAndBody(modelRX,JointModelRX(),0,pos,"rx",inertia);
  addJointAndBody(modelRevoluteUnaligned,joint_model_RU,0,pos,"revolute-unaligned",inertia);

  Data dataRX(modelRX);
  Data dataRevoluteUnaligned(modelRevoluteUnaligned);

  Eigen::VectorXd q = Eigen::VectorXd::Ones (modelRX.nq);
  Eigen::VectorXd v = Eigen::VectorXd::Ones (modelRX.nv);
  Eigen::VectorXd tauRX = Eigen::VectorXd::Ones (modelRX.nv);
  Eigen::VectorXd tauRevoluteUnaligned = Eigen::VectorXd::Ones (modelRevoluteUnaligned.nv);
  Eigen::VectorXd aRX = Eigen::VectorXd::Ones (modelRX.nv);
  Eigen::VectorXd aRevoluteUnaligned(aRX);

  forwardKinematics(modelRX, dataRX, q, v);
  forwardKinematics(modelRevoluteUnaligned, dataRevoluteUnaligned, q, v);

  computeAllTerms(modelRX, dataRX, q, v);
  computeAllTerms(modelRevoluteUnaligned, dataRevoluteUnaligned, q, v);

  BOOST_CHECK(dataRevoluteUnaligned.oMi[1].isApprox(dataRX.oMi[1]));
  BOOST_CHECK(dataRevoluteUnaligned.liMi[1].isApprox(dataRX.liMi[1]));
  BOOST_CHECK(dataRevoluteUnaligned.Ycrb[1].matrix().isApprox(dataRX.Ycrb[1].matrix()));
  BOOST_CHECK(dataRevoluteUnaligned.f[1].toVector().isApprox(dataRX.f[1].toVector()));
  
  BOOST_CHECK(dataRevoluteUnaligned.nle.isApprox(dataRX.nle));
  BOOST_CHECK(dataRevoluteUnaligned.com[0].isApprox(dataRX.com[0]));

  // InverseDynamics == rnea
  tauRX = rnea(modelRX, dataRX, q, v, aRX);
  tauRevoluteUnaligned = rnea(modelRevoluteUnaligned, dataRevoluteUnaligned, q, v, aRevoluteUnaligned);

  BOOST_CHECK(tauRX.isApprox(tauRevoluteUnaligned));

  // ForwardDynamics == aba
  Eigen::VectorXd aAbaRX = aba(modelRX,dataRX, q, v, tauRX);
  Eigen::VectorXd aAbaRevoluteUnaligned = aba(modelRevoluteUnaligned,dataRevoluteUnaligned, q, v, tauRevoluteUnaligned);

  BOOST_CHECK(aAbaRX.isApprox(aAbaRevoluteUnaligned));

  // CRBA
  crba(modelRX, dataRX,q);
  crba(modelRevoluteUnaligned, dataRevoluteUnaligned, q);

  BOOST_CHECK(dataRX.M.isApprox(dataRevoluteUnaligned.M));
   
  // Jacobian
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobianRX;jacobianRX.resize(6,1); jacobianRX.setZero();
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobianRevoluteUnaligned;jacobianRevoluteUnaligned.resize(6,1);jacobianRevoluteUnaligned.setZero();
  computeJointJacobians(modelRX, dataRX, q);
  computeJointJacobians(modelRevoluteUnaligned, dataRevoluteUnaligned, q);
  getJointJacobian(modelRX, dataRX, 1, LOCAL, jacobianRX);
  getJointJacobian(modelRevoluteUnaligned, dataRevoluteUnaligned, 1, LOCAL, jacobianRevoluteUnaligned);


  BOOST_CHECK(jacobianRX.isApprox(jacobianRevoluteUnaligned));
}
BOOST_AUTO_TEST_SUITE_END()
  
BOOST_AUTO_TEST_SUITE(JointRevoluteUnboundedUnaligned)
  
  BOOST_AUTO_TEST_CASE(vsRUX)
  {
    using namespace pinocchio;
    typedef SE3::Vector3 Vector3;
    typedef SE3::Matrix3 Matrix3;
    
    Vector3 axis;
    axis << 1.0, 0.0, 0.0;
    
    Model modelRUX, modelRevoluteUboundedUnaligned;
    
    Inertia inertia (1., Vector3 (0.5, 0., 0.0), Matrix3::Identity ());
    SE3 pos(1); pos.translation() = SE3::LinearType(1.,0.,0.);
    
    JointModelRevoluteUnboundedUnaligned joint_model_RUU(axis);
    typedef traits< JointRevoluteUnboundedUnalignedTpl<double> >::TangentVector_t TangentVector;
    
    addJointAndBody(modelRUX,JointModelRUBX(),0,pos,"rux",inertia);
    addJointAndBody(modelRevoluteUboundedUnaligned,joint_model_RUU,0,pos,"revolute-unbounded-unaligned",inertia);
    
    Data dataRUX(modelRUX);
    Data dataRevoluteUnboundedUnaligned(modelRevoluteUboundedUnaligned);
    
    Eigen::VectorXd q = Eigen::VectorXd::Ones (modelRUX.nq);
    TangentVector v = TangentVector::Ones (modelRUX.nv);
    Eigen::VectorXd tauRX = Eigen::VectorXd::Ones (modelRUX.nv);
    Eigen::VectorXd tauRevoluteUnaligned = Eigen::VectorXd::Ones (modelRevoluteUboundedUnaligned.nv);
    Eigen::VectorXd aRX = Eigen::VectorXd::Ones (modelRUX.nv);
    Eigen::VectorXd aRevoluteUnaligned(aRX);
    
    forwardKinematics(modelRUX, dataRUX, q, v);
    forwardKinematics(modelRevoluteUboundedUnaligned, dataRevoluteUnboundedUnaligned, q, v);
    
    computeAllTerms(modelRUX, dataRUX, q, v);
    computeAllTerms(modelRevoluteUboundedUnaligned, dataRevoluteUnboundedUnaligned, q, v);
    
    BOOST_CHECK(dataRevoluteUnboundedUnaligned.oMi[1].isApprox(dataRUX.oMi[1]));
    BOOST_CHECK(dataRevoluteUnboundedUnaligned.liMi[1].isApprox(dataRUX.liMi[1]));
    BOOST_CHECK(dataRevoluteUnboundedUnaligned.Ycrb[1].matrix().isApprox(dataRUX.Ycrb[1].matrix()));
    BOOST_CHECK(dataRevoluteUnboundedUnaligned.f[1].toVector().isApprox(dataRUX.f[1].toVector()));
    
    BOOST_CHECK(dataRevoluteUnboundedUnaligned.nle.isApprox(dataRUX.nle));
    BOOST_CHECK(dataRevoluteUnboundedUnaligned.com[0].isApprox(dataRUX.com[0]));
    
    // InverseDynamics == rnea
    tauRX = rnea(modelRUX, dataRUX, q, v, aRX);
    tauRevoluteUnaligned = rnea(modelRevoluteUboundedUnaligned, dataRevoluteUnboundedUnaligned, q, v, aRevoluteUnaligned);
    
    BOOST_CHECK(tauRX.isApprox(tauRevoluteUnaligned));
    
    // ForwardDynamics == aba
    Eigen::VectorXd aAbaRX = aba(modelRUX, dataRUX, q, v, tauRX);
    Eigen::VectorXd aAbaRevoluteUnaligned = aba(modelRevoluteUboundedUnaligned,dataRevoluteUnboundedUnaligned, q, v, tauRevoluteUnaligned);
    
    BOOST_CHECK(aAbaRX.isApprox(aAbaRevoluteUnaligned));
    
    // CRBA
    crba(modelRUX, dataRUX,q);
    crba(modelRevoluteUboundedUnaligned, dataRevoluteUnboundedUnaligned, q);
    
    BOOST_CHECK(dataRUX.M.isApprox(dataRevoluteUnboundedUnaligned.M));
    
    // Jacobian
    Data::Matrix6x jacobianRUX;jacobianRUX.resize(6,1); jacobianRUX.setZero();
    Data::Matrix6x jacobianRevoluteUnboundedUnaligned;
    jacobianRevoluteUnboundedUnaligned.resize(6,1); jacobianRevoluteUnboundedUnaligned.setZero();
    
    computeJointJacobians(modelRUX, dataRUX, q);
    computeJointJacobians(modelRevoluteUboundedUnaligned, dataRevoluteUnboundedUnaligned, q);
    getJointJacobian(modelRUX, dataRUX, 1, LOCAL, jacobianRUX);
    getJointJacobian(modelRevoluteUboundedUnaligned, dataRevoluteUnboundedUnaligned, 1, LOCAL, jacobianRevoluteUnboundedUnaligned);
    
    BOOST_CHECK(jacobianRUX.isApprox(jacobianRevoluteUnboundedUnaligned));
  }

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(JointRevoluteUnbounded)
  
  BOOST_AUTO_TEST_CASE(spatial)
  {
    SE3 M(SE3::Random());
    Motion v(Motion::Random());
    
    MotionRevoluteTpl<double,0,0> mp_x(2.);
    Motion mp_dense_x(mp_x);
    
    BOOST_CHECK(M.act(mp_x).isApprox(M.act(mp_dense_x)));
    BOOST_CHECK(M.actInv(mp_x).isApprox(M.actInv(mp_dense_x)));
    
    BOOST_CHECK(v.cross(mp_x).isApprox(v.cross(mp_dense_x)));
    
    MotionRevoluteTpl<double,0,1> mp_y(2.);
    Motion mp_dense_y(mp_y);
    
    BOOST_CHECK(M.act(mp_y).isApprox(M.act(mp_dense_y)));
    BOOST_CHECK(M.actInv(mp_y).isApprox(M.actInv(mp_dense_y)));
    
    BOOST_CHECK(v.cross(mp_y).isApprox(v.cross(mp_dense_y)));
    
    MotionRevoluteTpl<double,0,2> mp_z(2.);
    Motion mp_dense_z(mp_z);
    
    BOOST_CHECK(M.act(mp_z).isApprox(M.act(mp_dense_z)));
    BOOST_CHECK(M.actInv(mp_z).isApprox(M.actInv(mp_dense_z)));
    
    BOOST_CHECK(v.cross(mp_z).isApprox(v.cross(mp_dense_z)));
  }

BOOST_AUTO_TEST_CASE(vsRX)
{
  typedef SE3::Vector3 Vector3;
  typedef SE3::Matrix3 Matrix3;

  Model modelRX, modelRevoluteUnbounded;

  Inertia inertia(1., Vector3(0.5, 0., 0.0), Matrix3::Identity());
  SE3 pos(1); pos.translation() = SE3::LinearType(1.,0.,0.);

  JointModelRUBX joint_model_RUX;
  addJointAndBody(modelRX,JointModelRX(),0,SE3::Identity(),"rx",inertia);
  addJointAndBody(modelRevoluteUnbounded,joint_model_RUX,0,SE3::Identity(),"revolute unbounded x",inertia);

  Data dataRX(modelRX);
  Data dataRevoluteUnbounded(modelRevoluteUnbounded);

  Eigen::VectorXd q_rx = Eigen::VectorXd::Ones(modelRX.nq);
  Eigen::VectorXd q_rubx = Eigen::VectorXd::Ones(modelRevoluteUnbounded.nq);
  double ca, sa; double alpha = q_rx(0); SINCOS(alpha, &sa, &ca);
  q_rubx(0) = ca;
  q_rubx(1) = sa;
  Eigen::VectorXd v_rx = Eigen::VectorXd::Ones(modelRX.nv);
  Eigen::VectorXd v_rubx = v_rx;
  Eigen::VectorXd tauRX = Eigen::VectorXd::Ones(modelRX.nv);
  Eigen::VectorXd tauRevoluteUnbounded = Eigen::VectorXd::Ones(modelRevoluteUnbounded.nv);
  Eigen::VectorXd aRX = Eigen::VectorXd::Ones(modelRX.nv);
  Eigen::VectorXd aRevoluteUnbounded = aRX;
  
  forwardKinematics(modelRX, dataRX, q_rx, v_rx);
  forwardKinematics(modelRevoluteUnbounded, dataRevoluteUnbounded, q_rubx, v_rubx);

  computeAllTerms(modelRX, dataRX, q_rx, v_rx);
  computeAllTerms(modelRevoluteUnbounded, dataRevoluteUnbounded, q_rubx, v_rubx);

  BOOST_CHECK(dataRevoluteUnbounded.oMi[1].isApprox(dataRX.oMi[1]));
  BOOST_CHECK(dataRevoluteUnbounded.liMi[1].isApprox(dataRX.liMi[1]));
  BOOST_CHECK(dataRevoluteUnbounded.Ycrb[1].matrix().isApprox(dataRX.Ycrb[1].matrix()));
  BOOST_CHECK(dataRevoluteUnbounded.f[1].toVector().isApprox(dataRX.f[1].toVector()));
  
  BOOST_CHECK(dataRevoluteUnbounded.nle.isApprox(dataRX.nle));
  BOOST_CHECK(dataRevoluteUnbounded.com[0].isApprox(dataRX.com[0]));

  // InverseDynamics == rnea
  tauRX = rnea(modelRX, dataRX, q_rx, v_rx, aRX);
  tauRevoluteUnbounded = rnea(modelRevoluteUnbounded, dataRevoluteUnbounded, q_rubx, v_rubx, aRevoluteUnbounded);

  BOOST_CHECK(tauRX.isApprox(tauRevoluteUnbounded));

  // ForwardDynamics == aba
  Eigen::VectorXd aAbaRX= aba(modelRX,dataRX, q_rx, v_rx, tauRX);
  Eigen::VectorXd aAbaRevoluteUnbounded = aba(modelRevoluteUnbounded,dataRevoluteUnbounded, q_rubx, v_rubx, tauRevoluteUnbounded);

  BOOST_CHECK(aAbaRX.isApprox(aAbaRevoluteUnbounded));

  // crba
  crba(modelRX, dataRX,q_rx);
  crba(modelRevoluteUnbounded, dataRevoluteUnbounded, q_rubx);

  BOOST_CHECK(dataRX.M.isApprox(dataRevoluteUnbounded.M));
   
  // Jacobian
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobianPX;jacobianPX.resize(6,1); jacobianPX.setZero();
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobianPrismaticUnaligned;jacobianPrismaticUnaligned.resize(6,1);jacobianPrismaticUnaligned.setZero();
  computeJointJacobians(modelRX, dataRX, q_rx);
  computeJointJacobians(modelRevoluteUnbounded, dataRevoluteUnbounded, q_rubx);
  getJointJacobian(modelRX, dataRX, 1, LOCAL, jacobianPX);
  getJointJacobian(modelRevoluteUnbounded, dataRevoluteUnbounded, 1, LOCAL, jacobianPrismaticUnaligned);

  BOOST_CHECK(jacobianPX.isApprox(jacobianPrismaticUnaligned));

}
BOOST_AUTO_TEST_SUITE_END()
  
BOOST_AUTO_TEST_SUITE(JointRevolute)
  
BOOST_AUTO_TEST_CASE(spatial)
{
  typedef TransformRevoluteTpl<double,0,0> TransformX;
  typedef TransformRevoluteTpl<double,0,1> TransformY;
  typedef TransformRevoluteTpl<double,0,2> TransformZ;
  
  typedef SE3::Vector3 Vector3;
  
  const double alpha = 0.2;
  double sin_alpha, cos_alpha; SINCOS(alpha,&sin_alpha,&cos_alpha);
  SE3 Mplain, Mrand(SE3::Random());
  
  TransformX Mx(sin_alpha,cos_alpha);
  Mplain = Mx;
  BOOST_CHECK(Mplain.translation().isZero());
  BOOST_CHECK(Mplain.rotation().isApprox(Eigen::AngleAxisd(alpha,Vector3::UnitX()).toRotationMatrix()));
  BOOST_CHECK((Mrand*Mplain).isApprox(Mrand*Mx));
  
  TransformY My(sin_alpha,cos_alpha);
  Mplain = My;
  BOOST_CHECK(Mplain.translation().isZero());
  BOOST_CHECK(Mplain.rotation().isApprox(Eigen::AngleAxisd(alpha,Vector3::UnitY()).toRotationMatrix()));
  BOOST_CHECK((Mrand*Mplain).isApprox(Mrand*My));
  
  TransformZ Mz(sin_alpha,cos_alpha);
  Mplain = Mz;
  BOOST_CHECK(Mplain.translation().isZero());
  BOOST_CHECK(Mplain.rotation().isApprox(Eigen::AngleAxisd(alpha,Vector3::UnitZ()).toRotationMatrix()));
  BOOST_CHECK((Mrand*Mplain).isApprox(Mrand*Mz));
  
  SE3 M(SE3::Random());
  Motion v(Motion::Random());
  
  MotionRevoluteTpl<double,0,0> mp_x(2.);
  Motion mp_dense_x(mp_x);
  
  BOOST_CHECK(M.act(mp_x).isApprox(M.act(mp_dense_x)));
  BOOST_CHECK(M.actInv(mp_x).isApprox(M.actInv(mp_dense_x)));
  
  BOOST_CHECK(v.cross(mp_x).isApprox(v.cross(mp_dense_x)));
  
  MotionRevoluteTpl<double,0,1> mp_y(2.);
  Motion mp_dense_y(mp_y);
  
  BOOST_CHECK(M.act(mp_y).isApprox(M.act(mp_dense_y)));
  BOOST_CHECK(M.actInv(mp_y).isApprox(M.actInv(mp_dense_y)));
  
  BOOST_CHECK(v.cross(mp_y).isApprox(v.cross(mp_dense_y)));
  
  MotionRevoluteTpl<double,0,2> mp_z(2.);
  Motion mp_dense_z(mp_z);
  
  BOOST_CHECK(M.act(mp_z).isApprox(M.act(mp_dense_z)));
  BOOST_CHECK(M.actInv(mp_z).isApprox(M.actInv(mp_dense_z)));
  
  BOOST_CHECK(v.cross(mp_z).isApprox(v.cross(mp_dense_z)));
}

BOOST_AUTO_TEST_SUITE_END()
  
BOOST_AUTO_TEST_SUITE(JointRevoluteUnaligned)
  
BOOST_AUTO_TEST_CASE(spatial)
{
  SE3 M(SE3::Random());
  Motion v(Motion::Random());
  
  MotionRevoluteUnaligned mp(MotionRevoluteUnaligned::Vector3(1.,2.,3.),6.);
  Motion mp_dense(mp);
  
  BOOST_CHECK(M.act(mp).isApprox(M.act(mp_dense)));
  BOOST_CHECK(M.actInv(mp).isApprox(M.actInv(mp_dense)));
  
  BOOST_CHECK(v.cross(mp).isApprox(v.cross(mp_dense)));
}
  
BOOST_AUTO_TEST_SUITE_END()
