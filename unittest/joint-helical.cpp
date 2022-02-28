//
// Copyright (c) 2022 CNRS INRIA
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

BOOST_AUTO_TEST_SUITE(JointHelical)

BOOST_AUTO_TEST_CASE(spatial)
{
  typedef TransformHelicalTpl<double,0,0> TransformX;
  typedef TransformHelicalTpl<double,0,1> TransformY;
  typedef TransformHelicalTpl<double,0,2> TransformZ;

  typedef TransformPrismaticTpl<double,0,0> TransformPX;
  typedef TransformRevoluteTpl<double,0,0> TransformRX;

  
  typedef SE3::Vector3 Vector3;
  
  const double alpha = 0.2, pitch = 0.1;
  double sin_alpha, cos_alpha; SINCOS(alpha,&sin_alpha,&cos_alpha);
  SE3 Mplain, Mrand(SE3::Random());
  
  // TODO Alpha is not necessary, it can be reconstrcuted, but where to put pitch if not here as input ?
  TransformX Mx(sin_alpha,cos_alpha,alpha,pitch);
  Mplain = Mx;
  BOOST_CHECK(Mplain.translation().isApprox(Vector3::UnitX()*alpha*pitch));
  BOOST_CHECK(Mplain.rotation().isApprox(Eigen::AngleAxisd(alpha,Vector3::UnitX()).toRotationMatrix()));
  BOOST_CHECK((Mrand*Mplain).isApprox(Mrand*Mx));

  // TODO: Test against combinatino of revolute and prismatic joint
  // TransformPX MPx(alpha*pitch);
  // TransformRX MRx(sin_alpha,cos_alpha);
  // // auto MRXPX = MPx * MRx;

  // BOOST_CHECK(Mplain.translation().isApprox(MPx.translation()+MRx.translation()));
  // BOOST_CHECK(Mplain.rotation().isApprox(MPx.rotation() * MRx.rotation()));
  // BOOST_CHECK((Mrand*Mplain).isApprox(Mrand*MPx*MRx));
  
  TransformY My(sin_alpha,cos_alpha,alpha,pitch);
  Mplain = My;
  BOOST_CHECK(Mplain.translation().isApprox(Vector3::UnitY()*alpha*pitch));
  BOOST_CHECK(Mplain.rotation().isApprox(Eigen::AngleAxisd(alpha,Vector3::UnitY()).toRotationMatrix()));
  BOOST_CHECK((Mrand*Mplain).isApprox(Mrand*My));
  
  TransformZ Mz(sin_alpha,cos_alpha,alpha,pitch);
  Mplain = Mz;
  BOOST_CHECK(Mplain.translation().isApprox(Vector3::UnitZ()*alpha*pitch));
  BOOST_CHECK(Mplain.rotation().isApprox(Eigen::AngleAxisd(alpha,Vector3::UnitZ()).toRotationMatrix()));
  BOOST_CHECK((Mrand*Mplain).isApprox(Mrand*Mz));
  
  SE3 M(SE3::Random());
  Motion v(Motion::Random());
  
  MotionHelicalTpl<double,0,0> mh_x(2., pitch);
  Motion mh_dense_x(mh_x);
  
  BOOST_CHECK(M.act(mh_x).isApprox(M.act(mh_dense_x)));
  BOOST_CHECK(M.actInv(mh_x).isApprox(M.actInv(mh_dense_x)));
  
  BOOST_CHECK(v.cross(mh_x).isApprox(v.cross(mh_dense_x)));
  
  MotionHelicalTpl<double,0,1> mh_y(2., pitch);
  Motion mh_dense_y(mh_y);
  
  BOOST_CHECK(M.act(mh_y).isApprox(M.act(mh_dense_y)));
  BOOST_CHECK(M.actInv(mh_y).isApprox(M.actInv(mh_dense_y)));
  
  BOOST_CHECK(v.cross(mh_y).isApprox(v.cross(mh_dense_y)));
  
  MotionHelicalTpl<double,0,2> mh_z(2., pitch);
  Motion mh_dense_z(mh_z);
  
  BOOST_CHECK(M.act(mh_z).isApprox(M.act(mh_dense_z)));
  BOOST_CHECK(M.actInv(mh_z).isApprox(M.actInv(mh_dense_z)));
  
  BOOST_CHECK(v.cross(mh_z).isApprox(v.cross(mh_dense_z)));

}

BOOST_AUTO_TEST_SUITE_END()