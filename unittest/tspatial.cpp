//
// Copyright (c) 2015-2016 CNRS
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
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

#include <iostream>

#include "pinocchio/spatial/force.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/spatial/act-on-set.hpp"
#include "pinocchio/spatial/explog.hpp"

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE tspatialTest
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>
#include "pinocchio/tools/matrix-comparison.hpp"


BOOST_AUTO_TEST_SUITE ( tspatialTest)

BOOST_AUTO_TEST_CASE ( test_SE3 )
{
using namespace se3;
  typedef Eigen::Matrix<double,4,4> Matrix4;
  typedef SE3::Matrix6 Matrix6;
  typedef SE3::Vector3 Vector3;

  SE3 amb = SE3::Random();
  SE3 bmc = SE3::Random();
  SE3 amc = amb*bmc;

  Matrix4 aMb = amb;
  Matrix4 bMc = bmc;

  // Test internal product
  Matrix4 aMc = amc;
  is_matrix_absolutely_closed(aMc, aMb*bMc, 1e-12);

  Matrix4 bMa = amb.inverse();
  is_matrix_absolutely_closed(bMa, aMb.inverse(), 1e-12);

  // Test point action
  Vector3 p = Vector3::Random();
  Eigen::Matrix<double,4,1> p4; p4.head(3) = p; p4[3] = 1;

  Vector3 Mp = (aMb*p4).head(3);
  is_matrix_absolutely_closed(amb.act(p), Mp, 1e-12);

  Vector3 Mip = (aMb.inverse()*p4).head(3);
  is_matrix_absolutely_closed(amb.actInv(p), Mip, 1e-12);


  // Test action matrix
  Matrix6 aXb = amb;
  Matrix6 bXc = bmc;
  Matrix6 aXc = amc;
  is_matrix_absolutely_closed(aXc, aXb*bXc, 1e-12);

  Matrix6 bXa = amb.inverse();
  is_matrix_absolutely_closed(bXa, aXb.inverse(), 1e-12);
}

BOOST_AUTO_TEST_CASE ( test_Motion )
{
  using namespace se3;
  typedef SE3::Matrix6 Matrix6;
  typedef Motion::Vector6 Vector6;

  SE3 amb = SE3::Random();
  SE3 bmc = SE3::Random();
  SE3 amc = amb*bmc;

  Motion bv = Motion::Random();
  Motion bv2 = Motion::Random();

  Vector6 bv_vec = bv;
  Vector6 bv2_vec = bv2;
  
  // Test .+.
  Vector6 bvPbv2_vec = bv+bv2;
  is_matrix_absolutely_closed(bvPbv2_vec, bv_vec+bv2_vec, 1e-12);

  // Test -.
  Vector6 Mbv_vec = -bv;
  is_matrix_absolutely_closed( Mbv_vec, -bv_vec, 1e-12);

  // Test .+=.
  Motion bv3 = bv; bv3 += bv2;
  is_matrix_absolutely_closed( bv3.toVector(), bv_vec+bv2_vec, 1e-12);

  // Test .=V6
  bv3 = bv2_vec;
  is_matrix_absolutely_closed( bv3.toVector(), bv2_vec, 1e-12);

  // Test constructor from V6
  Motion bv4(bv2_vec);
  is_matrix_absolutely_closed( bv4.toVector(), bv2_vec, 1e-12);

  // Test action
  Matrix6 aXb = amb;
  is_matrix_absolutely_closed(amb.act(bv).toVector(), aXb*bv_vec, 1e-12);

  // Test action inverse
  Matrix6 bXc = bmc;
  is_matrix_absolutely_closed(bmc.actInv(bv).toVector(), bXc.inverse()*bv_vec, 1e-12);

  // Test double action
  Motion cv = Motion::Random();
  bv = bmc.act(cv);
  is_matrix_absolutely_closed(amb.act(bv).toVector(), amc.act(cv).toVector(), 1e-12);

  // Simple test for cross product vxv
  Motion vxv = bv.cross(bv);
  BOOST_CHECK_SMALL(vxv.toVector().tail(3).norm(), 1e-3); //previously ensure that (vxv.toVector().tail(3).isMuchSmallerThan(1e-3));

  // Simple test for cross product vxf
  Force f = Force(bv.toVector());
  Force vxf = bv.cross(f);
  is_matrix_absolutely_closed(vxf.linear(), bv.angular().cross(f.linear()), 1e-12);
  BOOST_CHECK_SMALL(vxf.angular().norm(), 1e-3);//previously ensure that ( vxf.angular().isMuchSmallerThan(1e-3));

  // Test frame change for vxf
  Motion av = Motion::Random();
  Force af = Force::Random();
  bv = amb.actInv(av);
  Force bf = amb.actInv(af);
  Force avxf = av.cross(af);
  Force bvxf = bv.cross(bf);
  is_matrix_absolutely_closed(avxf.toVector(), amb.act(bvxf).toVector(), 1e-12);

  // Test frame change for vxv
  av = Motion::Random();
  Motion aw = Motion::Random();
  bv = amb.actInv(av);
  Motion bw = amb.actInv(aw);
  Motion avxw = av.cross(aw);
  Motion bvxw = bv.cross(bw);
  is_matrix_absolutely_closed(avxw.toVector(), amb.act(bvxw).toVector(), 1e-12);
}

BOOST_AUTO_TEST_CASE ( test_Force )
{
  using namespace se3;
  typedef SE3::Matrix6 Matrix6;
  typedef Force::Vector6 Vector6;

  SE3 amb = SE3::Random();
  SE3 bmc = SE3::Random();
  SE3 amc = amb*bmc;

  Force bf = Force::Random();
  Force bf2 = Force::Random();

  Vector6 bf_vec = bf;
  Vector6 bf2_vec = bf2;
  
  // Test .+.
  Vector6 bfPbf2_vec = bf+bf2;
  is_matrix_absolutely_closed(bfPbf2_vec, bf_vec+bf2_vec, 1e-12);

  // Test -.
  Vector6 Mbf_vec = -bf;
  is_matrix_absolutely_closed(Mbf_vec, -bf_vec, 1e-12);

  // Test .+=.
  Force bf3 = bf; bf3 += bf2;
  is_matrix_absolutely_closed(bf3.toVector(), bf_vec+bf2_vec, 1e-12);

  // Test .= V6
  bf3 = bf2_vec;
  is_matrix_absolutely_closed(bf3.toVector(), bf2_vec, 1e-12);

  // Test constructor from V6
  Force bf4(bf2_vec);
  is_matrix_absolutely_closed(bf4.toVector(), bf2_vec, 1e-12);


  // Test action
  Matrix6 aXb = amb;
  is_matrix_absolutely_closed(amb.act(bf).toVector(), aXb.inverse().transpose()*bf_vec, 1e-12);

  // Test action inverse
  Matrix6 bXc = bmc;
  is_matrix_absolutely_closed(bmc.actInv(bf).toVector(), bXc.transpose()*bf_vec, 1e-12);

  // Test double action
  Force cf = Force::Random();
  bf = bmc.act(cf);
  is_matrix_absolutely_closed(amb.act(bf).toVector(), amc.act(cf).toVector(), 1e-12);

  // Simple test for cross product
  // Force vxv = bf.cross(bf);
  // ensure that (vxv.toVector().isMuchSmallerThan(bf.toVector()));
}

BOOST_AUTO_TEST_CASE ( test_Inertia )
{
  using namespace se3;
  typedef Inertia::Matrix6 Matrix6;
  typedef Inertia::Matrix3 Matrix3;

  Inertia aI = Inertia::Random();
  Matrix6 matI = aI;
  BOOST_CHECK_EQUAL(matI(0,0), aI.mass());
  BOOST_CHECK_EQUAL(matI(1,1), aI.mass());
  BOOST_CHECK_EQUAL(matI(2,2), aI.mass()); // 1,1 before unifying 

  BOOST_CHECK_SMALL((matI-matI.transpose()).norm(),matI.norm()); //previously ensure that( (matI-matI.transpose()).isMuchSmallerThan(matI) );
  BOOST_CHECK_SMALL((matI.topRightCorner<3,3>()*aI.lever()).norm(),
            aI.lever().norm()); //previously ensure that( (matI.topRightCorner<3,3>()*aI.lever()).isMuchSmallerThan(aI.lever()) );

  Inertia I1 = Inertia::Identity();
  is_matrix_absolutely_closed(I1.matrix(), Matrix6::Identity(), 1e-12); 

  // Test motion-to-force map
  Motion v = Motion::Random();
  Force f = I1 * v;
  is_matrix_absolutely_closed(f.toVector(), v.toVector(), 1e-12); 
  
  // Test Inertia group application
  SE3 bma = SE3::Random(); 
  Inertia bI = bma.act(aI);
  Matrix6 bXa = bma;
  is_matrix_absolutely_closed((bma.rotation()*aI.inertia().matrix()*bma.rotation().transpose()),
                               (Matrix3)bI.inertia(), 1e-12); 
  is_matrix_absolutely_closed((bXa.transpose().inverse() * aI.matrix() * bXa.inverse()),
                              bI.matrix(), 1e-12); 

  // Test inverse action
  is_matrix_absolutely_closed((bXa.transpose() * bI.matrix() * bXa),
                              bma.actInv(bI).matrix(), 1e-12);

  // Test vxIv cross product
  v = Motion::Random(); 
  f = aI*v;
  Force vxf = v.cross(f);
  Force vxIv = aI.vxiv(v);
  is_matrix_absolutely_closed(vxf.toVector(), vxIv.toVector(), 1e-12);

  // Test operator+
  I1 = Inertia::Random();
  Inertia I2 = Inertia::Random();
  is_matrix_absolutely_closed(I1.matrix()+I2.matrix(), (I1+I2).matrix(), 1e-12);

  // operator +=
  Inertia I12 = I1;
  I12 += I2;
  is_matrix_absolutely_closed(I1.matrix()+I2.matrix(), I12.matrix(), 1e-12);
  
  // Test operator vtiv
  double kinetic_ref = v.toVector().transpose() * aI.matrix() * v.toVector();
  double kinetic = aI.vtiv(v);
  BOOST_CHECK_SMALL(kinetic_ref - kinetic, 1e-12);
}

BOOST_AUTO_TEST_CASE ( test_ActOnSet )
{
  const int N = 20;
  typedef Eigen::Matrix<double,6,N> Matrix6N;
  se3::SE3 jMi = se3::SE3::Random();

  Matrix6N iF = Matrix6N::Random(),jF;
  se3::forceSet::se3Action(jMi,iF,jF);
  for( int k=0;k<N;++k )
    is_matrix_absolutely_closed(jMi.act(se3::Force(iF.col(k))).toVector(), jF.col(k), 1e-12);
    

  Matrix6N iV = Matrix6N::Random(),jV;
  se3::motionSet::se3Action(jMi,iV,jV);
  for( int k=0;k<N;++k )
    is_matrix_absolutely_closed(jMi.act(se3::Motion(iV.col(k))).toVector(), jV.col(k), 1e-12);

}

BOOST_AUTO_TEST_CASE ( test_Explog )
{
  typedef se3::SE3::Vector3 Vector3;
  typedef se3::SE3::Matrix3 Matrix3;
  typedef Eigen::Matrix4d Matrix4;
  typedef se3::Motion::Vector6 Vector6;

  const double EPSILON = 1e-12;

  // exp3 and log3.
  Vector3 v3(Vector3::Random());
  Matrix3 R(se3::exp3(v3));
  is_matrix_absolutely_closed(R.transpose(), R.inverse(), EPSILON);
  BOOST_CHECK_SMALL(R.determinant() - 1.0, EPSILON);
  Vector3 v3FromLog(se3::log3(R));
  is_matrix_absolutely_closed(v3, v3FromLog, EPSILON);

  // exp6 and log6.
  se3::Motion nu = se3::Motion::Random();
  se3::SE3 m = se3::exp6(nu);
  is_matrix_absolutely_closed(m.rotation().transpose(), m.rotation().inverse(),
                              EPSILON);
  BOOST_CHECK_SMALL(m.rotation().determinant() - 1.0, EPSILON);
  se3::Motion nuFromLog(se3::log6(m));
  is_matrix_absolutely_closed(nu.linear(), nuFromLog.linear(), EPSILON);
  is_matrix_absolutely_closed(nu.angular(), nuFromLog.angular(), EPSILON);

  Vector6 v6(Vector6::Random());
  se3::SE3 m2(se3::exp6(v6));
  is_matrix_absolutely_closed(m2.rotation().transpose(), m2.rotation().inverse(),
                              EPSILON);
  BOOST_CHECK_SMALL(m2.rotation().determinant() - 1.0, EPSILON);
  Matrix4 M = m2.toHomogeneousMatrix();
  se3::Motion nu2FromLog(se3::log6(M));
  Vector6 v6FromLog(nu2FromLog.toVector());
  is_matrix_absolutely_closed(v6, v6FromLog, EPSILON);
}

BOOST_AUTO_TEST_SUITE_END ()
