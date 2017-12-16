//
// Copyright (c) 2015-2017 CNRS
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
#include "pinocchio/spatial/skew.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

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
  BOOST_CHECK(aMc.isApprox(aMb*bMc, 1e-12));

  Matrix4 bMa = amb.inverse();
  BOOST_CHECK(bMa.isApprox(aMb.inverse(), 1e-12));

  // Test point action
  Vector3 p = Vector3::Random();
  Eigen::Matrix<double,4,1> p4; p4.head(3) = p; p4[3] = 1;

  Vector3 Mp = (aMb*p4).head(3);
  BOOST_CHECK(amb.act(p).isApprox(Mp, 1e-12));

  Vector3 Mip = (aMb.inverse()*p4).head(3);
  BOOST_CHECK(amb.actInv(p).isApprox(Mip, 1e-12));

  // Test action matrix
  Matrix6 aXb = amb;
  Matrix6 bXc = bmc;
  Matrix6 aXc = amc;
  BOOST_CHECK(aXc.isApprox(aXb*bXc, 1e-12));
  
  Matrix6 bXa = amb.inverse();
  BOOST_CHECK(bXa.isApprox(aXb.inverse(), 1e-12));
  
  // Test dual action matrix
  BOOST_CHECK(aXb.inverse().transpose().isApprox(amb.toDualActionMatrix(),1e-12));

  // Test isIdentity
  SE3 identity = SE3::Identity();
  BOOST_CHECK(identity.isIdentity());
  
  // Test isApprox
  BOOST_CHECK(identity.isApprox(identity));
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
  BOOST_CHECK(bvPbv2_vec.isApprox(bv_vec+bv2_vec, 1e-12));

  // Test -.
  Vector6 Mbv_vec = -bv;
  BOOST_CHECK( Mbv_vec.isApprox(-bv_vec, 1e-12));

  // Test .+=.
  Motion bv3 = bv; bv3 += bv2;
  BOOST_CHECK( bv3.toVector().isApprox(bv_vec+bv2_vec, 1e-12));

  // Test .=V6
  bv3 = bv2_vec;
  BOOST_CHECK( bv3.toVector().isApprox(bv2_vec, 1e-12));
  
  // Test scalar*M6
  Motion twicebv(2.*bv);
  BOOST_CHECK(twicebv.isApprox(Motion(2.*bv.toVector())));

  // Test constructor from V6
  Motion bv4(bv2_vec);
  BOOST_CHECK( bv4.toVector().isApprox(bv2_vec, 1e-12));

  // Test action
  Matrix6 aXb = amb;
  BOOST_CHECK(amb.act(bv).toVector().isApprox(aXb*bv_vec, 1e-12));

  // Test action inverse
  Matrix6 bXc = bmc;
  BOOST_CHECK(bmc.actInv(bv).toVector().isApprox(bXc.inverse()*bv_vec, 1e-12));

  // Test double action
  Motion cv = Motion::Random();
  bv = bmc.act(cv);
  BOOST_CHECK(amb.act(bv).toVector().isApprox(amc.act(cv).toVector(), 1e-12));

  // Simple test for cross product vxv
  Motion vxv = bv.cross(bv);
  BOOST_CHECK_SMALL(vxv.toVector().tail(3).norm(), 1e-3); //previously ensure that (vxv.toVector().tail(3).isMuchSmallerThan(1e-3));

  // Test Action Matrix
  Motion v2xv = bv2.cross(bv);
  Motion::ActionMatrix_t actv2 = bv2.toActionMatrix();
  
  BOOST_CHECK(v2xv.toVector().isApprox(actv2*bv.toVector()));
  
  // Test Dual Action Matrix
  Force f(bv.toVector());
  Force v2xf = bv2.cross(f);
  Motion::ActionMatrix_t dualactv2 = bv2.toDualActionMatrix();
  
  BOOST_CHECK(v2xf.toVector().isApprox(dualactv2*f.toVector()));
  BOOST_CHECK(dualactv2.isApprox(-actv2.transpose()));
  
  // Simple test for cross product vxf
  Force vxf = bv.cross(f);
  BOOST_CHECK(vxf.linear().isApprox(bv.angular().cross(f.linear()), 1e-12));
  BOOST_CHECK_SMALL(vxf.angular().norm(), 1e-3);//previously ensure that ( vxf.angular().isMuchSmallerThan(1e-3));

  // Test frame change for vxf
  Motion av = Motion::Random();
  Force af = Force::Random();
  bv = amb.actInv(av);
  Force bf = amb.actInv(af);
  Force avxf = av.cross(af);
  Force bvxf = bv.cross(bf);
  BOOST_CHECK(avxf.toVector().isApprox(amb.act(bvxf).toVector(), 1e-12));

  // Test frame change for vxv
  av = Motion::Random();
  Motion aw = Motion::Random();
  bv = amb.actInv(av);
  Motion bw = amb.actInv(aw);
  Motion avxw = av.cross(aw);
  Motion bvxw = bv.cross(bw);
  BOOST_CHECK(avxw.toVector().isApprox(amb.act(bvxw).toVector(), 1e-12));
  
  // Test isApprox
  const double eps = 1e-6;
  BOOST_CHECK(bv == bv);
  BOOST_CHECK(bv.isApprox(bv));
  Motion bv_approx(bv);
  bv_approx.linear()[0] += eps;
  BOOST_CHECK(bv_approx.isApprox(bv,eps));
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
  BOOST_CHECK(bfPbf2_vec.isApprox(bf_vec+bf2_vec, 1e-12));

  // Test -.
  Vector6 Mbf_vec = -bf;
  BOOST_CHECK(Mbf_vec.isApprox(-bf_vec, 1e-12));

  // Test .+=.
  Force bf3 = bf; bf3 += bf2;
  BOOST_CHECK(bf3.toVector().isApprox(bf_vec+bf2_vec, 1e-12));

  // Test .= V6
  bf3 = bf2_vec;
  BOOST_CHECK(bf3.toVector().isApprox(bf2_vec, 1e-12));

  // Test constructor from V6
  Force bf4(bf2_vec);
  BOOST_CHECK(bf4.toVector().isApprox(bf2_vec, 1e-12));


  // Test action
  Matrix6 aXb = amb;
  BOOST_CHECK(amb.act(bf).toVector().isApprox(aXb.inverse().transpose()*bf_vec, 1e-12));

  // Test action inverse
  Matrix6 bXc = bmc;
  BOOST_CHECK(bmc.actInv(bf).toVector().isApprox(bXc.transpose()*bf_vec, 1e-12));

  // Test double action
  Force cf = Force::Random();
  bf = bmc.act(cf);
  BOOST_CHECK(amb.act(bf).toVector().isApprox(amc.act(cf).toVector(), 1e-12));

  // Simple test for cross product
  // Force vxv = bf.cross(bf);
  // ensure that (vxv.toVector().isMuchSmallerThan(bf.toVector()));
  
  // Test isApprox
  const double eps = 1e-6;
  BOOST_CHECK(bf == bf);
  BOOST_CHECK(bf.isApprox(bf));
  Force bf_approx(bf);
  bf_approx.linear()[0] += eps;
  BOOST_CHECK(bf_approx.isApprox(bf,eps));
}

BOOST_AUTO_TEST_CASE ( test_Inertia )
{
  using namespace se3;
  typedef Inertia::Matrix6 Matrix6;

  Inertia aI = Inertia::Random();
  Matrix6 matI = aI;
  BOOST_CHECK_EQUAL(matI(0,0), aI.mass());
  BOOST_CHECK_EQUAL(matI(1,1), aI.mass());
  BOOST_CHECK_EQUAL(matI(2,2), aI.mass()); // 1,1 before unifying 

  BOOST_CHECK_SMALL((matI-matI.transpose()).norm(),matI.norm()); //previously ensure that( (matI-matI.transpose()).isMuchSmallerThan(matI) );
  BOOST_CHECK_SMALL((matI.topRightCorner<3,3>()*aI.lever()).norm(),
            aI.lever().norm()); //previously ensure that( (matI.topRightCorner<3,3>()*aI.lever()).isMuchSmallerThan(aI.lever()) );

  Inertia I1 = Inertia::Identity();
  BOOST_CHECK(I1.matrix().isApprox(Matrix6::Identity(), 1e-12)); 

  // Test motion-to-force map
  Motion v = Motion::Random();
  Force f = I1 * v;
  BOOST_CHECK(f.toVector().isApprox(v.toVector(), 1e-12)); 
  
  // Test Inertia group application
  SE3 bma = SE3::Random(); 
  Inertia bI = bma.act(aI);
  Matrix6 bXa = bma;
  BOOST_CHECK((bma.rotation()*aI.inertia().matrix()*bma.rotation().transpose())
                               .isApprox(bI.inertia().matrix(), 1e-12));
  BOOST_CHECK((bXa.transpose().inverse() * aI.matrix() * bXa.inverse())
                              .isApprox(bI.matrix(), 1e-12)); 

  // Test inverse action
  BOOST_CHECK((bXa.transpose() * bI.matrix() * bXa)
                              .isApprox(bma.actInv(bI).matrix(), 1e-12));

  // Test vxIv cross product
  v = Motion::Random(); 
  f = aI*v;
  Force vxf = v.cross(f);
  Force vxIv = aI.vxiv(v);
  BOOST_CHECK(vxf.toVector().isApprox(vxIv.toVector(), 1e-12));

  // Test operator+
  I1 = Inertia::Random();
  Inertia I2 = Inertia::Random();
  BOOST_CHECK((I1.matrix()+I2.matrix()).isApprox((I1+I2).matrix(), 1e-12));

  // operator +=
  Inertia I12 = I1;
  I12 += I2;
  BOOST_CHECK((I1.matrix()+I2.matrix()).isApprox(I12.matrix(), 1e-12));
  
  // Test operator vtiv
  double kinetic_ref = v.toVector().transpose() * aI.matrix() * v.toVector();
  double kinetic = aI.vtiv(v);
  BOOST_CHECK_SMALL(kinetic_ref - kinetic, 1e-12);

  // Test constructor (Matrix6)
  Inertia I1_bis(I1.matrix());
  BOOST_CHECK(I1.matrix().isApprox(I1_bis.matrix(), 1e-12));

  // Test Inertia from ellipsoid
  I1 = Inertia::FromEllipsoid(2., 3., 4., 5.);
  BOOST_CHECK_SMALL(I1.mass() - 2, 1e-12);
  BOOST_CHECK_SMALL(I1.lever().norm(), 1e-12);
  BOOST_CHECK(I1.inertia().matrix().isApprox(Symmetric3(
          16.4, 0., 13.6, 0., 0., 10.).matrix(), 1e-12));

  // Test Inertia from Cylinder
  I1 = Inertia::FromCylinder(2., 4., 6.);
  BOOST_CHECK_SMALL(I1.mass() - 2, 1e-12);
  BOOST_CHECK_SMALL(I1.lever().norm(), 1e-12);
  BOOST_CHECK(I1.inertia().matrix().isApprox(Symmetric3(
        14., 0., 14., 0., 0., 16.).matrix(), 1e-12));

  // Test Inertia from Box
  I1 = Inertia::FromBox(2., 6., 12., 18.);
  BOOST_CHECK_SMALL(I1.mass() - 2, 1e-12);
  BOOST_CHECK_SMALL(I1.lever().norm(), 1e-12);
  BOOST_CHECK(I1.inertia().matrix().isApprox(Symmetric3(
        78., 0., 60., 0., 0., 30.).matrix(), 1e-12));
  
  // Copy operator
  Inertia aI_copy(aI);
  BOOST_CHECK(aI_copy == aI);
  
  // Test isApprox
  const double eps = 1e-6;
  BOOST_CHECK(aI == aI);
  BOOST_CHECK(aI.isApprox(aI));
  Inertia aI_approx(aI);
  aI_approx.mass() += eps;
  BOOST_CHECK(aI_approx.isApprox(aI,eps));
  
  // Test Variation
  Inertia::Matrix6 aIvariation = aI.variation(v);
  
  Motion::ActionMatrix_t vAction = v.toActionMatrix();
  Motion::ActionMatrix_t vDualAction = v.toDualActionMatrix();
  
  Inertia::Matrix6 aImatrix = aI.matrix();
  Inertia::Matrix6 aIvariation_ref = vDualAction * aImatrix - aImatrix * vAction;
  
  BOOST_CHECK(aIvariation.isApprox(aIvariation_ref));
  BOOST_CHECK(vxIv.isApprox(Force(aIvariation*v.toVector())));
  
  // Test vxI operator
  {
    typedef Inertia::Matrix6 Matrix6;
    Inertia I(Inertia::Random());
    Motion v(Motion::Random());
    
    const Matrix6 M_ref(v.toDualActionMatrix()*I.matrix());
    Matrix6 M; Inertia::vxi(v,I,M);
    
    BOOST_CHECK(M.isApprox(M_ref));
    BOOST_CHECK(I.vxi(v).isApprox(M_ref));
  }
  
  // Test Ivx operator
  {
    typedef Inertia::Matrix6 Matrix6;
    Inertia I(Inertia::Random());
    Motion v(Motion::Random());
    
    const Matrix6 M_ref(I.matrix()*v.toActionMatrix());
    Matrix6 M; Inertia::ivx(v,I,M);
    
    BOOST_CHECK(M.isApprox(M_ref));
    BOOST_CHECK(I.ivx(v).isApprox(M_ref));
  }
  
  // Text variation against vxI - Ivx operator
  {
    typedef Inertia::Matrix6 Matrix6;
    Inertia I(Inertia::Random());
    Motion v(Motion::Random());
    
    Matrix6 Ivariation = I.variation(v);
    
    Matrix6 M1; Inertia::vxi(v,I,M1);
    Matrix6 M2; Inertia::ivx(v,I,M2);
    Matrix6 M3(M1-M2);
    
    BOOST_CHECK(M3.isApprox(Ivariation));
  }
  
}

BOOST_AUTO_TEST_CASE ( test_ActOnSet )
{
  using namespace se3;
  const int N = 20;
  typedef Eigen::Matrix<double,6,N> Matrix6N;
  SE3 jMi = SE3::Random();
  Motion v = Motion::Random();

  // Forcet SET
  Matrix6N iF = Matrix6N::Random(),jF,jFinv;
  forceSet::se3Action(jMi,iF,jF);
  for( int k=0;k<N;++k )
    BOOST_CHECK(jMi.act(Force(iF.col(k))).toVector().isApprox(jF.col(k), 1e-12));
  
  forceSet::se3ActionInverse(jMi.inverse(),iF,jFinv);
  BOOST_CHECK(jFinv.isApprox(jF));
  
  forceSet::se3Action(v,iF,jF);
  for( int k=0;k<N;++k )
    BOOST_CHECK(v.cross(Force(iF.col(k))).toVector().isApprox(jF.col(k), 1e-12));
    
  // Motion SET
  Matrix6N iV = Matrix6N::Random(),jV,jVinv;
  motionSet::se3Action(jMi,iV,jV);
  for( int k=0;k<N;++k )
    BOOST_CHECK(jMi.act(Motion(iV.col(k))).toVector().isApprox(jV.col(k), 1e-12));
  
  motionSet::se3ActionInverse(jMi.inverse(),iV,jVinv);
  BOOST_CHECK(jVinv.isApprox(jV));
  
  motionSet::se3Action(v,iV,jV);
  for( int k=0;k<N;++k )
    BOOST_CHECK(v.cross(Motion(iV.col(k))).toVector().isApprox(jV.col(k), 1e-12));
  
  

}

BOOST_AUTO_TEST_CASE(test_skew)
{
  using namespace se3;
  typedef SE3::Vector3 Vector3;
  typedef SE3::Vector6 Vector6;
  
  Vector3 v3(Vector3::Random());
  Vector6 v6(Vector6::Random());
  
  Vector3 res1 = unSkew(skew(v3));
  BOOST_CHECK(res1.isApprox(v3));
  
  Vector3 res2 = unSkew(skew(v6.head<3>()));
  BOOST_CHECK(res2.isApprox(v6.head<3>()));
  
  Vector3 res3 = skew(v3)*v3;
  BOOST_CHECK(res3.isZero());
  
  Vector3 rhs(Vector3::Random());
  Vector3 res41 = skew(v3)*rhs;
  Vector3 res42 = v3.cross(rhs);
  
  BOOST_CHECK(res41.isApprox(res42));
  
}

BOOST_AUTO_TEST_CASE(test_skew_square)
{
  using namespace se3;
  typedef SE3::Vector3 Vector3;
  typedef SE3::Matrix3 Matrix3;
  
  Vector3 u(Vector3::Random());
  Vector3 v(Vector3::Random());
  
  Matrix3 ref = skew(u) * skew(v);
  
  Matrix3 res = skewSquare(u,v);
  
  BOOST_CHECK(res.isApprox(ref));
}

BOOST_AUTO_TEST_SUITE_END ()
