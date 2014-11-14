#include <iostream>

#include "pinocchio/spatial/force.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/spatial/act-on-set.hpp"

#define TEST_SE3
#define TEST_MOTION
#define TEST_FORCE
#define TEST_INERTIA
#define TEST_SYM3


bool testSE3()
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
  assert(aMc.isApprox(aMb*bMc));

  Matrix4 bMa = amb.inverse();
  assert(bMa.isApprox(aMb.inverse()));

  { // Test point action
    Vector3 p = Vector3::Random();
    Eigen::Matrix<double,4,1> p4; p4.head(3) = p; p4[3] = 1;

    Vector3 Mp = (aMb*p4).head(3);
    assert(amb.act(p).isApprox(Mp));
    Vector3 Mip = (aMb.inverse()*p4).head(3);
    assert(amb.actInv(p).isApprox(Mip));
  }

  { // Test action matrix
    Matrix6 aXb = amb;
    Matrix6 bXc = bmc;
    Matrix6 aXc = amc;
    assert(aXc.isApprox(aXb*bXc));

    Matrix6 bXa = amb.inverse();
    assert(bXa.isApprox(aXb.inverse()));
  }


  return true;
}

bool testMotion()
{
  using namespace se3;
  typedef Eigen::Matrix<double,4,4> Matrix4;
  typedef SE3::Matrix6 Matrix6;
  typedef SE3::Vector3 Vector3;
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
  assert( bvPbv2_vec.isApprox(bv_vec+bv2_vec) );
  // Test -.
  Vector6 Mbv_vec = -bv;
  assert( Mbv_vec.isApprox(-bv_vec) );
  // Test .+=.
  Motion bv3 = bv; bv3 += bv2;
  assert(bv3.toVector().isApprox(bv_vec+bv2_vec));
  // Test .=V6
  bv3 = bv2_vec;
  assert(bv3.toVector().isApprox(bv2_vec));
  // Test constructor from V6
  Motion bv4(bv2_vec);
  assert(bv4.toVector().isApprox(bv2_vec));

  // Test action
  Matrix6 aXb = amb;
  assert( amb.act(bv).toVector().isApprox(aXb*bv_vec));
  // Test action inverse
  Matrix6 bXc = bmc;
  assert( bmc.actInv(bv).toVector().isApprox(bXc.inverse()*bv_vec));

  // Test double action
  Motion cv = Motion::Random();
  bv = bmc.act(cv);
  assert( amb.act(bv).toVector().isApprox(amc.act(cv).toVector()) );

  // Simple test for cross product vxv
  Motion vxv = bv.cross(bv);
  assert(vxv.toVector().tail(3).isMuchSmallerThan(1e-3));

  // Simple test for cross product vxf
  Force f = Force(bv.toVector());
  Force vxf = bv.cross(f);
  assert( vxf.linear().isApprox( bv.angular().cross(f.linear())));
  assert( vxf.angular().isMuchSmallerThan(1e-3));

  // Test frame change for vxf
  Motion av = Motion::Random();
  Force af = Force::Random();
  bv = amb.actInv(av);
  Force bf = amb.actInv(af);
  Force avxf = av.cross(af);
  Force bvxf = bv.cross(bf);
  assert( avxf.toVector().isApprox( amb.act(bvxf).toVector()) );

  // Test frame change for vxv
  av = Motion::Random();
  Motion aw = Motion::Random();
  bv = amb.actInv(av);
  Motion bw = amb.actInv(aw);
  Motion avxw = av.cross(aw);
  Motion bvxw = bv.cross(bw);
  assert( avxw.toVector().isApprox( amb.act(bvxw).toVector()) );

  return true;
}


bool testForce()
{
  using namespace se3;
  typedef Eigen::Matrix<double,4,4> Matrix4;
  typedef SE3::Matrix6 Matrix6;
  typedef SE3::Vector3 Vector3;
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
  assert( bfPbf2_vec.isApprox(bf_vec+bf2_vec) );
  // Test -.
  Vector6 Mbf_vec = -bf;
  assert( Mbf_vec.isApprox(-bf_vec) );
  // Test .+=.
  Force bf3 = bf; bf3 += bf2;
  assert(bf3.toVector().isApprox(bf_vec+bf2_vec));
  // Test .= V6
  bf3 = bf2_vec;
  assert(bf3.toVector().isApprox(bf2_vec));
  // Test constructor from V6
  Force bf4(bf2_vec);
  assert(bf4.toVector().isApprox(bf2_vec));

  // Test action
  Matrix6 aXb = amb;
  assert( amb.act(bf).toVector().isApprox(aXb.inverse().transpose()*bf_vec));
  // Test action inverse
  Matrix6 bXc = bmc;
  assert( bmc.actInv(bf).toVector().isApprox(bXc.transpose()*bf_vec));
  // Test double action
  Force cf = Force::Random();
  bf = bmc.act(cf);
  assert( amb.act(bf).toVector().isApprox(amc.act(cf).toVector()) );

  // Simple test for cross product
  // Force vxv = bf.cross(bf);
  // assert(vxv.toVector().isMuchSmallerThan(bf.toVector()));

  return true;
}

bool testInertia()
{
  using namespace se3;
  typedef Inertia::Matrix6 Matrix6;
  typedef Inertia::Matrix3 Matrix3;
  typedef Inertia::Vector3 Vector3;
  typedef Eigen::Matrix<double,4,4> Matrix4;

  Inertia aI = Inertia::Random();
  Matrix6 matI = aI;
  assert( (matI(0,0) == aI.mass())
	  && (matI(1,1) == aI.mass())
	  && (matI(1,1) == aI.mass()) );
  assert( (matI-matI.transpose()).isMuchSmallerThan(matI) );
  assert( (matI.topRightCorner<3,3>()*aI.lever()).isMuchSmallerThan(aI.lever()) );

  Inertia I1 = Inertia::Identity();
  assert( I1.matrix() == Matrix6::Identity() );

  // Test motion-to-force map
  Motion v = Motion::Random();
  Force f = I1 * v;
  assert( f.toVector() == v.toVector() );
  
  // Test Inertia group application
  SE3 bma = SE3::Random(); 
  Inertia bI = bma.act(aI);
  Matrix6 bXa = bma;
  assert( (bma.rotation()*aI.inertia().matrix()*bma.rotation().transpose())
	  .isApprox((Matrix3)bI.inertia()) );
  assert( (bXa.transpose().inverse() * aI.matrix() * bXa.inverse()).isApprox( bI.matrix()) );

  // Test inverse action
  assert( (bXa.transpose() * bI.matrix() * bXa).isApprox( bma.actInv(bI).matrix()) );

  // Test vxIv cross product
  v = Motion::Random(); 
  f = aI*v;
  Force vxf = v.cross(f);
  Force vxIv = aI.vxiv(v);
  assert( vxf.toVector().isApprox(vxIv.toVector()) );

  // Test operator+
  I1 = Inertia::Random();
  Inertia I2 = Inertia::Random();
  assert( (I1.matrix()+I2.matrix()).isApprox((I1+I2).matrix()) );
  // operator +=
  Inertia I12 = I1;
  I12 += I2;
  assert( (I1.matrix()+I2.matrix()).isApprox(I12.matrix()) );

  return true;
}

bool testActOnSet()
{
  const int N = 20;
  typedef Eigen::Matrix<double,6,N> Matrix6N;
  se3::SE3 jMi = se3::SE3::Random();

  Matrix6N iF = Matrix6N::Random(),jF;
  se3::forceSet::se3Action(jMi,iF,jF);
  for( int k=0;k<N;++k )
    assert( jMi.act(se3::Force(iF.col(k))).toVector().isApprox( jF.col(k) ));

  Matrix6N iV = Matrix6N::Random(),jV;
  se3::motionSet::se3Action(jMi,iV,jV);
  for( int k=0;k<N;++k )
    assert( jMi.act(se3::Motion(iV.col(k))).toVector().isApprox( jV.col(k) ));

  return true;
}


int main()
{
  testSE3();
  testMotion();
  testForce();
  testInertia();
  testActOnSet();
  return 0;
}

