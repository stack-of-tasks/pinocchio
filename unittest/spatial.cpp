//
// Copyright (c) 2015-2020 CNRS INRIA
//

#include <iostream>

#include "pinocchio/spatial/force.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/spatial/act-on-set.hpp"
#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/spatial/skew.hpp"
#include "pinocchio/spatial/cartesian-axis.hpp"
#include "pinocchio/spatial/spatial-axis.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE ( test_SE3 )
{
  using namespace pinocchio;
  typedef SE3::HomogeneousMatrixType HomogeneousMatrixType;
  typedef SE3::ActionMatrixType ActionMatrixType;
  typedef SE3::Vector3 Vector3;
  typedef Eigen::Matrix<double,4,1> Vector4;
  
  const SE3 identity = SE3::Identity();

  typedef SE3::Quaternion Quaternion;
  typedef SE3::Vector4 Vector4;
  Quaternion quat(Vector4::Random().normalized());

  SE3 m_from_quat(quat,Vector3::Random());

  SE3 amb = SE3::Random();
  SE3 bmc = SE3::Random();
  SE3 amc = amb*bmc;

  HomogeneousMatrixType aMb = amb;
  HomogeneousMatrixType bMc = bmc;

  // Test internal product
  HomogeneousMatrixType aMc = amc;
  BOOST_CHECK(aMc.isApprox(aMb*bMc));

  HomogeneousMatrixType bMa = amb.inverse();
  BOOST_CHECK(bMa.isApprox(aMb.inverse()));

  // Test point action
  Vector3 p = Vector3::Random();
  Vector4 p4; p4.head(3) = p; p4[3] = 1;

  Vector3 Mp = (aMb*p4).head(3);
  BOOST_CHECK(amb.act(p).isApprox(Mp));

  Vector3 Mip = (aMb.inverse()*p4).head(3);
  BOOST_CHECK(amb.actInv(p).isApprox(Mip));

  // Test action matrix
  ActionMatrixType aXb = amb;
  ActionMatrixType bXc = bmc;
  ActionMatrixType aXc = amc;
  BOOST_CHECK(aXc.isApprox(aXb*bXc));
  
  ActionMatrixType bXa = amb.inverse();
  BOOST_CHECK(bXa.isApprox(aXb.inverse()));
  
  ActionMatrixType X_identity = identity.toActionMatrix();
  BOOST_CHECK(X_identity.isIdentity());
  
  ActionMatrixType X_identity_inverse = identity.toActionMatrixInverse();
  BOOST_CHECK(X_identity_inverse.isIdentity());
  
  // Test dual action matrix
  BOOST_CHECK(aXb.inverse().transpose().isApprox(amb.toDualActionMatrix()));

  // Test isIdentity
  BOOST_CHECK(identity.isIdentity());
  
  // Test isApprox
  BOOST_CHECK(identity.isApprox(identity));
  
  // Test cast
  typedef SE3Tpl<float> SE3f;
  SE3f::Matrix3 rot_float(amb.rotation().cast<float>());
  SE3f amb_float = amb.cast<float>();
  BOOST_CHECK(amb_float.isApprox(amb.cast<float>()));
  
  // Test actInv
  const SE3 M = SE3::Random();
  const SE3 Minv = M.inverse();
  
  BOOST_CHECK(Minv.actInv(Minv).isIdentity());
  BOOST_CHECK(M.actInv(identity).isApprox(Minv));
  
  // Test normalization
  {
    const double prec = Eigen::NumTraits<double>::dummy_precision();
    SE3 M(SE3::Random());
    M.rotation() += prec * SE3::Matrix3::Random();
    BOOST_CHECK(!M.isNormalized());
    
    SE3 M_normalized = M.normalized();
    BOOST_CHECK(M_normalized.isNormalized());
    
    M.normalize();
    BOOST_CHECK(M.isNormalized());
  }
  
}

BOOST_AUTO_TEST_CASE ( test_Motion )
{
  using namespace pinocchio;
  typedef SE3::ActionMatrixType ActionMatrixType;
  typedef Motion::Vector6 Vector6;

  SE3 amb = SE3::Random();
  SE3 bmc = SE3::Random();
  SE3 amc = amb*bmc;

  Motion bv = Motion::Random();
  Motion bv2 = Motion::Random();
  
  typedef MotionBase<Motion> Base;

  Vector6 bv_vec = bv;
  Vector6 bv2_vec = bv2;
  
  // std::stringstream
  std::stringstream ss;
  ss << bv << std::endl;
  BOOST_CHECK(!ss.str().empty());
  
  // Test .+.
  Vector6 bvPbv2_vec = bv+bv2;
  BOOST_CHECK(bvPbv2_vec.isApprox(bv_vec+bv2_vec));
  
  Motion bplus = static_cast<Base &>(bv) + static_cast<Base &>(bv2);
  BOOST_CHECK((bv+bv2).isApprox(bplus));
  
  Motion v_not_zero(Vector6::Ones());
  BOOST_CHECK(!v_not_zero.isZero());
  
  Motion v_zero(Vector6::Zero());
  BOOST_CHECK(v_zero.isZero());
  
  // Test == and !=
  BOOST_CHECK(bv == bv);
  BOOST_CHECK(!(bv != bv));

  // Test -.
  Vector6 Mbv_vec = -bv;
  BOOST_CHECK( Mbv_vec.isApprox(-bv_vec));

  // Test .+=.
  Motion bv3 = bv; bv3 += bv2;
  BOOST_CHECK( bv3.toVector().isApprox(bv_vec+bv2_vec));

  // Test .=V6
  bv3 = bv2_vec;
  BOOST_CHECK( bv3.toVector().isApprox(bv2_vec));
  
  // Test scalar*M6
  Motion twicebv(2.*bv);
  BOOST_CHECK(twicebv.isApprox(Motion(2.*bv.toVector())));
  
  // Test M6*scalar
  Motion bvtwice(bv*2.);
  BOOST_CHECK(bvtwice.isApprox(twicebv));
  
  // Test M6/scalar
  Motion bvdividedbytwo(bvtwice/2.);
  BOOST_CHECK(bvdividedbytwo.isApprox(bv));

  // Test constructor from V6
  Motion bv4(bv2_vec);
  BOOST_CHECK( bv4.toVector().isApprox(bv2_vec));

  // Test action
  ActionMatrixType aXb = amb;
  BOOST_CHECK(amb.act(bv).toVector().isApprox(aXb*bv_vec));

  // Test action inverse
  ActionMatrixType bXc = bmc;
  BOOST_CHECK(bmc.actInv(bv).toVector().isApprox(bXc.inverse()*bv_vec));

  // Test double action
  Motion cv = Motion::Random();
  bv = bmc.act(cv);
  BOOST_CHECK(amb.act(bv).toVector().isApprox(amc.act(cv).toVector()));

  // Simple test for cross product vxv
  Motion vxv = bv.cross(bv);
  BOOST_CHECK_SMALL(vxv.toVector().tail(3).norm(), 1e-3); //previously ensure that (vxv.toVector().tail(3).isMuchSmallerThan(1e-3));

  // Test Action Matrix
  Motion v2xv = bv2.cross(bv);
  Motion::ActionMatrixType actv2 = bv2.toActionMatrix();
  
  BOOST_CHECK(v2xv.toVector().isApprox(actv2*bv.toVector()));
  
  // Test Dual Action Matrix
  Force f(bv.toVector());
  Force v2xf = bv2.cross(f);
  Motion::ActionMatrixType dualactv2 = bv2.toDualActionMatrix();
  
  BOOST_CHECK(v2xf.toVector().isApprox(dualactv2*f.toVector()));
  BOOST_CHECK(dualactv2.isApprox(-actv2.transpose()));
  
  // Simple test for cross product vxf
  Force vxf = bv.cross(f);
  BOOST_CHECK(vxf.linear().isApprox(bv.angular().cross(f.linear())));
  BOOST_CHECK_SMALL(vxf.angular().norm(), 1e-3);//previously ensure that ( vxf.angular().isMuchSmallerThan(1e-3));

  // Test frame change for vxf
  Motion av = Motion::Random();
  Force af = Force::Random();
  bv = amb.actInv(av);
  Force bf = amb.actInv(af);
  Force avxf = av.cross(af);
  Force bvxf = bv.cross(bf);
  BOOST_CHECK(avxf.toVector().isApprox(amb.act(bvxf).toVector()));

  // Test frame change for vxv
  av = Motion::Random();
  Motion aw = Motion::Random();
  bv = amb.actInv(av);
  Motion bw = amb.actInv(aw);
  Motion avxw = av.cross(aw);
  Motion bvxw = bv.cross(bw);
  BOOST_CHECK(avxw.toVector().isApprox(amb.act(bvxw).toVector()));
  
  // Test isApprox
  bv.toVector().setOnes();
  const double eps = 1e-6;
  BOOST_CHECK(bv == bv);
  BOOST_CHECK(bv.isApprox(bv));
  Motion bv_approx(bv);
  bv_approx.linear()[0] += eps;
  BOOST_CHECK(bv_approx.isApprox(bv,eps));
  
  // Test ref() method
  {
    Motion a(Motion::Random());
    BOOST_CHECK(a.ref().isApprox(a));
    
    const Motion b(a);
    BOOST_CHECK(b.isApprox(a.ref()));
  }
  
  // Test cast
  {
    typedef MotionTpl<float> Motionf;
    Motion a(Motion::Random());
    Motionf a_float = a.cast<float>();
    BOOST_CHECK(a_float.isApprox(a.cast<float>()));
  }
}

BOOST_AUTO_TEST_CASE (test_motion_ref)
{
  using namespace pinocchio;
  typedef Motion::Vector6 Vector6;
  
  typedef MotionRef<Vector6> MotionV6;
  
  Motion v_ref(Motion::Random());
  MotionV6 v(v_ref.toVector());
  
  BOOST_CHECK(v_ref.isApprox(v));
  
  MotionV6::MotionPlain v2(v*2.);
  Motion v2_ref(v_ref*2.);
  
  BOOST_CHECK(v2_ref.isApprox(v2));
  
  v2 = v_ref + v;
  BOOST_CHECK(v2_ref.isApprox(v2));
  
  v = v2;
  BOOST_CHECK(v2.isApprox(v));
  
  v2 = v - v;
  BOOST_CHECK(v2.isApprox(Motion::Zero()));
  
  SE3 M(SE3::Identity());
  v2 = M.act(v);
  BOOST_CHECK(v2.isApprox(v));
  
  v2 = M.actInv(v);
  BOOST_CHECK(v2.isApprox(v));
  
  Motion v3(Motion::Random());
  v_ref.setRandom();
  v = v_ref;
  v2 = v.cross(v3);
  v2_ref = v_ref.cross(v3);
  
  BOOST_CHECK(v2.isApprox(v2_ref));
  
  v.setRandom();
  v.setZero();
  BOOST_CHECK(v.isApprox(Motion::Zero()));
  
  // Test ref() method
  {
    Vector6 v6(Vector6::Random());
    MotionV6 a(v6);
    BOOST_CHECK(a.ref().isApprox(a));
    
    const Motion b(a);
    BOOST_CHECK(b.isApprox(a.ref()));
  }
  
}

BOOST_AUTO_TEST_CASE(test_motion_zero)
{
  using namespace pinocchio;
  Motion v((MotionZero()));
  
  BOOST_CHECK(v.toVector().isZero());
  BOOST_CHECK(MotionZero() == Motion::Zero());
  
  // SE3.act
  SE3 m(SE3::Random());
  BOOST_CHECK(m.act(MotionZero()) == Motion::Zero());
  BOOST_CHECK(m.actInv(MotionZero()) == Motion::Zero());
  
  // Motion.cross
  Motion v2(Motion::Random());
  BOOST_CHECK(v2.cross(MotionZero()) == Motion::Zero());
}

BOOST_AUTO_TEST_CASE ( test_Force )
{
  using namespace pinocchio;
  typedef SE3::ActionMatrixType ActionMatrixType;
  typedef Force::Vector6 Vector6;

  SE3 amb = SE3::Random();
  SE3 bmc = SE3::Random();
  SE3 amc = amb*bmc;

  Force bf = Force::Random();
  Force bf2 = Force::Random();

  Vector6 bf_vec = bf;
  Vector6 bf2_vec = bf2;
  
  // std::stringstream
  std::stringstream ss;
  ss << bf << std::endl;
  BOOST_CHECK(!ss.str().empty());
  
  // Test .+.
  Vector6 bfPbf2_vec = bf+bf2;
  BOOST_CHECK(bfPbf2_vec.isApprox(bf_vec+bf2_vec));

  // Test -.
  Vector6 Mbf_vec = -bf;
  BOOST_CHECK(Mbf_vec.isApprox(-bf_vec));

  // Test .+=.
  Force bf3 = bf; bf3 += bf2;
  BOOST_CHECK(bf3.toVector().isApprox(bf_vec+bf2_vec));

  // Test .= V6
  bf3 = bf2_vec;
  BOOST_CHECK(bf3.toVector().isApprox(bf2_vec));

  // Test constructor from V6
  Force bf4(bf2_vec);
  BOOST_CHECK(bf4.toVector().isApprox(bf2_vec));

  // Test action
  ActionMatrixType aXb = amb;
  BOOST_CHECK(amb.act(bf).toVector().isApprox(aXb.inverse().transpose()*bf_vec));

  // Test action inverse
  ActionMatrixType bXc = bmc;
  BOOST_CHECK(bmc.actInv(bf).toVector().isApprox(bXc.transpose()*bf_vec));

  // Test double action
  Force cf = Force::Random();
  bf = bmc.act(cf);
  BOOST_CHECK(amb.act(bf).toVector().isApprox(amc.act(cf).toVector()));

  // Simple test for cross product
  // Force vxv = bf.cross(bf);
  // ensure that (vxv.toVector().isMuchSmallerThan(bf.toVector()));
  
  Force f_not_zero(Vector6::Ones());
  BOOST_CHECK(!f_not_zero.isZero());
  
  Force f_zero(Vector6::Zero());
  BOOST_CHECK(f_zero.isZero());
  
  // Test isApprox
  
  BOOST_CHECK(bf == bf);
  bf.setRandom();
  bf2.setZero();
  BOOST_CHECK(bf == bf);
  BOOST_CHECK(bf != bf2);
  BOOST_CHECK(bf.isApprox(bf));
  BOOST_CHECK(!bf.isApprox(bf2));
  
  const double eps = 1e-6;
  Force bf_approx(bf);
  bf_approx.linear()[0] += 2.*eps;
  BOOST_CHECK(!bf_approx.isApprox(bf,eps));
  
  // Test ref() method
  {
    Force a(Force::Random());
    BOOST_CHECK(a.ref().isApprox(a));
    
    const Force b(a);
    BOOST_CHECK(b.isApprox(a.ref()));
  }
  
  // Test cast
  {
    typedef ForceTpl<float> Forcef;
    Force a(Force::Random());
    Forcef a_float = a.cast<float>();
    BOOST_CHECK(a_float.isApprox(a.cast<float>()));
  }
  
  // Test scalar multiplication
  const double alpha = 1.5;
  Force b(Force::Random());
  Force alpha_f = alpha * b;
  Force f_alpha = b * alpha;
  
  BOOST_CHECK(alpha_f == f_alpha);
}

BOOST_AUTO_TEST_CASE (test_force_ref)
{
  using namespace pinocchio;
  typedef Force::Vector6 Vector6;

  typedef ForceRef<Vector6> ForceV6;
  
  Force f_ref(Force::Random());
  ForceV6 f(f_ref.toVector());
  
  BOOST_CHECK(f_ref.isApprox(f));
  
  ForceV6::ForcePlain f2(f*2.);
  Force f2_ref(f_ref*2.);
  
  BOOST_CHECK(f2_ref.isApprox(f2));
  
  f2 = f_ref + f;
  BOOST_CHECK(f2_ref.isApprox(f2));
  
  f = f2;
  BOOST_CHECK(f2.isApprox(f));
  
  f2 = f - f;
  BOOST_CHECK(f2.isApprox(Force::Zero()));
  
  SE3 M(SE3::Identity());
  f2 = M.act(f);
  BOOST_CHECK(f2.isApprox(f));
  
  f2 = M.actInv(f);
  BOOST_CHECK(f2.isApprox(f));
  
  Motion v(Motion::Random());
  f_ref.setRandom();
  f = f_ref;
  f2 = v.cross(f);
  f2_ref = v.cross(f_ref);
  
  BOOST_CHECK(f2.isApprox(f2_ref));
  
  f.setRandom();
  f.setZero();
  BOOST_CHECK(f.isApprox(Force::Zero()));
  
  // Test ref() method
  {
    Vector6 v6(Vector6::Random());
    ForceV6 a(v6);
    BOOST_CHECK(a.ref().isApprox(a));
    
    const Force b(a);
    BOOST_CHECK(b.isApprox(a.ref()));
  }
}

BOOST_AUTO_TEST_CASE ( test_Inertia )
{
  using namespace pinocchio;
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
  BOOST_CHECK(I1.matrix().isApprox(Matrix6::Identity()));

  // Test motion-to-force map
  Motion v = Motion::Random();
  Force f = I1 * v;
  BOOST_CHECK(f.toVector().isApprox(v.toVector()));
  
  // Test Inertia group application
  SE3 bma = SE3::Random(); 
  Inertia bI = bma.act(aI);
  Matrix6 bXa = bma;
  BOOST_CHECK((bma.rotation()*aI.inertia().matrix()*bma.rotation().transpose())
                               .isApprox(bI.inertia().matrix()));
  BOOST_CHECK((bXa.transpose().inverse() * aI.matrix() * bXa.inverse())
                              .isApprox(bI.matrix()));

  // Test inverse action
  BOOST_CHECK((bXa.transpose() * bI.matrix() * bXa)
                              .isApprox(bma.actInv(bI).matrix()));

  // Test vxIv cross product
  v = Motion::Random(); 
  f = aI*v;
  Force vxf = v.cross(f);
  Force vxIv = aI.vxiv(v);
  BOOST_CHECK(vxf.toVector().isApprox(vxIv.toVector()));

  // Test operator+
  I1 = Inertia::Random();
  Inertia I2 = Inertia::Random();
  BOOST_CHECK((I1.matrix()+I2.matrix()).isApprox((I1+I2).matrix()));

  // operator +=
  Inertia I12 = I1;
  I12 += I2;
  BOOST_CHECK((I1.matrix()+I2.matrix()).isApprox(I12.matrix()));
  
  // Test operator vtiv
  double kinetic_ref = v.toVector().transpose() * aI.matrix() * v.toVector();
  double kinetic = aI.vtiv(v);
  BOOST_CHECK_SMALL(kinetic_ref - kinetic, 1e-12);

  // Test constructor (Matrix6)
  Inertia I1_bis(I1.matrix());
  BOOST_CHECK(I1.matrix().isApprox(I1_bis.matrix()));
  
  // Test Inertia from ellipsoid
  const double sphere_mass = 5.;
  const double sphere_radius = 2.;
  I1 = Inertia::FromSphere(sphere_mass, sphere_radius);
  const double L_sphere = 2./5. * sphere_mass * sphere_radius * sphere_radius;
  BOOST_CHECK_SMALL(I1.mass() - sphere_mass, 1e-12);
  BOOST_CHECK(I1.lever().isZero());
  BOOST_CHECK(I1.inertia().matrix().isApprox(Symmetric3(L_sphere, 0., L_sphere , 0., 0., L_sphere).matrix()));

  // Test Inertia from ellipsoid
  I1 = Inertia::FromEllipsoid(2., 3., 4., 5.);
  BOOST_CHECK_SMALL(I1.mass() - 2, 1e-12);
  BOOST_CHECK_SMALL(I1.lever().norm(), 1e-12);
  BOOST_CHECK(I1.inertia().matrix().isApprox(Symmetric3(
          16.4, 0., 13.6, 0., 0., 10.).matrix()));

  // Test Inertia from Cylinder
  I1 = Inertia::FromCylinder(2., 4., 6.);
  BOOST_CHECK_SMALL(I1.mass() - 2, 1e-12);
  BOOST_CHECK_SMALL(I1.lever().norm(), 1e-12);
  BOOST_CHECK(I1.inertia().matrix().isApprox(Symmetric3(
        14., 0., 14., 0., 0., 16.).matrix()));

  // Test Inertia from Box
  I1 = Inertia::FromBox(2., 6., 12., 18.);
  BOOST_CHECK_SMALL(I1.mass() - 2, 1e-12);
  BOOST_CHECK_SMALL(I1.lever().norm(), 1e-12);
  BOOST_CHECK(I1.inertia().matrix().isApprox(Symmetric3(
        78., 0., 60., 0., 0., 30.).matrix()));
  
  // Copy operator
  Inertia aI_copy(aI);
  BOOST_CHECK(aI_copy == aI);
  
  // Test isZero
  Inertia I_not_zero = Inertia::Identity();
  BOOST_CHECK(!I_not_zero.isZero());
  
  Inertia I_zero = Inertia::Zero();
  BOOST_CHECK(I_zero.isZero());
  
  // Test isApprox
  const double eps = 1e-6;
  BOOST_CHECK(aI == aI);
  BOOST_CHECK(aI.isApprox(aI));
  Inertia aI_approx(aI);
  aI_approx.mass() += eps/2.;
  BOOST_CHECK(aI_approx.isApprox(aI,eps));
  
  // Test Variation
  Inertia::Matrix6 aIvariation = aI.variation(v);
  
  Motion::ActionMatrixType vAction = v.toActionMatrix();
  Motion::ActionMatrixType vDualAction = v.toDualActionMatrix();
  
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
  
  // Test variation against vxI - Ivx operator
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

  // Test dynamic parameters
  {
    Inertia I(Inertia::Random());

    Inertia::Vector10 v = I.toDynamicParameters();

    BOOST_CHECK_CLOSE(v[0], I.mass(), 1e-12);

    BOOST_CHECK(v.segment<3>(1).isApprox(I.mass()*I.lever()));

    Eigen::Matrix3d I_o = I.inertia() + I.mass()*skew(I.lever()).transpose()*skew(I.lever());
    Eigen::Matrix3d I_ov;
    I_ov << v[4], v[5], v[7],
            v[5], v[6], v[8],
            v[7], v[8], v[9];

    BOOST_CHECK(I_o.isApprox(I_ov));

    Inertia I2 = Inertia::FromDynamicParameters(v);
    BOOST_CHECK(I2.isApprox(I));

  }
  
  // Test disp
  std::cout << aI << std::endl;

}

BOOST_AUTO_TEST_CASE(cast_inertia)
{
  using namespace pinocchio;
  Inertia Y(Inertia::Random());
  
  BOOST_CHECK(Y.cast<double>() == Y);
  BOOST_CHECK(Y.cast<long double>().cast<double>() == Y);
}

BOOST_AUTO_TEST_CASE ( test_ActOnSet )
{
  using namespace pinocchio;
  const int N = 20;
  typedef Eigen::Matrix<double,6,N> Matrix6N;
  SE3 jMi = SE3::Random();
  Motion v = Motion::Random();

  // Forcet SET
  Matrix6N iF = Matrix6N::Random(),jF,jFinv,jF_ref,jFinv_ref;
  
  // forceSet::se3Action
  forceSet::se3Action(jMi,iF,jF);
  for( int k=0;k<N;++k )
    BOOST_CHECK(jMi.act(Force(iF.col(k))).toVector().isApprox(jF.col(k)));
  
  jF_ref = jMi.toDualActionMatrix()*iF;
  BOOST_CHECK(jF_ref.isApprox(jF));
  
  forceSet::se3ActionInverse(jMi.inverse(),iF,jFinv);
  BOOST_CHECK(jFinv.isApprox(jF));
  
  Matrix6N iF2 = Matrix6N::Random();
  jF_ref += jMi.toDualActionMatrix() * iF2;
  
  forceSet::se3Action<ADDTO>(jMi,iF2,jF);
  BOOST_CHECK(jF.isApprox(jF_ref));
  
  Matrix6N iF3 = Matrix6N::Random();
  jF_ref -= jMi.toDualActionMatrix() * iF3;
  
  forceSet::se3Action<RMTO>(jMi,iF3,jF);
  BOOST_CHECK(jF.isApprox(jF_ref));
  
  // forceSet::se3ActionInverse
  forceSet::se3ActionInverse(jMi,iF,jFinv);
  jFinv_ref = jMi.inverse().toDualActionMatrix() * iF;
  BOOST_CHECK(jFinv_ref.isApprox(jFinv));
  
  jFinv_ref += jMi.inverse().toDualActionMatrix() * iF2;
  forceSet::se3ActionInverse<ADDTO>(jMi,iF2,jFinv);
  BOOST_CHECK(jFinv.isApprox(jFinv_ref));
  
  jFinv_ref -= jMi.inverse().toDualActionMatrix() * iF3;
  forceSet::se3ActionInverse<RMTO>(jMi,iF3,jFinv);
  BOOST_CHECK(jFinv.isApprox(jFinv_ref));
  
  // forceSet::motionAction
  forceSet::motionAction(v,iF,jF);
  for( int k=0;k<N;++k )
    BOOST_CHECK(v.cross(Force(iF.col(k))).toVector().isApprox(jF.col(k)));
  
  jF_ref = v.toDualActionMatrix() * iF;
  BOOST_CHECK(jF.isApprox(jF_ref));
  
  jF_ref += v.toDualActionMatrix() * iF2;
  forceSet::motionAction<ADDTO>(v,iF2,jF);
  BOOST_CHECK(jF.isApprox(jF_ref));
  
  jF_ref -= v.toDualActionMatrix() * iF3;
  forceSet::motionAction<RMTO>(v,iF3,jF);
  BOOST_CHECK(jF.isApprox(jF_ref));
  
  // Motion SET
  Matrix6N iV = Matrix6N::Random(),jV,jV_ref,jVinv,jVinv_ref;
  
  // motionSet::se3Action
  motionSet::se3Action(jMi,iV,jV);
  for( int k=0;k<N;++k )
    BOOST_CHECK(jMi.act(Motion(iV.col(k))).toVector().isApprox(jV.col(k)));
  
  jV_ref = jMi.toActionMatrix()*iV;
  BOOST_CHECK(jV.isApprox(jV_ref));
  
  motionSet::se3ActionInverse(jMi.inverse(),iV,jVinv);
  BOOST_CHECK(jVinv.isApprox(jV));
  
  Matrix6N iV2 = Matrix6N::Random();
  jV_ref += jMi.toActionMatrix()*iV2;
  motionSet::se3Action<ADDTO>(jMi,iV2,jV);
  BOOST_CHECK(jV.isApprox(jV_ref));
  
  Matrix6N iV3 = Matrix6N::Random();
  jV_ref -= jMi.toActionMatrix()*iV3;
  motionSet::se3Action<RMTO>(jMi,iV3,jV);
  BOOST_CHECK(jV.isApprox(jV_ref));
  
  // motionSet::se3ActionInverse
  motionSet::se3ActionInverse(jMi,iV,jVinv);
  jVinv_ref = jMi.inverse().toActionMatrix() * iV;
  BOOST_CHECK(jVinv.isApprox(jVinv_ref));
  
  jVinv_ref += jMi.inverse().toActionMatrix()*iV2;
  motionSet::se3ActionInverse<ADDTO>(jMi,iV2,jVinv);
  BOOST_CHECK(jVinv.isApprox(jVinv_ref));
  
  jVinv_ref -= jMi.inverse().toActionMatrix()*iV3;
  motionSet::se3ActionInverse<RMTO>(jMi,iV3,jVinv);
  BOOST_CHECK(jVinv.isApprox(jVinv_ref));
  
  // motionSet::motionAction
  motionSet::motionAction(v,iV,jV);
  for( int k=0;k<N;++k )
    BOOST_CHECK(v.cross(Motion(iV.col(k))).toVector().isApprox(jV.col(k)));
  
  jV_ref = v.toActionMatrix()*iV;
  BOOST_CHECK(jV.isApprox(jV_ref));
  
  jV_ref += v.toActionMatrix()*iV2;
  motionSet::motionAction<ADDTO>(v,iV2,jV);
  BOOST_CHECK(jV.isApprox(jV_ref));
  
  jV_ref -= v.toActionMatrix()*iV3;
  motionSet::motionAction<RMTO>(v,iV3,jV);
  BOOST_CHECK(jV.isApprox(jV_ref));
  
  // motionSet::inertiaAction
  const Inertia I(Inertia::Random());
  motionSet::inertiaAction(I,iV,jV);
  for( int k=0;k<N;++k )
    BOOST_CHECK((I*(Motion(iV.col(k)))).toVector().isApprox(jV.col(k)));
  
  jV_ref = I.matrix()*iV;
  BOOST_CHECK(jV.isApprox(jV_ref));
  
  jV_ref += I.matrix()*iV2;
  motionSet::inertiaAction<ADDTO>(I,iV2,jV);
  BOOST_CHECK(jV.isApprox(jV_ref));
  
  jV_ref -= I.matrix()*iV3;
  motionSet::inertiaAction<RMTO>(I,iV3,jV);
  BOOST_CHECK(jV.isApprox(jV_ref));
 
  // motionSet::act
  Force f = Force::Random();
  motionSet::act(iV,f,jF);
  for( int k=0;k<N;++k )
    BOOST_CHECK(Motion(iV.col(k)).cross(f).toVector().isApprox(jF.col(k)));
  
  for( int k=0;k<N;++k )
    jF_ref.col(k) = Force(Motion(iV.col(k)).cross(f)).toVector();
  BOOST_CHECK(jF.isApprox(jF_ref));
  
  for( int k=0;k<N;++k )
    jF_ref.col(k) += Force(Motion(iV2.col(k)).cross(f)).toVector();
  motionSet::act<ADDTO>(iV2,f,jF);
  BOOST_CHECK(jF.isApprox(jF_ref));
  
  for( int k=0;k<N;++k )
    jF_ref.col(k) -= Force(Motion(iV3.col(k)).cross(f)).toVector();
  motionSet::act<RMTO>(iV3,f,jF);
  BOOST_CHECK(jF.isApprox(jF_ref));
}

BOOST_AUTO_TEST_CASE(test_skew)
{
  using namespace pinocchio;
  typedef SE3::Vector3 Vector3;
  typedef Motion::Vector6 Vector6;
  
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

BOOST_AUTO_TEST_CASE(test_addSkew)
{
  using namespace pinocchio;
  typedef SE3::Vector3 Vector3;
  typedef SE3::Matrix3 Matrix3;
  
  Vector3 v(Vector3::Random());
  Matrix3 M(Matrix3::Random());
  Matrix3 Mcopy(M);
  
  addSkew(v,M);
  Matrix3 Mref = Mcopy + skew(v);
  BOOST_CHECK(M.isApprox(Mref));
  
  Mref += skew(-v);
  addSkew(-v,M);
  BOOST_CHECK(M.isApprox(Mcopy));
  
  M.setZero();
  addSkew(v,M);
  BOOST_CHECK(M.isApprox(skew(v)));
}

BOOST_AUTO_TEST_CASE(test_skew_square)
{
  using namespace pinocchio;
  typedef SE3::Vector3 Vector3;
  typedef SE3::Matrix3 Matrix3;
  
  Vector3 u(Vector3::Random());
  Vector3 v(Vector3::Random());
  
  Matrix3 ref = skew(u) * skew(v);
  
  Matrix3 res = skewSquare(u,v);
  
  BOOST_CHECK(res.isApprox(ref));
}

template<int axis>
struct test_scalar_multiplication_cartesian_axis
{
  typedef pinocchio::CartesianAxis<axis> Axis;
  typedef double Scalar;
  typedef Eigen::Matrix<Scalar,3,1> Vector3;
  
  static void run()
  {
    const Scalar alpha = static_cast <Scalar> (rand()) / static_cast <Scalar> (RAND_MAX);
    const Vector3 r1 = Axis() * alpha;
    const Vector3 r2 = alpha * Axis();
    
    BOOST_CHECK(r1.isApprox(r2));
    
    for(int k = 0; k < Axis::dim; ++k)
    {
      if(k==axis)
      {
        BOOST_CHECK(r1[k] == alpha);
        BOOST_CHECK(r2[k] == alpha);
      }
      else
      {
        BOOST_CHECK(r1[k] == Scalar(0));
        BOOST_CHECK(r2[k] == Scalar(0));
      }
    }
  }
};

BOOST_AUTO_TEST_CASE(test_cartesian_axis)
{
  using namespace Eigen;
  using namespace pinocchio;
  Vector3d v(Vector3d::Random());
  const double alpha = 3;
  Vector3d v2(alpha*v);
  
  BOOST_CHECK(AxisX::cross(v).isApprox(Vector3d::Unit(0).cross(v)));
  BOOST_CHECK(AxisY::cross(v).isApprox(Vector3d::Unit(1).cross(v)));
  BOOST_CHECK(AxisZ::cross(v).isApprox(Vector3d::Unit(2).cross(v)));
  BOOST_CHECK(AxisX::alphaCross(alpha,v).isApprox(Vector3d::Unit(0).cross(v2)));
  BOOST_CHECK(AxisY::alphaCross(alpha,v).isApprox(Vector3d::Unit(1).cross(v2)));
  BOOST_CHECK(AxisZ::alphaCross(alpha,v).isApprox(Vector3d::Unit(2).cross(v2)));
  
  test_scalar_multiplication_cartesian_axis<0>::run();
  test_scalar_multiplication_cartesian_axis<1>::run();
  test_scalar_multiplication_cartesian_axis<2>::run();
}

template<int axis>
struct test_scalar_multiplication
{
  typedef pinocchio::SpatialAxis<axis> Axis;
  typedef double Scalar;
  typedef pinocchio::MotionTpl<Scalar> Motion;
  
  static void run()
  {
    const Scalar alpha = static_cast <Scalar> (rand()) / static_cast <Scalar> (RAND_MAX);
    const Motion r1 = Axis() * alpha;
    const Motion r2 = alpha * Axis();
    
    BOOST_CHECK(r1.isApprox(r2));
    
    for(int k = 0; k < Axis::dim; ++k)
    {
      if(k==axis)
      {
        BOOST_CHECK(r1.toVector()[k] == alpha);
        BOOST_CHECK(r2.toVector()[k] == alpha);
      }
      else
      {
        BOOST_CHECK(r1.toVector()[k] == Scalar(0));
        BOOST_CHECK(r2.toVector()[k] == Scalar(0));
      }
    }
  }
};

BOOST_AUTO_TEST_CASE(test_spatial_axis)
{
  using namespace pinocchio;
  
  Motion v(Motion::Random());
  Force f(Force::Random());

  Motion vaxis;
  vaxis << AxisVX();
  BOOST_CHECK(AxisVX::cross(v).isApprox(vaxis.cross(v)));
  BOOST_CHECK(v.cross(AxisVX()).isApprox(v.cross(vaxis)));
  BOOST_CHECK(AxisVX::cross(f).isApprox(vaxis.cross(f)));
  
  vaxis << AxisVY();
  BOOST_CHECK(AxisVY::cross(v).isApprox(vaxis.cross(v)));
  BOOST_CHECK(v.cross(AxisVY()).isApprox(v.cross(vaxis)));
  BOOST_CHECK(AxisVY::cross(f).isApprox(vaxis.cross(f)));
  
  vaxis << AxisVZ();
  BOOST_CHECK(AxisVZ::cross(v).isApprox(vaxis.cross(v)));
  BOOST_CHECK(v.cross(AxisVZ()).isApprox(v.cross(vaxis)));
  BOOST_CHECK(AxisVZ::cross(f).isApprox(vaxis.cross(f)));
  
  vaxis << AxisWX();
  BOOST_CHECK(AxisWX::cross(v).isApprox(vaxis.cross(v)));
  BOOST_CHECK(v.cross(AxisWX()).isApprox(v.cross(vaxis)));
  BOOST_CHECK(AxisWX::cross(f).isApprox(vaxis.cross(f)));
  
  vaxis << AxisWY();
  BOOST_CHECK(AxisWY::cross(v).isApprox(vaxis.cross(v)));
  BOOST_CHECK(v.cross(AxisWY()).isApprox(v.cross(vaxis)));
  BOOST_CHECK(AxisWY::cross(f).isApprox(vaxis.cross(f)));
  
  vaxis << AxisWZ();
  BOOST_CHECK(AxisWZ::cross(v).isApprox(vaxis.cross(v)));
  BOOST_CHECK(v.cross(AxisWZ()).isApprox(v.cross(vaxis)));
  BOOST_CHECK(AxisWZ::cross(f).isApprox(vaxis.cross(f)));
  
  // Test operation Axis * Scalar
  test_scalar_multiplication<0>::run();
  test_scalar_multiplication<1>::run();
  test_scalar_multiplication<2>::run();
  test_scalar_multiplication<3>::run();
  test_scalar_multiplication<4>::run();
  test_scalar_multiplication<5>::run();
  
  // Operations of Constraint on forces Sxf
  typedef Motion::ActionMatrixType ActionMatrixType;
  typedef ActionMatrixType::ColXpr ColType;
  typedef ForceRef<ColType> ForceRefOnColType;
  typedef MotionRef<ColType> MotionRefOnColType;
  ActionMatrixType Sxf,Sxf_ref;
  ActionMatrixType S(ActionMatrixType::Identity());
  
  SpatialAxis<0>::cross(f,ForceRefOnColType(Sxf.col(0)));
  SpatialAxis<1>::cross(f,ForceRefOnColType(Sxf.col(1)));
  SpatialAxis<2>::cross(f,ForceRefOnColType(Sxf.col(2)));
  SpatialAxis<3>::cross(f,ForceRefOnColType(Sxf.col(3)));
  SpatialAxis<4>::cross(f,ForceRefOnColType(Sxf.col(4)));
  SpatialAxis<5>::cross(f,ForceRefOnColType(Sxf.col(5)));

  for(int k = 0; k < 6; ++k)
  {
    MotionRefOnColType Scol(S.col(k));
    ForceRefOnColType(Sxf_ref.col(k)) = Scol.cross(f);
  }
  
  BOOST_CHECK(Sxf.isApprox(Sxf_ref));
}

BOOST_AUTO_TEST_SUITE_END ()
