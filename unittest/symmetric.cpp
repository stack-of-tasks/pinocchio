//
// Copyright (c) 2015-2016,2018 CNRS
//

/* --- Unitary test symmetric.cpp This code tests and compares two ways of
 * expressing symmetric matrices. In addition to the unitary validation (test
 * of the basic operations), the code is validating the computation
 * performances of each methods.
 *
 * The three methods are:
 * - Eigen SelfAdjoint (a mask atop of a classical dense matrix) ==> the least efficient.
 * - Pinocchio rewritting of Metapod code with LTI factor as well and minor improvement.
 *
 * IMPORTANT: the following timings seems outdated.
 * Expected time scores on a I7 2.1GHz:
 * - Eigen: 2.5us
 * - Pinocchio: 6us
 */

#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/skew.hpp"
#include "pinocchio/utils/timer.hpp"

#include <boost/random.hpp>
#include <Eigen/Geometry>

#include "pinocchio/spatial/symmetric3.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>


#include <Eigen/StdVector>
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(pinocchio::Symmetric3)

void timeSym3(const pinocchio::Symmetric3 & S,
        const pinocchio::Symmetric3::Matrix3 & R,
        pinocchio::Symmetric3 & res)
{
  res = S.rotate(R);
}

#ifdef WITH_METAPOD

#include <metapod/tools/spatial/lti.hh>
#include <metapod/tools/spatial/rm-general.hh>

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(metapod::Spatial::ltI<double>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(metapod::Spatial::RotationMatrixTpl<double>)

void timeLTI(const metapod::Spatial::ltI<double>& S,
       const metapod::Spatial::RotationMatrixTpl<double>& R, 
       metapod::Spatial::ltI<double> & res)
{
  res = R.rotTSymmetricMatrix(S);
}

#endif

void timeSelfAdj( const Eigen::Matrix3d & A,
      const Eigen::Matrix3d & Sdense,
      Eigen::Matrix3d & ASA )
{
  typedef Eigen::SelfAdjointView<const Eigen::Matrix3d,Eigen::Upper> Sym3;
  Sym3 S(Sdense);
  ASA.triangularView<Eigen::Upper>()
    = A * S * A.transpose();
}

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

/* --- PINOCCHIO ------------------------------------------------------------ */
/* --- PINOCCHIO ------------------------------------------------------------ */
/* --- PINOCCHIO ------------------------------------------------------------ */
BOOST_AUTO_TEST_CASE ( test_pinocchio_Sym3 )
{
  using namespace pinocchio;
  typedef Symmetric3::Matrix3 Matrix3;
  typedef Symmetric3::Vector3 Vector3;
  
  { 
    // op(Matrix3)
    {
      Matrix3 M = Matrix3::Random(); M = M*M.transpose();
      Symmetric3 S(M);
      BOOST_CHECK(S.matrix().isApprox(M, 1e-12));
    }
    
    // S += S
    {
      Symmetric3
      S = Symmetric3::Random(),
      S2 = Symmetric3::Random();
      Symmetric3 Scopy = S;
      S+=S2;
      BOOST_CHECK(S.matrix().isApprox(S2.matrix()+Scopy.matrix(), 1e-12));
    }

    // S + M
    {
      Symmetric3 S = Symmetric3::Random();
      Matrix3 M = Matrix3::Random(); M = M*M.transpose();

      Symmetric3 S2 = S + M;
      BOOST_CHECK(S2.matrix().isApprox(S.matrix()+M, 1e-12));

      S2 = S - M;
      BOOST_CHECK(S2.matrix().isApprox(S.matrix()-M, 1e-12));
    }

    // S*v
    {
      Symmetric3 S = Symmetric3::Random();
      Vector3 v = Vector3::Random(); 
      Vector3 Sv = S*v;
      BOOST_CHECK(Sv.isApprox(S.matrix()*v, 1e-12));
    }

    // Random
    for(int i=0;i<100;++i )
    {
      Matrix3 M = Matrix3::Random(); M = M*M.transpose();
      Symmetric3 S = Symmetric3::RandomPositive();
      Vector3 v = Vector3::Random();
      BOOST_CHECK_GT( (v.transpose()*(S*v))[0] , 0);
    }

    // Identity
    { 
      BOOST_CHECK(Symmetric3::Identity().matrix().isApprox(Matrix3::Identity(), 1e-12));
    }

    // Skew2
    {
      Vector3 v = Vector3::Random();
      Symmetric3 vxvx = Symmetric3::SkewSquare(v);

      Vector3 p = Vector3::UnitX();
      BOOST_CHECK((vxvx*p).isApprox(v.cross(v.cross(p)), 1e-12));

      p = Vector3::UnitY();
      BOOST_CHECK((vxvx*p).isApprox(v.cross(v.cross(p)), 1e-12));

      p = Vector3::UnitZ();
      BOOST_CHECK((vxvx*p).isApprox(v.cross(v.cross(p)), 1e-12));

      Matrix3 vx = skew(v);
      Matrix3 vxvx2 = (vx*vx).eval();
      BOOST_CHECK(vxvx.matrix().isApprox(vxvx2, 1e-12));

      Symmetric3 S = Symmetric3::RandomPositive();
      BOOST_CHECK((S-Symmetric3::SkewSquare(v)).matrix()
                                        .isApprox(S.matrix()-vxvx2, 1e-12));

      double m = Eigen::internal::random<double>()+1;
      BOOST_CHECK((S-m*Symmetric3::SkewSquare(v)).matrix()
                                        .isApprox(S.matrix()-m*vxvx2, 1e-12));


      Symmetric3 S2 = S;
      S -= Symmetric3::SkewSquare(v);
      BOOST_CHECK(S.matrix().isApprox(S2.matrix()-vxvx2, 1e-12));

      S = S2; S -= m*Symmetric3::SkewSquare(v);
      BOOST_CHECK(S.matrix().isApprox(S2.matrix()-m*vxvx2, 1e-12));

    }

    // (i,j)
    {
      Matrix3 M = Matrix3::Random(); M = M*M.transpose();
      Symmetric3 S(M);
      for(int i=0;i<3;++i)
        for(int j=0;j<3;++j)
          BOOST_CHECK_SMALL(S(i,j) - M(i,j), Eigen::NumTraits<double>::dummy_precision());
      }
    }

    // SRS
    {
      Symmetric3 S = Symmetric3::RandomPositive();
      Matrix3 R = (Eigen::Quaterniond(Eigen::Matrix<double,4,1>::Random())).normalized().matrix();
      
      Symmetric3 RSRt = S.rotate(R);
      BOOST_CHECK(RSRt.matrix().isApprox(R*S.matrix()*R.transpose(), 1e-12));

      Symmetric3 RtSR = S.rotate(R.transpose());
      BOOST_CHECK(RtSR.matrix().isApprox(R.transpose()*S.matrix()*R, 1e-12));
    }
  
  // Test operator vtiv
  {
    Symmetric3 S = Symmetric3::RandomPositive();
    Vector3 v = Vector3::Random();
    double kinetic_ref = v.transpose() * S.matrix() * v;
    double kinetic = S.vtiv(v);
    BOOST_CHECK_SMALL(kinetic_ref - kinetic, 1e-12);
  }
  
  // Test v x S3
  {
    Symmetric3 S = Symmetric3::RandomPositive();
    Vector3 v = Vector3::Random();
    Matrix3 Vcross = skew(v);
    Matrix3 M_ref(Vcross * S.matrix());
    
    Matrix3 M_res;
    Symmetric3::vxs(v,S,M_res);
    BOOST_CHECK(M_res.isApprox(M_ref));
    
    BOOST_CHECK(S.vxs(v).isApprox(M_ref));
  }
  
  // Test S3 vx
  {
    Symmetric3 S = Symmetric3::RandomPositive();
    Vector3 v = Vector3::Random();
    Matrix3 Vcross = skew(v);
    Matrix3 M_ref(S.matrix() * Vcross);
    
    Matrix3 M_res;
    Symmetric3::svx(v,S,M_res);
    BOOST_CHECK(M_res.isApprox(M_ref));
    
    BOOST_CHECK(S.svx(v).isApprox(M_ref));
  }
  
  // Test isZero
  {
    Symmetric3 S_not_zero = Symmetric3::Identity();
    BOOST_CHECK(!S_not_zero.isZero());
    
    Symmetric3 S_zero = Symmetric3::Zero();
    BOOST_CHECK(S_zero.isZero());
  }
  
  // Test isApprox
  {
    Symmetric3 S1 = Symmetric3::RandomPositive();
    Symmetric3 S2 = S1;
    
    BOOST_CHECK(S1.isApprox(S2));
    
    Symmetric3 S3 = S1;
    S3 += S3;
    BOOST_CHECK(!S1.isApprox(S3));
  }

    // Time test
    {
      const size_t NBT = 100000;
      Symmetric3 S = Symmetric3::RandomPositive();

      std::vector<Symmetric3> Sres (NBT);
      std::vector<Matrix3> Rs (NBT);
      for(size_t i=0;i<NBT;++i) 
        Rs[i] = (Eigen::Quaterniond(Eigen::Matrix<double,4,1>::Random())).normalized().matrix();

      std::cout << "Pinocchio: ";
      PinocchioTicToc timer(PinocchioTicToc::US); timer.tic();
      SMOOTH(NBT)
      {
        timeSym3(S,Rs[_smooth],Sres[_smooth]);
      }
      timer.toc(std::cout,NBT);
    }
}

/* --- EIGEN SYMMETRIC ------------------------------------------------------ */
/* --- EIGEN SYMMETRIC ------------------------------------------------------ */
/* --- EIGEN SYMMETRIC ------------------------------------------------------ */

BOOST_AUTO_TEST_CASE ( test_eigen_SelfAdj )
{
  using namespace pinocchio;
  typedef Eigen::Matrix3d Matrix3;
  typedef Eigen::SelfAdjointView<Matrix3,Eigen::Upper> Sym3;

  Matrix3 M = Matrix3::Random();
  Sym3 S(M);
  {
    Matrix3 Scp = S;
    BOOST_CHECK((Scp-Scp.transpose()).isApprox(Matrix3::Zero(), 1e-16));
  }

  Matrix3 M2 = Matrix3::Random();
  M.triangularView<Eigen::Upper>() = M2;

  Matrix3 A = Matrix3::Random(), ASA1, ASA2;
  ASA1.triangularView<Eigen::Upper>() = A * S * A.transpose();
  timeSelfAdj(A,M,ASA2);

  {
    Matrix3 Masa1 = ASA1.selfadjointView<Eigen::Upper>();
    Matrix3 Masa2 = ASA2.selfadjointView<Eigen::Upper>();
    BOOST_CHECK(Masa1.isApprox(Masa2, 1e-16));
  }

  const size_t NBT = 100000;
  std::vector<Eigen::Matrix3d> Sres (NBT);
  std::vector<Eigen::Matrix3d> Rs (NBT);
  for(size_t i=0;i<NBT;++i) 
    Rs[i] = (Eigen::Quaterniond(Eigen::Matrix<double,4,1>::Random())).normalized().matrix();

  std::cout << "Eigen: ";
  PinocchioTicToc timer(PinocchioTicToc::US); timer.tic();
  SMOOTH(NBT)
  {
    timeSelfAdj(Rs[_smooth],M,Sres[_smooth]);
  }
  timer.toc(std::cout,NBT);
}

BOOST_AUTO_TEST_CASE(comparison)
{
  using namespace pinocchio;
  Symmetric3 sym1(Symmetric3::Random());
  
  Symmetric3 sym2(sym1);
  sym2.data() *= 2;
  
  BOOST_CHECK(sym2 != sym1);
  BOOST_CHECK(sym1 == sym1);
}

BOOST_AUTO_TEST_CASE(cast)
{
  using namespace pinocchio;
  Symmetric3 sym(Symmetric3::Random());
  
  BOOST_CHECK(sym.cast<double>() == sym);
  BOOST_CHECK(sym.cast<long double>().cast<double>() == sym);
  
}
BOOST_AUTO_TEST_SUITE_END ()

