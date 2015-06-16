//
// Copyright (c) 2015 CNRS
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

/* --- Unitary test symmetric.cpp This code tests and compares three ways of
 * expressing symmetric matrices. In addition to the unitary validation (test
 * of the basic operations), the code is validating the computation
 * performances of each methods.
 *
 * The three methods are:
 * - Eigen SelfAdjoint (a mask atop of a classical dense matrix) ==> the least efficient.
 * - Metapod Symmetric with LTI factorization.
 * - Pinocchio rewritting of Metapod code with LTI factor as well and minor improvement.
 *
 * Expected time scores on a I7 2.1GHz:
 * - Eigen: 2.5us
 * - Metapod: 4us
 * - Pinocchio: 6us
 */

#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/tools/timer.hpp"

#include <boost/random.hpp>



#include "pinocchio/spatial/symmetric3.hpp"

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE symmetricTest
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>
#include "pinocchio/tools/matrix-comparison.hpp"


#include <Eigen/StdVector>
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(se3::Symmetric3)

void timeSym3(const se3::Symmetric3 & S,
        const se3::Symmetric3::Matrix3 & R,
        se3::Symmetric3 & res)
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

BOOST_AUTO_TEST_SUITE ( symmetricTest)

/* --- PINOCCHIO ------------------------------------------------------------ */
/* --- PINOCCHIO ------------------------------------------------------------ */
/* --- PINOCCHIO ------------------------------------------------------------ */
BOOST_AUTO_TEST_CASE ( test_pinocchio_Sym3 )
{
  using namespace se3;
  typedef Symmetric3::Matrix3 Matrix3;
  typedef Symmetric3::Vector3 Vector3;
  
  { 
    // op(Matrix3)
    {
      Matrix3 M = Matrix3::Random(); M = M*M.transpose();
      Symmetric3 S(M);
      BOOST_CHECK( S.matrix().isApprox(M,1e-12) );
    }
    
    // S += S
    {
      Symmetric3
      S = Symmetric3::Random(),
      S2 = Symmetric3::Random();
      Symmetric3 Scopy = S;
      S+=S2;
      BOOST_CHECK( S.matrix().isApprox(S2.matrix()+Scopy.matrix(),1e-12) );
    }

    // S + M
    {
      Symmetric3 S = Symmetric3::Random();
      Matrix3 M = Matrix3::Random(); M = M*M.transpose();

      Symmetric3 S2 = S + M;
      BOOST_CHECK( S2.matrix().isApprox(S.matrix()+M,1e-12) );

      S2 = S - M;
      BOOST_CHECK( S2.matrix().isApprox(S.matrix()-M,1e-12) );
    }

    // S*v
    {
      Symmetric3 S = Symmetric3::Random();
      Vector3 v = Vector3::Random(); 
      Vector3 Sv = S*v;
      BOOST_CHECK( Sv.isApprox(S.matrix()*v,1e-12) );
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
      BOOST_CHECK( Symmetric3::Identity().matrix().isApprox(Matrix3::Identity(),1e-12) );
    }

    // Skew2
    {
      Vector3 v = Vector3::Random();
      Symmetric3 vxvx = Symmetric3::SkewSquare(v);

      Vector3 p = Vector3::UnitX();
      BOOST_CHECK( (vxvx*p).isApprox(v.cross(v.cross(p)),1e-12) );

      p = Vector3::UnitY();
      BOOST_CHECK( (vxvx*p).isApprox(v.cross(v.cross(p)),1e-12) );

      p = Vector3::UnitZ();
      BOOST_CHECK( (vxvx*p).isApprox(v.cross(v.cross(p)),1e-12) );

      Matrix3 vx = skew(v);
      Matrix3 vxvx2 = (vx*vx).eval();
      BOOST_CHECK( vxvx.matrix().isApprox(vxvx2,1e-12) );

      Symmetric3 S = Symmetric3::RandomPositive();
      BOOST_CHECK( (S-Symmetric3::SkewSquare(v)).matrix().isApprox(
                                    S.matrix()-vxvx2,1e-12) );

      double m = Eigen::internal::random<double>()+1;
      BOOST_CHECK( (S-m*Symmetric3::SkewSquare(v)).matrix().isApprox(
                                    S.matrix()-m*vxvx2,1e-12) );


      Symmetric3 S2 = S;
      S -= Symmetric3::SkewSquare(v);
      BOOST_CHECK( S.matrix().isApprox(S2.matrix()-vxvx2,1e-12) );

      S = S2; S -= m*Symmetric3::SkewSquare(v);
      BOOST_CHECK( S.matrix().isApprox(S2.matrix()-m*vxvx2,1e-12) );

    }

    // (i,j)
    {
      Matrix3 M = Matrix3::Random(); M = M*M.transpose();
      Symmetric3 S(M);
      for(int i=0;i<3;++i)
        for(int j=0;j<3;++j)
          BOOST_CHECK_EQUAL(S(i,j), M(i,j) );
      }
    }

    // SRS
    {
      Symmetric3 S = Symmetric3::RandomPositive();
      Matrix3 R = (Eigen::Quaterniond(Eigen::Matrix<double,4,1>::Random())).normalized().matrix();
      
      Symmetric3 RSRt = S.rotate(R);
      BOOST_CHECK( RSRt.matrix().isApprox(R*S.matrix()*R.transpose(),1e-12) );

      Symmetric3 RtSR = S.rotate(R.transpose());
      BOOST_CHECK( RtSR.matrix().isApprox(R.transpose()*S.matrix()*R,1e-12) );
    }

    // Time test 
    {
      const int NBT = 100000;
      Symmetric3 S = Symmetric3::RandomPositive();

      std::vector<Symmetric3> Sres (NBT);
      std::vector<Matrix3> Rs (NBT);
      for(int i=0;i<NBT;++i) 
        Rs[(std::size_t)i] = (Eigen::Quaterniond(Eigen::Matrix<double,4,1>::Random())).normalized().matrix();

      std::cout << "Pinocchio: ";
      StackTicToc timer(StackTicToc::US); timer.tic();
      SMOOTH(NBT)
      {
        timeSym3(S,Rs[(std::size_t)_smooth],Sres[(std::size_t)_smooth]);
      }
      timer.toc(std::cout,NBT);
    }
}

/* --- METAPOD -------------------------------------------------------------- */
/* --- METAPOD -------------------------------------------------------------- */
/* --- METAPOD -------------------------------------------------------------- */

BOOST_AUTO_TEST_CASE ( test_metapod_LTI )
{
#ifdef WITH_METAPOD
  using namespace metapod::Spatial;

  typedef ltI<double> Sym3;
  typedef Eigen::Matrix3d Matrix3;
  typedef RotationMatrixTpl<double> R3;

  Matrix3 M = Matrix3::Random();
  Sym3 S(M),S2;

  R3 R; R.randomInit();

  R.rotTSymmetricMatrix(S);
  timeLTI(S,R,S2);
  BOOST_CHECK( S2.toMatrix().isApprox(R.toMatrix().transpose()*S.toMatrix()*R.toMatrix(),1e-12) );
  
  const int NBT = 100000;
  std::vector<Sym3> Sres (NBT);
  std::vector<R3> Rs (NBT);
  for(int i=0;i<NBT;++i) 
    Rs[i].randomInit();
  
  std::cout << "Metapod: ";
  StackTicToc timer(StackTicToc::US); timer.tic();
  SMOOTH(NBT)
  {
    timeLTI(S, Rs[_smooth], Sres[_smooth]);
  }
  timer.toc(std::cout,NBT);
#else
  std::cout << "Metapod is not installed ... skipping this test. " << std::endl;
#endif
}

/* --- EIGEN SYMMETRIC ------------------------------------------------------ */
/* --- EIGEN SYMMETRIC ------------------------------------------------------ */
/* --- EIGEN SYMMETRIC ------------------------------------------------------ */

BOOST_AUTO_TEST_CASE ( test_eigen_SelfAdj )
{
  using namespace se3;
  typedef Eigen::Matrix3d Matrix3;
  typedef Eigen::SelfAdjointView<Matrix3,Eigen::Upper> Sym3;

  Matrix3 M = Matrix3::Random();
  Sym3 S(M);
  {
    Matrix3 Scp = S;
    BOOST_CHECK( (Scp-Scp.transpose()).isApprox(Matrix3::Zero(),1e-16) );
  }

  Matrix3 M2 = Matrix3::Random();
  M.triangularView<Eigen::Upper>() = M2;

  Matrix3 A = Matrix3::Random(), ASA1, ASA2;
  ASA1.triangularView<Eigen::Upper>() = A * S * A.transpose();
  timeSelfAdj(A,M,ASA2);

  {
    Matrix3 Masa1 = ASA1.selfadjointView<Eigen::Upper>();
    Matrix3 Masa2 = ASA2.selfadjointView<Eigen::Upper>();
    BOOST_CHECK( Masa1.isApprox(Masa2,1e-16) );
  }

  const int NBT = 100000;
  std::vector<Eigen::Matrix3d> Sres (NBT);
  std::vector<Eigen::Matrix3d> Rs (NBT);
  for(int i=0;i<NBT;++i) 
    Rs[(std::size_t)i] = (Eigen::Quaterniond(Eigen::Matrix<double,4,1>::Random())).normalized().matrix();

  std::cout << "Eigen: ";
  StackTicToc timer(StackTicToc::US); timer.tic();
  SMOOTH(NBT)
  {
    timeSelfAdj(Rs[(std::size_t)_smooth],M,Sres[(std::size_t)_smooth]);
  }
  timer.toc(std::cout,NBT);
}
BOOST_AUTO_TEST_SUITE_END ()

