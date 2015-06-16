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

/*
 * Validate the sparse Cholesky decomposition of the mass matrix.  The code
 * tests both the numerical value and the computation time. For a strong
 * computation benchmark, see benchmark/timings.
 *
 */

#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/cholesky.hpp"
#include "pinocchio/multibody/parser/sample-models.hpp"
#include "pinocchio/tools/timer.hpp"

#include <iostream>
#ifdef NDEBUG
#  include <Eigen/Cholesky>
#endif

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE CholeskyTest
#include <boost/test/unit_test.hpp>
#include "pinocchio/tools/matrix-comparison.hpp"
#include <boost/utility/binary.hpp>


BOOST_AUTO_TEST_SUITE ( CholeskyTest)

BOOST_AUTO_TEST_CASE ( test_cholesky )
{
  using namespace Eigen;
  using namespace se3;

  se3::Model model;
  se3::buildModels::humanoidSimple(model,true);
  se3::Data data(model);

  VectorXd q = VectorXd::Zero(model.nq);
  data.M.fill(0); // Only nonzero coeff of M are initialized by CRBA.
  crba(model,data,q);
 
  se3::cholesky::decompose(model,data);
  data.M.triangularView<Eigen::StrictlyLower>() = 
  data.M.triangularView<Eigen::StrictlyUpper>().transpose();
  
  const Eigen::MatrixXd & U = data.U;
  const Eigen::VectorXd & D = data.D;
  const Eigen::MatrixXd & M = data.M;

  #ifndef NDEBUG
    std::cout << "M = [\n" << M << "];" << std::endl;
    std::cout << "U = [\n" << U << "];" << std::endl;
    std::cout << "D = [\n" << D.transpose() << "];" << std::endl;
  #endif
      
  BOOST_CHECK( M.isApprox(U*D.asDiagonal()*U.transpose() ,1e-12) );

  Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
// std::cout << "v = [" << v.transpose() << "]';" << std::endl;

  Eigen::VectorXd Uv = v; se3::cholesky::Uv(model,data,Uv);
  BOOST_CHECK( Uv.isApprox(U*v,1e-12) );

  Eigen::VectorXd Utv = v; se3::cholesky::Utv(model,data,Utv);
  BOOST_CHECK( Utv.isApprox(U.transpose()*v,1e-12) );

  Eigen::VectorXd Uiv = v; se3::cholesky::Uiv(model,data,Uiv);
  BOOST_CHECK( Uiv.isApprox(U.inverse()*v,1e-12) );


  Eigen::VectorXd Utiv = v; se3::cholesky::Utiv(model,data,Utiv);
  BOOST_CHECK( Utiv.isApprox(U.transpose().inverse()*v,1e-12) );

  Eigen::VectorXd Miv = v; se3::cholesky::solve(model,data,Miv);
  BOOST_CHECK( Miv.isApprox(M.inverse()*v,1e-12) );

  Eigen::VectorXd Mv = v; se3::cholesky::Mv(model,data,Mv,true);
  BOOST_CHECK( Mv.isApprox(M*v,1e-12) );
  Mv = v;                 se3::cholesky::Mv(model,data,Mv,false);
  BOOST_CHECK( Mv.isApprox(M*v,1e-12) );
}


/* The flag triger the following timers:
 * 000001: sparse UDUt cholesky
 * 000010: dense Eigen LDLt cholesky (with pivot)
 * 000100: sparse resolution 
 * 001000: sparse U*v multiplication
 * 010000: sparse U\v substitution
 * 100000: sparse M*v multiplication without Cholesky
 */
BOOST_AUTO_TEST_CASE ( test_timings )
{
  using namespace Eigen;
  using namespace se3;

  se3::Model model;
  se3::buildModels::humanoidSimple(model,true);
  se3::Data data(model);

  VectorXd q = VectorXd::Zero(model.nq);
  data.M.fill(0); // Only nonzero coeff of M are initialized by CRBA.
  crba(model,data,q);
  

  long flag = BOOST_BINARY(1000101);
  StackTicToc timer(StackTicToc::US); 
  #ifdef NDEBUG
    #ifdef _INTENSE_TESTING_
      const int NBT = 1000*1000;
    #else
      const int NBT = 10;
    #endif
  #else 
    const int NBT = 1;
    std::cout << "(the time score in debug mode is not relevant)  " ;
  #endif

  bool verbose = flag & (flag-1) ; // True is two or more binaries of the flag are 1.
  if(verbose) std::cout <<"--" << std::endl;

  if( flag >> 0 & 1 )
    {
      timer.tic();
      SMOOTH(NBT)
      {
	se3::cholesky::decompose(model,data);
      }
      if(verbose) std::cout << "Decompose =\t";
      timer.toc(std::cout,NBT);
    }

  if( flag >> 1 & 1 )
    {
      timer.tic();
      Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
      Eigen::VectorXd res(model.nv);
      SMOOTH(NBT)
      {
	Eigen::LDLT <Eigen::MatrixXd> Mchol(data.M);
	res = Mchol.solve(v);
      }
      if(verbose) std::cout << "Eigen::LDLt =\t";
      timer.toc(std::cout,NBT);
    }

  if( flag >> 2 & 31 )
    {
      std::vector<Eigen::VectorXd> randvec(NBT);
      for(int i=0;i<NBT;++i ) randvec[(std::size_t)i] = Eigen::VectorXd::Random(model.nv);
      Eigen::VectorXd zero = Eigen::VectorXd(model.nv);
      Eigen::VectorXd res (model.nv);


      if( flag >> 2 & 1 )
	{
	  timer.tic();
	  SMOOTH(NBT)
	  {
	    se3::cholesky::solve(model,data,randvec[(std::size_t)_smooth]);
	  }
	  if(verbose) std::cout << "solve =\t\t";
	  timer.toc(std::cout,NBT);
	}

      if( flag >> 3 & 1 )
	{
	  timer.tic();
	  SMOOTH(NBT)
	  {
	    se3::cholesky::Uv(model,data,randvec[(std::size_t)_smooth]);
	  }
	  if(verbose) std::cout << "Uv =\t\t";
	  timer.toc(std::cout,NBT);
	}

      if( flag >> 4 & 1 )
	{
	  timer.tic();
	  SMOOTH(NBT)
	  {
	    se3::cholesky::Uiv(model,data,randvec[(std::size_t)_smooth]);
	  }
	  if(verbose) std::cout << "Uiv =\t\t";
	  timer.toc(std::cout,NBT);
	}
      if( flag >> 5 & 1 )
	{
	  timer.tic();
	  Eigen::VectorXd res;
	  SMOOTH(NBT)
	  {
	    res = se3::cholesky::Mv(model,data,randvec[(std::size_t)_smooth]);
	  }
	  if(verbose) std::cout << "Mv =\t\t";
	  timer.toc(std::cout,NBT);
	}
      if( flag >> 6 & 1 )
	{
	  timer.tic();
	  SMOOTH(NBT)
	  {
	    se3::cholesky::Mv(model,data,randvec[(std::size_t)_smooth],true);
	  }
	  if(verbose) std::cout << "UDUtv =\t\t";
	  timer.toc(std::cout,NBT);
	}
    }

}

BOOST_AUTO_TEST_SUITE_END ()
