//
// Copyright (c) 2015-2019 CNRS INRIA
//

/*
 * Validate the sparse Cholesky decomposition of the mass matrix.  The code
 * tests both the numerical value and the computation time. For a strong
 * computation benchmark, see benchmark/timings.
 *
 */

#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/cholesky.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/utils/timer.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include <iostream>
#ifdef NDEBUG
#  include <Eigen/Cholesky>
#endif

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>


BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE ( test_cholesky )
{
  using namespace Eigen;
  using namespace pinocchio;

  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model,true);
  pinocchio::Data data(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  data.M.fill(0); // Only nonzero coeff of M are initialized by CRBA.
  crba(model,data,q);
 
  pinocchio::cholesky::decompose(model,data);
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
      
  BOOST_CHECK(M.isApprox(U*D.asDiagonal()*U.transpose() , 1e-12));

  Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
// std::cout << "v = [" << v.transpose() << "]';" << std::endl;

  Eigen::VectorXd Uv = v; pinocchio::cholesky::Uv(model,data,Uv);
  BOOST_CHECK(Uv.isApprox(U*v, 1e-12));

  Eigen::VectorXd Utv = v; pinocchio::cholesky::Utv(model,data,Utv);
  BOOST_CHECK(Utv.isApprox(U.transpose()*v, 1e-12));

  Eigen::VectorXd Uiv = v; pinocchio::cholesky::Uiv(model,data,Uiv);
  BOOST_CHECK(Uiv.isApprox(U.inverse()*v, 1e-12));


  Eigen::VectorXd Utiv = v; pinocchio::cholesky::Utiv(model,data,Utiv);
  BOOST_CHECK(Utiv.isApprox(U.transpose().inverse()*v, 1e-12));

  Eigen::VectorXd Miv = v; pinocchio::cholesky::solve(model,data,Miv);
  BOOST_CHECK(Miv.isApprox(M.inverse()*v, 1e-12));

  Eigen::VectorXd Mv = v; Mv = pinocchio::cholesky::Mv(model,data,Mv);
  BOOST_CHECK(Mv.isApprox(M*v, 1e-12));
  Mv = v;                 pinocchio::cholesky::UDUtv(model,data,Mv);
  BOOST_CHECK(Mv.isApprox(M*v, 1e-12));
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
  using namespace pinocchio;

  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model,true);
  pinocchio::Data data(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  data.M.fill(0); // Only nonzero coeff of M are initialized by CRBA.
  crba(model,data,q);
  

  long flag = BOOST_BINARY(1111111);
  PinocchioTicToc timer(PinocchioTicToc::US); 
  #ifdef NDEBUG
    #ifdef _INTENSE_TESTING_
      const size_t NBT = 1000*1000;
    #else
      const size_t NBT = 10;
    #endif
  #else 
    const size_t NBT = 1;
    std::cout << "(the time score in debug mode is not relevant)  " ;
  #endif

  bool verbose = flag & (flag-1) ; // True is two or more binaries of the flag are 1.
  if(verbose) std::cout <<"--" << std::endl;

  if( flag >> 0 & 1 )
    {
      timer.tic();
      SMOOTH(NBT)
      {
	pinocchio::cholesky::decompose(model,data);
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
      for(size_t i=0;i<NBT;++i ) randvec[i] = Eigen::VectorXd::Random(model.nv);
      Eigen::VectorXd zero = Eigen::VectorXd(model.nv);
      Eigen::VectorXd res (model.nv);


      if( flag >> 2 & 1 )
	{
	  timer.tic();
	  SMOOTH(NBT)
	  {
	    pinocchio::cholesky::solve(model,data,randvec[_smooth]);
	  }
	  if(verbose) std::cout << "solve =\t\t";
	  timer.toc(std::cout,NBT);
	}

      if( flag >> 3 & 1 )
	{
	  timer.tic();
	  SMOOTH(NBT)
	  {
	    pinocchio::cholesky::Uv(model,data,randvec[_smooth]);
	  }
	  if(verbose) std::cout << "Uv =\t\t";
	  timer.toc(std::cout,NBT);
	}

      if( flag >> 4 & 1 )
	{
	  timer.tic();
	  SMOOTH(NBT)
	  {
	    pinocchio::cholesky::Uiv(model,data,randvec[_smooth]);
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
	    res = pinocchio::cholesky::Mv(model,data,randvec[_smooth]);
	  }
	  if(verbose) std::cout << "Mv =\t\t";
	  timer.toc(std::cout,NBT);
	}
      if( flag >> 6 & 1 )
	{
    timer.tic();
    SMOOTH(NBT)
    {
      pinocchio::cholesky::UDUtv(model,data,randvec[_smooth]);
    }
    if(verbose) std::cout << "UDUtv =\t\t";
    timer.toc(std::cout,NBT);
	}
    }
}
  
  BOOST_AUTO_TEST_CASE(test_Minv_from_cholesky)
  {
    using namespace Eigen;
    using namespace pinocchio;
    
    pinocchio::Model model;
    pinocchio::buildModels::humanoidRandom(model,true);
    pinocchio::Data data(model);
    
    model.lowerPositionLimit.head<3>().fill(-1.);
    model.upperPositionLimit.head<3>().fill(1.);
    VectorXd q = randomConfiguration(model);
    crba(model,data,q);
    data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.triangularView<Eigen::StrictlyUpper>().transpose();
    MatrixXd Minv_ref(data.M.inverse());
    
    cholesky::decompose(model,data);
    VectorXd v_unit(VectorXd::Unit(model.nv,0));

    VectorXd Ui_v_unit(model.nv);
    VectorXd Ui_v_unit_ref(model.nv);
    
    for(int k = 0; k < model.nv; ++k)
    {
      v_unit = VectorXd::Unit(model.nv,k);
      Ui_v_unit.setZero();
      cholesky::internal::Miunit(model,data,k,Ui_v_unit);
      Ui_v_unit_ref = v_unit;
      cholesky::Uiv(model,data,Ui_v_unit_ref);
      Ui_v_unit_ref.array() *= data.Dinv.array();
      cholesky::Utiv(model,data,Ui_v_unit_ref);

      BOOST_CHECK(Ui_v_unit.isApprox(Ui_v_unit_ref));
      
      Ui_v_unit_ref = v_unit;
      cholesky::solve(model,data,Ui_v_unit_ref);
      BOOST_CHECK(Ui_v_unit.isApprox(Ui_v_unit_ref));
      
//      std::cout << "Ui_v_unit : " << Ui_v_unit.transpose() << std::endl;
//      std::cout << "Ui_v_unit_ref : " << Ui_v_unit_ref.transpose() << std::endl << std::endl;
    }
    
    MatrixXd Minv(model.nv,model.nv);
    Minv.setZero();
    cholesky::computeMinv(model,data,Minv);
    
    BOOST_CHECK(Minv.isApprox(Minv_ref));
    
    // Check second call to cholesky::computeMinv
    cholesky::computeMinv(model,data,Minv);
    BOOST_CHECK(Minv.isApprox(Minv_ref));
    
    // Call the second signature of cholesky::computeMinv
    Data data_bis(model);
    crba(model,data_bis,q);
    cholesky::decompose(model,data_bis);
    cholesky::computeMinv(model,data_bis);
    BOOST_CHECK(data_bis.Minv.isApprox(Minv_ref));
  }

BOOST_AUTO_TEST_SUITE_END ()
