#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/multibody/joint.hpp"
#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/cholesky.hpp"

#include <iostream>
#ifdef NDEBUG
#  include <Eigen/Cholesky>
#endif

#include "pinocchio/tools/timer.hpp"
#include "pinocchio/multibody/parser/sample-models.hpp"

//#define __SSE3__
#include <fenv.h>
#ifdef __SSE3__
#include <pmmintrin.h>
#endif


void timings(const se3::Model & model, se3::Data& data, int flag = 1)
{
  StackTicToc timer(StackTicToc::US); 
#ifdef NDEBUG
  const int NBT = 1000*100;
#else 
  const int NBT = 1;
#endif

  if( flag & 1 )
    {
      timer.tic();
      SMOOTH(NBT)
      {
	se3::cholesky::decompose(model,data);
      }
      timer.toc(std::cout,NBT);
    }

  if( flag & 2 )
    {
      timer.tic();
      SMOOTH(NBT)
      {
	Eigen::LDLT <Eigen::MatrixXd> Mchol(data.M);
      }
      std::cout << "\t\t"; timer.toc(std::cout,1000);
    }
}

void assertValues(const se3::Model & model, se3::Data& data)
{
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
      
  assert( M.isApprox(U*D.asDiagonal()*U.transpose()) );

  Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
  const Eigen::VectorXd & UDv = se3::cholesky::UDv(model,data,v);
  std::cout << "v = [" << v.transpose() << "]';" << std::endl;
  std::cout << "UDv = [" << UDv.transpose() << "]';" << std::endl;
  assert( UDv.isApprox(U*D.asDiagonal()*v));
  const Eigen::VectorXd & Uv = se3::cholesky::Uv(model,data,v);
  assert( Uv.isApprox(U*v));

  const Eigen::VectorXd & DUtv = se3::cholesky::DUtv(model,data,v);
  assert( DUtv.isApprox(D.asDiagonal()*U.transpose()*v));
  const Eigen::VectorXd & Utv = se3::cholesky::Utv(model,data,v);
  assert( Utv.isApprox(U.transpose()*v));

  const Eigen::VectorXd & Uiv = se3::cholesky::Uiv(model,data,v);
  assert( Uiv.isApprox(U.inverse()*v));

  const Eigen::VectorXd & Utiv = se3::cholesky::Utiv(model,data,v);
  assert( Utiv.isApprox(U.transpose().inverse()*v));
  const Eigen::VectorXd & UtiDiv = se3::cholesky::UtiDiv(model,data,v);
  assert( UtiDiv.isApprox(U.transpose().inverse()*D.asDiagonal().inverse()*v));
  
}

int main()//int argc, const char ** argv)
{
#ifdef __SSE3__
  _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
#endif


  using namespace Eigen;
  using namespace se3;

  SE3::Matrix3 I3 = SE3::Matrix3::Identity();

  se3::Model model;
  //se3::buildModels::humanoidSimple(model,true);
  se3::buildModels::humanoid2d(model);

  se3::Data data(model);
  VectorXd q = VectorXd::Zero(model.nq);
  data.M.fill(0);
  crba(model,data,q);

#ifndef NDEBUG 
  assertValues(model,data);
#else
  timings(model,data);
#endif

  return 0;
}
