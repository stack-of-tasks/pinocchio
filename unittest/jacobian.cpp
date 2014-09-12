#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include <iostream>
#include <boost/utility/binary.hpp>

#include "pinocchio/tools/timer.hpp"
#include "pinocchio/multibody/parser/sample-models.hpp"


void timings(const se3::Model & model, se3::Data& data, long flag)
{
  using namespace se3;
  StackTicToc timer(StackTicToc::US); 
#ifdef NDEBUG
  const int NBT = 1000*1000;
#else 
  const int NBT = 1;
#endif

  bool verbose = flag & (flag-1) ; // True is two or more binaries of the flag are 1.
  if(verbose) std::cout <<"--" << std::endl;
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);

  if( flag >> 0 & 1 )
    {
      timer.tic();
      SMOOTH(NBT)
      {
	computeJacobian(model,data,q);
      }
      if(verbose) std::cout << "Compute =\t";
      timer.toc(std::cout,NBT);
    }
  if( flag >> 1 & 1 )
    {
      computeJacobian(model,data,q);
      Model::Index idx = model.existBodyName("rarm6")?model.getBodyId("rarm6"):model.nbody-1; 
      Eigen::MatrixXd Jrh(6,model.nv); Jrh.fill(0);

      timer.tic();
      SMOOTH(NBT)
      {
	getJacobian<false>(model,data,idx,Jrh);
      }
      if(verbose) std::cout << "Copy =\t";
      timer.toc(std::cout,NBT);
    }
}

void assertValues(const se3::Model & model, se3::Data& data)
{
  using namespace Eigen;
  using namespace se3;

  VectorXd q = VectorXd::Zero(model.nq);
  computeJacobian(model,data,q);

  Model::Index idx = model.existBodyName("rarm2")?model.getBodyId("rarm2"):model.nbody-1; 
  MatrixXd Jrh(6,model.nv); Jrh.fill(0);
  getJacobian<false>(model,data,idx,Jrh);

  VectorXd qdot = VectorXd::Random(model.nv);
  VectorXd qddot = VectorXd::Zero(model.nv);
  rnea( model,data,q,qdot,qddot );
  
  Motion v = data.oMi[idx].act( data.v[idx] );
  assert( v.toVector().isApprox( Jrh*qdot ));

  std::cout << "Jrh = [ " << Jrh << " ];" << std::endl;
  std::cout << "J = [ " << data.J << " ];" << std::endl;
}
  
int main()
{
  using namespace Eigen;
  using namespace se3;

  se3::Model model;
  se3::buildModels::humanoidSimple(model);
  se3::Data data(model);

#ifndef NDEBUG 
  assertValues(model,data);
#else
  timings(model,data,BOOST_BINARY(11));
#endif

  return 0;
}
