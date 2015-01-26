/*
 * Test the CRBA algorithm. The code validates both the computation times and
 * the value by comparing the results of the CRBA with the reconstruction of
 * the mass matrix using the RNEA.
 * For a strong timing benchmark, see benchmark/timings.
 *
 */

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/multibody/parser/sample-models.hpp"
#include "pinocchio/tools/timer.hpp"

#include <iostream>

void timings(const se3::Model & model, se3::Data& data, int NBT = 100000)
{
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
 
  StackTicToc timer(StackTicToc::US); 
	double duration = 0.;
	int num_iterations = 1e6;

	for (int i = 0; i < num_iterations; i++) 
  {
		VectorXd q = VectorXd::Random(model.nq);

		timer.tic();
		crba(model,data,q);
		duration += timer.toc (StackTicToc::US);
  }

	std::cout << "Mean duration of CRBA : " << duration / (double) num_iterations << " us" << std::endl;
}

void assertValues(const se3::Model & model, se3::Data& data)
{
  using namespace Eigen;
  using namespace se3;

  Eigen::MatrixXd M(model.nv,model.nv);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
  Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);
  data.M.fill(0);  crba(model,data,q);
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();

  /* Joint inertia from iterative crba. */
  const Eigen::VectorXd bias = rnea(model,data,q,v,a);
  for(int i=0;i<model.nv;++i)
    { 
      M.col(i) = rnea(model,data,q,v,Eigen::VectorXd::Unit(model.nv,i)) - bias;
    }

  // std::cout << "Mcrb = [ " << data.M << "  ];" << std::endl;
  // std::cout << "Mrne = [  " << M << " ]; " << std::endl;
  assert( M.isApprox(data.M) );
}

int main()
{
  se3::Model model;
  se3::buildModels::humanoidSimple(model);
  se3::Data data(model);

#ifdef NDEBUG
  timings(model,data);
#else
  assertValues(model,data);
  std::cout << "(the time score in debug mode is not relevant)  " ;
  timings(model,data,1);
#endif // ifndef NDEBUG

  return 0;
}
