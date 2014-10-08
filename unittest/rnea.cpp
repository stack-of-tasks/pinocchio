#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/multibody/joint.hpp"
#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/multibody/parser/sample-models.hpp"

#include <iostream>

#include "pinocchio/tools/timer.hpp"


//#define __SSE3__
#include <fenv.h>
#ifdef __SSE3__
#include <pmmintrin.h>
#endif

int main()
{
#ifdef __SSE3__
  _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
#endif

  using namespace Eigen;
  using namespace se3;

  se3::Model model; buildModels::humanoidSimple(model);
  
  se3::Data data(model);
  data.v[0] = Motion::Zero();
  data.a[0] = -model.gravity;

  VectorXd q = VectorXd::Random(model.nq);
  VectorXd v = VectorXd::Random(model.nv);
  VectorXd a = VectorXd::Random(model.nv);
 
	double duration = 0.;
	int num_iterations = 1e6;
	StackTicToc timer(StackTicToc::US); 

	for(int i = 0; i < num_iterations; i++)
	{
		q = VectorXd::Random(model.nq);
		v = VectorXd::Random(model.nv);
		a = VectorXd::Random(model.nv);

		timer.tic();
		rnea(model,data,q,v,a);
		duration += timer.toc (StackTicToc::US);
	}

	std::cout << "Duration : " << duration / (double) num_iterations <<  " us" << std::endl;
  return 0;
}
