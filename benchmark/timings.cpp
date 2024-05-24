//
// Copyright (c) 2015-2020 CNRS INRIA
//

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/cholesky.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/sample-models.hpp"

#include <iostream>

#include "pinocchio/utils/timer.hpp"

namespace pinocchio
{
  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  struct EmptyForwardStepUnaryVisit
  : fusion::JointUnaryVisitorBase<EmptyForwardStepUnaryVisit<Scalar, Options, JointCollectionTpl>>
  {
    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

    typedef fusion::NoArg ArgsType;

    template<typename JointModel>
    static void
    algo(const JointModelBase<JointModel> &, JointDataBase<typename JointModel::JointDataDerived> &)
    { // do nothing
    }
  };

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  inline void emptyForwardPassUnaryVisit(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data)
  {
    assert(model.check(data) && "data is not consistent with model.");

    typedef typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointIndex JointIndex;
    typedef EmptyForwardStepUnaryVisit<Scalar, Options, JointCollectionTpl> Algo;

    for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
    {
      Algo::run(model.joints[i], data.joints[i]);
    }
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  struct EmptyForwardStepUnaryVisitNoData
  : fusion::JointUnaryVisitorBase<
      EmptyForwardStepUnaryVisitNoData<Scalar, Options, JointCollectionTpl>>
  {
    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

    typedef fusion::NoArg ArgsType;

    template<typename JointModel>
    EIGEN_DONT_INLINE static void algo(const JointModelBase<JointModel> &)
    { // do nothing
    }
  };

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  inline void emptyForwardPassUnaryVisitNoData(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data)
  {
    PINOCCHIO_UNUSED_VARIABLE(data);
    assert(model.check(data) && "data is not consistent with model.");

    typedef typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointIndex JointIndex;
    typedef EmptyForwardStepUnaryVisitNoData<Scalar, Options, JointCollectionTpl> Algo;

    for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
    {
      Algo::run(model.joints[i]);
    }
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  struct EmptyForwardStepBinaryVisit
  : fusion::JointBinaryVisitorBase<EmptyForwardStepBinaryVisit<Scalar, Options, JointCollectionTpl>>
  {
    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

    typedef fusion::NoArg ArgsType;

    template<typename JointModel1, typename JointModel2>
    EIGEN_DONT_INLINE static void algo(
      const JointModelBase<JointModel1> &,
      const JointModelBase<JointModel2> &,
      JointDataBase<typename JointModel1::JointDataDerived> &,
      JointDataBase<typename JointModel2::JointDataDerived> &)
    { // do nothing
    }
  };

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  inline void emptyForwardPassBinaryVisit(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data)
  {
    assert(model.check(data) && "data is not consistent with model.");

    typedef typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointIndex JointIndex;
    typedef EmptyForwardStepBinaryVisit<Scalar, Options, JointCollectionTpl> Algo;

    for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
    {
      Algo::run(model.joints[i], model.joints[i], data.joints[i], data.joints[i]);
    }
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  struct EmptyForwardStepBinaryVisitNoData
  : fusion::JointBinaryVisitorBase<
      EmptyForwardStepBinaryVisitNoData<Scalar, Options, JointCollectionTpl>>
  {
    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

    typedef fusion::NoArg ArgsType;

    template<typename JointModel1, typename JointModel2>
    EIGEN_DONT_INLINE static void
    algo(const JointModelBase<JointModel1> &, const JointModelBase<JointModel2> &)
    { // do nothing
    }
  };

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  inline void emptyForwardPassBinaryVisitNoData(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data)
  {
    PINOCCHIO_UNUSED_VARIABLE(data);
    assert(model.check(data) && "data is not consistent with model.");

    typedef typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointIndex JointIndex;
    typedef EmptyForwardStepBinaryVisitNoData<Scalar, Options, JointCollectionTpl> Algo;

    for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
    {
      Algo::run(model.joints[i], model.joints[i]);
    }
  }
} // namespace pinocchio

int main(int argc, const char ** argv)
{
  using namespace Eigen;
  using namespace pinocchio;

  PinocchioTicToc timer(PinocchioTicToc::US);
#ifdef NDEBUG
  const int NBT = 1000 * 100;
#else
  const int NBT = 1;
  std::cout << "(the time score in debug mode is not relevant) " << std::endl;
#endif

  pinocchio::Model model;

  std::string filename = PINOCCHIO_MODEL_DIR + std::string("/simple_humanoid.urdf");
  if (argc > 1)
    filename = argv[1];

  bool with_ff = true;
  if (argc > 2)
  {
    const std::string ff_option = argv[2];
    if (ff_option == "-no-ff")
      with_ff = false;
  }

  if (filename == "HS")
    pinocchio::buildModels::humanoidRandom(model, true);
  else if (with_ff)
    pinocchio::urdf::buildModel(filename, JointModelFreeFlyer(), model);
  else
    pinocchio::urdf::buildModel(filename, model);
  std::cout << "nq = " << model.nq << std::endl;
  std::cout << "nv = " << model.nv << std::endl;
  std::cout << "--" << std::endl;

  pinocchio::Data data(model);
  const VectorXd qmax = Eigen::VectorXd::Ones(model.nq);

  PINOCCHIO_ALIGNED_STD_VECTOR(VectorXd) qs(NBT);
  PINOCCHIO_ALIGNED_STD_VECTOR(VectorXd) qdots(NBT);
  PINOCCHIO_ALIGNED_STD_VECTOR(VectorXd) qddots(NBT);
  PINOCCHIO_ALIGNED_STD_VECTOR(VectorXd) taus(NBT);
  for (size_t i = 0; i < NBT; ++i)
  {
    qs[i] = randomConfiguration(model, -qmax, qmax);
    qdots[i] = Eigen::VectorXd::Random(model.nv);
    qddots[i] = Eigen::VectorXd::Random(model.nv);
    taus[i] = Eigen::VectorXd::Random(model.nv);
  }

  timer.tic();
  SMOOTH(NBT)
  {
    rnea(model, data, qs[_smooth], qdots[_smooth], qddots[_smooth]);
  }
  std::cout << "RNEA = \t\t\t\t";
  timer.toc(std::cout, NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    nonLinearEffects(model, data, qs[_smooth], qdots[_smooth]);
  }
  std::cout << "NLE = \t\t\t\t";
  timer.toc(std::cout, NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    rnea(model, data, qs[_smooth], qdots[_smooth], Eigen::VectorXd::Zero(model.nv));
  }
  std::cout << "NLE via RNEA = \t\t\t";
  timer.toc(std::cout, NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    crba(model, data, qs[_smooth], Convention::LOCAL);
  }
  std::cout << "CRBA (original) = \t\t";
  timer.toc(std::cout, NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    crba(model, data, qs[_smooth], Convention::WORLD);
  }
  std::cout << "CRBA = \t\t";
  timer.toc(std::cout, NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    computeAllTerms(model, data, qs[_smooth], qdots[_smooth]);
  }
  std::cout << "computeAllTerms = \t\t";
  timer.toc(std::cout, NBT);

  double total = 0;
  SMOOTH(NBT)
  {
    crba(model, data, qs[_smooth], Convention::WORLD);
    timer.tic();
    cholesky::decompose(model, data);
    total += timer.toc(timer.DEFAULT_UNIT);
  }
  std::cout << "Sparse Cholesky = \t\t" << (total / NBT) << " "
            << timer.unitName(timer.DEFAULT_UNIT) << std::endl;

  total = 0;
  Eigen::LDLT<Eigen::MatrixXd> Mldlt(data.M);
  SMOOTH(NBT)
  {
    crba(model, data, qs[_smooth], Convention::WORLD);
    data.M.triangularView<Eigen::StrictlyLower>() =
      data.M.transpose().triangularView<Eigen::StrictlyLower>();
    timer.tic();
    Mldlt.compute(data.M);
    total += timer.toc(timer.DEFAULT_UNIT);
  }
  std::cout << "Dense Cholesky = \t\t" << (total / NBT) << " " << timer.unitName(timer.DEFAULT_UNIT)
            << std::endl;

  timer.tic();
  SMOOTH(NBT)
  {
    computeJointJacobians(model, data, qs[_smooth]);
  }
  std::cout << "Jacobian = \t\t\t";
  timer.toc(std::cout, NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    computeJointJacobiansTimeVariation(model, data, qs[_smooth], qdots[_smooth]);
  }
  std::cout << "Jacobian Derivative = \t\t";
  timer.toc(std::cout, NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    jacobianCenterOfMass(model, data, qs[_smooth], true);
  }
  std::cout << "COM+Jcom = \t\t\t";
  timer.toc(std::cout, NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    centerOfMass(model, data, qs[_smooth], qdots[_smooth], qddots[_smooth], true);
  }
  std::cout << "COM+vCOM+aCOM = \t\t";
  timer.toc(std::cout, NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    forwardKinematics(model, data, qs[_smooth]);
  }
  std::cout << "Forward Kinematics(q) = \t";
  timer.toc(std::cout, NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    forwardKinematics(model, data, qs[_smooth], qdots[_smooth]);
  }
  std::cout << "Forward Kinematics(q,v) = \t";
  timer.toc(std::cout, NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    forwardKinematics(model, data, qs[_smooth], qdots[_smooth], qddots[_smooth]);
  }
  std::cout << "Forward Kinematics(q,v,a) = \t";
  timer.toc(std::cout, NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    framesForwardKinematics(model, data, qs[_smooth]);
  }
  std::cout << "Frame Placement(q) = \t\t";
  timer.toc(std::cout, NBT);

  total = 0.;
  SMOOTH(NBT)
  {
    forwardKinematics(model, data, qs[_smooth]);
    timer.tic();
    updateFramePlacements(model, data);
    total += timer.toc(timer.DEFAULT_UNIT);
  }
  std::cout << "Update Frame Placement = \t" << (total / NBT) << " "
            << timer.unitName(timer.DEFAULT_UNIT) << std::endl;

  timer.tic();
  SMOOTH(NBT)
  {
    ccrba(model, data, qs[_smooth], qdots[_smooth]);
  }
  std::cout << "CCRBA = \t\t\t";
  timer.toc(std::cout, NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    aba(model, data, qs[_smooth], qdots[_smooth], taus[_smooth], Convention::LOCAL);
  }
  std::cout << "ABA (minimal) = \t\t";
  timer.toc(std::cout, NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    aba(model, data, qs[_smooth], qdots[_smooth], taus[_smooth], Convention::WORLD);
  }
  std::cout << "ABA = \t\t";
  timer.toc(std::cout, NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    computeCoriolisMatrix(model, data, qs[_smooth], qdots[_smooth]);
  }
  std::cout << "Coriolis Matrix = \t\t";
  timer.toc(std::cout, NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    computeMinverse(model, data, qs[_smooth]);
  }
  std::cout << "Minv(q) = \t\t\t";
  timer.toc(std::cout, NBT);

  total = 0;
  SMOOTH(NBT)
  {
    aba(model, data, qs[_smooth], qdots[_smooth], taus[_smooth], Convention::WORLD);
    timer.tic();
    computeMinverse(model, data);
    total += timer.toc(timer.DEFAULT_UNIT);
  }
  std::cout << "Minv() = \t\t\t" << (total / NBT) << " " << timer.unitName(timer.DEFAULT_UNIT)
            << std::endl;
  std::cout << "--" << std::endl;

  timer.tic();
  SMOOTH(NBT)
  {
    emptyForwardPassUnaryVisit(model, data);
  }
  std::cout << "Forward Pass(jmodel,jdata) = \t\t\t";
  timer.toc(std::cout, NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    emptyForwardPassUnaryVisitNoData(model, data);
  }
  std::cout << "Forward Pass(jmodel) = \t\t\t\t";
  timer.toc(std::cout, NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    emptyForwardPassBinaryVisit(model, data);
  }
  std::cout << "Forward Pass(jmodel1,jmodel2,jdata1,jdata2) = \t";
  timer.toc(std::cout, NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    emptyForwardPassBinaryVisitNoData(model, data);
  }
  std::cout << "Forward Pass(jmodel1,jmodel2) = \t\t";
  timer.toc(std::cout, NBT);

  std::cout << "--" << std::endl;
  return 0;
}
