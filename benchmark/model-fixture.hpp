//
// Copyright (c) 2025 CNRS INRIA
//

#ifndef __pinocchio_benchmark_model_fixture_hpp__
#define __pinocchio_benchmark_model_fixture_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/sample-models.hpp"

#include "pinocchio/parsers/urdf.hpp"

#include <boost/optional.hpp>
#include <boost/none.hpp>

#include <benchmark/benchmark.h>

#include <Eigen/Core>

#include <iostream>

/// Store custom command line arguments
struct ExtraArgs
{
  bool with_ff = true;
  std::string model_filename;

  ExtraArgs()
  : model_filename(PINOCCHIO_MODEL_DIR + std::string("/simple_humanoid.urdf"))
  {
  }
};

/// ModelFixture store a Model, Data and associated random q, v, a and tau vector.
/// ModelFixture::GlobalSetUp load a Model in ModelFixture::MODEL from the parsed ExtraArgs.
struct ModelFixture : benchmark::Fixture
{
  void SetUp(benchmark::State &)
  {
    model = MODEL;
    data = pinocchio::Data(model);

    const Eigen::VectorXd qmax(Eigen::VectorXd::Ones(model.nq));
    q = randomConfiguration(model, -qmax, qmax);
    v = Eigen::VectorXd::Random(model.nv);
    a = Eigen::VectorXd::Random(model.nv);
    tau = Eigen::VectorXd::Random(model.nv);
  }

  void TearDown(benchmark::State &)
  {
  }

  static void GlobalSetUp(const ExtraArgs & extra_args)
  {
    if (extra_args.model_filename == "HS")
    {
      pinocchio::buildModels::humanoidRandom(ModelFixture::MODEL, extra_args.with_ff);
    }
    else if (extra_args.with_ff)
    {
      pinocchio::urdf::buildModel(
        extra_args.model_filename,
        pinocchio::JointModelFreeFlyerTpl<
          pinocchio::context::Scalar, pinocchio::context::Options>(),
        ModelFixture::MODEL);
    }
    else
    {
      pinocchio::urdf::buildModel(extra_args.model_filename, ModelFixture::MODEL);
    }
    std::cout << "nq = " << ModelFixture::MODEL.nq << std::endl;
    std::cout << "nv = " << ModelFixture::MODEL.nv << std::endl;
    std::cout << "name = " << ModelFixture::MODEL.name << std::endl;
    std::cout << "--" << std::endl;
  }

  pinocchio::Model model;
  pinocchio::Data data;
  Eigen::VectorXd q;
  Eigen::VectorXd v;
  Eigen::VectorXd a;
  Eigen::VectorXd tau;

  static pinocchio::Model MODEL;
};

// Parse --no-ff and --model arguments
inline boost::optional<ExtraArgs> parseExtraArgs(int argc, char ** argv)
{
  ExtraArgs args;
  for (int i = 1; i < argc; ++i)
  {
    if (std::strcmp(argv[i], "--no-ff") == 0)
    {
      args.with_ff = false;
    }
    else if (std::strcmp(argv[i], "--model") == 0)
    {
      if (argc > (i + 1))
      {
        args.model_filename = argv[i + 1];
        --argc;
      }
      else
      {
        std::cerr
          << argv[0]
          << ": error: unrecognized command-line flag: --model should be followed by an urdf path"
          << std::endl;
        return boost::none;
      }
    }
    else
    {
      std::cerr << argv[0] << ": error: unrecognized command-line flag: " << argv[i] << std::endl;
      return boost::none;
    }
  }
  return args;
}

// BENCHMARK_MAIN() macro expansion edited to take some extra argument
// GLOBAL_SETUP is a function to setup global variable (void(const ExtraArgs& args)).
#define PINOCCHIO_BENCHMARK_MAIN_WITH_SETUP(GLOBAL_SETUP)                                          \
  pinocchio::Model ModelFixture::MODEL;                                                            \
  int main(int argc, char ** argv)                                                                 \
  {                                                                                                \
    char arg0_default[] = "benchmark";                                                             \
    char * args_default = arg0_default;                                                            \
    if (!argv)                                                                                     \
    {                                                                                              \
      argc = 1;                                                                                    \
      argv = &args_default;                                                                        \
    }                                                                                              \
    ::benchmark::Initialize(&argc, argv);                                                          \
                                                                                                   \
    /* It's important to report errors so typos on google benchmark arguments */                   \
    /* are detected. */                                                                            \
    auto extra_args = parseExtraArgs(argc, argv);                                                  \
    if (!extra_args)                                                                               \
    {                                                                                              \
      return 1;                                                                                    \
    }                                                                                              \
    GLOBAL_SETUP(*extra_args);                                                                     \
                                                                                                   \
    ::benchmark ::RunSpecifiedBenchmarks();                                                        \
    ::benchmark ::Shutdown();                                                                      \
    return 0;                                                                                      \
  }                                                                                                \
  int main(int, char **)

// BENCHMARK_MAIN() macro expansion edited to take some extra argument
#define PINOCCHIO_BENCHMARK_MAIN() PINOCCHIO_BENCHMARK_MAIN_WITH_SETUP(ModelFixture::GlobalSetUp)

#endif // #ifndef __pinocchio_benchmark_model_fixture_hpp__
