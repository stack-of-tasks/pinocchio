#include <pinocchio/spatial/se3.hpp>

#include <benchmark/benchmark.h>

using SE3 = pinocchio::SE3Tpl<double>;

// Standard SE3Tpl operation
struct SE3TplAct
{
  EIGEN_DONT_INLINE void operator()(SE3 & amc, const SE3 & amb, const SE3 & bmc)
  {
    amc = amb * bmc;
  }
};

// Use of SE3Expr without noalias
struct SE3ExprAct
{
  EIGEN_DONT_INLINE void operator()(SE3 & amc, const SE3 & amb, const SE3 & bmc)
  {
    amc.expr() = amb.const_expr() * bmc.const_expr();
  }
};

// Use of SE3Expr with noalias
struct SE3ExprNoaliasAct
{
  EIGEN_DONT_INLINE void operator()(SE3 & amc, const SE3 & amb, const SE3 & bmc)
  {
    amc.expr().noalias() = amb.const_expr() * bmc.const_expr();
  }
};

// Apply SE3 manualy with noalias
struct SE3ManualAct
{
  EIGEN_DONT_INLINE void operator()(SE3 & amc, const SE3 & amb, const SE3 & bmc)
  {
    amc.rotation().noalias() = amb.rotation() * bmc.rotation();
    amc.translation().noalias() = amb.translation();
    amc.translation().noalias() += amb.rotation() * bmc.translation();
  }
};

template<class Fun>
void bench_se3_act(benchmark::State & state)
{
  Fun f;
  SE3 amc;
  SE3 amb(SE3::Random());
  SE3 bmc(SE3::Random());
  for (auto _ : state)
  {
    for (int64_t i = 0; i < state.range(0); ++i)
    {
      f(amc, amb, bmc);
      benchmark::DoNotOptimize(amc);
    }
  }
}

static void CustomArguments(benchmark::internal::Benchmark * b)
{
  b->ArgNames({"iter"})->MinWarmUpTime(3.);
  for (int64_t i = 1; i < 4; ++i)
  {
    std::vector<int64_t> args;
    args.push_back(static_cast<int64_t>(std::pow(10, i)));
    b->Args(args);
  }
}

BENCHMARK(bench_se3_act<SE3TplAct>)->Apply(CustomArguments);
BENCHMARK(bench_se3_act<SE3ExprAct>)->Apply(CustomArguments);
BENCHMARK(bench_se3_act<SE3ExprNoaliasAct>)->Apply(CustomArguments);
BENCHMARK(bench_se3_act<SE3ManualAct>)->Apply(CustomArguments);

BENCHMARK_MAIN();
