# Benchmarks

To build benchmarks use the `all-benchmark` pixi environment then build the `bench` target:

```bash
pixi shell -e all-benchmark
pixi run configure
cd build
ninja bench
```

Then, you can run benchmark manually:

```bash
./benchmark/pinocchio-timings-xxx
```

Where `pinocchio-timings-xxx` is the benchmark you want to run.


## CLI arguments

Benchmarks are developed with [google benchmark](https://github.com/google/benchmark) library.
You can use all [google benchmark CLI arguments](https://github.com/google/benchmark/blob/main/docs/user_guide.md#output-formats).

Here a list of commonly used arguments:

- `--benchmark_list_tests`: List all benchmarks
- `--benchmark_filter=<regex>`: Only run benchmarks that match `regex`
- `--benchmark_repetitions=<N>`: Run benchmarks `N` time to compute variance
- `--benchmark_out=<filename>`: Write results in `filename` (JSON format by default)
- `--benchmark_time_unit={ns|us|ms|s}`: Use a different time unit in the report

Also, on some benchmarks, we provide the following custom arguments:

- `--no-ff`: Use a model without free flyer joint
- `--model <urdf_path>`: Use a custom model


## Parallel benchmarks

On some CPU architectures, all core doesn't have the same power (on I9 we have E-Core and P-Core where E-core are less powerful than P-core).

It's really important to run parallel tests on the same core type.
If a thread is launched on a less powerful core, results can be wrong.
Also, variance can be improved by only running 1 thread by core.

On GNU/Linux, you can check that by running:

```bash
lscpu --all --extended
```

Then you can look at the CORE or MAXMHZ column as explained [here](https://stackoverflow.com/a/71282744).

As an example on an I9 CPU:

```
CPU NODE SOCKET CORE L1d:L1i:L2:L3 ONLINE    MAXMHZ   MINMHZ      MHZ
  0    0      0    0 0:0:0:0          yes 5300.0000 800.0000 1144.586
  1    0      0    0 0:0:0:0          yes 5300.0000 800.0000 1253.787
  2    0      0    1 4:4:1:0          yes 5300.0000 800.0000  800.000
  3    0      0    1 4:4:1:0          yes 5300.0000 800.0000  800.000
  4    0      0    2 8:8:2:0          yes 5300.0000 800.0000  800.000
  5    0      0    2 8:8:2:0          yes 5300.0000 800.0000  800.000
  6    0      0    3 12:12:3:0        yes 5300.0000 800.0000  998.313
  7    0      0    3 12:12:3:0        yes 5300.0000 800.0000  800.000
  8    0      0    4 16:16:4:0        yes 5500.0000 800.0000 1422.816
  9    0      0    4 16:16:4:0        yes 5500.0000 800.0000  800.000
 10    0      0    5 20:20:5:0        yes 5500.0000 800.0000  823.734
 11    0      0    5 20:20:5:0        yes 5500.0000 800.0000  800.000
 12    0      0    6 24:24:6:0        yes 5300.0000 800.0000  885.377
 13    0      0    6 24:24:6:0        yes 5300.0000 800.0000  800.000
 14    0      0    7 28:28:7:0        yes 5300.0000 800.0000  800.161
 15    0      0    7 28:28:7:0        yes 5300.0000 800.0000  800.000
 16    0      0    8 32:32:8:0        yes 4000.0000 800.0000 1600.027
 17    0      0    9 33:33:8:0        yes 4000.0000 800.0000 1599.988
 18    0      0   10 34:34:8:0        yes 4000.0000 800.0000 1800.008
 19    0      0   11 35:35:8:0        yes 4000.0000 800.0000 1599.950
 20    0      0   12 36:36:9:0        yes 4000.0000 800.0000 1800.033
 21    0      0   13 37:37:9:0        yes 4000.0000 800.0000  800.000
 22    0      0   14 38:38:9:0        yes 4000.0000 800.0000  800.000
 23    0      0   15 39:39:9:0        yes 4000.0000 800.0000  800.000
 24    0      0   16 40:40:10:0       yes 4000.0000 800.0000  800.000
 25    0      0   17 41:41:10:0       yes 4000.0000 800.0000  800.000
 26    0      0   18 42:42:10:0       yes 4000.0000 800.0000  800.000
 27    0      0   19 43:43:10:0       yes 4000.0000 800.0000 1711.773
 28    0      0   20 44:44:11:0       yes 4000.0000 800.0000  800.000
 29    0      0   21 45:45:11:0       yes 4000.0000 800.0000 1600.001
 30    0      0   22 46:46:11:0       yes 4000.0000 800.0000 1596.722
 31    0      0   23 47:47:11:0       yes 4000.0000 800.0000  800.000
```

CPU 0 to 15 are P-CORE (2 CPU on one CORE) ad CPU 16 to 31 are E-CORE (1 CPU per CORE).

Now, we can constraint OpenMP to only run on CPU 0, 2, 4, 6, 8, 10, 12 and 14 by running
parallel benchmarks with the following environment variables:

```bash
GOMP_CPU_AFFINITY="0-15:2" OMP_PROC_BIND=TRUE ./benchmark/pinocchio-timings-parallel
```

Here some [documentation](https://gcc.gnu.org/onlinedocs/libgomp/openmp-environment-variables/gompcpuaffinity.html)
about OpenMP environment variables.


## Reducing variance

To reduce the variance, you can follow [the following instructions](https://github.com/google/benchmark/blob/main/docs/reducing_variance.md).
