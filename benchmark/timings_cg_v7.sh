#!/bin/bash


# compile GCC Code as
$(g++ timings_cg_v7.cpp  -DNDEBUG -I /usr/include/eigen3 -I /usr/include -O0 -L /usr/local/lib -l pinocchio -L /usr/local/lib -l casadi  -ldl -o timings_cg_v7) # compiling using gcc here

#$(g++ timings_cg_v7.cpp  -DNDEBUG -I /usr/include/eigen3 -I /usr/include -O0 -L /usr/local/lib -l pinocchio -L /usr/local/lib -l casadi  -march=native -ldl -o timings_cg_v7) # compiling using gcc here
