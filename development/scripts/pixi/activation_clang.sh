#! /bin/bash
# Clang activation script

export CC=clang
export CXX=clang++

# activation.sh set this variable to gcc, we must override it here
export CCACHE_COMPILERTYPE=clang
