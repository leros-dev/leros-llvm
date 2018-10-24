#!/bin/sh

BUILD_DIRECTORY=build-leros-llvm-Clang-Release

# Make sure current working directory is the directory of the build script
cd "${0%/*}"

SOURCE_ROOT=$(pwd)

mkdir ../$BUILD_DIRECTORY
cd ../$BUILD_DIRECTORY

# Run CMake configuration
cmake -DLLVM_TARGETS_TO_BUILD="" -DLLVM_EXPERIMENTAL_TARGETS_TO_BUILD="Leros" -DCLANG_BUILD_TOOLS=OFF -DCMAKE_BUILD_TYPE="Release" $SOURCE_ROOT

# Get total number of cores
NPROC=$(nproc)

# Build
cmake --build . -j $NPROC