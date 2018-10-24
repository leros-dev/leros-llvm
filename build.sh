#!/bin/bash

BUILD_DIRECTORY=build-leros-llvm-Clang-Release

# Make sure current working directory is the directory of the build script
cd "${0%/*}"

SOURCE_ROOT=$(pwd)

mkdir ../$BUILD_DIRECTORY
cd ../$BUILD_DIRECTORY

# Run CMake configuration
cmake -DLLVM_TOOL_CLANG_BUILD=OFF -DLLVM_TARGETS_TO_BUILD="" -DLLVM_EXPERIMENTAL_TARGETS_TO_BUILD="Leros" -DCMAKE_BUILD_TYPE="Release" $SOURCE_ROOT

# Use all cores when building
if [[ "$OSTYPE" == "linux-gnu" ]]; then
        export MAKEFLAGS=-j$(nproc)
elif [[ "$OSTYPE" == "darwin"* ]]; then
        export MAKEFLAGS=-j$(sysctl -n hw.ncpu)
fi

# Build
cmake --build .