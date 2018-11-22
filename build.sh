#!/bin/bash

BUILD_DIRECTORY=leros-llvm

# Make sure current working directory is the directory of the build script
cd "${0%/*}"

SOURCE_ROOT=$(pwd)

CLANG_VERSION="8.0.0"

# Binary folder of the built compiler
BUILD_ROOT="$SOURCE_ROOT/../$BUILD_DIRECTORY"
BUILD_BIN_DIR="$BUILD_ROOT/bin"
BUILD_CLANG_LIB_DIR="$BUILD_ROOT/lib/clang/$CLANG_VERSION"

mkdir ../$BUILD_DIRECTORY
cd ../$BUILD_DIRECTORY

# Run CMake configuration
cmake -DLLVM_TOOL_CLANG_BUILD=OFF -DLLVM_TARGETS_TO_BUILD="" LLVM_DEFAULT_TARGET_TRIPLE="leros32-unknown-elf" -DLLVM_EXPERIMENTAL_TARGETS_TO_BUILD="Leros" -DCMAKE_BUILD_TYPE="Release" $SOURCE_ROOT

# Use all cores when building
if [[ "$OSTYPE" == "linux-gnu" ]]; then
        export MAKEFLAGS=-j$(nproc)
elif [[ "$OSTYPE" == "darwin"* ]]; then
        export MAKEFLAGS=-j$(sysctl -n hw.ncpu)
fi

# Build
cmake --build .

# ============ leros-lib =================
cd ..
# Check if the leros-lib repo is checked out next to the leros-llvm repository
if [ -d "leros-lib" ]; then
  # make sure the repository is up-to-date
  cd leros-lib
  git pull
else
  # clone the leros-lib repository
  git clone https://github.com/leros-dev/leros-lib
  cd leros-lib
fi

# Build crt0.leros.o
cd runtime
$BUILD_BIN_DIR/clang crt0.leros.c -c -ffreestanding -o crt0.leros.o
mv crt0.leros.o "$BUILD_CLANG_LIB_DIR/crt0.leros.o"

# Build the runtime library
RUNTIME_LIBRARY_NAME="libclang_rt.builtins-leros32"
cd ..
/bin/bash build_builtins.sh "$BUILD_BIN_DIR"
cd build-leros-compiler-rt-builtins
cd lib
cd linux
mv "$RUNTIME_LIBRARY_NAME" "$BUILD_CLANG_LIB_DIR/lib/$RUNTIME_LIBRARY_NAME"


# Download crt0.leros.o from leros-lib
cd bin
wget "https://github.com/leros-dev/leros-lib/blob/master/runtime/crt0.leros.o"

