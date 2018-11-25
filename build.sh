#!/bin/bash

CLANG_VERSION="8.0.0"
BUILD_DIRECTORY=build-leros-llvm

# Make sure current working directory is leros-llvm
cd "${0%/*}"

SOURCE_ROOT=$(pwd)
PARENT_DIR="$(dirname "${SOURCE_ROOT}")"

# Binary folder of the built compiler
BUILD_ROOT="$PARENT_DIR/$BUILD_DIRECTORY"
BUILD_BIN_DIR="$BUILD_ROOT/bin"
BUILD_CLANG_LIB_DIR="$BUILD_ROOT/lib/clang/$CLANG_VERSION"

# Create build folder next to source
mkdir $BUILD_ROOT
cd $BUILD_ROOT

# Run CMake configuration of leros-llvm inside build directory
cmake -DLLVM_TOOL_CLANG_BUILD=OFF -DLLVM_TARGETS_TO_BUILD="" -DLLVM_DEFAULT_TARGET_TRIPLE="leros32-unknown-elf" -DLLVM_EXPERIMENTAL_TARGETS_TO_BUILD="Leros" -DCMAKE_BUILD_TYPE="Release" $SOURCE_ROOT

# Use all cores when building
if [[ "$OSTYPE" == "linux-gnu" ]]; then
        export MAKEFLAGS=-j$(nproc)
elif [[ "$OSTYPE" == "darwin"* ]]; then
        export MAKEFLAGS=-j$(sysctl -n hw.ncpu)
fi

# Build
cmake --build .

# ============ leros-lib =================
LEROS_LIB_DIR="$PARENT_DIR/leros-lib"
LEROS_LIB_DIR="$PARENT_DIR/leros-lib"
cd "$PARENT_DIR"

# Check if the leros-lib repo is checked out next to the leros-llvm repository
if [ -d "$LEROS_LIB_DIR" ]; then
  # make sure the repository is up-to-date
  cd $LEROS_LIB_DIR
  git pull
else
  # clone the leros-lib repository
  git clone https://github.com/leros-dev/leros-lib
  cd $LEROS_LIB_DIR
fi

# Build crt0.leros.o
cd runtime
$BUILD_BIN_DIR/clang crt0.leros.c -c -ffreestanding -o crt0.leros.o
mv crt0.leros.o "$BUILD_CLANG_LIB_DIR/crt0.leros.o"

# Build the runtime library
RUNTIME_LIBRARY_NAME="libclang_rt.builtins-leros32.a"
cd ..
/bin/bash build_runtime.sh "$BUILD_BIN_DIR"
cd "build-leros-compiler-rt-builtins"
cd "lib"
cd "linux"
mkdir "${BUILD_CLANG_LIB_DIR}/lib"
mv "$RUNTIME_LIBRARY_NAME" "$BUILD_CLANG_LIB_DIR/lib/$RUNTIME_LIBRARY_NAME"
