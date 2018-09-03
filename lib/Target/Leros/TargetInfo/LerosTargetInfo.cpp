//===- LerosTargetInfo.cpp - Leros Target Implementation ----------- *- C++
//-*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "Leros.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

Target &llvm::getTheLeros16Target() {
  static Target TheLeros16Target;
  return TheLeros16Target;
}

Target &llvm::getTheLeros32Target() {
  static Target TheLeros32Target;
  return TheLeros32Target;
}

Target &llvm::getTheLeros64Target() {
  static Target TheLeros64Target;
  return TheLeros64Target;
}

extern "C" void LLVMInitializeLerosTargetInfo() {
  RegisterTarget<Triple::Leros> X(getTheLeros16Target(), "Leros16", "Leros 16",
                                  "Leros");
  RegisterTarget<Triple::Leros> Y(getTheLeros32Target(), "Leros32", "Leros 32",
                                  "Leros");
  RegisterTarget<Triple::Leros> Z(getTheLeros32Target(), "Leros64", "Leros 64",
                                  "Leros");
}
