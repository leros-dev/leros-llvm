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

namespace llvm {

Target &getTheLeros32Target() {
  static Target TheLeros32Target;
  return TheLeros32Target;
}

Target &getTheLeros64Target() {
  static Target TheLeros64Target;
  return TheLeros64Target;
}

extern "C" void LLVMInitializeLerosTargetInfo() {
  RegisterTarget<Triple::leros32> Y(getTheLeros32Target(), "leros32",
                                    "32-bit Leros", "Leros");
  RegisterTarget<Triple::leros64> Z(getTheLeros64Target(), "leros64",
                                    "64-bit Leros", "Leros");
}
}
