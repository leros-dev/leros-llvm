//===- LerosSubtarget.cpp - Leros Subtarget Information ---------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the Leros specific subclass of TargetSubtarget.
//
//===----------------------------------------------------------------------===//

#include "LerosSubtarget.h"

using namespace llvm;

void LerosSubtarget::initializeSubtargetDependencies(StringRef CPU,
                                                     StringRef FS,
                                                     const Triple &TT) {
  std::string CPUName = CPU;
  if (CPUName.empty()) {
    if (TT.isArch16Bit()) {
      CPUName = "generic-leros16";
    } else if (TT.isArch32Bit()) {
      CPUName = "generic-leros32";
    } else if (TT.isArch64Bit()) {
      CPUName = "generic-leros64";
    }
  }
  ParseSubtargetFeatures(CPUName, FS);
}

LerosSubtarget::LerosSubtarget(const Triple &TT, const std::string &CPU,
                               const std::string &FS, const TargetMachine &TM)
    : LerosGenSubtargetInfo(TT, CPU, FS) {

  initializeSubtargetDependencies(CPU, FS, TT);
}
