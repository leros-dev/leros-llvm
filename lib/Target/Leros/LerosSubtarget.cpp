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

LerosSubtarget::LerosSubtarget(const Triple &TT, const std::string &CPU,
                               const std::string &FS, const TargetMachine &TM)
    : LerosGenSubtargetInfo(TT, CPU, FS) {}
