
//===-- LerosTargetObjectFile.h - Leros Object Info -*- C++ ---------*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_Leros_LerosTARGETOBJECTFILE_H
#define LLVM_LIB_TARGET_Leros_LerosTARGETOBJECTFILE_H

#include "llvm/CodeGen/TargetLoweringObjectFileImpl.h"

namespace llvm {
class LerosTargetMachine;

/// This implementation is used for Leros ELF targets.
class LerosELFTargetObjectFile : public TargetLoweringObjectFileELF {
  void Initialize(MCContext &Ctx, const TargetMachine &TM) override;
};

} // end namespace llvm

#endif
