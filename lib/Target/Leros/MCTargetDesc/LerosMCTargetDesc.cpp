//===- LerosMCTargetDesc.cpp - Leros Target Descriptions ------------*- C++
//-*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file provides Leros specific target descriptions.
//
//===----------------------------------------------------------------------===//

#include "LerosMCTargetDesc.h"
#include "InstPrinter/LerosInstPrinter.h"
#include "LerosMCAsmInfo.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Support/TargetRegistry.h"

#define GET_INSTRINFO_MC_DESC
#include "LerosGenInstrInfo.inc"

#define GET_SUBTARGETINFO_MC_DESC
#include "LerosGenSubtargetInfo.inc"

#define GET_REGINFO_MC_DESC
#include "LerosGenRegisterInfo.inc"

namespace llvm {

static MCInstrInfo *createLerosMCInstrInfo() {
  auto *X = new MCInstrInfo();
  InitLerosMCInstrInfo(X);
  return X;
}

static MCRegisterInfo *createLerosMCRegisterInfo(const Triple &TT) {
  auto *X = new MCRegisterInfo();
  InitLerosMCRegisterInfo(X, Leros::R1);
  return X;
}

static MCInstPrinter *createLerosMCInstPrinter(const Triple &T,
                                               unsigned SyntaxVariant,
                                               const MCAsmInfo &MAI,
                                               const MCInstrInfo &MII,
                                               const MCRegisterInfo &MRI) {
  return new LerosInstPrinter(MAI, MII, MRI);
}

static MCSubtargetInfo *
createLerosMCSubtargetInfo(const Triple &TT, StringRef CPU, StringRef FS) {
  return createLerosMCSubtargetInfoImpl(TT, CPU, FS);
}

static MCAsmInfo *createLerosMCAsmInfo(const MCRegisterInfo &MRI,
                                       const Triple &TT) {
  return new LerosMCAsmInfo(TT);
}

// Force static initialization.
extern "C" void LLVMInitializeLerosTargetMC() {
  // Register the MC asm info.
  for (Target *T : {&getTheLeros32Target(), &getTheLeros64Target()}) {
    TargetRegistry::RegisterMCAsmInfo(*T, createLerosMCAsmInfo);
    TargetRegistry::RegisterMCInstrInfo(*T, createLerosMCInstrInfo);
    TargetRegistry::RegisterMCRegInfo(*T, createLerosMCRegisterInfo);
    TargetRegistry::RegisterMCSubtargetInfo(*T, createLerosMCSubtargetInfo);
    TargetRegistry::RegisterMCInstPrinter(*T, createLerosMCInstPrinter);
  }
}
}
