//===- LerosMCTargetDesc.cpp - Leros Target Descriptions ------------*- C++ -*-===//
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
#include "LerosMCAsmInfo.h"
#include "LerosTargetStreamer.h"
#include "InstPrinter/LerosInstPrinter.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

#define GET_INSTRINFO_MC_DESC
#include "LerosGenInstrInfo.inc"

#define GET_SUBTARGETINFO_MC_DESC
#include "LerosGenSubtargetInfo.inc"

#define GET_REGINFO_MC_DESC
#include "LerosGenRegisterInfo.inc"

static MCInstrInfo *createLerosMCInstrInfo() {
  auto *X = new MCInstrInfo();
  InitLerosMCInstrInfo(X);
  return X;
}

static MCRegisterInfo *createLerosMCRegisterInfo(const Triple &TT) {
  auto *X = new MCRegisterInfo();
  InitLerosMCRegisterInfo(X, Leros::BLINK);
  return X;
}

static MCSubtargetInfo *createLerosMCSubtargetInfo(const Triple &TT,
                                                 StringRef CPU, StringRef FS) {
  return createLerosMCSubtargetInfoImpl(TT, CPU, FS);
}

static MCAsmInfo *createLerosMCAsmInfo(const MCRegisterInfo &MRI,
                                     const Triple &TT) {
  MCAsmInfo *MAI = new LerosMCAsmInfo(TT);

  // Initial state of the frame pointer is SP.
  MCCFIInstruction Inst = MCCFIInstruction::createDefCfa(nullptr, Leros::SP, 0);
  MAI->addInitialFrameState(Inst);

  return MAI;
}

static MCInstPrinter *createLerosMCInstPrinter(const Triple &T,
                                             unsigned SyntaxVariant,
                                             const MCAsmInfo &MAI,
                                             const MCInstrInfo &MII,
                                             const MCRegisterInfo &MRI) {
  return new LerosInstPrinter(MAI, MII, MRI);
}

LerosTargetStreamer::LerosTargetStreamer(MCStreamer &S) : MCTargetStreamer(S) {}
LerosTargetStreamer::~LerosTargetStreamer() = default;

static MCTargetStreamer *createTargetAsmStreamer(MCStreamer &S,
                                                 formatted_raw_ostream &OS,
                                                 MCInstPrinter *InstPrint,
                                                 bool isVerboseAsm) {
  return new LerosTargetStreamer(S);
}

// Force static initialization.
extern "C" void LLVMInitializeLerosTargetMC() {
  // Register the MC asm info.
  Target &TheLerosTarget = getTheLerosTarget();
  RegisterMCAsmInfoFn X(TheLerosTarget, createLerosMCAsmInfo);

  // Register the MC instruction info.
  TargetRegistry::RegisterMCInstrInfo(TheLerosTarget, createLerosMCInstrInfo);

  // Register the MC register info.
  TargetRegistry::RegisterMCRegInfo(TheLerosTarget, createLerosMCRegisterInfo);

  // Register the MC subtarget info.
  TargetRegistry::RegisterMCSubtargetInfo(TheLerosTarget,
                                          createLerosMCSubtargetInfo);

  // Register the MCInstPrinter
  TargetRegistry::RegisterMCInstPrinter(TheLerosTarget, createLerosMCInstPrinter);

  TargetRegistry::RegisterAsmTargetStreamer(TheLerosTarget,
                                            createTargetAsmStreamer);
}
