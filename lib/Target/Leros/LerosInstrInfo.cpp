//===-- LerosInstrInfo.cpp - Leros Instruction Information ------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the Leros implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#include "LerosInstrInfo.h"
#include "Leros.h"
#include "LerosSubtarget.h"
#include "LerosTargetMachine.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/RegisterScavenging.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/TargetRegistry.h"

#define GET_INSTRINFO_CTOR_DTOR
#include "LerosGenInstrInfo.inc"

namespace llvm {

void LerosInstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                                 MachineBasicBlock::iterator MBBI,
                                 const DebugLoc &DL, unsigned DstReg,
                                 unsigned SrcReg, bool KillSrc) const {
  if (Leros::GPRRegClass.contains(DstReg, SrcReg)) {
    BuildMI(MBB, MBBI, DL, get(Leros::INSTR_MOV), DstReg)
        .addReg(SrcReg, getKillRegState(KillSrc))
        .addImm(0);
    return;
  }
}

void LerosInstrInfo::expandMOV(MachineBasicBlock &MBB,
                               MachineBasicBlock::iterator I) const {
  const unsigned &dst = I->getOperand(1).getReg(),
                 &src = I->getOperand(2).getReg();
  BuildMI(MBB, I, I->getDebugLoc(), get(ISD::ADD)).addReg(src);
  BuildMI(MBB, I, I->getDebugLoc(), get(ISD::ADD)).addReg(dst);
}

bool LerosInstrInfo::expandPostRAPseudo(MachineInstr &MI) const {
  MachineBasicBlock &MBB = *MI.getParent();
  switch (MI.getDesc().getOpcode()) {
  default:
    return false;
  case Leros::INSTR_MOV:
    expandMOV(MBB, MI);
    break;
  }

  MBB.erase(MI);
  return true;
}
}
