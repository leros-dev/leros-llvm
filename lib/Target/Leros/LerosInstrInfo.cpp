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

  BuildMI(MBB, MBBI, DL, get(Leros::MOV), DstReg)
      .addReg(SrcReg, getKillRegState(KillSrc));

  return;
}

void LerosInstrInfo::expandMOV(MachineBasicBlock &MBB,
                               MachineBasicBlock::iterator I) const {
  const unsigned &dst = I->getOperand(1).getReg(),
                 &src = I->getOperand(2).getReg();
  BuildMI(MBB, I, I->getDebugLoc(), get(ISD::ADD)).addReg(src);
  BuildMI(MBB, I, I->getDebugLoc(), get(ISD::ADD)).addReg(dst);
}

void LerosInstrInfo::expandRET(MachineBasicBlock &MBB,
                               MachineBasicBlock::iterator I) const {
  const unsigned &dst = I->getOperand(0).getReg();
  BuildMI(MBB, I, I->getDebugLoc(), get(Leros::JAL)).addReg(dst);
}

void LerosInstrInfo::expandRR(MachineBasicBlock &MBB, MachineInstr &MI) const {
#define OPCASE(instr, postfix)                                                 \
  case Leros::##instr##_R##postfix##_PSEUDO:                                   \
    opcode = Leros::##instr##_A##postfix##;                                    \
    break;

  unsigned opcode;
  switch (MI.getDesc().getOpcode()) {
    OPCASE(ADD, R)
    OPCASE(SUB, R)
    OPCASE(SHR, R)
    OPCASE(AND, R)
    OPCASE(OR, R)
    OPCASE(XOR, R)
  }
#undef OPCASE
  const unsigned &dst = MI.getOperand(0).getReg(),
                 &rs1 = MI.getOperand(1).getReg(),
                 &rs2 = MI.getOperand(2).getReg();
  BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::LOAD_R)).addReg(rs1);
  BuildMI(MBB, MI, MI.getDebugLoc(), get(opcode)).addReg(rs2);
  BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::STORE)).addReg(dst);
}

bool LerosInstrInfo::expandPostRAPseudo(MachineInstr &MI) const {
  MachineBasicBlock &MBB = *MI.getParent();

  if (MI.getDesc().TSFlags & LEROSIF::RegisterRegister) {
    expandRR(MBB, MI);
  } else {
    switch (MI.getDesc().getOpcode()) {
    default:
      llvm_unreachable("All pseudo-instructions must be expandable");
      return false;
    case Leros::MOV:
      expandMOV(MBB, MI);
      break;
    case Leros::RET:
      expandRET(MBB, MI);
      break;
    }
  }

  MBB.erase(MI);
  return true;
}

void LerosInstrInfo::storeRegToStackSlot(MachineBasicBlock &MBB,
                                         MachineBasicBlock::iterator I,
                                         unsigned SrcReg, bool IsKill, int FI,
                                         const TargetRegisterClass *RC,
                                         const TargetRegisterInfo *TRI) const {
  DebugLoc DL;
  if (I != MBB.end())
    DL = I->getDebugLoc();

  if (SrcReg == Leros::ACC) {
    // Store directly to register
    BuildMI(MBB, I, DL, get(Leros::STORE))
        .addReg(SrcReg, getKillRegState(IsKill))
        .addFrameIndex(FI)
        .addImm(0);
  } else {
    // Load into accumulator and store
  }
}
}
