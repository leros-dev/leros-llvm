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

void LerosInstrInfo::expandRRR(MachineBasicBlock &MBB, MachineInstr &MI) const {
#define OPCASE(instr, postfix)                                                 \
  case instr##_R##postfix##_PSEUDO:                                            \
    opcode = instr##_A##postfix;                                               \
    break;

  unsigned opcode = MI.getDesc().getOpcode();
  switch (opcode) {
  default:
    llvm_unreachable("Unhandled opcode");
    break;
    OPCASE(Leros::ADD, R)
    OPCASE(Leros::SUB, R)
    OPCASE(Leros::SHR, R)
    OPCASE(Leros::AND, R)
    OPCASE(Leros::OR, R)
    OPCASE(Leros::XOR, R)
  }
#undef OPCASE
  const unsigned &dst = MI.getOperand(0).getReg(),
                 &rs1 = MI.getOperand(1).getReg(),
                 &rs2 = MI.getOperand(2).getReg();
  BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::LOAD_R)).addReg(rs1);
  BuildMI(MBB, MI, MI.getDebugLoc(), get(opcode)).addReg(rs2);
  BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::STORE)).addReg(dst);
}

void LerosInstrInfo::expandRRI(MachineBasicBlock &MBB, MachineInstr &MI) const {
#define OPCASE(instr, postfix)                                                 \
  case instr##_R##postfix##_PSEUDO:                                            \
    opcode = instr##_A##postfix;                                               \
    break;

  unsigned opcode = MI.getDesc().getOpcode();
  switch (opcode) {
  default:
    llvm_unreachable("Unhandled opcode");
    break;
    OPCASE(Leros::ADD, I)
    OPCASE(Leros::SUB, I)
    OPCASE(Leros::SHR, I)
    OPCASE(Leros::AND, I)
    OPCASE(Leros::OR, I)
    OPCASE(Leros::XOR, I)
    OPCASE(Leros::LOADH, I)
    OPCASE(Leros::LOADH2, I)
    OPCASE(Leros::LOADH3, I)
  }
#undef OPCASE
  const unsigned &dst = MI.getOperand(0).getReg(),
                 &rs1 = MI.getOperand(1).getReg(),
                 &imm = MI.getOperand(2).getImm();
  BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::LOAD_R)).addReg(rs1);
  BuildMI(MBB, MI, MI.getDebugLoc(), get(opcode)).addImm(imm);
  BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::STORE)).addReg(dst);
}

void LerosInstrInfo::expandRI(MachineBasicBlock &MBB, MachineInstr &MI) const {
  unsigned opcode = MI.getDesc().getOpcode();
  switch (opcode) {
  default:
    llvm_unreachable("Unhandled opcode");
    break;
  case Leros::LOAD_RI_PSEUDO: {
    opcode = Leros::LOAD_R;
    break;
  }
  }
  const unsigned &dst = MI.getOperand(0).getReg(),
                 &imm = MI.getOperand(1).getImm();
  BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::LOAD_I)).addImm(imm);
  BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::STORE)).addReg(dst);
}

void LerosInstrInfo::expandBRCC(MachineBasicBlock &MBB,
                                MachineInstr &MI) const {
#define OPCASE(instr)                                                          \
  case instr##_PSEUDO:                                                         \
    opcode = instr##_IMPL;                                                     \
    break;

  unsigned opcode = MI.getDesc().getOpcode();
  switch (opcode) {
  default:
    llvm_unreachable("Unhandled opcode");
    break;
    OPCASE(Leros::BR)
    OPCASE(Leros::BRZ)
    OPCASE(Leros::BRNZ)
    OPCASE(Leros::BRP)
    OPCASE(Leros::BRN)
  }
#undef OPCASE
  const unsigned &rs1 = MI.getOperand(0).getReg(),
                 &rs2 = MI.getOperand(1).getReg();
  const auto &bb = MI.getOperand(2).getMBB();
  BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::LOAD_R)).addReg(rs1);
  BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::SUB_AR)).addReg(rs2);
  BuildMI(MBB, MI, MI.getDebugLoc(), get(opcode)).addMBB(bb);
}

void LerosInstrInfo::expandBR(MachineBasicBlock &MBB, MachineInstr &MI) const {
  const auto &bb = MI.getOperand(0).getMBB();
  BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::BR_IMPL)).addMBB(bb);
}

bool LerosInstrInfo::expandPostRAPseudo(MachineInstr &MI) const {
  MachineBasicBlock &MBB = *MI.getParent();

  switch (MI.getDesc().TSFlags) {
  default:
    llvm_unreachable("All pseudo-instructions must be expandable");
    break;
  case LEROSIF::RRR: {
    expandRRR(MBB, MI);
    break;
  }
  case LEROSIF::RRI: {
    expandRRI(MBB, MI);
    break;
  }
  case LEROSIF::RI: {
    expandRI(MBB, MI);
    break;
  }
  case LEROSIF::BranchCC: {
    expandBRCC(MBB, MI);
    break;
  }
  case LEROSIF::Branch: {
    expandBR(MBB, MI);
    break;
  }
  case LEROSIF::NoFormat: {
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
} // namespace llvm
