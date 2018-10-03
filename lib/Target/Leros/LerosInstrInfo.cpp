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
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/TargetRegistry.h"

#define GET_INSTRINFO_CTOR_DTOR
#include "LerosGenInstrInfo.inc"

namespace llvm {

LerosInstrInfo::LerosInstrInfo()
    : LerosGenInstrInfo(Leros::ADJCALLSTACKDOWN, Leros::ADJCALLSTACKUP) {}

void LerosInstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                                 MachineBasicBlock::iterator MBBI,
                                 const DebugLoc &DL, unsigned DstReg,
                                 unsigned SrcReg, bool KillSrc) const {
  expandMOV(MBB, MBBI, false, DstReg, SrcReg);
  return;
}

bool LerosInstrInfo::isBranchOffsetInRange(unsigned, int64_t BrOffset) const {
  return isInt<8>(BrOffset);
}

void LerosInstrInfo::movImm32(MachineBasicBlock &MBB,
                              MachineBasicBlock::iterator MBBI,
                              const DebugLoc &DL, unsigned DstReg,
                              uint64_t val) const {
  assert(isInt<32>(val) && "Can only materialize 32-bit constants");

  // Materialize a constant into a register through repeated usage of loadh
  // operations
  if (isInt<8>(val)) {
    BuildMI(MBB, MBBI, DL, get(Leros::LOAD_I)).addImm((val)&0xff);
  } else if (isInt<16>(val)) {
    BuildMI(MBB, MBBI, DL, get(Leros::LOAD_I)).addImm((val)&0xff);
    BuildMI(MBB, MBBI, DL, get(Leros::LOADH_AI)).addImm((val >> 8) & 0xff);
  } else if (isInt<24>(val)) {
    BuildMI(MBB, MBBI, DL, get(Leros::LOAD_I)).addImm(val & 0xff);
    BuildMI(MBB, MBBI, DL, get(Leros::LOADH_AI)).addImm((val >> 8) & 0xff);
    BuildMI(MBB, MBBI, DL, get(Leros::LOADH2_AI)).addImm((val >> 16) & 0xff);
  } else {
    BuildMI(MBB, MBBI, DL, get(Leros::LOAD_I)).addImm((val)&0xff);
    BuildMI(MBB, MBBI, DL, get(Leros::LOADH_AI)).addImm((val >> 8) & 0xff);
    BuildMI(MBB, MBBI, DL, get(Leros::LOADH2_AI)).addImm((val >> 16) & 0xff);
    BuildMI(MBB, MBBI, DL, get(Leros::LOADH3_AI)).addImm((val >> 24) & 0xff);
  }

  BuildMI(MBB, MBBI, DL, get(Leros::STORE_R), DstReg);
}

void LerosInstrInfo::expandMOV(MachineBasicBlock &MBB,
                               MachineBasicBlock::iterator I,
                               bool BBHasOperands, unsigned dst,
                               unsigned src) const {
  if (BBHasOperands) {
    dst = I->getOperand(1).getReg();
    src = I->getOperand(2).getReg();
  }
  BuildMI(MBB, I, I->getDebugLoc(), get(Leros::LOAD_R)).addReg(src);
  BuildMI(MBB, I, I->getDebugLoc(), get(Leros::STORE_R)).addReg(dst);
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
  BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::STORE_R)).addReg(dst);
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
  BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::STORE_R)).addReg(dst);
}

void LerosInstrInfo::expandRI(MachineBasicBlock &MBB, MachineInstr &MI) const {
  const unsigned &dst = MI.getOperand(0).getReg(),
                 &imm = MI.getOperand(1).getImm();
  BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::LOAD_I)).addImm(imm);
  BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::STORE_R)).addReg(dst);
}

unsigned LerosInstrInfo::getInstSizeInBytes(const MachineInstr &MI) const {
  return 2; // All leros instructions are 16 bits long
}

void LerosInstrInfo::loadRegFromStackSlot(MachineBasicBlock &MBB,
                                          MachineBasicBlock::iterator I,
                                          unsigned DstReg, int FrameIndex,
                                          const TargetRegisterClass *RC,
                                          const TargetRegisterInfo *TRI) const {
  DebugLoc DL;
  if (I != MBB.end())
    DL = I->getDebugLoc();

  BuildMI(MBB, I, DL, get(Leros::LOAD_M_PSEUDO), DstReg)
      .addFrameIndex(FrameIndex)
      .addImm(0);
}

void LerosInstrInfo::expandCALL(MachineBasicBlock &MBB,
                                MachineInstr &MI) const {
  // For now, do not expand call - this should be done in some MCCodeEmitter by
  // the linker

  /*
unsigned Ra = Leros::R0;
unsigned opcode = MI.getDesc().getOpcode();

MCOperand &Func = MI.getOperand(0);
const MCExpr *Expr = Func.getExpr();

// Load function address
movImm32(MBB, MI, MI.getDebugLoc(), Ra, Expr);

// Create function call expression CallExpr for JAL.
BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::JAL))
    .addReg(Ra, RegState::Define);
    */
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

void LerosInstrInfo::expandLS(MachineBasicBlock &MBB, MachineInstr &MI) const {
  const unsigned &rs2 = MI.getOperand(0).getReg(),
                 &rs1 = MI.getOperand(1).getReg();
  const auto &imm = MI.getOperand(2).getImm();
  unsigned opcode = MI.getDesc().getOpcode();

  if (opcode == Leros::STORE_M_PSEUDO) {
    BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::LOAD_R)).addReg(rs2);
    BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::LDADDR)).addReg(rs1);
    BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::STIND)).addImm(imm);
  } else if (opcode == Leros::LOAD_M_PSEUDO) {
    BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::LDADDR)).addReg(rs1);
    BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::LDIND)).addImm(imm);
    BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::STORE_R)).addReg(rs2);
  } else {
    llvm_unreachable("Unknown opcode for LS pseudoinstruction format");
  }
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
  case LEROSIF::LoadStore: {
    expandLS(MBB, MI);
    break;
  }
  case LEROSIF::NoFormat: {
    const auto opc = MI.getDesc().getOpcode();
    switch (opc) {
    default:
      return false;
    case Leros::PseudoCALL:
      expandCALL(MBB, MI);
      break;
    case Leros::MOV:
      expandMOV(MBB, MI, true);
      break;
    case Leros::RET:
      expandRET(MBB, MI);
      break;
    }
  }
  }

  if (MI.getDesc().getOpcode() != Leros::PseudoCALL)
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

  BuildMI(MBB, I, DL, get(Leros::STORE_M_PSEUDO))
      .addReg(SrcReg, getKillRegState(IsKill))
      .addFrameIndex(FI)
      .addImm(0);
}
} // namespace llvm
