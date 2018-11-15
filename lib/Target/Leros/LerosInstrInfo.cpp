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

#define DEBUG_TYPE "leros-instr-info"

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
                              const DebugLoc &DL, unsigned DstReg, uint64_t val,
                              MachineInstr::MIFlag Flag) const {
  assert(isInt<32>(val) && "Can only materialize 32-bit constants");

  // Materialize a constant into a register through repeated usage of loadh
  // operations
  if (isInt<8>(val)) {
    BuildMI(MBB, MBBI, DL, get(Leros::LOAD_I))
        .addImm((val)&0xff)
        .setMIFlag(Flag);
  } else if (isInt<16>(val)) {
    BuildMI(MBB, MBBI, DL, get(Leros::LOAD_I))
        .addImm((val)&0xff)
        .setMIFlag(Flag);
    BuildMI(MBB, MBBI, DL, get(Leros::LOADH_AI))
        .addImm((val >> 8) & 0xff)
        .setMIFlag(Flag);
  } else if (isInt<24>(val)) {
    BuildMI(MBB, MBBI, DL, get(Leros::LOAD_I))
        .addImm(val & 0xff)
        .setMIFlag(Flag);
    BuildMI(MBB, MBBI, DL, get(Leros::LOADH_AI))
        .addImm((val >> 8) & 0xff)
        .setMIFlag(Flag);
    BuildMI(MBB, MBBI, DL, get(Leros::LOADH2_AI))
        .addImm((val >> 16) & 0xff)
        .setMIFlag(Flag);
  } else {
    BuildMI(MBB, MBBI, DL, get(Leros::LOAD_I))
        .addImm((val)&0xff)
        .setMIFlag(Flag);
    BuildMI(MBB, MBBI, DL, get(Leros::LOADH_AI))
        .addImm((val >> 8) & 0xff)
        .setMIFlag(Flag);
    BuildMI(MBB, MBBI, DL, get(Leros::LOADH2_AI))
        .addImm((val >> 16) & 0xff)
        .setMIFlag(Flag);
    BuildMI(MBB, MBBI, DL, get(Leros::LOADH3_AI))
        .addImm((val >> 24) & 0xff)
        .setMIFlag(Flag);
  }

  BuildMI(MBB, MBBI, DL, get(Leros::STORE_R), DstReg).setMIFlag(Flag);
}

void LerosInstrInfo::movUImm32(MachineBasicBlock &MBB,
                               MachineBasicBlock::iterator MBBI,
                               const DebugLoc &DL, unsigned DstReg,
                               uint64_t Val) const {
  assert(isUInt<32>(Val) && "Can only materialize 32-bit constants");

  movImm32(MBB, MBBI, DL, DstReg, static_cast<int32_t>(Val));
}

void LerosInstrInfo::expandMOV(MachineBasicBlock &MBB,
                               MachineBasicBlock::iterator I,
                               bool BBHasOperands, unsigned dst,
                               unsigned src) const {
  if (BBHasOperands) {
    dst = I->getOperand(0).getReg();
    src = I->getOperand(1).getReg();
  }
  BuildMI(MBB, I, I->getDebugLoc(), get(Leros::LOAD_R)).addReg(src);
  BuildMI(MBB, I, I->getDebugLoc(), get(Leros::STORE_R)).addReg(dst);
}

void LerosInstrInfo::expandRET(MachineBasicBlock &MBB,
                               MachineBasicBlock::iterator I) const {
  BuildMI(MBB, I, I->getDebugLoc(), get(Leros::LOAD_R)).addReg(Leros::R0);
  BuildMI(MBB, I, I->getDebugLoc(), get(Leros::JAL_ret)).addReg(Leros::R0);
}

void LerosInstrInfo::expandSHR(MachineBasicBlock &MBB, MachineInstr &I) const {
  const unsigned &dst = I.getOperand(0).getReg(),
                 &src = I.getOperand(1).getReg();
  auto DL = I.getDebugLoc();
  BuildMI(MBB, I, DL, get(Leros::LOAD_R)).addReg(src);
  // Reg and immediate does nothing, but required for the signature of SHR
  BuildMI(MBB, I, DL, get(Leros::SHR_IMPL)).addImm(0);
  BuildMI(MBB, I, DL, get(Leros::STORE_R)).addReg(dst);
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
    OPCASE(Leros::AND, I)
    OPCASE(Leros::OR, I)
    OPCASE(Leros::XOR, I)
    OPCASE(Leros::LOADH, I)
    OPCASE(Leros::LOADH2, I)
    OPCASE(Leros::LOADH3, I)
  }
#undef OPCASE
  const unsigned &dst = MI.getOperand(0).getReg(),
                 &rs1 = MI.getOperand(1).getReg();

  BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::LOAD_R)).addReg(rs1);
  if (MI.getOperand(2).getType() == MachineOperand::MO_GlobalAddress) {
    BuildMI(MBB, MI, MI.getDebugLoc(), get(opcode))
        .addGlobalAddress(MI.getOperand(2).getGlobal());
  } else if (MI.getOperand(2).getType() == MachineOperand::MO_BlockAddress) {
    BuildMI(MBB, MI, MI.getDebugLoc(), get(opcode))
        .addBlockAddress(MI.getOperand(2).getBlockAddress());
  } else {
    BuildMI(MBB, MI, MI.getDebugLoc(), get(opcode))
        .addImm(MI.getOperand(2).getImm());
  }
  BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::STORE_R)).addReg(dst);
}

void LerosInstrInfo::expandRI(MachineBasicBlock &MBB, MachineInstr &MI) const {
  const auto DL = MI.getDebugLoc();
  const unsigned &dst = MI.getOperand(0).getReg();

  if (MI.getOperand(1).getType() == MachineOperand::MO_GlobalAddress) {
    BuildMI(MBB, MI, DL, get(Leros::LOAD_I))
        .addGlobalAddress(MI.getOperand(1).getGlobal());
  } else if (MI.getOperand(1).getType() == MachineOperand::MO_BlockAddress) {
    BuildMI(MBB, MI, DL, get(Leros::LOAD_I))
        .addBlockAddress(MI.getOperand(1).getBlockAddress());
  } else {
    BuildMI(MBB, MI, DL, get(Leros::LOAD_I)).addImm(MI.getOperand(1).getImm());
  }

  BuildMI(MBB, MI, DL, get(Leros::STORE_R)).addReg(dst);
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
  const auto DL = MI.getDebugLoc();

  auto operand = MI.getOperand(0);

  if (operand.getType() == MachineOperand::MO_GlobalAddress) {
    auto op = operand.getGlobal();
    BuildMI(MBB, MI, DL, get(Leros::LOAD_I))
        .addGlobalAddress(op, 0, LEROSTF::MO_B0);
    BuildMI(MBB, MI, DL, get(Leros::LOADH_AI))
        .addGlobalAddress(op, 0, LEROSTF::MO_B1);
    BuildMI(MBB, MI, DL, get(Leros::LOADH2_AI))
        .addGlobalAddress(op, 0, LEROSTF::MO_B2);
    BuildMI(MBB, MI, DL, get(Leros::LOADH3_AI))
        .addGlobalAddress(op, 0, LEROSTF::MO_B3);
  } else if (operand.getType() == MachineOperand::MO_BlockAddress) {
    auto op = operand.getBlockAddress();
    BuildMI(MBB, MI, DL, get(Leros::LOAD_I))
        .addBlockAddress(op, 0, LEROSTF::MO_B0);
    BuildMI(MBB, MI, DL, get(Leros::LOADH_AI))
        .addBlockAddress(op, 0, LEROSTF::MO_B1);
    BuildMI(MBB, MI, DL, get(Leros::LOADH2_AI))
        .addBlockAddress(op, 0, LEROSTF::MO_B2);
    BuildMI(MBB, MI, DL, get(Leros::LOADH3_AI))
        .addBlockAddress(op, 0, LEROSTF::MO_B3);
  }

  BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::JAL_call)).addReg(Leros::R0);
}

// Inserts a branch into the end of the specific MachineBasicBlock, returning
// the number of instructions inserted.
unsigned LerosInstrInfo::insertBranch(
    MachineBasicBlock &MBB, MachineBasicBlock *TBB, MachineBasicBlock *FBB,
    ArrayRef<MachineOperand> Cond, const DebugLoc &DL, int *BytesAdded) const {
  if (BytesAdded)
    *BytesAdded = 0;

  // Shouldn't be a fall through.
  assert(TBB && "InsertBranch must not be told to insert a fallthrough");
  assert((Cond.size() == 3 || Cond.size() == 0) &&
         "Leros pseudo branch conditions have two components!");

  // Unconditional branch.
  if (Cond.empty()) {
    MachineInstr &MI = *BuildMI(&MBB, DL, get(Leros::PseudoBR)).addMBB(TBB);
    if (BytesAdded)
      *BytesAdded += getInstSizeInBytes(MI);
    return 1;
  }

  // Either a one or two-way conditional branch.
  unsigned Opc = Cond[0].getImm();
  MachineInstr &CondMI =
      *BuildMI(&MBB, DL, get(Opc)).add(Cond[1]).add(Cond[2]).addMBB(TBB);
  if (BytesAdded)
    *BytesAdded += getInstSizeInBytes(CondMI);

  // One-way conditional branch.
  if (!FBB)
    return 1;

  // Two-way conditional branch.
  MachineInstr &MI = *BuildMI(&MBB, DL, get(Leros::PseudoBR)).addMBB(FBB);
  if (BytesAdded)
    *BytesAdded += getInstSizeInBytes(MI);
  return 2;
}

void LerosInstrInfo::expandBRCC(MachineBasicBlock &MBB, MachineInstr &MI,
                                bool hasPrecalcCC) const {
#define OPCASE(instr)                                                          \
  case instr##_PSEUDO:                                                         \
    opcode = instr##_IMPL;                                                     \
    break;

  unsigned opcode = MI.getDesc().getOpcode();
  switch (opcode) {
  default:
    llvm_unreachable("Unhandled opcode");
    break;
    OPCASE(Leros::BRZ)
    OPCASE(Leros::BRNZ)
    OPCASE(Leros::BRP)
    OPCASE(Leros::BRN)
  case Leros::PseudoBRNZ: {
    opcode = Leros::BRNZ_IMPL;
    break;
  }
  case Leros::PseudoBRZ: {
    opcode = Leros::BRZ_IMPL;
    break;
  }
  case Leros::PseudoBRP: {
    opcode = Leros::BRP_IMPL;
    break;
  }
  case Leros::PseudoBRN: {
    opcode = Leros::BRN_IMPL;
    break;
  }
  }
#undef OPCASE

  MachineBasicBlock *bb;
  if (opcode != Leros::BR_IMPL) {
    bb = MI.getOperand(hasPrecalcCC ? 1 : 2).getMBB();
    const unsigned &rs1 = MI.getOperand(0).getReg();

    BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::LOAD_R)).addReg(rs1);
    if (!hasPrecalcCC) {
      BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::SUB_AR))
          .addReg(MI.getOperand(1).getReg());
    }
  } else {
    bb = MI.getOperand(0).getMBB();
  }

  BuildMI(MBB, MI, MI.getDebugLoc(), get(opcode)).addMBB(bb);
}

void LerosInstrInfo::expandBR(MachineBasicBlock &MBB, MachineInstr &MI) const {
  const auto &bb = MI.getOperand(0).getMBB();
  BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::BR_IMPL)).addMBB(bb);
}

void LerosInstrInfo::expandBRIND(MachineBasicBlock &MBB,
                                 MachineInstr &MI) const {
  const auto &reg = MI.getOperand(0).getReg();
  // Since we do not want to link here, we use a scratch register to dump the
  // linked address into
  auto scratchReg = Leros::GPRPseudoExpandRegClass.getRegister(0);
  BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::LOAD_R)).addReg(reg);
  BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::JAL_call)).addReg(scratchReg);
}

void LerosInstrInfo::expandNOP(MachineBasicBlock &MBB, MachineInstr &MI) const {
  BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::NOP_IMPL)).addImm(0);
}

void LerosInstrInfo::expandLS(MachineBasicBlock &MBB, MachineInstr &MI) const {
  const unsigned &rs2 = MI.getOperand(0).getReg(),
                 &rs1 = MI.getOperand(1).getReg();
  const auto &imm = MI.getOperand(2).getImm();
  unsigned opcode = MI.getDesc().getOpcode();
  DebugLoc DL = MI.getDebugLoc();

  switch (opcode) {
  default:
    llvm_unreachable("Unknown opcode for LS pseudoinstruction format");
  case Leros::STORE_M_PSEUDO: {
    BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::LOAD_R)).addReg(rs2);
    BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::LDADDR)).addReg(rs1);
    BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::STIND)).addImm(imm);
    break;
  }
  case Leros::STORE_8_M_PSEUDO: {
    BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::LOAD_R)).addReg(rs2);
    BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::LDADDR)).addReg(rs1);
    BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::STINDB)).addImm(imm);
    break;
  }
  case Leros::LOAD_U8_M_PSEUDO: {
    BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::LDADDR)).addReg(rs1);
    BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::LDINDBU)).addImm(imm);
    BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::STORE_R)).addReg(rs2);
    break;
  }
  case Leros::LOAD_U16_M_PSEUDO:
  case Leros::LOAD_M_PSEUDO: {
    BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::LDADDR)).addReg(rs1);
    BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::LDIND)).addImm(imm);
    BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::STORE_R)).addReg(rs2);

    // Check whether we have to zero or sign extend the load if this was not a
    // full-word load
    switch (MI.getDesc().TSFlags) {
    default: {
      // Full word load, we exit here
      break;
    }
    case LEROSIF::Unsigned16BitLoad: {
      // Mask the lower halfword
      // Build the operand which we have to OR with. We use a register from
      // GPRPseudoExpandRegClass since we are post reg allocation
      auto scratchReg = Leros::GPRPseudoExpandRegClass.getRegister(0);
      movImm32(MBB, MI, MI.getDebugLoc(), scratchReg, 0xFFFF);
      BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::STORE_R))
          .addReg(scratchReg);
      BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::LOAD_R)).addReg(rs2);
      BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::AND_AR)).addReg(scratchReg);
      BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::STORE_R)).addReg(rs2);
      break;
    }
    }
    break;
  }
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
    expandBRCC(MBB, MI, false);
    break;
  }
  case LEROSIF::Branch: {
    expandBR(MBB, MI);
    break;
  }
  case LEROSIF::Signed8BitLoad:
  case LEROSIF::Signed16BitLoad:
  case LEROSIF::Unsigned8BitLoad:
  case LEROSIF::Unsigned16BitLoad:
  case LEROSIF::LoadStore: {
    expandLS(MBB, MI);
    break;
  }
  case LEROSIF::NoFormat: {
    const auto opc = MI.getDesc().getOpcode();
    switch (opc) {
    default:
      return false;
    case Leros::SHRByOne_Pseudo:
      expandSHR(MBB, MI);
      break;
    case Leros::NOP:
      expandNOP(MBB, MI);
      break;
    case Leros::PseudoCALL:
      expandCALL(MBB, MI);
      break;
    case Leros::MOV:
      expandMOV(MBB, MI, true);
      break;
    case Leros::RET:
      expandRET(MBB, MI);
      break;
    case Leros::PseudoBRZ:
    case Leros::PseudoBRNZ:
    case Leros::PseudoBRP:
    case Leros::PseudoBRN:
      expandBRCC(MBB, MI, true);
      break;
    case Leros::PseudoBRIND:
      expandBRIND(MBB, MI);
      break;
    }
  }
  }

  MBB.erase(MI);
  return true;
}
// The contents of values added to Cond are not examined outside of
// LerosInstrInfo, giving us flexibility in what to push to it. For Leros, we
// push BranchOpcode, Reg1, Reg2.
static void parseCondBranch(MachineInstr &LastInst, MachineBasicBlock *&Target,
                            SmallVectorImpl<MachineOperand> &Cond) {
  // Block ends with fall-through condbranch.
  assert(LastInst.getDesc().isConditionalBranch() &&
         "Unknown conditional branch");
  Target = LastInst.getOperand(2).getMBB();
  Cond.push_back(MachineOperand::CreateImm(LastInst.getOpcode()));
  Cond.push_back(LastInst.getOperand(0));
  Cond.push_back(LastInst.getOperand(1));
}

unsigned LerosInstrInfo::removeBranch(MachineBasicBlock &MBB,
                                      int *BytesRemoved) const {
  if (BytesRemoved)
    *BytesRemoved = 0;
  MachineBasicBlock::iterator I = MBB.getLastNonDebugInstr();
  if (I == MBB.end())
    return 0;

  if (!I->getDesc().isUnconditionalBranch() &&
      !I->getDesc().isConditionalBranch())
    return 0;

  // Remove the branch.
  I->eraseFromParent();
  if (BytesRemoved)
    *BytesRemoved += getInstSizeInBytes(*I);

  I = MBB.end();

  if (I == MBB.begin())
    return 1;
  --I;
  if (!I->getDesc().isConditionalBranch())
    return 1;

  // Remove the branch.
  I->eraseFromParent();
  if (BytesRemoved)
    *BytesRemoved += getInstSizeInBytes(*I);
  return 2;
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
