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
    dst = I->getOperand(0).getReg();
    src = I->getOperand(1).getReg();
  }
  BuildMI(MBB, I, I->getDebugLoc(), get(Leros::LOAD_R)).addReg(src);
  BuildMI(MBB, I, I->getDebugLoc(), get(Leros::STORE_R)).addReg(dst);
}

void LerosInstrInfo::expandRET(MachineBasicBlock &MBB,
                               MachineBasicBlock::iterator I) const {
  BuildMI(MBB, I, I->getDebugLoc(), get(Leros::JAL_ret)).addReg(Leros::RA);
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
  const auto DL = MI.getDebugLoc();
  const unsigned &dst = MI.getOperand(0).getReg();
  if (MI.getOperand(1).getType() == MachineOperand::MO_GlobalAddress) {
    // Emit a load sequence for the global address that will get translated
    // during linking
    const auto GA = MI.getOperand(1).getGlobal();
    BuildMI(MBB, MI, DL, get(Leros::LOAD_I)).addGlobalAddress(GA);
    BuildMI(MBB, MI, DL, get(Leros::LOADH_AI)).addGlobalAddress(GA);
    BuildMI(MBB, MI, DL, get(Leros::LOADH2_AI)).addGlobalAddress(GA);
    BuildMI(MBB, MI, DL, get(Leros::LOADH3_AI)).addGlobalAddress(GA);
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
  case Leros::LOAD_S8_M_PSEUDO:
  case Leros::LOAD_S16_M_PSEUDO: {
    // Based on the load type, we now emit small algorithms for sign- or zero
    // extension
    const auto TF = MI.getDesc().TSFlags;
    switch (TF) {
    case LEROSIF::Signed8BitLoad:
    case LEROSIF::Signed16BitLoad: {
      // To insert a sign extension instruction, we insert a triangle pattern
      // control-flow pattern where, based on the sign of the input operand at
      // the
      // 7th or 15th bit position, we sign extend by 0 or 1
      const BasicBlock *LLVM_BB = MBB.getBasicBlock();
      MachineFunction::iterator I = ++MBB.getIterator();

      MachineBasicBlock *HeadMBB = &MBB;
      MachineFunction *F = HeadMBB->getParent();
      MachineBasicBlock *NegMBB = F->CreateMachineBasicBlock(LLVM_BB);
      MachineBasicBlock *endMBB = F->CreateMachineBasicBlock(LLVM_BB);

      F->insert(I, NegMBB);
      F->insert(I, endMBB);
      // Move all remaining instructions to TailMBB.
      endMBB->splice(endMBB->begin(), HeadMBB,
                     std::next(MachineBasicBlock::iterator(MI)),
                     HeadMBB->end());
      // Update machine-CFG edges by transferring all successors of the current
      // block to the new block which will contain the Phi node for the select.
      endMBB->transferSuccessorsAndUpdatePHIs(HeadMBB);
      // Set the successors for HeadMBB.
      HeadMBB->addSuccessor(NegMBB);
      HeadMBB->addSuccessor(endMBB);
      NegMBB->addSuccessor(endMBB);

      // Load memory operand
      BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::LDADDR)).addReg(rs1);
      BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::LDIND)).addImm(imm);

      // Extract sign from the loaded operand and insert branch
      BuildMI(HeadMBB, DL, get(Leros::SHR_AI))
          .addImm(TF == LEROSIF::Signed8BitLoad ? 7 : 15);
      BuildMI(HeadMBB, DL, get(Leros::AND_AI)).addImm(1);

      // Branch if sign == 1 (negative)
      BuildMI(HeadMBB, DL, get(Leros::BRZ_IMPL)).addMBB(NegMBB);

      // Positive sign extension (fallthrough from previous branch)
      if (LEROSIF::Signed8BitLoad) {
        BuildMI(HeadMBB, DL, get(Leros::LOADH_AI)).addImm(0x0);
      } else {
        BuildMI(HeadMBB, DL, get(Leros::LOADH2_AI)).addImm(0x0);
      }
      BuildMI(HeadMBB, DL, get(Leros::BRZ_IMPL)).addMBB(endMBB);

      // Negative sign extension
      if (LEROSIF::Signed8BitLoad) {
        BuildMI(HeadMBB, DL, get(Leros::LOADH_AI)).addImm(0xFF);
      } else {
        BuildMI(HeadMBB, DL, get(Leros::LOADH2_AI)).addImm(0xFF);
      }
      break;
    }
    }
    break;
  }
  case Leros::LOAD_U8_M_PSEUDO:
  case Leros::LOAD_U16_M_PSEUDO:
  case Leros::LOAD_M_PSEUDO: {
    BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::LDADDR)).addReg(rs1);
    BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::LDIND)).addImm(imm);

    // Check whether we have to zero or sign extend the load if this was not a
    // full-word load
    switch (MI.getDesc().TSFlags) {
    default: {
      // Full word load, just store the value
      BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::STORE_R)).addReg(rs2);
    }
    case LEROSIF::Signed8BitLoad: {
      // todo implement this
      LLVM_DEBUG("Implement logic for signed masking and loading!");
      BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::STORE_R)).addReg(rs2);
      break;
    }
    case LEROSIF::Signed16BitLoad: {
      LLVM_DEBUG("Implement logic for signed masking and loading!");
      BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::STORE_R)).addReg(rs2);
      break;
    }
    case LEROSIF::Unsigned8BitLoad: {
      BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::AND_AI)).addImm(0xFF);
      BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::STORE_R)).addReg(rs2);
      break;
    }
    case LEROSIF::Unsigned16BitLoad: {
      BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::STORE_R)).addReg(rs2);
      // Build the operand which we have to OR with. We use a register from
      // GPRPseudoExpandRegClass since we are post reg allocation
      auto scratchReg = Leros::GPRPseudoExpandRegClass.getRegister(0);
      movImm32(MBB, MI, MI.getDebugLoc(), scratchReg, 0xFFFF);
      BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::LOADH2_AI))
          .addImm(0x0); // ensure that the topmost bits are 0
      BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::STORE_R))
          .addReg(scratchReg);
      BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::LOAD_R)).addReg(rs2);
      BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::AND_AR)).addReg(scratchReg);
      BuildMI(MBB, MI, MI.getDebugLoc(), get(Leros::STORE_R))
          .addReg(scratchReg);
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
      expandBRCC(MBB, MI, true);
    }
  }
  }

  if (MI.getDesc().getOpcode() != Leros::PseudoCALL)
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
