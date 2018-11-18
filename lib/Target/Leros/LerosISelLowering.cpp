//===-- LerosISelLowering.cpp - Leros DAG Lowering Implementation ---------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the LerosTargetLowering class.
//
//===----------------------------------------------------------------------===//
#include <iostream>

#include "Leros.h"
#include "LerosISelLowering.h"
#include "LerosMachineFunctionInfo.h"
#include "LerosRegisterInfo.h"
#include "LerosTargetMachine.h"
#include "llvm/CodeGen/CallingConvLower.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineJumpTableInfo.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/SelectionDAGISel.h"
#include "llvm/CodeGen/TargetLoweringObjectFileImpl.h"
#include "llvm/CodeGen/ValueTypes.h"
#include "llvm/IR/CallingConv.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/DerivedTypes.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/GlobalAlias.h"
#include "llvm/IR/GlobalVariable.h"
#include "llvm/IR/Intrinsics.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"

#include "llvm/Support/GraphWriter.h"

#define DEBUG_TYPE "leros-lower"

namespace llvm {

LerosTargetLowering::LerosTargetLowering(const TargetMachine &TM,
                                         const LerosSubtarget &STI)
    : TargetLowering(TM), Subtarget(STI) {

  // Set up the register classes.
  MVT XLenVT = Subtarget.getXLenVT();
  addRegisterClass(XLenVT, &Leros::GPRRegClass);

  setStackPointerRegisterToSaveRestore(Leros::R1);

  // Compute derived properties from the register classes
  computeRegisterProperties(Subtarget.getRegisterInfo());

  setOperationAction(ISD::BR_CC, XLenVT, Expand);
  setOperationAction(ISD::SELECT_CC, XLenVT, Expand);

  for (auto N : {ISD::EXTLOAD, ISD::SEXTLOAD, ISD::ZEXTLOAD})
    setLoadExtAction(N, XLenVT, MVT::i1, Promote);

  // TODO: add all necessary setOperationAction calls.
  setOperationAction(ISD::DYNAMIC_STACKALLOC, XLenVT, Expand);

  setOperationAction(ISD::BR_JT, MVT::Other, Expand);
  setOperationAction(ISD::BR_CC, XLenVT, Expand);
  setOperationAction(ISD::SELECT, XLenVT, Custom);
  setOperationAction(ISD::SELECT_CC, XLenVT, Expand);

  setOperationAction(ISD::STACKSAVE, MVT::Other, Expand);
  setOperationAction(ISD::STACKRESTORE, MVT::Other, Expand);

  setOperationAction(ISD::VASTART, MVT::Other, Expand);
  setOperationAction(ISD::VAARG, MVT::Other, Expand);
  setOperationAction(ISD::VACOPY, MVT::Other, Expand);
  setOperationAction(ISD::VAEND, MVT::Other, Expand);

  for (auto VT : {MVT::i1, MVT::i8, MVT::i16})
    setOperationAction(ISD::SIGN_EXTEND_INREG, VT, Expand);

  setOperationAction(ISD::UMUL_LOHI, XLenVT, Promote);
  setOperationAction(ISD::SMUL_LOHI, XLenVT, Promote);
  setOperationAction(ISD::MUL, XLenVT, Expand);
  setOperationAction(ISD::MULHS, XLenVT, Expand);
  setOperationAction(ISD::MULHU, XLenVT, Expand);
  setOperationAction(ISD::SDIV, XLenVT, Expand);
  setOperationAction(ISD::UDIV, XLenVT, Expand);
  setOperationAction(ISD::SREM, XLenVT, Expand);
  setOperationAction(ISD::UREM, XLenVT, Expand);

  setOperationAction(ISD::SDIVREM, XLenVT, Expand);
  setOperationAction(ISD::UDIVREM, XLenVT, Expand);

  setOperationAction(ISD::SHL_PARTS, XLenVT, Expand);
  setOperationAction(ISD::SRL_PARTS, XLenVT, Expand);
  setOperationAction(ISD::SRA_PARTS, XLenVT, Expand);

  setOperationAction(ISD::ROTL, XLenVT, Expand);
  setOperationAction(ISD::ROTR, XLenVT, Expand);
  setOperationAction(ISD::BSWAP, XLenVT, Expand);
  setOperationAction(ISD::CTTZ, XLenVT, Expand);
  setOperationAction(ISD::CTLZ, XLenVT, Expand);
  setOperationAction(ISD::CTPOP, XLenVT, Expand);

  ISD::CondCode FPCCToExtend[] = {
      ISD::SETOGT, ISD::SETOGE, ISD::SETONE, ISD::SETO,   ISD::SETUEQ,
      ISD::SETUGT, ISD::SETUGE, ISD::SETULT, ISD::SETULE, ISD::SETUNE,
      ISD::SETGT,  ISD::SETGE,  ISD::SETNE};

  setOperationAction(ISD::FMINNUM, MVT::f32, Expand);
  setOperationAction(ISD::FMAXNUM, MVT::f32, Expand);
  for (auto CC : FPCCToExtend)
    setCondCodeAction(CC, MVT::f32, Expand);
  setOperationAction(ISD::SELECT_CC, MVT::f32, Expand);
  setOperationAction(ISD::SELECT, MVT::f32, Expand);
  setOperationAction(ISD::BR_CC, MVT::f32, Expand);

  setOperationAction(ISD::FMINNUM, MVT::f64, Expand);
  setOperationAction(ISD::FMAXNUM, MVT::f64, Expand);
  for (auto CC : FPCCToExtend)
    setCondCodeAction(CC, MVT::f64, Expand);
  setOperationAction(ISD::SELECT_CC, MVT::f64, Expand);
  setOperationAction(ISD::SELECT, MVT::f64, Expand);
  setOperationAction(ISD::BR_CC, MVT::f64, Expand);
  setLoadExtAction(ISD::EXTLOAD, MVT::f64, MVT::f32, Expand);
  setTruncStoreAction(MVT::f64, MVT::f32, Expand);

  setOperationAction(ISD::GlobalAddress, XLenVT, Custom);
  setOperationAction(ISD::BlockAddress, XLenVT, Custom);
  setOperationAction(ISD::ConstantPool, XLenVT, Custom);

  setBooleanContents(ZeroOrOneBooleanContent);

  // Function alignments (log2).
  unsigned FunctionAlignment = 1; // align to 2^1 bytes boundaries
  setMinFunctionAlignment(FunctionAlignment);
  setPrefFunctionAlignment(FunctionAlignment);

  // Effectively disable jump table generation.
  setMinimumJumpTableEntries(INT_MAX);
}

SDValue LerosTargetLowering::lowerGlobalAddress(SDValue Op,
                                                SelectionDAG &DAG) const {
  SDLoc DL(Op);
  EVT Ty = Op.getValueType();
  GlobalAddressSDNode *N = cast<GlobalAddressSDNode>(Op);
  const GlobalValue *GV = N->getGlobal();
  int64_t Offset = N->getOffset();
  MVT XLenVT = Subtarget.getXLenVT();

  // Create the machine node
  SDValue GAB0 = DAG.getTargetGlobalAddress(GV, DL, Ty, 0, LEROSTF::MO_B0);
  SDValue GAB1 = DAG.getTargetGlobalAddress(GV, DL, Ty, 0, LEROSTF::MO_B1);
  SDValue GAB2 = DAG.getTargetGlobalAddress(GV, DL, Ty, 0, LEROSTF::MO_B2);
  SDValue GAB3 = DAG.getTargetGlobalAddress(GV, DL, Ty, 0, LEROSTF::MO_B3);

  SDValue MNB0 =
      SDValue(DAG.getMachineNode(Leros::LOAD_RI_PSEUDO, DL, Ty, GAB0), 0);
  SDValue MNB1 = SDValue(
      DAG.getMachineNode(Leros::LOADH_RI_PSEUDO, DL, Ty, MNB0, GAB1), 0);
  SDValue MNB2 = SDValue(
      DAG.getMachineNode(Leros::LOADH2_RI_PSEUDO, DL, Ty, MNB1, GAB2), 0);
  SDValue MNB3 = SDValue(
      DAG.getMachineNode(Leros::LOADH3_RI_PSEUDO, DL, Ty, MNB2, GAB3), 0);

  if (Offset != 0)
    return DAG.getNode(ISD::ADD, DL, Ty, MNB3,
                       DAG.getConstant(Offset, DL, XLenVT));
  return MNB3;
}

SDValue LerosTargetLowering::lowerConstantPool(SDValue Op,
                                               SelectionDAG &DAG) const {
  llvm_unreachable("LowerConstantPool unimplemented");
  return SDValue();
}

SDValue LerosTargetLowering::lowerBlockAddress(SDValue Op,
                                               SelectionDAG &DAG) const {
  SDLoc DL(Op);
  EVT Ty = Op.getValueType();
  BlockAddressSDNode *N = cast<BlockAddressSDNode>(Op);
  const BlockAddress *BA = N->getBlockAddress();
  int64_t Offset = N->getOffset();

  if (isPositionIndependent())
    report_fatal_error("Unable to lowerBlockAddress");

  SDValue BAB0 = DAG.getTargetBlockAddress(BA, Ty, Offset, LEROSTF::MO_B0);
  SDValue BAB1 = DAG.getTargetBlockAddress(BA, Ty, Offset, LEROSTF::MO_B1);
  SDValue BAB2 = DAG.getTargetBlockAddress(BA, Ty, Offset, LEROSTF::MO_B2);
  SDValue BAB3 = DAG.getTargetBlockAddress(BA, Ty, Offset, LEROSTF::MO_B3);

  SDValue MNB0 =
      SDValue(DAG.getMachineNode(Leros::LOAD_RI_PSEUDO, DL, Ty, BAB0), 0);
  SDValue MNB1 = SDValue(
      DAG.getMachineNode(Leros::LOADH_RI_PSEUDO, DL, Ty, MNB0, BAB1), 0);
  SDValue MNB2 = SDValue(
      DAG.getMachineNode(Leros::LOADH2_RI_PSEUDO, DL, Ty, MNB1, BAB2), 0);
  SDValue MNB3 = SDValue(
      DAG.getMachineNode(Leros::LOADH3_RI_PSEUDO, DL, Ty, MNB2, BAB3), 0);

  return MNB3;
}

SDValue LerosTargetLowering::lowerSELECT(SDValue Op, SelectionDAG &DAG) const {
  SDValue CondV = Op.getOperand(0);
  SDValue TrueV = Op.getOperand(1);
  SDValue FalseV = Op.getOperand(2);
  SDLoc DL(Op);
  MVT XLenVT = Subtarget.getXLenVT();

  // (select condv, truev, falsev)
  // -> (lerosisd::select_cc condv, truev, falsev)

  SDVTList VTs = DAG.getVTList(Op.getValueType(), MVT::Glue);
  SDValue Ops[] = {CondV, TrueV, FalseV};

  return DAG.getNode(LEROSISD::SELECT_CC, DL, VTs, Ops);
}

SDValue LerosTargetLowering::lowerFRAMEADDR(SDValue Op,
                                            SelectionDAG &DAG) const {
  const LerosRegisterInfo &RI = *Subtarget.getRegisterInfo();
  MachineFunction &MF = DAG.getMachineFunction();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  MFI.setFrameAddressIsTaken(true);
  unsigned FrameReg = RI.getFrameRegister(MF);
  int XLenInBytes = Subtarget.getXLen() / 8;

  EVT VT = Op.getValueType();
  SDLoc DL(Op);
  SDValue FrameAddr = DAG.getCopyFromReg(DAG.getEntryNode(), DL, FrameReg, VT);
  unsigned Depth = cast<ConstantSDNode>(Op.getOperand(0))->getZExtValue();
  while (Depth--) {
    int Offset = -(XLenInBytes * 2);
    SDValue Ptr = DAG.getNode(ISD::ADD, DL, VT, FrameAddr,
                              DAG.getIntPtrConstant(Offset, DL));
    FrameAddr =
        DAG.getLoad(VT, DL, DAG.getEntryNode(), Ptr, MachinePointerInfo());
  }
  return FrameAddr;
}

SDValue LerosTargetLowering::lowerRETURNADDR(SDValue Op,
                                             SelectionDAG &DAG) const {
  const LerosRegisterInfo &RI = *Subtarget.getRegisterInfo();
  MachineFunction &MF = DAG.getMachineFunction();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  MFI.setReturnAddressIsTaken(true);
  MVT XLenVT = Subtarget.getXLenVT();
  int XLenInBytes = Subtarget.getXLen() / 8;

  if (verifyReturnAddressArgumentIsConstant(Op, DAG))
    return SDValue();

  EVT VT = Op.getValueType();
  SDLoc DL(Op);
  unsigned Depth = cast<ConstantSDNode>(Op.getOperand(0))->getZExtValue();
  if (Depth) {
    int Off = -XLenInBytes;
    SDValue FrameAddr = lowerFRAMEADDR(Op, DAG);
    SDValue Offset = DAG.getConstant(Off, DL, VT);
    return DAG.getLoad(VT, DL, DAG.getEntryNode(),
                       DAG.getNode(ISD::ADD, DL, VT, FrameAddr, Offset),
                       MachinePointerInfo());
  }

  // Return the value of the return address register, marking it an implicit
  // live-in.
  unsigned Reg = MF.addLiveIn(RI.getRARegister(), getRegClassFor(XLenVT));
  return DAG.getCopyFromReg(DAG.getEntryNode(), DL, Reg, XLenVT);
}

SDValue LerosTargetLowering::LowerOperation(SDValue Op,
                                            SelectionDAG &DAG) const {

  switch (Op.getOpcode()) {
  default:
    llvm_unreachable("Uninmplemented operand");
  case ISD::ConstantPool:
    return lowerConstantPool(Op, DAG);
  case ISD::GlobalAddress:
    return lowerGlobalAddress(Op, DAG);
  case ISD::BlockAddress:
    return lowerBlockAddress(Op, DAG);
  case ISD::SELECT:
    return lowerSELECT(Op, DAG);
  case ISD::FRAMEADDR:
    return lowerFRAMEADDR(Op, DAG);
  case ISD::RETURNADDR:
    return lowerRETURNADDR(Op, DAG);
  }
}

static unsigned getBranchOpcodeForIntCondCode(ISD::CondCode CC) {
  switch (CC) {
  default:
    llvm_unreachable("Unsupported CondCode");
  case ISD::SETEQ:
    return Leros::BRZ_PSEUDO;
  case ISD::SETNE:
    return Leros::BRNZ_PSEUDO;
  case ISD::SETLT:
    return Leros::BRN_PSEUDO;
  case ISD::SETGE:
    return Leros::BRP_PSEUDO;
  case ISD::SETULT:
    return Leros::BRN_PSEUDO;
  case ISD::SETUGE:
    return Leros::BRP_PSEUDO;
  }
}

MachineBasicBlock *LerosTargetLowering::EmitSHL(MachineInstr &MI,
                                                MachineBasicBlock *BB) const {

  const LerosInstrInfo &TII =
      *BB->getParent()->getSubtarget<LerosSubtarget>().getInstrInfo();
  const BasicBlock *LLVM_BB = BB->getBasicBlock();
  DebugLoc DL = MI.getDebugLoc();
  MachineFunction::iterator I = ++BB->getIterator();
  MachineRegisterInfo &MRI = BB->getParent()->getRegInfo();

  MachineBasicBlock *HeadMBB = BB;
  MachineFunction *F = BB->getParent();

  const unsigned &dstReg = MI.getOperand(0).getReg(),
                 &rs1 = MI.getOperand(1).getReg();
  unsigned rs2;
  int64_t imm;

  if (MI.getOpcode() == Leros::SHL_RI_PSEUDO) {
    imm = MI.getOperand(2).getImm();
    if (imm > Subtarget.getXLen()) {
      llvm_unreachable("Immediate value must be lteq XLen");
    }
    if (imm == 1) {
      // A single rs1 self addition is sufficient
      BuildMI(*BB, MI, DL, TII.get(Leros::ADD_RR_PSEUDO), dstReg)
          .addReg(rs1)
          .addReg(rs1);
      MI.eraseFromParent(); // The pseudo instruction is gone now.
      return BB;
    }
  } else {
    rs2 = MI.getOperand(2).getReg();
  }

  MachineBasicBlock *TailMBB = F->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *shiftMBB = F->CreateMachineBasicBlock(LLVM_BB);
  F->insert(I, shiftMBB);
  F->insert(I, TailMBB);

  // Move all remaining instructions to TailMBB.
  TailMBB->splice(TailMBB->begin(), HeadMBB,
                  std::next(MachineBasicBlock::iterator(MI)), HeadMBB->end());
  // Update machine-CFG edges by transferring all successors of the current
  // block to the new block which will contain the Phi node for the select.
  TailMBB->transferSuccessorsAndUpdatePHIs(HeadMBB);

  HeadMBB->addSuccessor(shiftMBB);
  shiftMBB->addSuccessor(TailMBB);
  shiftMBB->addSuccessor(shiftMBB);

  /** @warning: We do NOT set shiftMBB as a successor of itself, even though
   * this would probably be the correct thing to do. When this is done, because
   * of the virtual registers used in the function, these are hoisted out of the
   * loop, and thus invalidated. Insted, we desire to force printing of the loop
   * label by triggering LerosAsmPrinter::isBlockOnlyReachableByFallthrough,
   * which analyzes whether any terminators of the loop is self referencing
   */

  if (MI.getOpcode() == Leros::SHL_RI_PSEUDO) {
    // shift by immediate operand
    // We here do the following control flow
    //     HeadMBB
    //       |
    //     load immediate & and input argument
    //       |
    //     shiftBB   <-------
    //       |              ^
    //     shl << 1 (repeat)^
    //       |
    //     TailMBB

    // Load immediate into scratch register
    unsigned ScratchReg = MRI.createVirtualRegister(&Leros::GPRRegClass);
    TII.movImm32(*HeadMBB, HeadMBB->end(), DL, ScratchReg, imm);

    // Fallthrough to shiftMBB

    // Determine reg to shift from whether we are starting the loop or iterating
    unsigned RegToShift = MRI.createVirtualRegister(&Leros::GPRRegClass);
    unsigned ShiftRes = MRI.createVirtualRegister(&Leros::GPRRegClass);
    BuildMI(*shiftMBB, shiftMBB->begin(), DL, TII.get(Leros::PHI), RegToShift)
        .addReg(rs1)
        .addMBB(HeadMBB)
        .addReg(ShiftRes)
        .addMBB(shiftMBB);

    unsigned RegToIter = MRI.createVirtualRegister(&Leros::GPRRegClass);
    unsigned SubRes = MRI.createVirtualRegister(&Leros::GPRRegClass);
    BuildMI(*shiftMBB, shiftMBB->begin(), DL, TII.get(Leros::PHI), RegToIter)
        .addReg(ScratchReg)
        .addMBB(HeadMBB)
        .addReg(SubRes)
        .addMBB(shiftMBB);

    BuildMI(shiftMBB, DL, TII.get(Leros::ADD_RR_PSEUDO), ShiftRes)
        .addReg(RegToShift)
        .addReg(RegToShift);

    BuildMI(shiftMBB, DL, TII.get(Leros::SUB_RI_PSEUDO), SubRes)
        .addReg(RegToIter)
        .addImm(1);
    // We can use PseudoBRC as the opcode, since we branche while SubRes > 0
    BuildMI(shiftMBB, DL, TII.get(Leros::PseudoBRNZ))
        .addReg(SubRes)
        .addMBB(shiftMBB);

    BuildMI(*TailMBB, TailMBB->begin(), DL, TII.get(Leros::MOV), dstReg)
        .addReg(ShiftRes);
    MI.eraseFromParent(); // The pseudo instruction is gone now.
    return TailMBB;
  } else {
    HeadMBB->addSuccessor(TailMBB);
    // We here do the following control flow
    //     HeadMBB
    //       |
    //     CheckIfZero
    //       | \
    //       |  moveArg
    //       |   \
    //      shiftBB   <-------
    //       |   |---------- ^
    //       |  /
    //       |
    //     TailMBB
    // Set the successors for HeadMBB.

    // Zero check
    BuildMI(HeadMBB, DL, TII.get(Leros::PseudoBRZ)).addReg(rs2).addMBB(TailMBB);

    // Determine reg to shift from whether we are starting the loop or iterating
    unsigned RegToShift = MRI.createVirtualRegister(&Leros::GPRRegClass);
    unsigned ShiftRes = MRI.createVirtualRegister(&Leros::GPRRegClass);
    BuildMI(*shiftMBB, shiftMBB->begin(), DL, TII.get(Leros::PHI), RegToShift)
        .addReg(rs1)
        .addMBB(HeadMBB)
        .addReg(ShiftRes)
        .addMBB(shiftMBB);

    // Determine iteration value reg from whether we are starting the loop or
    // iterating
    unsigned RegWithIter = MRI.createVirtualRegister(&Leros::GPRRegClass);
    unsigned SubRes = MRI.createVirtualRegister(&Leros::GPRRegClass);
    BuildMI(*shiftMBB, shiftMBB->begin(), DL, TII.get(Leros::PHI), RegWithIter)
        .addReg(rs2)
        .addMBB(HeadMBB)
        .addReg(SubRes)
        .addMBB(shiftMBB);

    // shift by register operand
    BuildMI(shiftMBB, DL, TII.get(Leros::ADD_RR_PSEUDO), ShiftRes)
        .addReg(RegToShift)
        .addReg(RegToShift);

    BuildMI(shiftMBB, DL, TII.get(Leros::SUB_RI_PSEUDO), SubRes)
        .addReg(RegWithIter)
        .addImm(1);

    BuildMI(shiftMBB, DL, TII.get(Leros::PseudoBRNZ))
        .addReg(SubRes)
        .addMBB(shiftMBB);

    // move rs1 to rsd as first instruction in TailMBB
    BuildMI(*TailMBB, TailMBB->begin(), DL, TII.get(Leros::PHI), dstReg)
        .addReg(rs1)
        .addMBB(HeadMBB)
        .addReg(ShiftRes)
        .addMBB(shiftMBB);
    MI.eraseFromParent(); // The pseudo instruction is gone now.
    return TailMBB;
  }
}

MachineBasicBlock *LerosTargetLowering::EmitSET(MachineInstr &MI,
                                                MachineBasicBlock *BB) const {
  const LerosInstrInfo &TII =
      *BB->getParent()->getSubtarget<LerosSubtarget>().getInstrInfo();
  const BasicBlock *LLVM_BB = BB->getBasicBlock();
  DebugLoc DL = MI.getDebugLoc();
  MachineFunction::iterator I = ++BB->getIterator();
  MachineRegisterInfo &MRI = BB->getParent()->getRegInfo();

  MachineBasicBlock *HeadMBB = BB;
  MachineFunction *F = BB->getParent();

  unsigned opcode;

  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Unknown SET opcode");
  case Leros::SETEQ_PSEUDO: {
    opcode = Leros::PseudoBRZ;
    break;
  }
  case Leros::SETGE_PSEUDO: {
    opcode = Leros::PseudoBRP;
    break;
  }
  case Leros::SETLT_PSEUDO: {
    opcode = Leros::PseudoBRN;
    break;
  }
  }

  const unsigned &dstReg = MI.getOperand(0).getReg(),
                 &rs1 = MI.getOperand(1).getReg(),
                 &rs2 = MI.getOperand(2).getReg();

  MachineBasicBlock *TailMBB = F->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *falseMBB = F->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *trueMBB = F->CreateMachineBasicBlock(LLVM_BB);

  // Insertion order matters, to properly handle BB fallthrough
  F->insert(I, falseMBB);
  F->insert(I, trueMBB);
  F->insert(I, TailMBB);

  // Move all remaining instructions to TailMBB.
  TailMBB->splice(TailMBB->begin(), HeadMBB,
                  std::next(MachineBasicBlock::iterator(MI)), HeadMBB->end());
  // Update machine-CFG edges by transferring all successors of the current
  // block to the new block which will contain the Phi node for the select.
  TailMBB->transferSuccessorsAndUpdatePHIs(HeadMBB);

  // Set successors for remaining MBBs
  HeadMBB->addSuccessor(falseMBB);
  HeadMBB->addSuccessor(trueMBB);

  trueMBB->addSuccessor(TailMBB);
  falseMBB->addSuccessor(TailMBB);

  // -------- HeadMBB --------------
  const unsigned ScratchReg = MRI.createVirtualRegister(&Leros::GPRRegClass);
  BuildMI(*HeadMBB, HeadMBB->end(), DL, TII.get(Leros::SUB_RR_PSEUDO),
          ScratchReg)
      .addReg(rs1)
      .addReg(rs2);

  // We can use PseudoBRC as the opcode, since we branche while SubRes > 0
  BuildMI(*HeadMBB, HeadMBB->end(), DL, TII.get(opcode))
      .addReg(ScratchReg)
      .addMBB(trueMBB);

  // fallthrough to falseMBB

  // ------- FalseMBB --------

  // From here, we know that comparison was false
  // register
  const unsigned falseRes = MRI.createVirtualRegister(&Leros::GPRRegClass);
  TII.movImm32(*falseMBB, falseMBB->end(), DL, falseRes, 0);

  BuildMI(*falseMBB, falseMBB->end(), DL, TII.get(Leros::PseudoBR))
      .addMBB(TailMBB);

  // -------- trueMBB --------------
  const unsigned trueRes = MRI.createVirtualRegister(&Leros::GPRRegClass);
  TII.movImm32(*trueMBB, trueMBB->end(), DL, trueRes, 1);

  // Fallthrough to tail

  // -------- tailMBB --------------
  // Get result through a phi node
  BuildMI(*TailMBB, TailMBB->begin(), DL, TII.get(Leros::PHI), dstReg)
      .addReg(falseRes)
      .addMBB(falseMBB)
      .addReg(trueRes)
      .addMBB(trueMBB);
  MI.eraseFromParent(); // The pseudo instruction is gone now.
  return TailMBB;
}

MachineBasicBlock *
LerosTargetLowering::EmitSEXTLOAD(MachineInstr &MI,
                                  MachineBasicBlock *BB) const {
  const LerosInstrInfo &TII =
      *BB->getParent()->getSubtarget<LerosSubtarget>().getInstrInfo();
  const BasicBlock *LLVM_BB = BB->getBasicBlock();
  DebugLoc DL = MI.getDebugLoc();
  MachineFunction::iterator I = ++BB->getIterator();
  MachineRegisterInfo &MRI = BB->getParent()->getRegInfo();

  MachineBasicBlock *HeadMBB = BB;
  MachineFunction *F = BB->getParent();

  unsigned opcode;
  unsigned memLoadOpcode;
  uint32_t signImm;

  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Unknown SEXT opcode");
  case Leros::LOAD_S8_M_PSEUDO: {
    memLoadOpcode = Leros::LOAD_U8_M_PSEUDO;
    opcode = Leros::LOADH_RI_PSEUDO;
    signImm = 0x80;
    break;
  }
  case Leros::LOAD_S16_M_PSEUDO: {
    memLoadOpcode = Leros::LOAD_M_PSEUDO;
    opcode = Leros::LOADH2_RI_PSEUDO;
    signImm = 0x8000;
    break;
  }
  }

  const unsigned &dstReg = MI.getOperand(0).getReg();
  auto op1 = MI.getOperand(1);
  const auto &imm = MI.getOperand(2).getImm();

  MachineBasicBlock *TailMBB = F->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *posMBB = F->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *negMBB = F->CreateMachineBasicBlock(LLVM_BB);

  // Insertion order matters, to properly handle BB fallthrough
  F->insert(I, posMBB);
  F->insert(I, negMBB);
  F->insert(I, TailMBB);

  // Move all remaining instructions to TailMBB.
  TailMBB->splice(TailMBB->begin(), HeadMBB,
                  std::next(MachineBasicBlock::iterator(MI)), HeadMBB->end());
  // Update machine-CFG edges by transferring all successors of the current
  // block to the new block which will contain the Phi node for the select.
  TailMBB->transferSuccessorsAndUpdatePHIs(HeadMBB);

  // Set successors for remaining MBBs
  HeadMBB->addSuccessor(posMBB);
  HeadMBB->addSuccessor(negMBB);

  negMBB->addSuccessor(TailMBB);
  posMBB->addSuccessor(TailMBB);

  // -------- HeadMBB --------------
  // Load memory
  const unsigned ScratchReg = MRI.createVirtualRegister(&Leros::GPRRegClass);
  if (op1.isReg()) {
    BuildMI(*HeadMBB, HeadMBB->end(), DL, TII.get(memLoadOpcode), ScratchReg)
        .addReg(op1.getReg())
        .addImm(imm);
  } else if (op1.isFI()) {
    BuildMI(*HeadMBB, HeadMBB->end(), DL, TII.get(memLoadOpcode), ScratchReg)
        .addFrameIndex(op1.getIndex())
        .addImm(imm);
  } else {
    llvm_unreachable("Unknown operand type for SEXT emission!");
  }

  // Mask sign of loaded operand
  const unsigned mask = MRI.createVirtualRegister(&Leros::GPRRegClass);
  TII.movUImm32(*HeadMBB, HeadMBB->end(), DL, mask, signImm);
  const unsigned maskedLoad = MRI.createVirtualRegister(&Leros::GPRRegClass);
  BuildMI(*HeadMBB, HeadMBB->end(), DL, TII.get(Leros::AND_RR_PSEUDO),
          maskedLoad)
      .addReg(ScratchReg)
      .addReg(mask);

  // result != 0 => negative number
  BuildMI(*HeadMBB, HeadMBB->end(), DL, TII.get(Leros::PseudoBRNZ))
      .addReg(maskedLoad)
      .addMBB(negMBB);

  // -------- posMBB --------------
  // From here we know that the loaded operand is positive - bitmask lower bits
  const unsigned ZEXTRes = MRI.createVirtualRegister(&Leros::GPRRegClass);
  BuildMI(*posMBB, posMBB->end(), DL, TII.get(opcode), ZEXTRes)
      .addReg(ScratchReg)
      .addImm(0x0);

  BuildMI(*posMBB, posMBB->end(), DL, TII.get(Leros::PseudoBR)).addMBB(TailMBB);

  // From here, we know that loaded operand is negative. Sign extend from either
  // the 8th or 16th bit (depending on $opcode)
  const unsigned SEXTRes = MRI.createVirtualRegister(&Leros::GPRRegClass);
  BuildMI(*negMBB, negMBB->end(), DL, TII.get(opcode), SEXTRes)
      .addReg(ScratchReg)
      .addImm(0xFF);
  // Fallthrough to tail

  // -------- tailMBB --------------
  // Get result through a phi node
  BuildMI(*TailMBB, TailMBB->begin(), DL, TII.get(Leros::PHI), dstReg)
      .addReg(SEXTRes)
      .addMBB(negMBB)
      .addReg(ZEXTRes)
      .addMBB(posMBB);
  MI.eraseFromParent(); // The pseudo instruction is gone now.
  return TailMBB;
}

MachineBasicBlock *LerosTargetLowering::EmitSRAI(MachineInstr &MI,
                                                 MachineBasicBlock *BB) const {
  const LerosInstrInfo &TII =
      *BB->getParent()->getSubtarget<LerosSubtarget>().getInstrInfo();
  const BasicBlock *LLVM_BB = BB->getBasicBlock();
  DebugLoc DL = MI.getDebugLoc();
  MachineFunction::iterator I = ++BB->getIterator();
  MachineRegisterInfo &MRI = BB->getParent()->getRegInfo();

  MachineBasicBlock *HeadMBB = BB;
  MachineFunction *F = BB->getParent();

  const unsigned &dstReg = MI.getOperand(0).getReg(),
                 &rs1 = MI.getOperand(1).getReg();
  const int64_t imm = MI.getOperand(2).getImm();

  MachineBasicBlock *TailMBB = F->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *negMBB = F->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *posMBB = F->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *buildSEConstantMBB = F->CreateMachineBasicBlock(LLVM_BB);

  // Insertion order matters, to properly handle BB fallthrough
  F->insert(I, buildSEConstantMBB);
  F->insert(I, negMBB);
  F->insert(I, posMBB);
  F->insert(I, TailMBB);

  // Move all remaining instructions to TailMBB.
  TailMBB->splice(TailMBB->begin(), HeadMBB,
                  std::next(MachineBasicBlock::iterator(MI)), HeadMBB->end());
  // Update machine-CFG edges by transferring all successors of the current
  // block to the new block which will contain the Phi node for the select.
  TailMBB->transferSuccessorsAndUpdatePHIs(HeadMBB);

  // Set successors for remaining MBBs
  HeadMBB->addSuccessor(buildSEConstantMBB);
  HeadMBB->addSuccessor(posMBB);

  buildSEConstantMBB->addSuccessor(negMBB);

  negMBB->addSuccessor(TailMBB);
  negMBB->addSuccessor(negMBB);

  posMBB->addSuccessor(TailMBB);
  posMBB->addSuccessor(posMBB);

  // -------- HeadMBB --------------
  // Load immediate into scratch register
  const unsigned ScratchReg = MRI.createVirtualRegister(&Leros::GPRRegClass);
  TII.movImm32(*HeadMBB, HeadMBB->end(), DL, ScratchReg, imm);

  // We can use PseudoBRC as the opcode, since we branche while SubRes > 0
  BuildMI(*HeadMBB, HeadMBB->end(), DL, TII.get(Leros::PseudoBRP))
      .addReg(rs1)
      .addMBB(posMBB);

  // -------- buildSEConstantMBB ------
  // From here, we know that it is a negative number - build sign extension
  // register
  const unsigned SER = MRI.createVirtualRegister(&Leros::GPRRegClass);
  TII.movUImm32(*buildSEConstantMBB, buildSEConstantMBB->end(), DL, SER,
                0x80000000);

  // fallthrough to NegMBB

  // -------- NegMBB --------------
  // PHI nodes must be subsequent at the start of the basic block

  unsigned NRegToShift = MRI.createVirtualRegister(&Leros::GPRRegClass);
  unsigned NShiftRes = MRI.createVirtualRegister(&Leros::GPRRegClass);
  const unsigned NSEShiftRes = MRI.createVirtualRegister(&Leros::GPRRegClass);
  BuildMI(*negMBB, negMBB->end(), DL, TII.get(Leros::PHI), NRegToShift)
      .addReg(rs1)
      .addMBB(buildSEConstantMBB)
      .addReg(NSEShiftRes)
      .addMBB(negMBB);

  unsigned RegToIter = MRI.createVirtualRegister(&Leros::GPRRegClass);
  unsigned IterRes = MRI.createVirtualRegister(&Leros::GPRRegClass);
  BuildMI(*negMBB, negMBB->end(), DL, TII.get(Leros::PHI), RegToIter)
      .addReg(ScratchReg)
      .addMBB(buildSEConstantMBB)
      .addReg(IterRes)
      .addMBB(negMBB);

  BuildMI(*negMBB, negMBB->end(), DL, TII.get(Leros::SHRByOne_Pseudo),
          NShiftRes)
      .addReg(NRegToShift);

  // Sign extend
  BuildMI(*negMBB, negMBB->end(), DL, TII.get(Leros::OR_RR_PSEUDO), NSEShiftRes)
      .addReg(NShiftRes)
      .addReg(SER);

  BuildMI(*negMBB, negMBB->end(), DL, TII.get(Leros::SUB_RI_PSEUDO), IterRes)
      .addReg(RegToIter)
      .addImm(1);

  BuildMI(*negMBB, negMBB->end(), DL, TII.get(Leros::PseudoBRNZ))
      .addReg(IterRes)
      .addMBB(negMBB);

  // Finished sign-extended shift - unconditional branch to tail
  BuildMI(*negMBB, negMBB->end(), DL, TII.get(Leros::PseudoBR)).addMBB(TailMBB);

  // -------- PosMBB --------------

  unsigned PRegToShift = MRI.createVirtualRegister(&Leros::GPRRegClass);
  unsigned PShiftRes = MRI.createVirtualRegister(&Leros::GPRRegClass);
  BuildMI(*posMBB, posMBB->end(), DL, TII.get(Leros::PHI), PRegToShift)
      .addReg(rs1)
      .addMBB(HeadMBB)
      .addReg(PShiftRes)
      .addMBB(posMBB);

  unsigned PRegToIter = MRI.createVirtualRegister(&Leros::GPRRegClass);
  unsigned PIterRes = MRI.createVirtualRegister(&Leros::GPRRegClass);
  BuildMI(*posMBB, posMBB->end(), DL, TII.get(Leros::PHI), PRegToIter)
      .addReg(ScratchReg)
      .addMBB(HeadMBB)
      .addReg(PIterRes)
      .addMBB(posMBB);

  BuildMI(*posMBB, posMBB->end(), DL, TII.get(Leros::SHRByOne_Pseudo),
          PShiftRes)
      .addReg(PRegToShift);

  BuildMI(*posMBB, posMBB->end(), DL, TII.get(Leros::SUB_RI_PSEUDO), PIterRes)
      .addReg(PRegToIter)
      .addImm(1);

  BuildMI(*posMBB, posMBB->end(), DL, TII.get(Leros::PseudoBRNZ))
      .addReg(PIterRes)
      .addMBB(posMBB);

  // Fallthrough to tail

  // -------- tailMBB --------------
  // Get result through a phi node
  BuildMI(*TailMBB, TailMBB->begin(), DL, TII.get(Leros::PHI), dstReg)
      .addReg(NSEShiftRes)
      .addMBB(negMBB)
      .addReg(PShiftRes)
      .addMBB(posMBB);
  MI.eraseFromParent(); // The pseudo instruction is gone now.
  return TailMBB;
}

MachineBasicBlock *LerosTargetLowering::EmitSRAR(MachineInstr &MI,
                                                 MachineBasicBlock *BB) const {
  const LerosInstrInfo &TII =
      *BB->getParent()->getSubtarget<LerosSubtarget>().getInstrInfo();
  const BasicBlock *LLVM_BB = BB->getBasicBlock();
  DebugLoc DL = MI.getDebugLoc();
  MachineFunction::iterator I = ++BB->getIterator();
  MachineRegisterInfo &MRI = BB->getParent()->getRegInfo();

  MachineBasicBlock *HeadMBB = BB;
  MachineFunction *F = BB->getParent();

  const unsigned &dstReg = MI.getOperand(0).getReg(),
                 &rs1 = MI.getOperand(1).getReg(),
                 &rs2 = MI.getOperand(2).getReg();

  MachineBasicBlock *TailMBB = F->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *PosCheck = F->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *negInit = F->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *negMBB = F->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *posMBB = F->CreateMachineBasicBlock(LLVM_BB);

  // Insertion order matters, to properly handle BB fallthrough
  F->insert(I, PosCheck);
  F->insert(I, negInit);
  F->insert(I, negMBB);
  F->insert(I, posMBB);
  F->insert(I, TailMBB);

  // Move all remaining instructions to TailMBB.
  TailMBB->splice(TailMBB->begin(), HeadMBB,
                  std::next(MachineBasicBlock::iterator(MI)), HeadMBB->end());
  // Update machine-CFG edges by transferring all successors of the current
  // block to the new block which will contain the Phi node for the select.
  TailMBB->transferSuccessorsAndUpdatePHIs(HeadMBB);

  // Set successors for remaining MBBs
  HeadMBB->addSuccessor(PosCheck);
  HeadMBB->addSuccessor(TailMBB);

  PosCheck->addSuccessor(posMBB);
  PosCheck->addSuccessor(negInit);

  negInit->addSuccessor(negMBB);

  negMBB->addSuccessor(TailMBB);
  negMBB->addSuccessor(negMBB);

  posMBB->addSuccessor(TailMBB);
  posMBB->addSuccessor(posMBB);

  // -------- HeadMBB --------------
  // Zero check
  BuildMI(*HeadMBB, HeadMBB->end(), DL, TII.get(Leros::PseudoBRZ))
      .addReg(rs2)
      .addMBB(TailMBB);

  // Check if positive
  BuildMI(*PosCheck, PosCheck->end(), DL, TII.get(Leros::PseudoBRP))
      .addReg(rs1)
      .addMBB(posMBB);

  // From here, we know that it is a negative number - build sign extension
  // register
  const unsigned SER = MRI.createVirtualRegister(&Leros::GPRRegClass);
  TII.movUImm32(*negInit, negInit->end(), DL, SER, 0x80000000);

  // fallthrough to NegMBB

  // -------- NegMBB --------------
  // Multiple PHI nodes must be subsequent at the start of the BB
  unsigned NRegToShift = MRI.createVirtualRegister(&Leros::GPRRegClass);
  unsigned NShiftRes = MRI.createVirtualRegister(&Leros::GPRRegClass);
  const unsigned NSEShiftRes = MRI.createVirtualRegister(&Leros::GPRRegClass);

  BuildMI(*negMBB, negMBB->begin(), DL, TII.get(Leros::PHI), NRegToShift)
      .addReg(rs1)
      .addMBB(negInit)
      .addReg(NSEShiftRes)
      .addMBB(negMBB);
  BuildMI(negMBB, DL, TII.get(Leros::SHRByOne_Pseudo), NShiftRes)
      .addReg(NRegToShift);

  unsigned NRegToIter = MRI.createVirtualRegister(&Leros::GPRRegClass);
  unsigned NIterRes = MRI.createVirtualRegister(&Leros::GPRRegClass);
  BuildMI(*negMBB, negMBB->begin(), DL, TII.get(Leros::PHI), NRegToIter)
      .addReg(rs2)
      .addMBB(negInit)
      .addReg(NIterRes)
      .addMBB(negMBB);

  // Sign extend
  BuildMI(negMBB, DL, TII.get(Leros::OR_RR_PSEUDO), NSEShiftRes)
      .addReg(NShiftRes)
      .addReg(SER);

  BuildMI(negMBB, DL, TII.get(Leros::SUB_RI_PSEUDO), NIterRes)
      .addReg(NRegToIter)
      .addImm(1);

  BuildMI(negMBB, DL, TII.get(Leros::PseudoBRNZ))
      .addReg(NIterRes)
      .addMBB(negMBB);

  // Finished sign-extended shift - unconditional branch to tail
  BuildMI(negMBB, DL, TII.get(Leros::PseudoBR)).addMBB(TailMBB);

  // -------- PosMBB --------------
  unsigned PRegToShift = MRI.createVirtualRegister(&Leros::GPRRegClass);
  unsigned PShiftRes = MRI.createVirtualRegister(&Leros::GPRRegClass);
  BuildMI(*posMBB, posMBB->begin(), DL, TII.get(Leros::PHI), PRegToShift)
      .addReg(rs1)
      .addMBB(PosCheck)
      .addReg(PShiftRes)
      .addMBB(posMBB);

  unsigned PRegToIter = MRI.createVirtualRegister(&Leros::GPRRegClass);
  unsigned PIterRes = MRI.createVirtualRegister(&Leros::GPRRegClass);
  BuildMI(*posMBB, posMBB->begin(), DL, TII.get(Leros::PHI), PRegToIter)
      .addReg(rs2)
      .addMBB(PosCheck)
      .addReg(PIterRes)
      .addMBB(posMBB);

  BuildMI(posMBB, DL, TII.get(Leros::SHRByOne_Pseudo), PShiftRes)
      .addReg(PRegToShift);

  BuildMI(posMBB, DL, TII.get(Leros::SUB_RI_PSEUDO), PIterRes)
      .addReg(PRegToIter)
      .addImm(1);

  BuildMI(posMBB, DL, TII.get(Leros::PseudoBRNZ))
      .addReg(PIterRes)
      .addMBB(posMBB);

  // Fallthrough to tail

  // -------- tailMBB --------------
  // Get result through a phi node
  BuildMI(*TailMBB, TailMBB->begin(), DL, TII.get(Leros::PHI), dstReg)
      .addReg(rs1) // If input shift amount was 0
      .addMBB(HeadMBB)
      .addReg(NSEShiftRes)
      .addMBB(negMBB)
      .addReg(PShiftRes)
      .addMBB(posMBB);
  MI.eraseFromParent(); // The pseudo instruction is gone now.
  return TailMBB;
}

MachineBasicBlock *LerosTargetLowering::EmitSRL(MachineInstr &MI,
                                                MachineBasicBlock *BB) const {
  const LerosInstrInfo &TII =
      *BB->getParent()->getSubtarget<LerosSubtarget>().getInstrInfo();
  const BasicBlock *LLVM_BB = BB->getBasicBlock();
  DebugLoc DL = MI.getDebugLoc();
  MachineFunction::iterator I = ++BB->getIterator();
  MachineRegisterInfo &MRI = BB->getParent()->getRegInfo();

  MachineBasicBlock *HeadMBB = BB;
  MachineFunction *F = BB->getParent();

  const unsigned &dstReg = MI.getOperand(0).getReg(),
                 &rs1 = MI.getOperand(1).getReg();
  unsigned rs2;
  int64_t imm;

  if (MI.getOpcode() == Leros::SRL_RI_PSEUDO) {
    imm = MI.getOperand(2).getImm();
    if (imm > Subtarget.getXLen()) {
      llvm_unreachable("Immediate value must be lteq XLen");
    }
    if (imm == 1) {
      // A single shr is sufficient
      BuildMI(*HeadMBB, MI, DL, TII.get(Leros::SHRByOne_Pseudo), dstReg)
          .addReg(rs1);
      MI.eraseFromParent(); // The pseudo instruction is gone now.
      return HeadMBB;
    }
  } else {
    rs2 = MI.getOperand(2).getReg();
  }

  MachineBasicBlock *TailMBB = F->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *shiftMBB = F->CreateMachineBasicBlock(LLVM_BB);
  F->insert(I, shiftMBB);
  F->insert(I, TailMBB);
  // Move all remaining instructions to TailMBB.
  TailMBB->splice(TailMBB->begin(), HeadMBB,
                  std::next(MachineBasicBlock::iterator(MI)), HeadMBB->end());
  // Update machine-CFG edges by transferring all successors of the current
  // block to the new block which will contain the Phi node for the select.
  TailMBB->transferSuccessorsAndUpdatePHIs(HeadMBB);

  HeadMBB->addSuccessor(shiftMBB);

  shiftMBB->addSuccessor(shiftMBB);
  shiftMBB->addSuccessor(TailMBB);

  if (MI.getOpcode() == Leros::SRL_RI_PSEUDO) {
    // shift by immediate operand
    // We here do the following control flow
    //     HeadMBB
    //       |
    //     load immediate
    //       |
    //     shiftBB   <-------
    //       |              ^
    //     shr 1 (repeat)   ^
    //       |
    //     TailMBB

    // Load immediate into scratch register
    unsigned ScratchReg = MRI.createVirtualRegister(&Leros::GPRRegClass);
    TII.movImm32(*HeadMBB, HeadMBB->end(), DL, ScratchReg, imm);

    // Create shiftBB
    unsigned RegToShift = MRI.createVirtualRegister(&Leros::GPRRegClass);
    unsigned ShiftRes = MRI.createVirtualRegister(&Leros::GPRRegClass);
    BuildMI(*shiftMBB, shiftMBB->end(), DL, TII.get(Leros::PHI), RegToShift)
        .addReg(rs1)
        .addMBB(HeadMBB)
        .addReg(ShiftRes)
        .addMBB(shiftMBB);

    unsigned RegToIter = MRI.createVirtualRegister(&Leros::GPRRegClass);
    unsigned SubRes = MRI.createVirtualRegister(&Leros::GPRRegClass);
    BuildMI(*shiftMBB, shiftMBB->end(), DL, TII.get(Leros::PHI), RegToIter)
        .addReg(ScratchReg)
        .addMBB(HeadMBB)
        .addReg(SubRes)
        .addMBB(shiftMBB);

    BuildMI(*shiftMBB, shiftMBB->end(), DL, TII.get(Leros::SHRByOne_Pseudo),
            ShiftRes)
        .addReg(RegToShift);

    BuildMI(*shiftMBB, shiftMBB->end(), DL, TII.get(Leros::SUB_RI_PSEUDO),
            SubRes)
        .addReg(RegToIter)
        .addImm(1);
    // We can use PseudoBRC as the opcode, since we branche while SubRes > 0
    BuildMI(*shiftMBB, shiftMBB->end(), DL, TII.get(Leros::PseudoBRNZ))
        .addReg(SubRes)
        .addMBB(shiftMBB);

    BuildMI(*TailMBB, TailMBB->begin(), DL, TII.get(Leros::MOV), dstReg)
        .addReg(ShiftRes);
    MI.eraseFromParent(); // The pseudo instruction is gone now.
    return TailMBB;
  } else {
    // We here do the following control flow
    //     HeadMBB
    //       |
    //     CheckIfZero
    //       | \
    //       |  shiftBB   <-------
    //       |  |            ^
    //       | /
    //       |
    //     TailMBB

    // Set the successors for HeadMBB.
    HeadMBB->addSuccessor(TailMBB);

    // Zero check
    BuildMI(*HeadMBB, HeadMBB->end(), DL, TII.get(Leros::PseudoBRZ))
        .addReg(rs2)
        .addMBB(TailMBB);
    HeadMBB->addSuccessor(shiftMBB);

    // shift by register operand
    unsigned RegToShift = MRI.createVirtualRegister(&Leros::GPRRegClass);
    unsigned ShiftRes = MRI.createVirtualRegister(&Leros::GPRRegClass);
    BuildMI(*shiftMBB, shiftMBB->end(), DL, TII.get(Leros::PHI), RegToShift)
        .addReg(rs1)
        .addMBB(HeadMBB)
        .addReg(ShiftRes)
        .addMBB(shiftMBB);

    unsigned RegToIter = MRI.createVirtualRegister(&Leros::GPRRegClass);
    unsigned SubRes = MRI.createVirtualRegister(&Leros::GPRRegClass);
    BuildMI(*shiftMBB, shiftMBB->end(), DL, TII.get(Leros::PHI), RegToIter)
        .addReg(rs2)
        .addMBB(HeadMBB)
        .addReg(SubRes)
        .addMBB(shiftMBB);

    BuildMI(*shiftMBB, shiftMBB->end(), DL, TII.get(Leros::SHRByOne_Pseudo),
            ShiftRes)
        .addReg(RegToShift);

    BuildMI(*shiftMBB, shiftMBB->end(), DL, TII.get(Leros::SUB_RI_PSEUDO),
            SubRes)
        .addReg(RegToIter)
        .addImm(1);
    BuildMI(*shiftMBB, shiftMBB->end(), DL, TII.get(Leros::PseudoBRNZ))
        .addReg(SubRes)
        .addMBB(shiftMBB);

    // move rs1 to rsd as first instruction in TailMBB
    BuildMI(*TailMBB, TailMBB->begin(), DL, TII.get(Leros::PHI), dstReg)
        .addReg(rs1)
        .addMBB(HeadMBB)
        .addReg(ShiftRes)
        .addMBB(shiftMBB);
    MI.eraseFromParent(); // The pseudo instruction is gone now.
    return TailMBB;
  }
}

MachineBasicBlock *
LerosTargetLowering::EmitSELECT(MachineInstr &MI, MachineBasicBlock *BB) const {
  const TargetInstrInfo &TII = *BB->getParent()->getSubtarget().getInstrInfo();
  const BasicBlock *LLVM_BB = BB->getBasicBlock();
  DebugLoc DL = MI.getDebugLoc();
  MachineFunction::iterator I = ++BB->getIterator();

  MachineBasicBlock *HeadMBB = BB;
  MachineFunction *F = BB->getParent();
  MachineBasicBlock *TailMBB = F->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *IfTrueMBB = F->CreateMachineBasicBlock(LLVM_BB);

  // We do not do any branching in our SELECT emission - it is expected that rs1
  // of the select instruction contains a boolean value which selects between
  // the two registers

  F->insert(I, IfTrueMBB);
  F->insert(I, TailMBB);
  // Move all remaining instructions to TailMBB.
  TailMBB->splice(TailMBB->begin(), HeadMBB,
                  std::next(MachineBasicBlock::iterator(MI)), HeadMBB->end());
  // Update machine-CFG edges by transferring all successors of the current
  // block to the new block which will contain the Phi node for the select.
  TailMBB->transferSuccessorsAndUpdatePHIs(HeadMBB);
  // Set the successors for HeadMBB.
  HeadMBB->addSuccessor(IfTrueMBB);
  HeadMBB->addSuccessor(TailMBB);

  // Insert appropriate branch. If CC is 0, we branch to TailMBB and the select
  // the 2nd register
  unsigned CCReg = MI.getOperand(1).getReg();
  BuildMI(HeadMBB, DL, TII.get(Leros::PseudoBRZ)).addReg(CCReg).addMBB(TailMBB);

  // IfTrueMBB just falls through to TailMBB. We select the 1st register here
  IfTrueMBB->addSuccessor(TailMBB);

  // %Result = phi [ %TrueValue, IfTrueMBB ], [ %FalseValue, HeadMBB ]
  BuildMI(*TailMBB, TailMBB->begin(), DL, TII.get(Leros::PHI),
          MI.getOperand(0).getReg())
      .addReg(MI.getOperand(3).getReg())
      .addMBB(HeadMBB)
      .addReg(MI.getOperand(2).getReg())
      .addMBB(IfTrueMBB);

  MI.eraseFromParent(); // The pseudo instruction is gone now.
  return TailMBB;
}

MachineBasicBlock *
LerosTargetLowering::EmitTruncatedStore(MachineInstr &MI,
                                        MachineBasicBlock *BB) const {
  const LerosInstrInfo &TII =
      *BB->getParent()->getSubtarget<LerosSubtarget>().getInstrInfo();
  DebugLoc DL = MI.getDebugLoc();
  MachineRegisterInfo &MRI = BB->getParent()->getRegInfo();
  MachineFunction::iterator I = ++BB->getIterator();

  const BasicBlock *LLVM_BB = BB->getBasicBlock();
  MachineBasicBlock *HeadMBB = BB;
  MachineFunction *F = BB->getParent();
  MachineBasicBlock *TailMBB = F->CreateMachineBasicBlock(LLVM_BB);

  // We dont explicitely need a new basic block here, but to keep the solution
  // clean and error free, we insert a new one (tailMBB), move all successors to
  // this, and add all of our new instructions onto the HeadMBB end iterator

  F->insert(I, TailMBB);
  // Move all remaining instructions to TailMBB.
  TailMBB->splice(TailMBB->begin(), HeadMBB,
                  std::next(MachineBasicBlock::iterator(MI)), HeadMBB->end());

  // Update machine-CFG edges by transferring all successors of the current
  // block to the new block which will contain the Phi node for the select.
  TailMBB->transferSuccessorsAndUpdatePHIs(HeadMBB);

  // HeadMBB falls through to TailMBB
  HeadMBB->addSuccessor(TailMBB);

  auto rstore = MI.getOperand(0).getReg();
  auto rmem = MI.getOperand(1);
  auto imm = MI.getOperand(2).getImm();

  if (MI.getOpcode() == Leros::STORE_8_M_PSEUDO) {
    // Utilize the byte-store of leros
    BuildMI(*HeadMBB, HeadMBB->end(), DL, TII.get(Leros::STORE_8_M_PSEUDO),
            rstore)
        .addReg(rmem.getReg())
        .addImm(imm);
  } else {
    // 16-bit store. Load the memory at the location, mask upper half-word and
    // OR it with the value which is to-be stored

    // Load the memory at the location
    const unsigned MemReg = MRI.createVirtualRegister(&Leros::GPRRegClass);
    if (rmem.isReg()) {
      BuildMI(*HeadMBB, HeadMBB->end(), DL, TII.get(Leros::LOAD_M_PSEUDO),
              MemReg)
          .addReg(rmem.getReg())
          .addImm(imm);
    } else if (rmem.isFI()) {
      BuildMI(*HeadMBB, HeadMBB->end(), DL, TII.get(Leros::LOAD_M_PSEUDO),
              MemReg)
          .addFrameIndex(rmem.getIndex())
          .addImm(imm);
    } else {
      llvm_unreachable("Unknown operand type for SEXT emission!");
    }

    // Mask the upper bits
    int64_t bitmask = -65536; // will expand to 0xFFFF0000
    const unsigned MaskReg1 = MRI.createVirtualRegister(&Leros::GPRRegClass);
    TII.movImm32(*HeadMBB, HeadMBB->end(), DL, MaskReg1, bitmask);
    const unsigned MaskedMem = MRI.createVirtualRegister(&Leros::GPRRegClass);
    BuildMI(*HeadMBB, HeadMBB->end(), DL, TII.get(Leros::AND_RR_PSEUDO),
            MaskedMem)
        .addReg(MemReg)
        .addReg(MaskReg1);

    // Mask the lower bits of the store register
    const unsigned MaskReg2 = MRI.createVirtualRegister(&Leros::GPRRegClass);
    bitmask = 0xFFFF;
    TII.movImm32(*HeadMBB, HeadMBB->end(), DL, MaskReg2, bitmask);
    const unsigned MaskedStore = MRI.createVirtualRegister(&Leros::GPRRegClass);
    BuildMI(*HeadMBB, HeadMBB->end(), DL, TII.get(Leros::AND_RR_PSEUDO),
            MaskedStore)
        .addReg(rstore)
        .addReg(MaskReg2);

    // OR the two registers
    const unsigned MaskedRes = MRI.createVirtualRegister(&Leros::GPRRegClass);
    BuildMI(*HeadMBB, HeadMBB->end(), DL, TII.get(Leros::OR_RR_PSEUDO),
            MaskedRes)
        .addReg(MaskedStore)
        .addReg(MaskedMem);

    // Store the memory
    if (rmem.isReg()) {
      BuildMI(*HeadMBB, HeadMBB->end(), DL, TII.get(Leros::STORE_M_PSEUDO))
          .addReg(MaskedRes)
          .addReg(rmem.getReg())
          .addImm(imm);
    } else if (rmem.isFI()) {
      BuildMI(*HeadMBB, HeadMBB->end(), DL, TII.get(Leros::STORE_M_PSEUDO))
          .addReg(MaskedRes)
          .addFrameIndex(rmem.getIndex())
          .addImm(imm);
    } else {
      llvm_unreachable("Unknown operand type for SEXT emission!");
    }
  }

  MI.eraseFromParent(); // The pseudo instruction is gone now.
  return TailMBB;
}

MachineBasicBlock *
LerosTargetLowering::EmitInstrWithCustomInserter(MachineInstr &MI,
                                                 MachineBasicBlock *BB) const {
  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Unexpected instr type to insert");
  case Leros::Select_GPR_Using_CC_GPR:
    return EmitSELECT(MI, BB);
  case Leros::SHL_RI_PSEUDO:
  case Leros::SHL_RR_PSEUDO:
    return EmitSHL(MI, BB);
  case Leros::SRL_RI_PSEUDO:
  case Leros::SRL_RR_PSEUDO:
    return EmitSRL(MI, BB);
  case Leros::SRA_RI_PSEUDO:
    return EmitSRAI(MI, BB);
  case Leros::SRA_RR_PSEUDO:
    return EmitSRAR(MI, BB);
  case Leros::SETEQ_PSEUDO:
  case Leros::SETGE_PSEUDO:
  case Leros::SETLT_PSEUDO:
    return EmitSET(MI, BB);
  case Leros::LOAD_S8_M_PSEUDO:
  case Leros::LOAD_S16_M_PSEUDO:
    return EmitSEXTLOAD(MI, BB);
  case Leros::STORE_8_M_PSEUDO:
  case Leros::STORE_16_M_PSEUDO:
    return EmitTruncatedStore(MI, BB);
  }
}

const char *LerosTargetLowering::getTargetNodeName(unsigned Opcode) const {
#define NODE(name)                                                             \
  case LEROSISD::name:                                                         \
    return "LEROSISD::" #name;
  switch (Opcode) {
  default: {
    llvm_unreachable("Unknown node name for target node");
    return nullptr;
  }
    NODE(Ret);
    NODE(Call);
    NODE(LOADH);
    NODE(LOADH2);
    NODE(LOADH3);
    NODE(Mov);
    NODE(SELECT_CC);
  }
#undef NODE
}

//===----------------------------------------------------------------------===//
//                      Calling Convention Implementation
//===----------------------------------------------------------------------===//

#include "LerosGenCallingConv.inc"

//===----------------------------------------------------------------------===//
//                  Call Calling Convention Implementation
//===----------------------------------------------------------------------===//

/// Leros call implementation
SDValue LerosTargetLowering::LowerCall(TargetLowering::CallLoweringInfo &CLI,
                                       SmallVectorImpl<SDValue> &InVals) const {
  SelectionDAG &DAG = CLI.DAG;
  SDLoc &dl = CLI.DL;
  SmallVectorImpl<ISD::OutputArg> &Outs = CLI.Outs;
  SmallVectorImpl<SDValue> &OutVals = CLI.OutVals;
  SmallVectorImpl<ISD::InputArg> &Ins = CLI.Ins;
  SDValue Chain = CLI.Chain;
  SDValue Callee = CLI.Callee;
  CallingConv::ID CallConv = CLI.CallConv;
  EVT PtrVT = getPointerTy(DAG.getDataLayout());
  bool isVarArg = CLI.IsVarArg;
  MachineFunction &MF = DAG.getMachineFunction();

  // Leros target does not yet support tail call optimization.
  CLI.IsTailCall = false;

  if (isVarArg) {
    llvm_unreachable("Unimplemented");
  }

  // Analyze operands of the call, assigning locations to each operand.
  SmallVector<CCValAssign, 16> ArgLocs;
  CCState CCInfo(CallConv, isVarArg, DAG.getMachineFunction(), ArgLocs,
                 *DAG.getContext());
  CCInfo.AnalyzeCallOperands(Outs, CC_Leros);

  // Get the size of the outgoing arguments stack space requirement.
  const unsigned NumBytes = CCInfo.getNextStackOffset();

  // Create local copies for byval args
  SmallVector<SDValue, 8> ByValArgs;
  for (unsigned i = 0, e = Outs.size(); i != e; ++i) {
    ISD::ArgFlagsTy Flags = Outs[i].Flags;
    if (!Flags.isByVal())
      continue;

    SDValue Arg = OutVals[i];
    unsigned Size = Flags.getByValSize();
    unsigned Align = Flags.getByValAlign();

    int FI = MF.getFrameInfo().CreateStackObject(Size, Align, /*isSS=*/false);
    SDValue FIPtr = DAG.getFrameIndex(FI, getPointerTy(DAG.getDataLayout()));
    SDValue SizeNode = DAG.getConstant(Size, dl, Subtarget.getXLenVT());

    Chain = DAG.getMemcpy(Chain, dl, FIPtr, Arg, SizeNode, Align,
                          /*IsVolatile=*/false,
                          /*AlwaysInline=*/false, /*IsTailCall=*/false,
                          MachinePointerInfo(), MachinePointerInfo());
    ByValArgs.push_back(FIPtr);
  }

  Chain = DAG.getCALLSEQ_START(Chain, NumBytes, 0, dl);

  SmallVector<std::pair<unsigned, SDValue>, 8> RegsToPass;
  SmallVector<SDValue, 8> MemOpChains;

  // Walk the register/memloc assignments, inserting copies/loads.
  for (unsigned i = 0, e = ArgLocs.size(); i != e; ++i) {
    CCValAssign &VA = ArgLocs[i];
    SDValue Arg = OutVals[i];

    // We only handle fully promoted arguments.
    assert(VA.getLocInfo() == CCValAssign::Full && "Unhandled loc info");

    if (VA.isRegLoc()) {
      RegsToPass.push_back(std::make_pair(VA.getLocReg(), Arg));
      continue;
    }

    assert(VA.isMemLoc() &&
           "Only support passing arguments through registers or via the stack");

    SDValue StackPtr = DAG.getRegister(Leros::R1, MVT::i32);
    SDValue PtrOff = DAG.getIntPtrConstant(VA.getLocMemOffset(), dl);
    PtrOff = DAG.getNode(ISD::ADD, dl, MVT::i32, StackPtr, PtrOff);
    MemOpChains.push_back(DAG.getStore(
        Chain, dl, Arg, PtrOff,
        MachinePointerInfo::getStack(MF, VA.getLocMemOffset()), 0));
  }

  // Emit all stores, make sure they occur before the call.
  if (!MemOpChains.empty()) {
    Chain = DAG.getNode(ISD::TokenFactor, dl, MVT::Other, MemOpChains);
  }

  // Build a sequence of copy-to-reg nodes chained together with token chain
  // and flag operands which copy the outgoing args into the appropriate regs.
  SDValue InFlag;
  for (auto &Reg : RegsToPass) {
    Chain = DAG.getCopyToReg(Chain, dl, Reg.first, Reg.second, InFlag);
    InFlag = Chain.getValue(1);
  }

  // If the callee is a GlobalAddress/ExternalSymbol node, turn it into a
  // TargetGlobalAddress/TargetExternalSymbol node so that legalize won't
  // split it and then direct call can be matched by PseudoCALL.
  if (GlobalAddressSDNode *S = dyn_cast<GlobalAddressSDNode>(Callee)) {
    Callee = DAG.getTargetGlobalAddress(S->getGlobal(), dl, PtrVT, 0, 0);
  } else if (ExternalSymbolSDNode *S = dyn_cast<ExternalSymbolSDNode>(Callee)) {
    Callee = DAG.getTargetExternalSymbol(S->getSymbol(), PtrVT, 0);
  }

  SmallVector<SDValue, 8> Ops;
  Ops.push_back(Chain);
  Ops.push_back(Callee);

  // Add argument registers to the end of the list so that they are known live
  // into the call.
  for (auto &Reg : RegsToPass) {
    Ops.push_back(DAG.getRegister(Reg.first, Reg.second.getValueType()));
  }

  // Add a register mask operand representing the call-preserved registers.
  const uint32_t *Mask;
  const TargetRegisterInfo *TRI = DAG.getSubtarget().getRegisterInfo();
  Mask = TRI->getCallPreservedMask(DAG.getMachineFunction(), CallConv);

  assert(Mask && "Missing call preserved mask for calling convention");
  Ops.push_back(DAG.getRegisterMask(Mask));

  if (InFlag.getNode()) {
    Ops.push_back(InFlag);
  }

  SDVTList NodeTys = DAG.getVTList(MVT::Other, MVT::Glue);

  // Returns a chain and a flag for retval copy to use.
  Chain = DAG.getNode(LEROSISD::Call, dl, NodeTys, Ops);
  InFlag = Chain.getValue(1);

  Chain = DAG.getCALLSEQ_END(Chain, DAG.getConstant(NumBytes, dl, PtrVT, true),
                             DAG.getConstant(0, dl, PtrVT, true), InFlag, dl);
  if (!Ins.empty()) {
    InFlag = Chain.getValue(1);
  }

  // Handle result values, copying them out of physregs into vregs that we
  // return.
  return LowerCallResult(Chain, InFlag, CallConv, isVarArg, Ins, dl, DAG,
                         InVals);
}

/// LowerCallResult - Lower the result values of a call into the
/// appropriate copies out of appropriate physical registers.
///
SDValue LerosTargetLowering::LowerCallResult(
    SDValue Chain, SDValue InFlag, CallingConv::ID CallConv, bool isVarArg,
    const SmallVectorImpl<ISD::InputArg> &Ins, const SDLoc &dl,
    SelectionDAG &DAG, SmallVectorImpl<SDValue> &InVals) const {

  assert(!isVarArg && "Unsupported");

  // Assign locations to each value returned by this call.
  SmallVector<CCValAssign, 16> RVLocs;
  CCState CCInfo(CallConv, isVarArg, DAG.getMachineFunction(), RVLocs,
                 *DAG.getContext());

  CCInfo.AnalyzeCallResult(Ins, RetCC_Leros);

  // Copy all of the result registers out of their specified physreg.
  for (auto &Loc : RVLocs) {
    Chain =
        DAG.getCopyFromReg(Chain, dl, Loc.getLocReg(), Loc.getValVT(), InFlag)
            .getValue(1);
    InFlag = Chain.getValue(2);
    InVals.push_back(Chain.getValue(0));
  }

  return Chain;
}

//===----------------------------------------------------------------------===//
//             Formal Arguments Calling Convention Implementation
//===----------------------------------------------------------------------===//

/// Leros formal arguments implementation
SDValue LerosTargetLowering::LowerFormalArguments(
    SDValue Chain, CallingConv::ID CallConv, bool isVarArg,
    const SmallVectorImpl<ISD::InputArg> &Ins, const SDLoc &dl,
    SelectionDAG &DAG, SmallVectorImpl<SDValue> &InVals) const {
  MachineFunction &MF = DAG.getMachineFunction();
  MachineRegisterInfo &RegInfo = MF.getRegInfo();

  assert(!isVarArg && "VarArg not supported");

  // Assign locations to all of the incoming arguments.
  SmallVector<CCValAssign, 16> ArgLocs;
  CCState CCInfo(CallConv, isVarArg, DAG.getMachineFunction(), ArgLocs,
                 *DAG.getContext());

  CCInfo.AnalyzeFormalArguments(Ins, CC_Leros);

  for (auto &VA : ArgLocs) {
    if (VA.isRegLoc()) {
      // Arguments passed in registers
      EVT RegVT = VA.getLocVT();
      assert(RegVT.getSimpleVT().SimpleTy == MVT::i32 &&
             "Only support MVT::i32 register passing");
      const unsigned VReg = RegInfo.createVirtualRegister(&Leros::GPRRegClass);
      RegInfo.addLiveIn(VA.getLocReg(), VReg);
      SDValue ArgIn = DAG.getCopyFromReg(Chain, dl, VReg, RegVT);

      InVals.push_back(ArgIn);
      continue;
    }

    assert(VA.isMemLoc() &&
           "Can only pass arguments as either registers or via the stack");

    const unsigned Offset = VA.getLocMemOffset();

    const int FI = MF.getFrameInfo().CreateFixedObject(4, Offset, true);
    EVT PtrTy = getPointerTy(DAG.getDataLayout());
    SDValue FIPtr = DAG.getFrameIndex(FI, PtrTy);

    assert(VA.getValVT() == MVT::i32 &&
           "Only support passing arguments as i32");
    SDValue Load = DAG.getLoad(VA.getValVT(), dl, Chain, FIPtr,
                               MachinePointerInfo::getFixedStack(MF, FI), 0);

    InVals.push_back(Load);
  }

  return Chain;
}

//===----------------------------------------------------------------------===//
//               Return Value Calling Convention Implementation
//===----------------------------------------------------------------------===//

bool LerosTargetLowering::isLegalICmpImmediate(int64_t Imm) const {
  return isInt<8>(Imm);
}

bool LerosTargetLowering::isLegalAddImmediate(int64_t Imm) const {
  return isInt<8>(Imm);
}

bool LerosTargetLowering::CanLowerReturn(
    CallingConv::ID CallConv, MachineFunction &MF, bool isVarArg,
    const SmallVectorImpl<ISD::OutputArg> &Outs, LLVMContext &Context) const {
  SmallVector<CCValAssign, 16> RVLocs;
  CCState CCInfo(CallConv, isVarArg, MF, RVLocs, Context);
  if (!CCInfo.CheckReturn(Outs, RetCC_Leros)) {
    return false;
  }
  if (CCInfo.getNextStackOffset() != 0 && isVarArg) {
    return false;
  }
  return true;
}

SDValue
LerosTargetLowering::LowerReturn(SDValue Chain, CallingConv::ID CallConv,
                                 bool isVarArg,
                                 const SmallVectorImpl<ISD::OutputArg> &Outs,
                                 const SmallVectorImpl<SDValue> &OutVals,
                                 const SDLoc &dl, SelectionDAG &DAG) const {
  if (isVarArg) {
    report_fatal_error("VarArg not supported");
  }

  // CCValAssign - represent the assignment of
  // the return value to a location
  SmallVector<CCValAssign, 16> RVLocs;

  // CCState - Info about the registers and stack slot.
  CCState CCInfo(CallConv, isVarArg, DAG.getMachineFunction(), RVLocs,
                 *DAG.getContext());

  CCInfo.AnalyzeReturn(Outs, RetCC_Leros);

  SDValue Flag;
  SmallVector<SDValue, 4> RetOps(1, Chain);

  // Copy the result values into the output registers.
  for (unsigned i = 0, e = RVLocs.size(); i < e; ++i) {
    CCValAssign &VA = RVLocs[i];
    assert(VA.isRegLoc() && "Can only return in registers!");

    Chain = DAG.getCopyToReg(Chain, dl, VA.getLocReg(), OutVals[i], Flag);

    Flag = Chain.getValue(1);
    RetOps.push_back(DAG.getRegister(VA.getLocReg(), VA.getLocVT()));
  }

  RetOps[0] = Chain; // Update chain.

  // Add the flag if we have it.
  if (Flag.getNode()) {
    RetOps.push_back(Flag);
  }

  return DAG.getNode(LEROSISD::Ret, dl, MVT::Other, RetOps);
}
} // namespace llvm
