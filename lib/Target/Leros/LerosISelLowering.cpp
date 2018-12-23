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

  // Variable argument support
  setOperationAction(ISD::VASTART, MVT::Other, Custom);
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

  setOperationAction(ISD::SHL, XLenVT, LibCall);
  setOperationAction(ISD::SRL, XLenVT, LibCall);
  setOperationAction(ISD::SRA, XLenVT, LibCall);

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
      SDValue(DAG.getMachineNode(Leros::LOADI_PSEUDO, DL, Ty, GAB0), 0);
  SDValue MNB1 =
      SDValue(DAG.getMachineNode(Leros::LOADHI_PSEUDO, DL, Ty, MNB0, GAB1), 0);
  SDValue MNB2 =
      SDValue(DAG.getMachineNode(Leros::LOADH2I_PSEUDO, DL, Ty, MNB1, GAB2), 0);
  SDValue MNB3 =
      SDValue(DAG.getMachineNode(Leros::LOADH3I_PSEUDO, DL, Ty, MNB2, GAB3), 0);

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
      SDValue(DAG.getMachineNode(Leros::LOADI_PSEUDO, DL, Ty, BAB0), 0);
  SDValue MNB1 =
      SDValue(DAG.getMachineNode(Leros::LOADHI_PSEUDO, DL, Ty, MNB0, BAB1), 0);
  SDValue MNB2 =
      SDValue(DAG.getMachineNode(Leros::LOADH2I_PSEUDO, DL, Ty, MNB1, BAB2), 0);
  SDValue MNB3 =
      SDValue(DAG.getMachineNode(Leros::LOADH3I_PSEUDO, DL, Ty, MNB2, BAB3), 0);

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

SDValue LerosTargetLowering::lowerVASTART(SDValue Op, SelectionDAG &DAG) const {
  MachineFunction &MF = DAG.getMachineFunction();
  LerosMachineFunctionInfo *FuncInfo = MF.getInfo<LerosMachineFunctionInfo>();
  auto PtrVT = getPointerTy(DAG.getDataLayout());

  // Frame index of first vararg argument
  SDValue FrameIndex =
      DAG.getFrameIndex(FuncInfo->getVarArgsFrameIndex(), PtrVT);
  const Value *SV = cast<SrcValueSDNode>(Op.getOperand(2))->getValue();

  // Create a store of the frame index to the location operand
  return DAG.getStore(Op.getOperand(0), SDLoc(Op), FrameIndex, Op.getOperand(1),
                      MachinePointerInfo(SV));
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
  case ISD::VASTART:
    return lowerVASTART(Op, DAG);
  case ISD::FRAMEADDR:
    return lowerFRAMEADDR(Op, DAG);
  case ISD::RETURNADDR:
    return lowerRETURNADDR(Op, DAG);
  }
}

MachineBasicBlock *LerosTargetLowering::EmitUSET(MachineInstr &MI,
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

  unsigned unsignedCMPOpcode;

  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Unknown USET opcode");
  case Leros::SETUGE_PSEUDO: {
    opcode = Leros::BRGTE_PSEUDO;
    unsignedCMPOpcode = Leros::BRN_PSEUDO;
    break;
  }
  case Leros::SETULT_PSEUDO: {
    opcode = Leros::BRLT_PSEUDO;
    unsignedCMPOpcode = Leros::BRP_PSEUDO;
    break;
  }
  }

  const unsigned &dstReg = MI.getOperand(0).getReg(),
                 &rs1 = MI.getOperand(1).getReg(),
                 &rs2 = MI.getOperand(2).getReg();

  MachineBasicBlock *SignedComparisonMBB = F->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *UnsignedComparisonMBB =
      F->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *UnsignedRS1Negative = F->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *UnsignedRS2Negative = F->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *UnsignedTailMBB = F->CreateMachineBasicBlock(LLVM_BB);

  MachineBasicBlock *falseMBB = F->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *trueMBB = F->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *SignedTailMBB = F->CreateMachineBasicBlock(LLVM_BB);

  MachineBasicBlock *TailMBB = F->CreateMachineBasicBlock(LLVM_BB);

  // Insertion order matters, to properly handle BB fallthrough
  F->insert(I, UnsignedComparisonMBB);
  F->insert(I, UnsignedRS2Negative);
  F->insert(I, UnsignedRS1Negative);
  F->insert(I, UnsignedTailMBB);

  F->insert(I, SignedComparisonMBB);
  F->insert(I, falseMBB);
  F->insert(I, trueMBB);
  F->insert(I, SignedTailMBB);

  F->insert(I, TailMBB);

  // Move all remaining instructions to TailMBB.
  TailMBB->splice(TailMBB->begin(), HeadMBB,
                  std::next(MachineBasicBlock::iterator(MI)), HeadMBB->end());

  // Update machine-CFG edges by transferring all successors of the current
  // block to the new block which will contain the Phi node for the select.
  TailMBB->transferSuccessorsAndUpdatePHIs(HeadMBB);

  HeadMBB->addSuccessor(SignedComparisonMBB);
  HeadMBB->addSuccessor(UnsignedComparisonMBB);

  UnsignedComparisonMBB->addSuccessor(UnsignedRS1Negative);
  UnsignedComparisonMBB->addSuccessor(UnsignedRS2Negative);

  UnsignedRS1Negative->addSuccessor(UnsignedTailMBB);
  UnsignedRS2Negative->addSuccessor(UnsignedTailMBB);

  UnsignedTailMBB->addSuccessor(TailMBB);

  // Set successors for signed comparison basic blocks
  SignedComparisonMBB->addSuccessor(falseMBB);
  SignedComparisonMBB->addSuccessor(trueMBB);

  trueMBB->addSuccessor(SignedTailMBB);
  falseMBB->addSuccessor(SignedTailMBB);

  SignedTailMBB->addSuccessor(TailMBB);

  // Generate the true- and false value registers. Machine PHI nodes must have
  // virtual registers as operands, which is why the constant registers are not
  // used
  unsigned TrueReg = MRI.createVirtualRegister(&Leros::GPRRegClass);
  unsigned FalseReg = MRI.createVirtualRegister(&Leros::GPRRegClass);
  TII.movImm32(*HeadMBB, HeadMBB->end(), DL, TrueReg, 1);
  TII.movImm32(*HeadMBB, HeadMBB->end(), DL, FalseReg, 0);

  // Mask input operands
  unsigned rs1_s = MRI.createVirtualRegister(&Leros::GPRRegClass);
  unsigned rs2_s = MRI.createVirtualRegister(&Leros::GPRRegClass);

  BuildMI(*HeadMBB, HeadMBB->end(), DL, TII.get(Leros::AND_PSEUDO), rs1_s)
      .addReg(rs1)
      .addReg(LEROSCREG::B32SIGN);

  BuildMI(*HeadMBB, HeadMBB->end(), DL, TII.get(Leros::AND_PSEUDO), rs2_s)
      .addReg(rs2)
      .addReg(LEROSCREG::B32SIGN);

  // XOR the two sign registers
  unsigned xor_res = MRI.createVirtualRegister(&Leros::GPRRegClass);
  BuildMI(*HeadMBB, HeadMBB->end(), DL, TII.get(Leros::XOR_PSEUDO), xor_res)
      .addReg(rs1_s)
      .addReg(rs2_s);

  // xor_res != 0 =>
  // Discrepancy in sign bit
  // Negative operand will always be the larger of the two in an unsigned
  // comparison.
  // Depending on if it is greater than or less than,
  // Detect which operand was negative or positive and return accordingly.
  // Branch to regular signed operation if we do not detect a discrepancy
  BuildMI(*HeadMBB, HeadMBB->end(), DL, TII.get(Leros::BRZ_PSEUDO))
      .addReg(xor_res)
      .addMBB(SignedComparisonMBB);

  // ----- fallthrough from head - discrepancy in sign bit.  -----
  // Unsigned comparison is required
  unsigned UnsignedResReg = MRI.createVirtualRegister(&Leros::GPRRegClass);

  BuildMI(*UnsignedComparisonMBB, UnsignedComparisonMBB->end(), DL,
          TII.get(unsignedCMPOpcode))
      .addReg(rs1_s)
      .addMBB(UnsignedRS1Negative);

  // ---- UnsignedRS2Negative rs2 is negative ----
  BuildMI(*UnsignedRS2Negative, UnsignedRS2Negative->end(), DL,
          TII.get(Leros::BR_IMPL))
      .addMBB(UnsignedTailMBB);

  // ---- UnsignedRS1Negative: rs1 is negative ----
  // This is a fallthrough , we handle the actual result in the phi node

  // Unsigned res PHI -
  BuildMI(*UnsignedTailMBB, UnsignedTailMBB->end(), DL, TII.get(Leros::PHI),
          UnsignedResReg)
      .addReg(TrueReg)
      .addMBB(UnsignedRS1Negative)
      .addReg(FalseReg)
      .addMBB(UnsignedRS2Negative);

  // Branch to tail, which now knows to grab the result from the unsigned
  // comparison

  BuildMI(*UnsignedTailMBB, UnsignedTailMBB->end(), DL, TII.get(Leros::BR_IMPL))
      .addMBB(TailMBB);

  // -------- SignedComparisonMBB --------------
  // This is pretty much equivalent as for the usual signed EmitSET
  const unsigned SignedResReg = MRI.createVirtualRegister(&Leros::GPRRegClass);
  BuildMI(*SignedComparisonMBB, SignedComparisonMBB->end(), DL, TII.get(opcode))
      .addReg(rs1)
      .addReg(rs2)
      .addMBB(trueMBB);

  // fallthrough to falseMBB

  // ------- FalseMBB --------

  // From here, we know that comparison was false
  // register
  BuildMI(*falseMBB, falseMBB->end(), DL, TII.get(Leros::BR_IMPL))
      .addMBB(SignedTailMBB);

  // -------- trueMBB --------------
  // Fallthrough to tail

  // -------- SignedTailMBB --------------
  // Get result through a phi node
  BuildMI(*SignedTailMBB, SignedTailMBB->end(), DL, TII.get(Leros::PHI),
          SignedResReg)
      .addReg(FalseReg)
      .addMBB(falseMBB)
      .addReg(TrueReg)
      .addMBB(trueMBB);
  // Fallthrough to tail

  // -------- TailMBB --------------
  // Get result through a phi node, depending on whether we did signed or
  // unsigned comparison
  BuildMI(*TailMBB, TailMBB->begin(), DL, TII.get(Leros::PHI), dstReg)
      .addReg(SignedResReg)
      .addMBB(SignedTailMBB)
      .addReg(UnsignedResReg)
      .addMBB(UnsignedTailMBB);

  MI.eraseFromParent(); // The pseudo instruction is gone now.
  return TailMBB;
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
    opcode = Leros::BREQ_PSEUDO;
    break;
  }
  case Leros::SETGE_PSEUDO: {
    opcode = Leros::BRGTE_PSEUDO;
    break;
  }
  case Leros::SETLT_PSEUDO: {
    opcode = Leros::BRLT_PSEUDO;
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

  BuildMI(*HeadMBB, HeadMBB->end(), DL, TII.get(opcode))
      .addReg(rs1)
      .addReg(rs2)
      .addMBB(trueMBB);

  // fallthrough to falseMBB

  // ------- FalseMBB --------

  // From here, we know that comparison was false
  // register
  const unsigned falseRes = MRI.createVirtualRegister(&Leros::GPRRegClass);
  TII.movImm32(*falseMBB, falseMBB->end(), DL, falseRes, 0);

  BuildMI(*falseMBB, falseMBB->end(), DL, TII.get(Leros::BR_IMPL))
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
  unsigned signMaskReg;

  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Unknown SEXT opcode");
  case Leros::LOAD_S8_M_PSEUDO: {
    memLoadOpcode = Leros::LOAD_U8_M_PSEUDO;
    opcode = Leros::LOADHI_PSEUDO;
    signMaskReg = LEROSCREG::B8SIGN;
    break;
  }
  case Leros::LOAD_S16_M_PSEUDO: {
    memLoadOpcode = Leros::LOAD_M_PSEUDO;
    opcode = Leros::LOADH2I_PSEUDO;
    signMaskReg = LEROSCREG::B16SIGN;
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

  const unsigned maskedLoad = MRI.createVirtualRegister(&Leros::GPRRegClass);
  BuildMI(*HeadMBB, HeadMBB->end(), DL, TII.get(Leros::AND_PSEUDO), maskedLoad)
      .addReg(ScratchReg)
      .addReg(signMaskReg);

  // result != 0 => negative number
  BuildMI(*HeadMBB, HeadMBB->end(), DL, TII.get(Leros::BRNZ_PSEUDO))
      .addReg(maskedLoad)
      .addMBB(negMBB);

  // -------- posMBB --------------
  // From here we know that the loaded operand is positive - bitmask lower bits
  const unsigned ZEXTRes = MRI.createVirtualRegister(&Leros::GPRRegClass);
  BuildMI(*posMBB, posMBB->end(), DL, TII.get(opcode), ZEXTRes)
      .addReg(ScratchReg)
      .addImm(0x0);

  BuildMI(*posMBB, posMBB->end(), DL, TII.get(Leros::BR_IMPL)).addMBB(TailMBB);

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

  IfTrueMBB->addSuccessor(TailMBB);

  // Insert appropriate branch. If CC is 0, we branch to TailMBB and the select
  // the 2nd register
  unsigned CCReg = MI.getOperand(1).getReg();
  BuildMI(*HeadMBB, HeadMBB->end(), DL, TII.get(Leros::BRZ_PSEUDO))
      .addReg(CCReg)
      .addMBB(TailMBB);

  // IfTrueMBB - nothing here, falls through to TailMBB

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
    const unsigned MaskedMem = MRI.createVirtualRegister(&Leros::GPRRegClass);
    BuildMI(*HeadMBB, HeadMBB->end(), DL, TII.get(Leros::AND_PSEUDO), MaskedMem)
        .addReg(MemReg)
        .addReg(LEROSCREG::UHMASK);

    // Mask the lower bits of the store register
    const unsigned MaskedStore = MRI.createVirtualRegister(&Leros::GPRRegClass);
    BuildMI(*HeadMBB, HeadMBB->end(), DL, TII.get(Leros::AND_PSEUDO),
            MaskedStore)
        .addReg(rstore)
        .addReg(LEROSCREG::LHMASK);

    // OR the two registers
    const unsigned MaskedRes = MRI.createVirtualRegister(&Leros::GPRRegClass);
    BuildMI(*HeadMBB, HeadMBB->end(), DL, TII.get(Leros::OR_PSEUDO), MaskedRes)
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
  case Leros::SETEQ_PSEUDO:
  case Leros::SETGE_PSEUDO:
  case Leros::SETLT_PSEUDO:
    return EmitSET(MI, BB);
  case Leros::SETUGE_PSEUDO:
  case Leros::SETULT_PSEUDO:
    return EmitUSET(MI, BB);
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

static const MCPhysReg ArgGPRs[] = {
    Leros::R4, Leros::R5, Leros::R6,  Leros::R7,
    Leros::R8, Leros::R9, Leros::R10,
};

// Pass a 2*XLEN argument that has been split into two XLEN values through
// registers or the stack as necessary.
static bool CC_LerosAssign2XLen(unsigned XLen, CCState &State, CCValAssign VA1,
                                ISD::ArgFlagsTy ArgFlags1, unsigned ValNo2,
                                MVT ValVT2, MVT LocVT2,
                                ISD::ArgFlagsTy ArgFlags2) {
  unsigned XLenInBytes = XLen / 8;
  if (unsigned Reg = State.AllocateReg(ArgGPRs)) {
    // At least one half can be passed via register.
    State.addLoc(CCValAssign::getReg(VA1.getValNo(), VA1.getValVT(), Reg,
                                     VA1.getLocVT(), CCValAssign::Full));
  } else {
    // Both halves must be passed on the stack, with proper alignment.
    unsigned StackAlign = std::max(XLenInBytes, ArgFlags1.getOrigAlign());
    State.addLoc(
        CCValAssign::getMem(VA1.getValNo(), VA1.getValVT(),
                            State.AllocateStack(XLenInBytes, StackAlign),
                            VA1.getLocVT(), CCValAssign::Full));
    State.addLoc(CCValAssign::getMem(
        ValNo2, ValVT2, State.AllocateStack(XLenInBytes, XLenInBytes), LocVT2,
        CCValAssign::Full));
    return false;
  }

  if (unsigned Reg = State.AllocateReg(ArgGPRs)) {
    // The second half can also be passed via register.
    State.addLoc(
        CCValAssign::getReg(ValNo2, ValVT2, Reg, LocVT2, CCValAssign::Full));
  } else {
    // The second half is passed via the stack, without additional alignment.
    State.addLoc(CCValAssign::getMem(
        ValNo2, ValVT2, State.AllocateStack(XLenInBytes, XLenInBytes), LocVT2,
        CCValAssign::Full));
  }

  return false;
}

// Implements the Leros calling convention. Returns true upon failure.
static bool CC_Leros(const DataLayout &DL, unsigned ValNo, MVT ValVT, MVT LocVT,
                     CCValAssign::LocInfo LocInfo, ISD::ArgFlagsTy ArgFlags,
                     CCState &State, bool IsFixed, bool IsRet, Type *OrigTy) {
  unsigned XLen = DL.getLargestLegalIntTypeSizeInBits();
  assert(XLen == 32 || XLen == 64);
  MVT XLenVT = XLen == 32 ? MVT::i32 : MVT::i64;
  if (ValVT == MVT::f32) {
    LocVT = MVT::i32;
    LocInfo = CCValAssign::BCvt;
  }

  // Any return value split in to more than two values can't be returned
  // directly.
  if (IsRet && ValNo > 1)
    return true;

  // If this is a variadic argument, the Leros calling convention requires
  // that it is assigned an 'even' or 'aligned' register if it has 8-byte
  // alignment (leros32). An aligned register should
  // be used regardless of whether the original argument was split during
  // legalisation or not. The argument will not be passed by registers if the
  // original type is larger than 2*XLEN, so the register alignment rule does
  // not apply.
  unsigned TwoXLenInBytes = (2 * XLen) / 8;
  if (!IsFixed && ArgFlags.getOrigAlign() == TwoXLenInBytes &&
      DL.getTypeAllocSize(OrigTy) == TwoXLenInBytes) {
    unsigned RegIdx = State.getFirstUnallocated(ArgGPRs);
    // Skip 'odd' register if necessary.
    if (RegIdx != array_lengthof(ArgGPRs) && RegIdx % 2 == 1)
      State.AllocateReg(ArgGPRs);
  }

  SmallVectorImpl<CCValAssign> &PendingLocs = State.getPendingLocs();
  SmallVectorImpl<ISD::ArgFlagsTy> &PendingArgFlags =
      State.getPendingArgFlags();

  assert(PendingLocs.size() == PendingArgFlags.size() &&
         "PendingLocs and PendingArgFlags out of sync");

  // Handle passing f64 on RV32D with a soft float ABI.
  if (XLen == 32 && ValVT == MVT::f64) {
    assert(!ArgFlags.isSplit() && PendingLocs.empty() &&
           "Can't lower f64 if it is split");
    // Depending on available argument GPRS, f64 may be passed in a pair of
    // GPRs, split between a GPR and the stack, or passed completely on the
    // stack. LowerCall/LowerFormalArguments/LowerReturn must recognise these
    // cases.
    unsigned Reg = State.AllocateReg(ArgGPRs);
    LocVT = MVT::i32;
    if (!Reg) {
      unsigned StackOffset = State.AllocateStack(8, 8);
      State.addLoc(
          CCValAssign::getMem(ValNo, ValVT, StackOffset, LocVT, LocInfo));
      return false;
    }
    if (!State.AllocateReg(ArgGPRs))
      State.AllocateStack(4, 4);
    State.addLoc(CCValAssign::getReg(ValNo, ValVT, Reg, LocVT, LocInfo));
    return false;
  }

  // Split arguments might be passed indirectly, so keep track of the pending
  // values.
  if (ArgFlags.isSplit() || !PendingLocs.empty()) {
    LocVT = XLenVT;
    LocInfo = CCValAssign::Indirect;
    PendingLocs.push_back(
        CCValAssign::getPending(ValNo, ValVT, LocVT, LocInfo));
    PendingArgFlags.push_back(ArgFlags);
    if (!ArgFlags.isSplitEnd()) {
      return false;
    }
  }

  // If the split argument only had two elements, it should be passed directly
  // in registers or on the stack.
  if (ArgFlags.isSplitEnd() && PendingLocs.size() <= 2) {
    assert(PendingLocs.size() == 2 && "Unexpected PendingLocs.size()");
    // Apply the normal calling convention rules to the first half of the
    // split argument.
    CCValAssign VA = PendingLocs[0];
    ISD::ArgFlagsTy AF = PendingArgFlags[0];
    PendingLocs.clear();
    PendingArgFlags.clear();
    return CC_LerosAssign2XLen(XLen, State, VA, AF, ValNo, ValVT, LocVT,
                               ArgFlags);
  }

  // Allocate to a register if possible, or else a stack slot.
  unsigned Reg = State.AllocateReg(ArgGPRs);
  unsigned StackOffset = Reg ? 0 : State.AllocateStack(XLen / 8, XLen / 8);

  // If we reach this point and PendingLocs is non-empty, we must be at the
  // end of a split argument that must be passed indirectly.
  if (!PendingLocs.empty()) {
    assert(ArgFlags.isSplitEnd() && "Expected ArgFlags.isSplitEnd()");
    assert(PendingLocs.size() > 2 && "Unexpected PendingLocs.size()");

    for (auto &It : PendingLocs) {
      if (Reg)
        It.convertToReg(Reg);
      else
        It.convertToMem(StackOffset);
      State.addLoc(It);
    }
    PendingLocs.clear();
    PendingArgFlags.clear();
    return false;
  }

  assert(LocVT == XLenVT && "Expected an XLenVT at this stage");

  if (Reg) {
    State.addLoc(CCValAssign::getReg(ValNo, ValVT, Reg, LocVT, LocInfo));
    return false;
  }

  if (ValVT == MVT::f32) {
    LocVT = MVT::f32;
    LocInfo = CCValAssign::Full;
  }
  State.addLoc(CCValAssign::getMem(ValNo, ValVT, StackOffset, LocVT, LocInfo));
  return false;
}

void LerosTargetLowering::analyzeInputArgs(
    MachineFunction &MF, CCState &CCInfo,
    const SmallVectorImpl<ISD::InputArg> &Ins, bool IsRet) const {
  unsigned NumArgs = Ins.size();
  FunctionType *FType = MF.getFunction().getFunctionType();

  for (unsigned i = 0; i != NumArgs; ++i) {
    MVT ArgVT = Ins[i].VT;
    ISD::ArgFlagsTy ArgFlags = Ins[i].Flags;

    Type *ArgTy = nullptr;
    if (IsRet)
      ArgTy = FType->getReturnType();
    else if (Ins[i].isOrigArg())
      ArgTy = FType->getParamType(Ins[i].getOrigArgIndex());

    if (CC_Leros(MF.getDataLayout(), i, ArgVT, ArgVT, CCValAssign::Full,
                 ArgFlags, CCInfo, /*IsRet=*/true, IsRet, ArgTy)) {
      LLVM_DEBUG(dbgs() << "InputArg #" << i << " has unhandled type "
                        << EVT(ArgVT).getEVTString() << '\n');
      llvm_unreachable(nullptr);
    }
  }
}

void LerosTargetLowering::analyzeOutputArgs(
    MachineFunction &MF, CCState &CCInfo,
    const SmallVectorImpl<ISD::OutputArg> &Outs, bool IsRet,
    CallLoweringInfo *CLI) const {
  unsigned NumArgs = Outs.size();

  for (unsigned i = 0; i != NumArgs; i++) {
    MVT ArgVT = Outs[i].VT;
    ISD::ArgFlagsTy ArgFlags = Outs[i].Flags;
    Type *OrigTy = CLI ? CLI->getArgs()[Outs[i].OrigArgIndex].Ty : nullptr;

    if (CC_Leros(MF.getDataLayout(), i, ArgVT, ArgVT, CCValAssign::Full,
                 ArgFlags, CCInfo, Outs[i].IsFixed, IsRet, OrigTy)) {
      LLVM_DEBUG(dbgs() << "OutputArg #" << i << " has unhandled type "
                        << EVT(ArgVT).getEVTString() << "\n");
      llvm_unreachable(nullptr);
    }
  }
}

static SDValue convertValVTToLocVT(SelectionDAG &DAG, SDValue Val,
                                   const CCValAssign &VA, const SDLoc &DL) {
  EVT LocVT = VA.getLocVT();

  switch (VA.getLocInfo()) {
  default:
    llvm_unreachable("Unexpected CCValAssign::LocInfo");
  case CCValAssign::Full:
    break;
  case CCValAssign::BCvt:
    Val = DAG.getNode(ISD::BITCAST, DL, LocVT, Val);
    break;
  }
  return Val;
}

// Convert Val to a ValVT. Should not be called for CCValAssign::Indirect
// values.
static SDValue convertLocVTToValVT(SelectionDAG &DAG, SDValue Val,
                                   const CCValAssign &VA, const SDLoc &DL) {
  switch (VA.getLocInfo()) {
  default:
    llvm_unreachable("Unexpected CCValAssign::LocInfo");
  case CCValAssign::Full:
    break;
  case CCValAssign::BCvt:
    Val = DAG.getNode(ISD::BITCAST, DL, VA.getValVT(), Val);
    break;
  }
  return Val;
}

/// Leros call implementation
SDValue LerosTargetLowering::LowerCall(TargetLowering::CallLoweringInfo &CLI,
                                       SmallVectorImpl<SDValue> &InVals) const {
  SelectionDAG &DAG = CLI.DAG;
  SDLoc &DL = CLI.DL;
  SmallVectorImpl<ISD::OutputArg> &Outs = CLI.Outs;
  SmallVectorImpl<SDValue> &OutVals = CLI.OutVals;
  SmallVectorImpl<ISD::InputArg> &Ins = CLI.Ins;
  SDValue Chain = CLI.Chain;
  SDValue Callee = CLI.Callee;
  bool &IsTailCall = CLI.IsTailCall;
  CallingConv::ID CallConv = CLI.CallConv;
  bool IsVarArg = CLI.IsVarArg;
  EVT PtrVT = getPointerTy(DAG.getDataLayout());
  MVT XLenVT = Subtarget.getXLenVT();

  MachineFunction &MF = DAG.getMachineFunction();

  // Analyze the operands of the call, assigning locations to each operand.
  SmallVector<CCValAssign, 16> ArgLocs;
  CCState ArgCCInfo(CallConv, IsVarArg, MF, ArgLocs, *DAG.getContext());
  analyzeOutputArgs(MF, ArgCCInfo, Outs, /*IsRet=*/false, &CLI);

  // Leros does not support tail calls
  IsTailCall = false;

  if (CLI.CS && CLI.CS.isMustTailCall())
    report_fatal_error("failed to perform tail call elimination on a call "
                       "site marked musttail");

  // Get a count of how many bytes are to be pushed on the stack.
  unsigned NumBytes = ArgCCInfo.getNextStackOffset();

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
    SDValue SizeNode = DAG.getConstant(Size, DL, XLenVT);

    Chain = DAG.getMemcpy(Chain, DL, FIPtr, Arg, SizeNode, Align,
                          /*IsVolatile=*/false,
                          /*AlwaysInline=*/false, IsTailCall,
                          MachinePointerInfo(), MachinePointerInfo());
    ByValArgs.push_back(FIPtr);
  }

  Chain = DAG.getCALLSEQ_START(Chain, NumBytes, 0, CLI.DL);

  // Copy argument values to their designated locations.
  SmallVector<std::pair<unsigned, SDValue>, 8> RegsToPass;
  SmallVector<SDValue, 8> MemOpChains;
  SDValue StackPtr;
  for (unsigned i = 0, j = 0, e = ArgLocs.size(); i != e; ++i) {
    CCValAssign &VA = ArgLocs[i];
    SDValue ArgValue = OutVals[i];
    ISD::ArgFlagsTy Flags = Outs[i].Flags;

    // Promote the value if needed.
    // For now, only handle fully promoted and indirect arguments.
    if (VA.getLocInfo() == CCValAssign::Indirect) {
      // Store the argument in a stack slot and pass its address.
      SDValue SpillSlot = DAG.CreateStackTemporary(Outs[i].ArgVT);
      int FI = cast<FrameIndexSDNode>(SpillSlot)->getIndex();
      MemOpChains.push_back(
          DAG.getStore(Chain, DL, ArgValue, SpillSlot,
                       MachinePointerInfo::getFixedStack(MF, FI)));
      // If the original argument was split (e.g. i128), we need
      // to store all parts of it here (and pass just one address).
      unsigned ArgIndex = Outs[i].OrigArgIndex;
      assert(Outs[i].PartOffset == 0);
      while (i + 1 != e && Outs[i + 1].OrigArgIndex == ArgIndex) {
        SDValue PartValue = OutVals[i + 1];
        unsigned PartOffset = Outs[i + 1].PartOffset;
        SDValue Address = DAG.getNode(ISD::ADD, DL, PtrVT, SpillSlot,
                                      DAG.getIntPtrConstant(PartOffset, DL));
        MemOpChains.push_back(
            DAG.getStore(Chain, DL, PartValue, Address,
                         MachinePointerInfo::getFixedStack(MF, FI)));
        ++i;
      }
      ArgValue = SpillSlot;
    } else {
      ArgValue = convertValVTToLocVT(DAG, ArgValue, VA, DL);
    }

    // Use local copy if it is a byval arg.
    if (Flags.isByVal())
      ArgValue = ByValArgs[j++];

    if (VA.isRegLoc()) {
      // Queue up the argument copies and emit them at the end.
      RegsToPass.push_back(std::make_pair(VA.getLocReg(), ArgValue));
    } else {
      assert(VA.isMemLoc() && "Argument not register or memory");
      assert(!IsTailCall && "Tail call not allowed if stack is used "
                            "for passing parameters");

      // Work out the address of the stack slot.
      if (!StackPtr.getNode())
        StackPtr = DAG.getCopyFromReg(Chain, DL, Leros::R1, PtrVT);
      SDValue Address =
          DAG.getNode(ISD::ADD, DL, PtrVT, StackPtr,
                      DAG.getIntPtrConstant(VA.getLocMemOffset(), DL));

      // Emit the store.
      MemOpChains.push_back(
          DAG.getStore(Chain, DL, ArgValue, Address, MachinePointerInfo()));
    }
  }

  // Join the stores, which are independent of one another.
  if (!MemOpChains.empty())
    Chain = DAG.getNode(ISD::TokenFactor, DL, MVT::Other, MemOpChains);

  SDValue Glue;

  // Build a sequence of copy-to-reg nodes, chained and glued together.
  for (auto &Reg : RegsToPass) {
    Chain = DAG.getCopyToReg(Chain, DL, Reg.first, Reg.second, Glue);
    Glue = Chain.getValue(1);
  }

  // If the callee is a GlobalAddress/ExternalSymbol node, turn it into a
  // TargetGlobalAddress/TargetExternalSymbol node so that legalize won't
  // split it and then direct call can be matched by PseudoCALL.
  if (GlobalAddressSDNode *S = dyn_cast<GlobalAddressSDNode>(Callee)) {
    Callee = DAG.getTargetGlobalAddress(S->getGlobal(), DL, PtrVT, 0, 0);
  } else if (ExternalSymbolSDNode *S = dyn_cast<ExternalSymbolSDNode>(Callee)) {
    Callee = DAG.getTargetExternalSymbol(S->getSymbol(), PtrVT, 0);
  }

  // The first call operand is the chain and the second is the target address.
  SmallVector<SDValue, 8> Ops;
  Ops.push_back(Chain);
  Ops.push_back(Callee);

  // Add argument registers to the end of the list so that they are
  // known live into the call.
  for (auto &Reg : RegsToPass)
    Ops.push_back(DAG.getRegister(Reg.first, Reg.second.getValueType()));

  if (!IsTailCall) {
    // Add a register mask operand representing the call-preserved registers.
    const TargetRegisterInfo *TRI = Subtarget.getRegisterInfo();
    const uint32_t *Mask = TRI->getCallPreservedMask(MF, CallConv);
    assert(Mask && "Missing call preserved mask for calling convention");
    Ops.push_back(DAG.getRegisterMask(Mask));
  }

  // Glue the call to the argument copies, if any.
  if (Glue.getNode())
    Ops.push_back(Glue);

  // Emit the call.
  SDVTList NodeTys = DAG.getVTList(MVT::Other, MVT::Glue);

  Chain = DAG.getNode(LEROSISD::Call, DL, NodeTys, Ops);
  Glue = Chain.getValue(1);

  // Mark the end of the call, which is glued to the call itself.
  Chain = DAG.getCALLSEQ_END(Chain, DAG.getConstant(NumBytes, DL, PtrVT, true),
                             DAG.getConstant(0, DL, PtrVT, true), Glue, DL);
  Glue = Chain.getValue(1);

  // Assign locations to each value returned by this call.
  SmallVector<CCValAssign, 16> RVLocs;
  CCState RetCCInfo(CallConv, IsVarArg, MF, RVLocs, *DAG.getContext());
  analyzeInputArgs(MF, RetCCInfo, Ins, /*IsRet=*/true);

  // Copy all of the result registers out of their specified physreg.
  for (auto &VA : RVLocs) {
    // Copy the value out
    SDValue RetValue =
        DAG.getCopyFromReg(Chain, DL, VA.getLocReg(), VA.getLocVT(), Glue);
    // Glue the RetValue to the end of the call sequence
    Chain = RetValue.getValue(1);
    Glue = RetValue.getValue(2);

    RetValue = convertLocVTToValVT(DAG, RetValue, VA, DL);

    InVals.push_back(RetValue);
  }

  return Chain;
}

/// LowerCallResult - Lower the result values of a call into the
/// appropriate copies out of appropriate physical registers.
///
SDValue LerosTargetLowering::LowerCallResult(
    SDValue Chain, SDValue InFlag, CallingConv::ID CallConv, bool isVarArg,
    const SmallVectorImpl<ISD::InputArg> &Ins, const SDLoc &dl,
    SelectionDAG &DAG, SmallVectorImpl<SDValue> &InVals) const {

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

// The caller is responsible for loading the full value if the argument is
// passed with CCValAssign::Indirect.
static SDValue unpackFromMemLoc(SelectionDAG &DAG, SDValue Chain,
                                const CCValAssign &VA, const SDLoc &DL) {
  MachineFunction &MF = DAG.getMachineFunction();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  EVT LocVT = VA.getLocVT();
  EVT ValVT = VA.getValVT();
  EVT PtrVT = MVT::getIntegerVT(DAG.getDataLayout().getPointerSizeInBits(0));
  int FI = MFI.CreateFixedObject(ValVT.getSizeInBits() / 8,
                                 VA.getLocMemOffset(), /*Immutable=*/true);
  SDValue FIN = DAG.getFrameIndex(FI, PtrVT);
  SDValue Val;

  ISD::LoadExtType ExtType;
  switch (VA.getLocInfo()) {
  default:
    llvm_unreachable("Unexpected CCValAssign::LocInfo");
  case CCValAssign::Full:
  case CCValAssign::Indirect:
    ExtType = ISD::NON_EXTLOAD;
    break;
  }
  Val = DAG.getExtLoad(
      ExtType, DL, LocVT, Chain, FIN,
      MachinePointerInfo::getFixedStack(DAG.getMachineFunction(), FI), ValVT);
  return Val;
}

// The caller is responsible for loading the full value if the argument is
// passed with CCValAssign::Indirect.
static SDValue unpackFromRegLoc(SelectionDAG &DAG, SDValue Chain,
                                const CCValAssign &VA, const SDLoc &DL) {
  MachineFunction &MF = DAG.getMachineFunction();
  MachineRegisterInfo &RegInfo = MF.getRegInfo();
  EVT LocVT = VA.getLocVT();
  SDValue Val;

  unsigned VReg = RegInfo.createVirtualRegister(&Leros::GPRRegClass);
  RegInfo.addLiveIn(VA.getLocReg(), VReg);
  Val = DAG.getCopyFromReg(Chain, DL, VReg, LocVT);

  if (VA.getLocInfo() == CCValAssign::Indirect)
    return Val;

  return convertLocVTToValVT(DAG, Val, VA, DL);
}

/// Leros formal arguments implementation
SDValue LerosTargetLowering::LowerFormalArguments(
    SDValue Chain, CallingConv::ID CallConv, bool IsVarArg,
    const SmallVectorImpl<ISD::InputArg> &Ins, const SDLoc &DL,
    SelectionDAG &DAG, SmallVectorImpl<SDValue> &InVals) const {
  switch (CallConv) {
  default:
    report_fatal_error("Unsupported calling convention");
  case CallingConv::C:
  case CallingConv::Fast:
    break;
  }

  MachineFunction &MF = DAG.getMachineFunction();

  const Function &Func = MF.getFunction();
  if (Func.hasFnAttribute("interrupt")) {
    if (!Func.arg_empty())
      report_fatal_error(
          "Functions with the interrupt attribute cannot have arguments!");

    StringRef Kind =
        MF.getFunction().getFnAttribute("interrupt").getValueAsString();

    if (!(Kind == "user" || Kind == "supervisor" || Kind == "machine"))
      report_fatal_error(
          "Function interrupt attribute argument not supported!");
  }

  EVT PtrVT = getPointerTy(DAG.getDataLayout());
  MVT XLenVT = Subtarget.getXLenVT();
  unsigned XLenInBytes = Subtarget.getXLen() / 8;
  // Used with vargs to acumulate store chains.
  std::vector<SDValue> OutChains;

  // Assign locations to all of the incoming arguments.
  SmallVector<CCValAssign, 16> ArgLocs;
  CCState CCInfo(CallConv, IsVarArg, MF, ArgLocs, *DAG.getContext());
  CCInfo.AnalyzeFormalArguments(Ins, CC_Leros);

  for (unsigned i = 0, e = ArgLocs.size(); i != e; ++i) {
    CCValAssign &VA = ArgLocs[i];
    SDValue ArgValue;
    if (VA.isRegLoc())
      ArgValue = unpackFromRegLoc(DAG, Chain, VA, DL);
    else
      ArgValue = unpackFromMemLoc(DAG, Chain, VA, DL);

    if (VA.getLocInfo() == CCValAssign::Indirect) {
      // If the original argument was split and passed by reference (e.g. i128
      // on RV32), we need to load all parts of it here (using the same
      // address).
      InVals.push_back(DAG.getLoad(VA.getValVT(), DL, Chain, ArgValue,
                                   MachinePointerInfo()));
      unsigned ArgIndex = Ins[i].OrigArgIndex;
      assert(Ins[i].PartOffset == 0);
      while (i + 1 != e && Ins[i + 1].OrigArgIndex == ArgIndex) {
        CCValAssign &PartVA = ArgLocs[i + 1];
        unsigned PartOffset = Ins[i + 1].PartOffset;
        SDValue Address = DAG.getNode(ISD::ADD, DL, PtrVT, ArgValue,
                                      DAG.getIntPtrConstant(PartOffset, DL));
        InVals.push_back(DAG.getLoad(PartVA.getValVT(), DL, Chain, Address,
                                     MachinePointerInfo()));
        ++i;
      }
      continue;
    }
    InVals.push_back(ArgValue);
  }

  if (IsVarArg) {
    ArrayRef<MCPhysReg> ArgRegs = makeArrayRef(ArgGPRs);
    unsigned Idx = CCInfo.getFirstUnallocated(ArgRegs);
    const TargetRegisterClass *RC = &Leros::GPRRegClass;
    MachineFrameInfo &MFI = MF.getFrameInfo();
    MachineRegisterInfo &RegInfo = MF.getRegInfo();
    LerosMachineFunctionInfo *LFI = MF.getInfo<LerosMachineFunctionInfo>();

    // Offset of the first variable argument from stack pointer, and size of
    // the vararg save area. For now, the varargs save area is either zero or
    // large enough to hold r4-r10.
    int VaArgOffset, VarArgsSaveSize;

    // If all registers are allocated, then all varargs must be passed on the
    // stack and we don't need to save any argregs.
    if (ArgRegs.size() == Idx) {
      VaArgOffset = CCInfo.getNextStackOffset();
      VarArgsSaveSize = 0;
    } else {
      VarArgsSaveSize = XLenInBytes * (ArgRegs.size() - Idx);
      VaArgOffset = -VarArgsSaveSize;
    }

    // Record the frame index of the first variable argument
    // which is a value necessary to VASTART.
    int FI = MFI.CreateFixedObject(XLenInBytes, VaArgOffset, true);
    LFI->setVarArgsFrameIndex(FI);

    // If saving an odd number of registers then create an extra stack slot to
    // ensure that the frame pointer is 2*XLEN-aligned, which in turn ensures
    // offsets to even-numbered registered remain 2*XLEN-aligned.
    if (Idx % 2) {
      FI = MFI.CreateFixedObject(XLenInBytes, VaArgOffset - (int)XLenInBytes,
                                 true);
      VarArgsSaveSize += XLenInBytes;
    }

    // Copy the integer registers that may have been used for passing varargs
    // to the vararg save area.
    for (unsigned I = Idx; I < ArgRegs.size();
         ++I, VaArgOffset += XLenInBytes) {
      const unsigned Reg = RegInfo.createVirtualRegister(RC);
      RegInfo.addLiveIn(ArgRegs[I], Reg);
      SDValue ArgValue = DAG.getCopyFromReg(Chain, DL, Reg, XLenVT);
      FI = MFI.CreateFixedObject(XLenInBytes, VaArgOffset, true);
      SDValue PtrOff = DAG.getFrameIndex(FI, getPointerTy(DAG.getDataLayout()));
      SDValue Store = DAG.getStore(Chain, DL, ArgValue, PtrOff,
                                   MachinePointerInfo::getFixedStack(MF, FI));
      cast<StoreSDNode>(Store.getNode())
          ->getMemOperand()
          ->setValue((Value *)nullptr);
      OutChains.push_back(Store);
    }
    LFI->setVarArgsSaveSize(VarArgsSaveSize);
  }

  // All stores are grouped in one node to allow the matching between
  // the size of Ins and InVals. This only happens for vararg functions.
  if (!OutChains.empty()) {
    OutChains.push_back(Chain);
    Chain = DAG.getNode(ISD::TokenFactor, DL, MVT::Other, OutChains);
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
    CallingConv::ID CallConv, MachineFunction &MF, bool IsVarArg,
    const SmallVectorImpl<ISD::OutputArg> &Outs, LLVMContext &Context) const {
  SmallVector<CCValAssign, 16> RVLocs;
  CCState CCInfo(CallConv, IsVarArg, MF, RVLocs, Context);
  for (unsigned i = 0, e = Outs.size(); i != e; ++i) {
    MVT VT = Outs[i].VT;
    ISD::ArgFlagsTy ArgFlags = Outs[i].Flags;
    if (CC_Leros(MF.getDataLayout(), i, VT, VT, CCValAssign::Full, ArgFlags,
                 CCInfo, /*IsFixed=*/true, /*IsRet=*/true, nullptr))
      return false;
  }
  return true;
}

SDValue
LerosTargetLowering::LowerReturn(SDValue Chain, CallingConv::ID CallConv,
                                 bool IsVarArg,
                                 const SmallVectorImpl<ISD::OutputArg> &Outs,
                                 const SmallVectorImpl<SDValue> &OutVals,
                                 const SDLoc &DL, SelectionDAG &DAG) const {

  // Stores the assignment of the return value to a location.
  SmallVector<CCValAssign, 16> RVLocs;

  // Info about the registers and stack slot.
  CCState CCInfo(CallConv, IsVarArg, DAG.getMachineFunction(), RVLocs,
                 *DAG.getContext());

  analyzeOutputArgs(DAG.getMachineFunction(), CCInfo, Outs, /*IsRet=*/true,
                    nullptr);

  SDValue Glue;
  SmallVector<SDValue, 4> RetOps(1, Chain);

  // Copy the result values into the output registers.
  for (unsigned i = 0, e = RVLocs.size(); i < e; ++i) {
    SDValue Val = OutVals[i];
    CCValAssign &VA = RVLocs[i];
    assert(VA.isRegLoc() && "Can only return in registers!");

    // Handle a 'normal' return.
    Val = convertValVTToLocVT(DAG, Val, VA, DL);
    Chain = DAG.getCopyToReg(Chain, DL, VA.getLocReg(), Val, Glue);

    // Guarantee that all emitted copies are stuck together.
    Glue = Chain.getValue(1);
    RetOps.push_back(DAG.getRegister(VA.getLocReg(), VA.getLocVT()));
  }

  RetOps[0] = Chain; // Update chain.

  // Add the glue node if we have it.
  if (Glue.getNode()) {
    RetOps.push_back(Glue);
  }

  return DAG.getNode(LEROSISD::Ret, DL, MVT::Other, RetOps);
}
} // namespace llvm
