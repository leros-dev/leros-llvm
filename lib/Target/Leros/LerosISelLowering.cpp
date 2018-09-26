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

namespace llvm {

LerosTargetLowering::LerosTargetLowering(const TargetMachine &TM,
                                         const LerosSubtarget &STI)
    : TargetLowering(TM), Subtarget(STI) {

  // Set up the register classes.
  MVT XLenVT = Subtarget.getXLenVT();
  addRegisterClass(XLenVT, &Leros::GPRRegClass);

  setStackPointerRegisterToSaveRestore(Leros::SPRegClass.getRegister(0));

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

  setOperationAction(ISD::VASTART, MVT::Other, Custom);
  setOperationAction(ISD::VAARG, MVT::Other, Expand);
  setOperationAction(ISD::VACOPY, MVT::Other, Expand);
  setOperationAction(ISD::VAEND, MVT::Other, Expand);

  for (auto VT : {MVT::i1, MVT::i8, MVT::i16})
    setOperationAction(ISD::SIGN_EXTEND_INREG, VT, Expand);

  setOperationAction(ISD::MUL, XLenVT, LibCall);
  setOperationAction(ISD::MULHS, XLenVT, LibCall);
  setOperationAction(ISD::MULHU, XLenVT, LibCall);
  setOperationAction(ISD::SDIV, XLenVT, LibCall);
  setOperationAction(ISD::UDIV, XLenVT, LibCall);
  setOperationAction(ISD::SREM, XLenVT, LibCall);
  setOperationAction(ISD::UREM, XLenVT, LibCall);

  setOperationAction(ISD::SDIVREM, XLenVT, LibCall);
  setOperationAction(ISD::UDIVREM, XLenVT, LibCall);

  setOperationAction(ISD::SHL_PARTS, XLenVT, LibCall);
  setOperationAction(ISD::SRL_PARTS, XLenVT, Legal);
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

  setOperationAction(ISD::FMINNUM, MVT::f32, LibCall);
  setOperationAction(ISD::FMAXNUM, MVT::f32, LibCall);
  for (auto CC : FPCCToExtend)
    setCondCodeAction(CC, MVT::f32, Expand);
  setOperationAction(ISD::SELECT_CC, MVT::f32, LibCall);
  setOperationAction(ISD::SELECT, MVT::f32, LibCall);
  setOperationAction(ISD::BR_CC, MVT::f32, LibCall);

  setOperationAction(ISD::FMINNUM, MVT::f64, LibCall);
  setOperationAction(ISD::FMAXNUM, MVT::f64, LibCall);
  for (auto CC : FPCCToExtend)
    setCondCodeAction(CC, MVT::f64, LibCall);
  setOperationAction(ISD::SELECT_CC, MVT::f64, LibCall);
  setOperationAction(ISD::SELECT, MVT::f64, LibCall);
  setOperationAction(ISD::BR_CC, MVT::f64, LibCall);
  setLoadExtAction(ISD::EXTLOAD, MVT::f64, MVT::f32, LibCall);
  setTruncStoreAction(MVT::f64, MVT::f32, Expand);

  setOperationAction(ISD::GlobalAddress, XLenVT, Custom);
  setOperationAction(ISD::BlockAddress, XLenVT, Custom);
  setOperationAction(ISD::ConstantPool, XLenVT, Custom);

  setBooleanContents(ZeroOrOneBooleanContent);
}

SDValue LerosTargetLowering::lowerGlobalAddress(SDValue Op,
                                                SelectionDAG &DAG) const {
  SDLoc DL(Op);
  EVT Ty = Op.getValueType();
  GlobalAddressSDNode *N = cast<GlobalAddressSDNode>(Op);
  const GlobalValue *GV = N->getGlobal();
  int64_t Offset = N->getOffset();
  MVT XLenVT = Subtarget.getXLenVT();

  if (isPositionIndependent())
    report_fatal_error("Unable to lowerGlobalAddress");

  SDValue GA = DAG.getTargetGlobalAddress(GV, DL, Ty, 0);
  SDValue MN =
      SDValue(DAG.getMachineNode(Leros::LOAD_RI_PSEUDO, DL, Ty, GA), 0);
  if (Offset != 0)
    return DAG.getNode(ISD::ADD, DL, Ty, MN,
                       DAG.getConstant(Offset, DL, XLenVT));
  return MN;
}

SDValue LerosTargetLowering::lowerConstantPool(SDValue Op,
                                               SelectionDAG &DAG) const {
  llvm_unreachable("LowerConstantPool unimplemented");
  return SDValue();
}

SDValue LerosTargetLowering::lowerBlockAddress(SDValue Op,
                                               SelectionDAG &DAG) const {
  llvm_unreachable("lowerBlockAddress unimplemented");
  return SDValue();
}

SDValue LerosTargetLowering::LowerOperation(SDValue Op,
                                            SelectionDAG &DAG) const {

  switch (Op.getOpcode()) {
  case ISD::ConstantPool:
    return lowerConstantPool(Op, DAG);
  case ISD::GlobalAddress:
    return lowerGlobalAddress(Op, DAG);
  case ISD::BlockAddress:
    return lowerBlockAddress(Op, DAG);
  }
  llvm_unreachable("Unhandled lowerOperation opcode");
  return SDValue();
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
  SDLoc &Loc = CLI.DL;
  SmallVectorImpl<ISD::OutputArg> &Outs = CLI.Outs;
  SmallVectorImpl<SDValue> &OutVals = CLI.OutVals;
  SmallVectorImpl<ISD::InputArg> &Ins = CLI.Ins;
  const TargetFrameLowering *TFL = Subtarget.getFrameLowering();
  SDValue Chain = CLI.Chain;
  SDValue Callee = CLI.Callee;
  CallingConv::ID CallConv = CLI.CallConv;
  const bool isVarArg = CLI.IsVarArg;

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
  unsigned NextStackOffset = CCInfo.getNextStackOffset();
  NextStackOffset = alignTo(NextStackOffset, TFL->getStackAlignment());
  SDValue NextStackOffsetVal =
      DAG.getIntPtrConstant(NextStackOffset, Loc, true);

  Chain = DAG.getCALLSEQ_START(Chain, NextStackOffset, 0, Loc);

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

    SDValue StackPtr =
        DAG.getRegister(Leros::SPRegClass.getRegister(0), MVT::i32);
    SDValue PtrOff = DAG.getIntPtrConstant(VA.getLocMemOffset(), Loc);
    PtrOff = DAG.getNode(ISD::ADD, Loc, MVT::i32, StackPtr, PtrOff);

    MemOpChains.push_back(
        DAG.getStore(Chain, Loc, Arg, PtrOff, MachinePointerInfo()));
  }

  // Emit all stores, make sure they occur before the call.
  if (!MemOpChains.empty()) {
    Chain = DAG.getNode(ISD::TokenFactor, Loc, MVT::Other, MemOpChains);
  }

  // Build a sequence of copy-to-reg nodes chained together with token chain
  // and flag operands which copy the outgoing args into the appropriate regs.
  SDValue InFlag;
  for (auto &Reg : RegsToPass) {
    Chain = DAG.getCopyToReg(Chain, Loc, Reg.first, Reg.second, InFlag);
    InFlag = Chain.getValue(1);
  }

  // We only support calling global addresses.
  GlobalAddressSDNode *G = dyn_cast<GlobalAddressSDNode>(Callee);
  assert(G && "We only support the calling of global addresses");

  EVT PtrVT = getPointerTy(DAG.getDataLayout());
  Callee = DAG.getGlobalAddress(G->getGlobal(), Loc, PtrVT, 0);

  // If the callee is a GlobalAddress/ExternalSymbol node, turn it into a
  // TargetGlobalAddress/TargetExternalSymbol node so that legalize won't
  // split it and then direct call can be matched by PseudoCALL.
  if (GlobalAddressSDNode *S = dyn_cast<GlobalAddressSDNode>(Callee)) {
    Callee = DAG.getTargetGlobalAddress(S->getGlobal(), Loc, PtrVT, 0, 0);
  } else if (ExternalSymbolSDNode *S = dyn_cast<ExternalSymbolSDNode>(Callee)) {
    Callee = DAG.getTargetExternalSymbol(S->getSymbol(), PtrVT, 0);
  }

  std::vector<SDValue> Ops;
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
  Chain = DAG.getNode(LEROSISD::Call, Loc, NodeTys, Ops);
  InFlag = Chain.getValue(1);

  Chain = DAG.getCALLSEQ_END(Chain, NextStackOffsetVal,
                             DAG.getIntPtrConstant(0, Loc, true), InFlag, Loc);
  if (!Ins.empty()) {
    InFlag = Chain.getValue(1);
  }

  // Handle result values, copying them out of physregs into vregs that we
  // return.
  return LowerCallResult(Chain, InFlag, CallConv, isVarArg, Ins, Loc, DAG,
                         InVals);
}

SDValue LerosTargetLowering::LowerCallResult(
    SDValue Chain, SDValue InGlue, CallingConv::ID CallConv, bool isVarArg,
    const SmallVectorImpl<ISD::InputArg> &Ins, SDLoc dl, SelectionDAG &DAG,
    SmallVectorImpl<SDValue> &InVals) const {
  if (isVarArg) {
    report_fatal_error("VarArg not supported");
  }
  // Assign locations to each value returned by this call.
  SmallVector<CCValAssign, 16> RVLocs;
  CCState CCInfo(CallConv, isVarArg, DAG.getMachineFunction(), RVLocs,
                 *DAG.getContext());

  CCInfo.AnalyzeCallResult(Ins, RetCC_Leros);

  // Copy all of the result registers out of their specified physreg.
  for (auto &Loc : RVLocs) {
    Chain =
        DAG.getCopyFromReg(Chain, dl, Loc.getLocReg(), Loc.getValVT(), InGlue)
            .getValue(1);
    InGlue = Chain.getValue(2);
    InVals.push_back(Chain.getValue(0));
  }

  return Chain;
}

//===----------------------------------------------------------------------===//
//             Formal Arguments Calling Convention Implementation
//===----------------------------------------------------------------------===//

/// Leros formal arguments implementation
SDValue LerosTargetLowering::LowerFormalArguments(
    SDValue Chain, CallingConv::ID CallConv, bool IsVarArg,
    const SmallVectorImpl<ISD::InputArg> &Ins, const SDLoc &DL,
    SelectionDAG &DAG, SmallVectorImpl<SDValue> &InVals) const {
  MachineFunction &MF = DAG.getMachineFunction();
  MachineRegisterInfo &RegInfo = MF.getRegInfo();

  if (IsVarArg) {
    report_fatal_error("VarArg not supported");
  }
  // Assign locations to all of the incoming arguments.
  SmallVector<CCValAssign, 16> ArgLocs;
  CCState CCInfo(CallConv, IsVarArg, DAG.getMachineFunction(), ArgLocs,
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
      SDValue ArgIn = DAG.getCopyFromReg(Chain, DL, VReg, RegVT);

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
    SDValue Load =
        DAG.getLoad(VA.getValVT(), DL, Chain, FIPtr, MachinePointerInfo());

    InVals.push_back(Load);
  }

  return Chain;
}

//===----------------------------------------------------------------------===//
//               Return Value Calling Convention Implementation
//===----------------------------------------------------------------------===//
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

bool LerosTargetLowering::isLegalICmpImmediate(int64_t Imm) const {
  return isInt<8>(Imm);
}

bool LerosTargetLowering::isLegalAddImmediate(int64_t Imm) const {
  return isInt<8>(Imm);
}

SDValue
LerosTargetLowering::LowerReturn(SDValue Chain, CallingConv::ID CallConv,
                                 bool isVarArg,
                                 const SmallVectorImpl<ISD::OutputArg> &Outs,
                                 const SmallVectorImpl<SDValue> &OutVals,
                                 const SDLoc &DL, SelectionDAG &DAG) const {
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

    Chain = DAG.getCopyToReg(Chain, DL, VA.getLocReg(), OutVals[i], Flag);

    Flag = Chain.getValue(1);
    RetOps.push_back(DAG.getRegister(VA.getLocReg(), VA.getLocVT()));
  }

  RetOps[0] = Chain; // Update chain.

  // Add the flag if we have it.
  if (Flag.getNode()) {
    RetOps.push_back(Flag);
  }

  // use jal as return function, with register as operand
  auto node = DAG.getNode(LEROSISD::Ret, DL, MVT::Other, RetOps);
  return node;
}
} // namespace llvm
