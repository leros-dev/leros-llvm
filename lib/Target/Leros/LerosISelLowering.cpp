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

  setOperationAction(ISD::GlobalAddress, XLenVT, Expand);
  setOperationAction(ISD::BlockAddress, XLenVT, Expand);
  setOperationAction(ISD::ConstantPool, XLenVT, Expand);

  setBooleanContents(ZeroOrOneBooleanContent);
}

SDValue LerosTargetLowering::LowerOperation(SDValue Op,
                                            SelectionDAG &DAG) const {
  report_fatal_error("We do not expect any unhandleable ops yet");
  return SDValue();
}

const char *LerosTargetLowering::getTargetNodeName(unsigned Opcode) const {
#define NODE(name)                                                             \
  case LEROSISD::name:                                                         \
    return #name
  switch (Opcode) {
  default:
    return nullptr;
    NODE(Ret);
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
  std::cout << "Leros: Implement me!" << std::endl;
  return SDValue();
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
}
