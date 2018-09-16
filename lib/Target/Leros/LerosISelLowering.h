//===-- LerosISelLowering.h - Leros DAG Lowering Interface ------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines the interfaces that Leros uses to lower LLVM code into a
// selection DAG.
//
//===----------------------------------------------------------------------===//

#ifndef LerosISELLOWERING_H
#define LerosISELLOWERING_H

#include "Leros.h"
#include "llvm/CodeGen/SelectionDAG.h"
#include "llvm/CodeGen/TargetLowering.h"

namespace llvm {

namespace LEROSISD {
enum NodeType {
  FIRST_NUMBER = ISD::BUILTIN_OP_END,
  Ret,    // Return pseudo-op
  LOADH,  // loads an 8-bit immediate into the high byte of a 2-byte value
  LOADH2, // - || -                            3rd byte of a 4-byte value
  LOADH3, // - || -                            4th byte of a 4-byte value
  Mov     // Move pseudo-op
};
}

namespace LEROSIF {
enum { NoFormat = 0, RegisterRegister = 1, RegisterImmediate = 2, Branch = 3 };
}

// Forward delcarations
class LerosSubtarget;
class LerosTargetMachine;

//===--------------------------------------------------------------------===//
// TargetLowering Implementation
//===--------------------------------------------------------------------===//
class LerosTargetLowering : public TargetLowering {
public:
  explicit LerosTargetLowering(const TargetMachine &TM,
                               const LerosSubtarget &STI);

  /// LowerOperation - Provide custom lowering hooks for some operations.
  virtual SDValue LowerOperation(SDValue Op, SelectionDAG &DAG) const override;

  // This method returns the name of a target specific DAG node.
  virtual const char *getTargetNodeName(unsigned Opcode) const override;

  bool isLegalICmpImmediate(int64_t Imm) const override;
  bool isLegalAddImmediate(int64_t Imm) const override;

private:
  const LerosSubtarget &Subtarget;

  SDValue LowerFormalArguments(SDValue Chain, CallingConv::ID CallConv,
                               bool IsVarArg,
                               const SmallVectorImpl<ISD::InputArg> &Ins,
                               const SDLoc &DL, SelectionDAG &DAG,
                               SmallVectorImpl<SDValue> &InVals) const override;

  SDValue LowerCall(TargetLowering::CallLoweringInfo &CLI,
                    SmallVectorImpl<SDValue> &InVals) const override;

  SDValue LowerReturn(SDValue Chain, CallingConv::ID CallConv, bool IsVarArg,
                      const SmallVectorImpl<ISD::OutputArg> &Outs,
                      const SmallVectorImpl<SDValue> &OutVals, const SDLoc &DL,
                      SelectionDAG &DAG) const override;

  SDValue LowerCallResult(SDValue Chain, SDValue InGlue,
                          CallingConv::ID CallConv, bool isVarArg,
                          const SmallVectorImpl<ISD::InputArg> &Ins, SDLoc dl,
                          SelectionDAG &DAG,
                          SmallVectorImpl<SDValue> &InVals) const;

  bool CanLowerReturn(CallingConv::ID CallConv, MachineFunction &MF,
                      bool IsVarArg,
                      const SmallVectorImpl<ISD::OutputArg> &Outs,
                      LLVMContext &Context) const override;
};
}

#endif // LerosISELLOWERING_H
