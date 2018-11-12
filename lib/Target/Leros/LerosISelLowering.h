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
#include "LerosBaseInfo.h"
#include "llvm/CodeGen/SelectionDAG.h"
#include "llvm/CodeGen/TargetLowering.h"

namespace llvm {

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

  MachineBasicBlock *
  EmitInstrWithCustomInserter(MachineInstr &MI,
                              MachineBasicBlock *BB) const override;

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

  SDValue lowerGlobalAddress(SDValue Op, SelectionDAG &DAG) const;
  SDValue lowerBlockAddress(SDValue Op, SelectionDAG &DAG) const;
  SDValue lowerConstantPool(SDValue Op, SelectionDAG &DAG) const;
  SDValue lowerSELECT(SDValue Op, SelectionDAG &DAG) const;
  SDValue lowerVASTART(SDValue Op, SelectionDAG &DAG) const;
  SDValue lowerFRAMEADDR(SDValue Op, SelectionDAG &DAG) const;
  SDValue lowerRETURNADDR(SDValue Op, SelectionDAG &DAG) const;

  bool CanLowerReturn(CallingConv::ID CallConv, MachineFunction &MF,
                      bool IsVarArg,
                      const SmallVectorImpl<ISD::OutputArg> &Outs,
                      LLVMContext &Context) const override;

  void analyzeInputArgs(MachineFunction &MF, CCState &CCInfo,
                        const SmallVectorImpl<ISD::InputArg> &Ins,
                        bool IsRet) const;
  void analyzeOutputArgs(MachineFunction &MF, CCState &CCInfo,
                         const SmallVectorImpl<ISD::OutputArg> &Outs,
                         bool IsRet, CallLoweringInfo *CLI) const;

  MachineBasicBlock *EmitSHL(MachineInstr &MI, MachineBasicBlock *BB) const;
  MachineBasicBlock *EmitTruncatedStore(MachineInstr &MI,
                                        MachineBasicBlock *BB) const;
  MachineBasicBlock *EmitSRAI(MachineInstr &MI, MachineBasicBlock *BB) const;
  MachineBasicBlock *EmitSEXTLOAD(MachineInstr &MI,
                                  MachineBasicBlock *BB) const;
  MachineBasicBlock *EmitSRAR(MachineInstr &MI, MachineBasicBlock *BB) const;
  MachineBasicBlock *EmitSRL(MachineInstr &MI, MachineBasicBlock *BB) const;
  MachineBasicBlock *EmitSET(MachineInstr &MI, MachineBasicBlock *BB) const;
  MachineBasicBlock *EmitSELECT(MachineInstr &MI, MachineBasicBlock *BB) const;
};
} // namespace llvm

#endif // LerosISELLOWERING_H
