//===-- Leros.h - Top-level interface for Leros -----------------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the entry points for global functions defined in the LLVM
// Leros back-end.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_Leros_Leros_H
#define LLVM_LIB_TARGET_Leros_Leros_H

#include "LerosBaseInfo.h"

namespace llvm {
class FunctionPass;
class MachineInstr;
class MCInst;
class AsmPrinter;
class MachineOperand;
class MCOperand;
class LerosTargetMachine;

void LowerLerosMachineInstrToMCInst(const MachineInstr *MI, MCInst &OutMI,
                                    const AsmPrinter &AP);
bool LowerLerosMachineOperandToMCOperand(const MachineOperand &MO,
                                         MCOperand &MCOp, const AsmPrinter &AP);

FunctionPass *createLerosISelDag(LerosTargetMachine &TM);

FunctionPass *createLerosUseAccumulatorPass();
void initializeLerosUseAccumulatorPass(PassRegistry &);
}

#endif
