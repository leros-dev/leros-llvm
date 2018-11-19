//===-- LerosBaseInfo.h - Top level definitions for Leros MC ----*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains small standalone enum definitions for the Leros target
// useful for the compiler back-end and the MC libraries.
//
//===----------------------------------------------------------------------===//
#ifndef LLVM_LIB_TARGET_Leros_MCTARGETDESC_LerosBASEINFO_H
#define LLVM_LIB_TARGET_Leros_MCTARGETDESC_LerosBASEINFO_H

#include "llvm/CodeGen/SelectionDAG.h"
#include "llvm/CodeGen/TargetLowering.h"

namespace llvm {

namespace LEROSISD {
enum {
  FIRST_NUMBER = ISD::BUILTIN_OP_END,
  Ret,  // Return pseudo-op
  Call, // Call SD Node
  LOAD,
  LOADH,  // loads an 8-bit immediate into the high byte of a 2-byte value
  LOADH2, // - || -                            3rd byte of a 4-byte value
  LOADH3, // - || -                            4th byte of a 4-byte value
  Mov,    // Move pseudo-op
  SELECT_CC
};
}

namespace LEROSIF {
/**
Instruction formats for the various pseudo instructions that creates the
3-operand wrapper ISA for the Leros instruction set.
@warning MUST be kept in sync with the format definitions defined in
LerosInstrFormats.td
*/
enum {
  NoFormat,
  RRR,
  RRI,
  BranchCmp,
  RI,
  BranchIndirect,
  LoadStore,
  Signed8BitLoad,
  Signed16BitLoad,
  Unsigned8BitLoad,
  Unsigned16BitLoad,
  ByteStore,
  BranchRs
};
} // namespace LEROSIF

// Leros expression target flags
namespace LEROSTF {
enum { MO_None, MO_B0, MO_B1, MO_B2, MO_B3 };
}

} // namespace llvm

#endif
