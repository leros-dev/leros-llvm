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

#include "MCTargetDesc/LerosMCTargetDesc.h"
#include "llvm/CodeGen/SelectionDAG.h"
#include "llvm/CodeGen/TargetLowering.h"

namespace llvm {

namespace LEROSCREG {
// Constant registers as defined by the Leros ABI
enum {
  ZERO = Leros::R100,    // 0x0
  ONE = Leros::R101,     // 0x1
  B8SIGN = Leros::R102,  // 0x80
  B16SIGN = Leros::R103, // 0x8000
  B32SIGN = Leros::R104, // 0x80000000
  LBMASK = Leros::R105,  // 0xFF
  LHMASK = Leros::R106,  // 0xFFFF
  UHMASK = Leros::R107   // 0xFFFF0000
};

const std::map<uint64_t, int> values = {
    {0x0, LEROSCREG::ZERO},           {0x1, LEROSCREG::ONE},
    {0x80, LEROSCREG::B8SIGN},        {0x8000, LEROSCREG::B16SIGN},
    {0x80000000, LEROSCREG::B32SIGN}, {0xFF, LEROSCREG::LBMASK},
    {0xFFFF, LEROSCREG::LHMASK},      {0xFFFF0000, LEROSCREG::UHMASK},
};

} // namespace LEROSCREG

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
enum { NoFormat, RRR, RRI, BranchCmp, RI, LoadStore, BranchRs };
} // namespace LEROSIF

// Leros expression target flags
namespace LEROSTF {
enum { MO_None, MO_B0, MO_B1, MO_B2, MO_B3 };
}

} // namespace llvm

#endif
