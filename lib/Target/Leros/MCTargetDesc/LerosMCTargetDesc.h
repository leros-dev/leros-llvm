//===- LerosMCTargetDesc.h - Leros Target Descriptions --------------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file provides Leros specific target descriptions.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_Leros_MCTARGETDESC_LerosMCTARGETDESC_H
#define LLVM_LIB_TARGET_Leros_MCTARGETDESC_LerosMCTARGETDESC_H

#include "llvm/Support/DataTypes.h"

namespace llvm {

class Target;

Target &getTheLeros16Target();
Target &getTheLeros32Target();

} // end namespace llvm

// Defines symbolic names for Leros registers.  This defines a mapping from
// register name to register number.
#define GET_REGINFO_ENUM
#include "LerosGenRegisterInfo.inc"

// Defines symbolic names for the Leros instructions.
#define GET_INSTRINFO_ENUM
#include "LerosGenInstrInfo.inc"

#define GET_SUBTARGETINFO_ENUM
#include "LerosGenSubtargetInfo.inc"

#endif // LLVM_LIB_TARGET_Leros_MCTARGETDESC_LerosMCTARGETDESC_H
