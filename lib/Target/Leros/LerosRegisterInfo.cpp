//===-- LerosRegisterInfo.cpp - Leros Register Information ------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the Leros implementation of the TargetRegisterInfo class.
//
//===----------------------------------------------------------------------===//
#include "Leros.h"

#include "LerosRegisterInfo.h"
#include "LerosSubtarget.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/RegisterScavenging.h"
#include "llvm/CodeGen/TargetFrameLowering.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/Support/ErrorHandling.h"

#define GET_REGINFO_TARGET_DESC
#include "LerosGenRegisterInfo.inc"

namespace llvm {

LerosRegisterInfo::LerosRegisterInfo(unsigned HwMode)
    : LerosGenRegisterInfo(Leros::R0, /*DwarfFlavour*/ 0, /*EHFlavor*/ 0,
                           /*PC*/ 0, HwMode) {}
}
