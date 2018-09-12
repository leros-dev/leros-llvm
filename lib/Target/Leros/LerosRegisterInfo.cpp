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
#include "LerosSubtarget.h"
#include "MCTargetDesc/LerosMCTargetDesc.h"
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
    : LerosGenRegisterInfo(Leros::R1, /*DwarfFlavour*/ 0, /*EHFlavor*/ 0,
                           /*PC*/ 0, HwMode) {}

const MCPhysReg *
LerosRegisterInfo::getCalleeSavedRegs(const MachineFunction *MF) const {
  return CSR_SaveList;
}

BitVector LerosRegisterInfo::getReservedRegs(const MachineFunction &MF) const {
  BitVector Reserved(getNumRegs());

  // Use markSuperRegs to ensure any register aliases are also reserved
  markSuperRegs(Reserved, Leros::R0); // ra
  markSuperRegs(Reserved, Leros::R1); // sp
  markSuperRegs(Reserved, Leros::R2); // gp
  markSuperRegs(Reserved, Leros::R3); // fp
  assert(checkAllSuperRegsMarked(Reserved));
  return Reserved;
}

void LerosRegisterInfo::eliminateFrameIndex(MachineBasicBlock::iterator II,
                                            int SPAdj, unsigned FIOperandNum,
                                            RegScavenger *RS) const {}

unsigned LerosRegisterInfo::getFrameRegister(const MachineFunction &MF) const {
  return Leros::R2;
}

const uint32_t *
LerosRegisterInfo::getCallPreservedMask(const MachineFunction &MF,
                                        CallingConv::ID /*CC*/) const {
  return CSR_RegMask;
}
}
