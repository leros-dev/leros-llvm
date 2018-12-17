//===-- LerosRegisterInfo.h - Leros Register Information Impl ---*- C++ -*-===//
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

#ifndef LLVM_LIB_TARGET_Leros_LerosREGISTERINFO_H
#define LLVM_LIB_TARGET_Leros_LerosREGISTERINFO_H

#include "llvm/CodeGen/TargetRegisterInfo.h"

#define GET_REGINFO_HEADER
#include "LerosGenRegisterInfo.inc"

namespace llvm {

struct LerosRegisterInfo : public LerosGenRegisterInfo {

  LerosRegisterInfo(unsigned HwMode);

  const uint32_t *getCallPreservedMask(const MachineFunction &MF,
                                       CallingConv::ID) const override;

  const MCPhysReg *getCalleeSavedRegs(const MachineFunction *MF) const override;

  const uint32_t *getNoPreservedMask() const override;

  bool isConstantPhysReg(unsigned PhysReg) const override;

  BitVector getReservedRegs(const MachineFunction &MF) const override;

  void eliminateFrameIndex(MachineBasicBlock::iterator MI, int SPAdj,
                           unsigned FIOperandNum,
                           RegScavenger *RS = nullptr) const override;

  unsigned getFrameRegister(const MachineFunction &MF) const override;

  bool requiresRegisterScavenging(const MachineFunction &MF) const override {
    return true;
  }

  // We may eliminate the frame index during PrologEpilogInserter, which is post
  // register alocation, and as such we need to enable frame index scavenging,
  // such that the virtual register defined in eliminateFrameIndex is handled
  bool requiresFrameIndexScavenging(const MachineFunction &MF) const override {
    return true;
  }
};
} // namespace llvm

#endif
