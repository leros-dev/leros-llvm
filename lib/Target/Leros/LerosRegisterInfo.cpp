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

#define DEBUG_TYPE "leros-reg-info"

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
                                            RegScavenger *RS) const {
  assert(SPAdj == 0 && "Unexpected non-zero SPAdj value");

  MachineInstr &MI = *II;
  MachineFunction &MF = *MI.getParent()->getParent();
  MachineRegisterInfo &MRI = MF.getRegInfo();
  const LerosInstrInfo *TII = MF.getSubtarget<LerosSubtarget>().getInstrInfo();
  DebugLoc DL = MI.getDebugLoc();

  int FrameIndex = MI.getOperand(FIOperandNum).getIndex();
  unsigned FrameReg;
  int Offset =
      getFrameLowering(MF)->getFrameIndexReference(MF, FrameIndex, FrameReg) +
      MI.getOperand(FIOperandNum + 1).getImm();

  if (!isInt<32>(Offset)) {
    report_fatal_error(
        "Frame offsets outside of the signed 32-bit range not supported");
  }

  MachineBasicBlock &MBB = *MI.getParent();
  bool FrameRegIsKill = false;

  if (!isInt<8>(Offset)) {
    assert(isInt<32>(Offset) && "Int32 expected");
    // The offset won't fit in an immediate, so use a scratch register instead
    // Modify Offset and FrameReg appropriately
    unsigned ScratchReg = MRI.createVirtualRegister(&Leros::GPRRegClass);
    TII->movImm32(MBB, II, DL, ScratchReg, Offset);
    BuildMI(MBB, II, DL, TII->get(Leros::ADD_RR_PSEUDO), ScratchReg)
        .addReg(FrameReg)
        .addReg(ScratchReg, RegState::Kill);
    Offset = 0;
    FrameReg = ScratchReg;
    FrameRegIsKill = true;
  }

  MI.getOperand(FIOperandNum)
      .ChangeToRegister(FrameReg, false, false, FrameRegIsKill);
  MI.getOperand(FIOperandNum + 1).ChangeToImmediate(Offset);
}

unsigned LerosRegisterInfo::getFrameRegister(const MachineFunction &MF) const {
  const TargetFrameLowering *TFI = getFrameLowering(MF);
  return TFI->hasFP(MF) ? Leros::R3 : Leros::R1;
}

const uint32_t *LerosRegisterInfo::getNoPreservedMask() const {
  return CSR_NoRegs_RegMask;
}

const uint32_t *
LerosRegisterInfo::getCallPreservedMask(const MachineFunction &MF,
                                        CallingConv::ID /*CC*/) const {
  return CSR_RegMask;
}
} // namespace llvm
