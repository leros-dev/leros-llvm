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
    : LerosGenRegisterInfo(Leros::SP, /*DwarfFlavour*/ 0, /*EHFlavor*/ 0,
                           /*PC*/ 0, HwMode) {}

const MCPhysReg *
LerosRegisterInfo::getCalleeSavedRegs(const MachineFunction *MF) const {
  return CSR_SaveList;
}

BitVector LerosRegisterInfo::getReservedRegs(const MachineFunction &MF) const {
  BitVector Reserved(getNumRegs());

  // Use markSuperRegs to ensure any register aliases are also reserved
  markSuperRegs(Reserved, Leros::RA); // ra
  markSuperRegs(Reserved, Leros::SP); // sp
  markSuperRegs(Reserved, Leros::GP); // gp
  markSuperRegs(Reserved, Leros::FP); // fp
  assert(checkAllSuperRegsMarked(Reserved));
  return Reserved;
}

void LerosRegisterInfo::eliminateFrameIndex(MachineBasicBlock::iterator II,
                                            int SPAdj, unsigned FIOperandNum,
                                            RegScavenger *RS) const {
  assert(SPAdj == 0 && "Unexpected non-zero SPAdj value");

  MachineInstr &MI = *II;
  MachineFunction &MF = *MI.getParent()->getParent();
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

  assert(isInt<32>(Offset) && "Int32 expected");
  // Move the offset into a scratch register which will be used by
  // loadaddr
  unsigned ScratchReg =
      RS->getRegsAvailable(&Leros::GPRNoReserveRegClass).find_first();
  TII->movImm32(MBB, II, DL, ScratchReg, Offset);
  BuildMI(MBB, II, DL, TII->get(Leros::ADD_RR_PSEUDO), ScratchReg)
      .addReg(FrameReg)
      .addReg(ScratchReg, RegState::Kill);
  Offset = 0;
  bool ScratchRegIsKill = true;
  MI.getOperand(FIOperandNum)
      .ChangeToRegister(ScratchReg, false, false, ScratchRegIsKill);
  MI.getOperand(FIOperandNum + 1).ChangeToImmediate(Offset);
}

unsigned LerosRegisterInfo::getFrameRegister(const MachineFunction &MF) const {
  const TargetFrameLowering *TFI = getFrameLowering(MF);
  return TFI->hasFP(MF) ? Leros::FP : Leros::SP;
}

const uint32_t *
LerosRegisterInfo::getCallPreservedMask(const MachineFunction &MF,
                                        CallingConv::ID /*CC*/) const {
  return CSR_RegMask;
}
} // namespace llvm
