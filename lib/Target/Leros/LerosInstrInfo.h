//===-- LerosInstrInfo.h - Leros Instruction Information --------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the Leros implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_Leros_LerosINSTRINFO_H
#define LLVM_LIB_TARGET_Leros_LerosINSTRINFO_H

#include "LerosRegisterInfo.h"
#include "llvm/CodeGen/TargetInstrInfo.h"

#define GET_INSTRINFO_HEADER
#include "LerosGenInstrInfo.inc"

namespace llvm {

class LerosInstrInfo : public LerosGenInstrInfo {

public:
  LerosInstrInfo() {}
  /*
    unsigned isLoadFromStackSlot(const MachineInstr &MI,
                                 int &FrameIndex) const override;
    unsigned isStoreToStackSlot(const MachineInstr &MI,
                                int &FrameIndex) const override;
  */
  void copyPhysReg(MachineBasicBlock &MBB, MachineBasicBlock::iterator MBBI,
                   const DebugLoc &DL, unsigned DstReg, unsigned SrcReg,
                   bool KillSrc) const override;

  void storeRegToStackSlot(MachineBasicBlock &MBB,
                           MachineBasicBlock::iterator MBBI, unsigned SrcReg,
                           bool IsKill, int FrameIndex,
                           const TargetRegisterClass *RC,
                           const TargetRegisterInfo *TRI) const override;
  /*
      void loadRegFromStackSlot(MachineBasicBlock &MBB,
                                MachineBasicBlock::iterator MBBI, unsigned
     DstReg,
                                int FrameIndex, const TargetRegisterClass *RC,
                                const TargetRegisterInfo *TRI) const override;
    */

  bool expandPostRAPseudo(MachineInstr &MI) const override;

private:
  void expandMOV(MachineBasicBlock &MBB, MachineBasicBlock::iterator I) const;
  void expandRET(MachineBasicBlock &MBB, MachineBasicBlock::iterator I) const;
  void expandRR(MachineBasicBlock &MBB, MachineInstr &MI) const;
};
}
#endif
