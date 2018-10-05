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
  LerosInstrInfo();
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

  void loadRegFromStackSlot(MachineBasicBlock &MBB,
                            MachineBasicBlock::iterator MBBI, unsigned DstReg,
                            int FrameIndex, const TargetRegisterClass *RC,
                            const TargetRegisterInfo *TRI) const override;

  bool expandPostRAPseudo(MachineInstr &MI) const override;

  void movImm32(MachineBasicBlock &MBB, MachineBasicBlock::iterator MBBI,
                const DebugLoc &DL, unsigned DstReg, uint64_t Val) const;

  unsigned insertBranch(MachineBasicBlock &MBB, MachineBasicBlock *TBB,
                        MachineBasicBlock *FBB, ArrayRef<MachineOperand> Cond,
                        const DebugLoc &dl,
                        int *BytesAdded = nullptr) const override;

  /// Determine if the branch target is in range.
  bool isBranchOffsetInRange(unsigned BranchOpc,
                             int64_t BrOffset) const override;

  unsigned getInstSizeInBytes(const MachineInstr &MI) const override;

  bool analyzeBranch(MachineBasicBlock &MBB, MachineBasicBlock *&TBB,
                     MachineBasicBlock *&FBB,
                     SmallVectorImpl<MachineOperand> &Cond,
                     bool AllowModify) const override;

private:
  void expandMOV(MachineBasicBlock &MBB, MachineBasicBlock::iterator I,
                 bool BBHasOperands, unsigned DstReg = 0,
                 unsigned SrcReg = 0) const;
  void expandRET(MachineBasicBlock &MBB, MachineBasicBlock::iterator I) const;
  void expandRRR(MachineBasicBlock &MBB, MachineInstr &MI) const;
  void expandRRI(MachineBasicBlock &MBB, MachineInstr &MI) const;
  void expandRI(MachineBasicBlock &MBB, MachineInstr &MI) const;
  void expandBRCC(MachineBasicBlock &MBB, MachineInstr &MI,
                  bool hasPrecalcCC) const;
  void expandBR(MachineBasicBlock &MBB, MachineInstr &MI) const;
  void expandLS(MachineBasicBlock &MBB, MachineInstr &MI) const;
  void expandCALL(MachineBasicBlock &MBB, MachineInstr &MI) const;
};
} // namespace llvm
#endif
