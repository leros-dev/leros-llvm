//===-- LerosUseAccumulatorInsts.cpp - Expand pseudo instructions -----------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains a pass that expands pseudo instructions into target
// instructions. This pass should be run after register allocation but before
// the post-regalloc scheduling pass.
//
//===----------------------------------------------------------------------===//

#include "Leros.h"
#include "LerosInstrInfo.h"
#include "LerosTargetMachine.h"

#include "llvm/CodeGen/LivePhysRegs.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"

using namespace llvm;

#define LEROS_USE_ACCUMULATOR_NAME "Leros accumulator usage pass"

namespace {

class LerosUseAccumulator : public MachineFunctionPass {
public:
  const LerosInstrInfo *TII;
  static char ID;

  LerosUseAccumulator() : MachineFunctionPass(ID) {
    initializeLerosUseAccumulatorPass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &MF) override;

  StringRef getPassName() const override { return LEROS_USE_ACCUMULATOR_NAME; }

private:
  bool analyzeMBB(MachineBasicBlock &MBB);

  bool removeStoreLoad(MachineBasicBlock &MBB, MachineBasicBlock::iterator MBBI,
                MachineBasicBlock::iterator &NextMBBI);
};

char LerosUseAccumulator::ID = 0;

bool LerosUseAccumulator::runOnMachineFunction(MachineFunction &MF) {
  TII = static_cast<const LerosInstrInfo *>(MF.getSubtarget().getInstrInfo());
  bool Modified = false;
  for (auto &MBB : MF)
    Modified |= analyzeMBB(MBB);
  return Modified;
}

bool LerosUseAccumulator::analyzeMBB(MachineBasicBlock &MBB) {
  bool Modified = false;

  MachineBasicBlock::iterator MBBI = MBB.begin(), E = MBB.end();
  while (MBBI != E) {
    MachineBasicBlock::iterator NMBBI = std::next(MBBI);
    Modified |= removeStoreLoad(MBB, MBBI, NMBBI);
    MBBI = NMBBI;
  }

  return Modified;
}

bool LerosUseAccumulator::removeStoreLoad(MachineBasicBlock &MBB,
                                 MachineBasicBlock::iterator MBBI,
                                 MachineBasicBlock::iterator &NextMBBI) {
    MachineInstr &MI = *MBBI;
    DebugLoc DL = MI.getDebugLoc();

    if(MBBI->getOpcode() == Leros::STORE_R && NextMBBI->getOpcode() == Leros::LOAD_R){
        // Check operands
        const unsigned& rs1 = MBBI->getOperand(0).getReg();
        const unsigned& rs2 =NextMBBI->getOperand(0).getReg();

        if(rs1 == rs2){
            // redundant load detected, remove the instruction
            MachineInstr &MI = *NextMBBI;
            NextMBBI = std::next(NextMBBI);
            MI.eraseFromParent();
            return true;
        }
    }
    return false;
}

} // end of anonymous namespace

INITIALIZE_PASS(LerosUseAccumulator, "Leros-use-accumulator",
                LEROS_USE_ACCUMULATOR_NAME, false, false)
namespace llvm {

FunctionPass *createLerosUseAccumulatorPass() { return new LerosUseAccumulator(); }

} // end of namespace llvm
