//===-- LerosUseAccumulatorInsts.cpp - Expand pseudo instructions
//-----------===//
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

#include <algorithm>
#include <iterator>
#include <utility>

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
  bool removeRedundantLoadMBB(MachineBasicBlock &MBB);
  bool removeRedundantStoreMBB(MachineBasicBlock &MBB);

  bool removeRedundantLoad(MachineBasicBlock &MBB,
                           MachineBasicBlock::iterator MBBI,
                           MachineBasicBlock::iterator &NextMBBI);
};

char LerosUseAccumulator::ID = 0;

bool LerosUseAccumulator::runOnMachineFunction(MachineFunction &MF) {
  TII = static_cast<const LerosInstrInfo *>(MF.getSubtarget().getInstrInfo());
  bool Modified = false;
  for (auto &MBB : MF) {
    Modified |= removeRedundantLoadMBB(MBB);
    Modified |= removeRedundantStoreMBB(MBB);
  }
  return Modified;
}

bool LerosUseAccumulator::removeRedundantStoreMBB(MachineBasicBlock &MBB) {
  bool Modified = false;
  int index = 0;

  std::map<unsigned, std::pair<int, MachineInstr *>> lastStores;
  std::map<unsigned, std::pair<int, MachineInstr *>> lastUsages;

  MachineBasicBlock::reverse_iterator MBBI = MBB.rbegin(), E = MBB.rend();
  while (MBBI != E) {
    bool eraseMBBI = false;
    if (MBBI->getOpcode() == Leros::STORE_R) {
      const unsigned &reg = MBBI->getOperand(0).getReg();
      if (lastStores.find(reg) == lastStores.end()) {
        // First (last) store to the reg is always valid
        lastStores[reg] = std::make_pair(index, &*MBBI);
      } else {
        // Determine if register is used between the two stores
        if (lastUsages.find(reg) == lastUsages.end() ||
            lastUsages[reg].first < lastStores[reg].first) {
          // No usage of register between the two stores - we can thus remove
          // the current store instruction, and maintain the last store
          // instruction
          eraseMBBI = true;
        } else {
          // Register is used between the two store operatiosn
          // update the most recent store to the reg
          lastStores[reg] = std::make_pair(index, &*MBBI);
        }
      }
    } else {
      if (MBBI->getOperand(0).isReg()) {
        // Update last usage of reg
        const unsigned &reg = MBBI->getOperand(0).getReg();
        lastUsages[reg] = std::make_pair(index, &*MBBI);
      }
    }

    // Iterate to next block
    MachineBasicBlock::reverse_iterator NMBBI = std::next(MBBI);
    if (eraseMBBI) {
      MBBI->eraseFromParent();
      Modified |= true;
    }
    MBBI = NMBBI;
    index++;
  }

  return Modified;
} // namespace

bool LerosUseAccumulator::removeRedundantLoadMBB(MachineBasicBlock &MBB) {
  bool Modified = false;

  MachineBasicBlock::iterator MBBI = MBB.begin(), E = MBB.end();
  while (MBBI != E) {
    MachineBasicBlock::iterator NMBBI = std::next(MBBI);
    Modified |= removeRedundantLoad(MBB, MBBI, NMBBI);
    MBBI = NMBBI;
  }

  return Modified;
}

bool LerosUseAccumulator::removeRedundantLoad(
    MachineBasicBlock &MBB, MachineBasicBlock::iterator MBBI,
    MachineBasicBlock::iterator &NextMBBI) {
  MachineInstr &MI = *MBBI;
  DebugLoc DL = MI.getDebugLoc();

  if (MBBI->getOpcode() == Leros::STORE_R &&
      NextMBBI->getOpcode() == Leros::LOAD_R) {
    // Check operands
    const unsigned &rs1 = MBBI->getOperand(0).getReg();
    const unsigned &rs2 = NextMBBI->getOperand(0).getReg();

    if (rs1 == rs2) {
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

FunctionPass *createLerosUseAccumulatorPass() {
  return new LerosUseAccumulator();
}

} // end of namespace llvm
