//===-- LerosAsmPrinter.cpp - Leros LLVM assembly writer ------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "InstPrinter/LerosInstPrinter.h"
#include "Leros.h"
#include "MCTargetDesc/LerosMCTargetDesc.h"
#include "llvm/Support/TargetRegistry.h"

#include "llvm/CodeGen/AsmPrinter.h"
#include "llvm/CodeGen/MachineInstrBundle.h"
#include "llvm/MC/MCStreamer.h"

namespace llvm {

namespace {
class LerosAsmPrinter : public AsmPrinter {

public:
  explicit LerosAsmPrinter(TargetMachine &TM,
                           std::unique_ptr<MCStreamer> Streamer)
      : AsmPrinter(TM, std::move(Streamer)) {}

  StringRef getPassName() const override { return "Leros Assembly Printer"; }

  virtual bool isBlockOnlyReachableByFallthrough(
      const MachineBasicBlock *MBB) const override;

  void EmitInstruction(const MachineInstr *MI) override;
};
} // end of anonymous namespace

void LerosAsmPrinter::EmitInstruction(const MachineInstr *MI) {
  MCInst TmpInst;
  LowerLerosMachineInstrToMCInst(MI, TmpInst, *this);
  EmitToStreamer(*OutStreamer, TmpInst);
}

bool LerosAsmPrinter::isBlockOnlyReachableByFallthrough(
    const MachineBasicBlock *MBB) const {
  // Since we emit tiny inlined loops for some operations, we need to check
  // whether the loop is self referencing. This is done by analyzing the
  // terminators of the BB, and whether they self-reference. This is equivalent
  // to what is done in AsmPrinter::isBlockOnlyReachableByFallthrough with the
  // difference that this triggers self-analysis of the MBB

  for (const auto &MI : MBB->terminators()) {
    for (ConstMIBundleOperands OP(MI); OP.isValid(); ++OP) {
      if (OP->isMBB() && OP->getMBB() == MBB)
        return false;
    }
  }

  return AsmPrinter::isBlockOnlyReachableByFallthrough(MBB);
}

// Force static initialization.
extern "C" void LLVMInitializeLerosAsmPrinter() {
  RegisterAsmPrinter<LerosAsmPrinter> X(getTheLeros32Target());
  RegisterAsmPrinter<LerosAsmPrinter> Y(getTheLeros64Target());
}
}
