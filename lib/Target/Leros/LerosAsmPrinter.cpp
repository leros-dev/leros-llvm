//===-- LerosAsmPrinter.cpp - Leros LLVM assembly writer ------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "LerosAsmPrinter.h"
#include "MCTargetDesc/LerosMCTargetDesc.h"
#include "llvm/Support/TargetRegistry.h"

namespace llvm {

void LerosAsmPrinter::EmitInstruction(const MachineInstr *MI) {
  MCInst tmpInst;
  LowerLerosMachineInstrToMCInst(MI, tmpInst, *this);
  AsmPrinter::EmitToStreamer(*OutStreamer, tmpInst);
}

bool LerosAsmPrinter::PrintAsmOperand(const MachineInstr *MI, unsigned OpNo,
                                      unsigned AsmVariant,
                                      const char *ExtraCode, raw_ostream &OS) {
  if (AsmVariant != 0)
    report_fatal_error("There are no defined alternate asm variants");

  // First try the generic code, which knows about modifiers like 'c' and 'n'.
  if (!AsmPrinter::PrintAsmOperand(MI, OpNo, AsmVariant, ExtraCode, OS))
    return false;

  return true;
}

bool LerosAsmPrinter::PrintAsmMemoryOperand(const MachineInstr *MI,
                                            unsigned OpNo, unsigned AsmVariant,
                                            const char *ExtraCode,
                                            raw_ostream &OS) {
  if (AsmVariant != 0)
    report_fatal_error("There are no defined alternate asm variants");

  return AsmPrinter::PrintAsmMemoryOperand(MI, OpNo, AsmVariant, ExtraCode, OS);
}

// Force static initialization.
extern "C" void LLVMInitializeLerosAsmPrinter() {
  RegisterAsmPrinter<LerosAsmPrinter> X(getTheLeros32Target());
  RegisterAsmPrinter<LerosAsmPrinter> Y(getTheLeros64Target());
}
}
