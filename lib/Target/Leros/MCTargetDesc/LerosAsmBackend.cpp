//===-- LerosAsmBackend.cpp - Leros Assembler Backend ---------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/LerosMCTargetDesc.h"
#include "llvm/ADT/APInt.h"
#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCDirectives.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCFixupKindInfo.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

namespace {
class LerosAsmBackend : public MCAsmBackend {
  const MCSubtargetInfo &STI;
  uint8_t OSABI;
  bool Is64Bit;

public:
  LerosAsmBackend(const MCSubtargetInfo &STI, uint8_t OSABI, bool Is64Bit)
      : MCAsmBackend(support::little), STI(STI), OSABI(OSABI),
        Is64Bit(Is64Bit) {}
  ~LerosAsmBackend() override {}

  unsigned getNumFixupKinds() const override { return 0; }

  void applyFixup(const MCAssembler &Asm, const MCFixup &Fixup,
                  const MCValue &Target, MutableArrayRef<char> Data,
                  uint64_t Value, bool IsResolved,
                  const MCSubtargetInfo *STI) const override {
    return;
  }

  bool mayNeedRelaxation(const MCInst &Inst,
                         const MCSubtargetInfo &STI) const override {
    return false;
  }

  bool fixupNeedsRelaxation(const MCFixup &Fixup, uint64_t Value,
                            const MCRelaxableFragment *DF,
                            const MCAsmLayout &Layout) const override {
    return false;
  }

  void relaxInstruction(const MCInst &Inst, const MCSubtargetInfo &STI,
                        MCInst &Res) const override {
    return;
  }

  std::unique_ptr<MCObjectTargetWriter>
  createObjectTargetWriter() const override;
  bool writeNopData(raw_ostream &OS, uint64_t Count) const override;
};

bool LerosAsmBackend::writeNopData(raw_ostream &OS, uint64_t Count) const {
  unsigned nopLen = 2;

  if ((Count % nopLen) != 0)
    return false;

  uint64_t Nop16Count = Count / 4;
  for (uint64_t i = Nop16Count; i != 0; --i)
    OS.write("\0\0", 2);

  return true;
}

std::unique_ptr<MCObjectTargetWriter>
LerosAsmBackend::createObjectTargetWriter() const {
  return createLerosELFObjectWriter(OSABI, Is64Bit);
}

} // end anonymous namespace

MCAsmBackend *llvm::createLerosAsmBackend(const Target &T,
                                          const MCSubtargetInfo &STI,
                                          const MCRegisterInfo &MRI,
                                          const MCTargetOptions &Options) {
  const Triple &TT = STI.getTargetTriple();
  uint8_t OSABI = MCELFObjectTargetWriter::getOSABI(TT.getOS());
  return new LerosAsmBackend(STI, OSABI, TT.isArch64Bit());
}
