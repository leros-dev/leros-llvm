//===-- LerosAsmBackend.cpp - Leros Assembler Backend ---------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/LerosFixupKinds.h"
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

  void applyFixup(const MCAssembler &Asm, const MCFixup &Fixup,
                  const MCValue &Target, MutableArrayRef<char> Data,
                  uint64_t Value, bool IsResolved,
                  const MCSubtargetInfo *STI) const override;

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

  unsigned getNumFixupKinds() const override {
    return Leros::NumTargetFixupKinds;
  }

  const MCFixupKindInfo &getFixupKindInfo(MCFixupKind Kind) const override {
    const static MCFixupKindInfo Infos[] = {
        // This table *must* be in the order that the fixup_* kinds are defined
        // in
        // LerosFixupKinds.h.
        //
        // name                      offset bits  flags
        {"fixup_leros_branch", 0, 12, MCFixupKindInfo::FKF_IsPCRel},
        {"fixup_leros_b0", 0, 8, 0},
        {"fixup_leros_b1", 0, 8, 0},
        {"fixup_leros_b2", 0, 8, 0},
        {"fixup_leros_b3", 0, 8, 0},
    };
    static_assert((array_lengthof(Infos)) == Leros::NumTargetFixupKinds,
                  "Not all fixup kinds added to Infos array");

    if (Kind < FirstTargetFixupKind)
      return MCAsmBackend::getFixupKindInfo(Kind);

    assert(unsigned(Kind - FirstTargetFixupKind) < getNumFixupKinds() &&
           "Invalid kind!");
    return Infos[Kind - FirstTargetFixupKind];
  }

  std::unique_ptr<MCObjectTargetWriter>
  createObjectTargetWriter() const override;
  bool writeNopData(raw_ostream &OS, uint64_t Count) const override;
};

static uint64_t adjustFixupValue(const MCFixup &Fixup, uint64_t Value,
                                 MCContext &Ctx) {
  unsigned Kind = Fixup.getKind();
  switch (Kind) {
  default:
    llvm_unreachable("Unknown fixup kind!");
  case FK_Data_1:
  case FK_Data_2:
  case FK_Data_4:
  case FK_Data_8:
    return Value;
  case Leros::fixup_leros_branch:
    if (!isInt<13>(Value))
      Ctx.reportError(Fixup.getLoc(), "fixup value out of range");
    if (Value & 0x1)
      Ctx.reportError(Fixup.getLoc(),
                      "branch fixup value must be 2-byte aligned");
    Value = (Value >> 1) & 0xfff;
    return Value;
  case Leros::fixup_leros_b0:
    return Value & 0xff;
  case Leros::fixup_leros_b1:
    return (Value >> 8) & 0xff;
  case Leros::fixup_leros_b2:
    return (Value >> 16) & 0xff;
  case Leros::fixup_leros_b3:
    return (Value >> 24) & 0xff;
  }
}

void LerosAsmBackend::applyFixup(const MCAssembler &Asm, const MCFixup &Fixup,
                                 const MCValue &Target,
                                 MutableArrayRef<char> Data, uint64_t Value,
                                 bool IsResolved,
                                 const MCSubtargetInfo *STI) const {
  MCContext &Ctx = Asm.getContext();
  MCFixupKindInfo Info = getFixupKindInfo(Fixup.getKind());
  if (!Value)
    return; // Doesn't change encoding.
  // Apply any target-specific value adjustments.
  Value = adjustFixupValue(Fixup, Value, Ctx);

  // Shift the value into position.
  Value <<= Info.TargetOffset;

  unsigned Offset = Fixup.getOffset();
  unsigned NumBytes = alignTo(Info.TargetSize + Info.TargetOffset, 8) / 8;

  assert(Offset + NumBytes <= Data.size() && "Invalid fixup offset!");

  // For each byte of the fragment that the fixup touches, mask in the
  // bits from the fixup value.
  for (unsigned i = 0; i != NumBytes; ++i) {
    Data[Offset + NumBytes - 1 - i] |= uint8_t((Value >> (i * 8)) & 0xff);
  }
}

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
