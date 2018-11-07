//===-- LerosMCCodeEmitter.cpp - Convert Leros code to machine code -------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the LerosMCCodeEmitter class.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/LerosFixupKinds.h"
#include "MCTargetDesc/LerosMCExpr.h"
#include "MCTargetDesc/LerosMCTargetDesc.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCCodeEmitter.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstBuilder.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/EndianStream.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

#define DEBUG_TYPE "mccodeemitter"

STATISTIC(MCNumEmitted, "Number of MC instructions emitted");
STATISTIC(MCNumFixups, "Number of MC fixups created");

namespace {
class LerosMCCodeEmitter : public MCCodeEmitter {
  LerosMCCodeEmitter(const LerosMCCodeEmitter &) = delete;
  void operator=(const LerosMCCodeEmitter &) = delete;
  MCContext &Ctx;
  MCInstrInfo const &MCII;

public:
  LerosMCCodeEmitter(MCContext &ctx, MCInstrInfo const &MCII)
      : Ctx(ctx), MCII(MCII) {}

  ~LerosMCCodeEmitter() override {}

  void encodeInstruction(const MCInst &MI, raw_ostream &OS,
                         SmallVectorImpl<MCFixup> &Fixups,
                         const MCSubtargetInfo &STI) const override;

  void expandFunctionCall(const MCInst &MI, raw_ostream &OS,
                          SmallVectorImpl<MCFixup> &Fixups,
                          const MCSubtargetInfo &STI) const;

  /// TableGen'erated function for getting the binary encoding for an
  /// instruction.
  uint64_t getBinaryCodeForInstr(const MCInst &MI,
                                 SmallVectorImpl<MCFixup> &Fixups,
                                 const MCSubtargetInfo &STI) const;

  /// Return binary encoding of operand. If the machine operand requires
  /// relocation, record the relocation and return zero.
  unsigned getMachineOpValue(const MCInst &MI, const MCOperand &MO,
                             SmallVectorImpl<MCFixup> &Fixups,
                             const MCSubtargetInfo &STI) const;

  unsigned getImmOpValueAsr1(const MCInst &MI, unsigned OpNo,
                             SmallVectorImpl<MCFixup> &Fixups,
                             const MCSubtargetInfo &STI) const;

  unsigned getImmOpValue(const MCInst &MI, unsigned OpNo,
                         SmallVectorImpl<MCFixup> &Fixups,
                         const MCSubtargetInfo &STI) const;
};
} // end anonymous namespace

MCCodeEmitter *llvm::createLerosMCCodeEmitter(const MCInstrInfo &MCII,
                                              const MCRegisterInfo &MRI,
                                              MCContext &Ctx) {
  return new LerosMCCodeEmitter(Ctx, MCII);
}

void LerosMCCodeEmitter::encodeInstruction(const MCInst &MI, raw_ostream &OS,
                                           SmallVectorImpl<MCFixup> &Fixups,
                                           const MCSubtargetInfo &STI) const {
  const MCInstrDesc &Desc = MCII.get(MI.getOpcode());
  // Get byte count of instruction.
  unsigned Size = Desc.getSize();

  switch (Size) {
  default:
    llvm_unreachable("Unhandled encodeInstruction length!");
  case 2: {
    uint16_t Bits = getBinaryCodeForInstr(MI, Fixups, STI);
    support::endian::write<uint16_t>(OS, Bits, support::little);
    break;
  }
  }

  ++MCNumEmitted; // Keep track of the # of mi's emitted.
}

unsigned
LerosMCCodeEmitter::getMachineOpValue(const MCInst &MI, const MCOperand &MO,
                                      SmallVectorImpl<MCFixup> &Fixups,
                                      const MCSubtargetInfo &STI) const {

  if (MO.isReg())
    return Ctx.getRegisterInfo()->getEncodingValue(MO.getReg());

  if (MO.isImm())
    return static_cast<unsigned>(MO.getImm());

  llvm_unreachable("Unhandled expression!");
  return 0;
}

unsigned
LerosMCCodeEmitter::getImmOpValueAsr1(const MCInst &MI, unsigned OpNo,
                                      SmallVectorImpl<MCFixup> &Fixups,
                                      const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  if (MO.isImm()) {
    unsigned Res = MO.getImm();
    assert((Res & 1) == 0 && "LSB is non-zero");
    return Res >> 1;
  }

  return getImmOpValue(MI, OpNo, Fixups, STI);
}

unsigned LerosMCCodeEmitter::getImmOpValue(const MCInst &MI, unsigned OpNo,
                                           SmallVectorImpl<MCFixup> &Fixups,
                                           const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  // If the destination is an immediate, there is nothing to do.
  if (MO.isImm())
    return MO.getImm();

  assert(MO.isExpr() && "getImmOpValue expects only expressions or immediates");
  const MCExpr *Expr = MO.getExpr();
  MCExpr::ExprKind Kind = Expr->getKind();

  Leros::Fixups FixupKind = Leros::fixup_leros_invalid;
  if (Kind == MCExpr::Target) {
    const LerosMCExpr *LerosExpr = cast<LerosMCExpr>(Expr);

    switch (LerosExpr->getKind()) {
    case LerosMCExpr::VK_Leros_None:
    case LerosMCExpr::VK_Leros_Invalid:
      llvm_unreachable("Unhandled fixup kind!");
    case LerosMCExpr::VK_Leros_B0:
      FixupKind = Leros::fixup_leros_b0;
      break;
    case LerosMCExpr::VK_Leros_B1:
      FixupKind = Leros::fixup_leros_b1;
      break;
    case LerosMCExpr::VK_Leros_B2:
      FixupKind = Leros::fixup_leros_b2;
      break;
    case LerosMCExpr::VK_Leros_B3:
      FixupKind = Leros::fixup_leros_b3;
      break;
    }
  } else if (Kind == MCExpr::SymbolRef &&
             cast<MCSymbolRefExpr>(Expr)->getKind() ==
                 MCSymbolRefExpr::VK_None) {
    // We might get a symbol reference that has been parsed through assembly, so
    // check for load instructions that use symbols
    switch (MI.getOpcode()) {
    case Leros::LOAD_I: {
      FixupKind = Leros::fixup_leros_b0;
      break;
    }
    case Leros::LOADH_AI: {
      FixupKind = Leros::fixup_leros_b1;
      break;
    }
    case Leros::LOADH2_AI: {
      FixupKind = Leros::fixup_leros_b2;
      break;
    }
    case Leros::LOADH3_AI: {
      FixupKind = Leros::fixup_leros_b3;
      break;
    }
    default: {
      // Default to a branch fixup
      FixupKind = Leros::fixup_leros_branch;
      break;
    }
    }
  }

  assert(FixupKind != Leros::fixup_leros_invalid && "Unhandled expression!");

  Fixups.push_back(
      MCFixup::create(0, Expr, MCFixupKind(FixupKind), MI.getLoc()));
  ++MCNumFixups;

  return 0;
}

#include "LerosGenMCCodeEmitter.inc"
