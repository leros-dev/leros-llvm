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

// Expand PseudoCALL and PseudoTAIL to AUIPC and JALR with relocation types.
// We expand PseudoCALL and PseudoTAIL while encoding, meaning AUIPC and JALR
// won't go through Leros MC to MC compressed instruction transformation. This
// is acceptable because AUIPC has no 16-bit form and C_JALR have no immediate
// operand field.  We let linker relaxation deal with it. When linker
// relaxation enabled, AUIPC and JALR have chance relax to JAL. If C extension
// is enabled, JAL has chance relax to C_JAL.
void LerosMCCodeEmitter::expandFunctionCall(const MCInst &MI, raw_ostream &OS,
                                            SmallVectorImpl<MCFixup> &Fixups,
                                            const MCSubtargetInfo &STI) const {
  MCInst TmpInst;
  MCOperand Func = MI.getOperand(0);
  unsigned Ra = Leros::RA;
  uint32_t Binary;

  assert(Func.isExpr() && "Expected expression");

  const MCExpr *Expr = Func.getExpr();

  // Create function call expression CallExpr for AUIPC.
  const MCExpr *CallExpr =
      LerosMCExpr::create(Expr, LerosMCExpr::VK_Leros_CALL, Ctx);

  // Emit AUIPC Ra, Func with R_Leros_CALL relocation type.
  TmpInst = MCInstBuilder(Leros::AUIPC)
                .addReg(Ra)
                .addOperand(MCOperand::createExpr(CallExpr));
  Binary = getBinaryCodeForInstr(TmpInst, Fixups, STI);
  support::endian::write(OS, Binary, support::little);

  if (MI.getOpcode() == Leros::PseudoTAIL)
    // Emit JALR X0, X6, 0
    TmpInst = MCInstBuilder(Leros::JALR).addReg(Leros::X0).addReg(Ra).addImm(0);
  else
    // Emit JALR X1, X1, 0
    TmpInst = MCInstBuilder(Leros::JALR).addReg(Ra).addReg(Ra).addImm(0);
  Binary = getBinaryCodeForInstr(TmpInst, Fixups, STI);
  support::endian::write(OS, Binary, support::little);
}

void LerosMCCodeEmitter::encodeInstruction(const MCInst &MI, raw_ostream &OS,
                                           SmallVectorImpl<MCFixup> &Fixups,
                                           const MCSubtargetInfo &STI) const {
  const MCInstrDesc &Desc = MCII.get(MI.getOpcode());
  // Get byte count of instruction.
  unsigned Size = Desc.getSize();

  if (MI.getOpcode() == Leros::PseudoCALL ||
      MI.getOpcode() == Leros::PseudoTAIL) {
    expandFunctionCall(MI, OS, Fixups, STI);
    MCNumEmitted += 2;
    return;
  }

  switch (Size) {
  default:
    llvm_unreachable("Unhandled encodeInstruction length!");
  case 2: {
    uint16_t Bits = getBinaryCodeForInstr(MI, Fixups, STI);
    support::endian::write<uint16_t>(OS, Bits, support::little);
    break;
  }
  case 4: {
    uint32_t Bits = getBinaryCodeForInstr(MI, Fixups, STI);
    support::endian::write(OS, Bits, support::little);
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
  bool EnableRelax = STI.getFeatureBits()[Leros::FeatureRelax];
  const MCOperand &MO = MI.getOperand(OpNo);

  MCInstrDesc const &Desc = MCII.get(MI.getOpcode());
  unsigned MIFrm = Desc.TSFlags & LerosII::InstFormatMask;

  // If the destination is an immediate, there is nothing to do.
  if (MO.isImm())
    return MO.getImm();

  assert(MO.isExpr() && "getImmOpValue expects only expressions or immediates");
  const MCExpr *Expr = MO.getExpr();
  MCExpr::ExprKind Kind = Expr->getKind();
  Leros::Fixups FixupKind = Leros::fixup_Leros_invalid;
  if (Kind == MCExpr::Target) {
    const LerosMCExpr *RVExpr = cast<LerosMCExpr>(Expr);

    switch (RVExpr->getKind()) {
    case LerosMCExpr::VK_Leros_None:
    case LerosMCExpr::VK_Leros_Invalid:
      llvm_unreachable("Unhandled fixup kind!");
    case LerosMCExpr::VK_Leros_LO:
      if (MIFrm == LerosII::InstFormatI)
        FixupKind = Leros::fixup_Leros_lo12_i;
      else if (MIFrm == LerosII::InstFormatS)
        FixupKind = Leros::fixup_Leros_lo12_s;
      else
        llvm_unreachable("VK_Leros_LO used with unexpected instruction format");
      break;
    case LerosMCExpr::VK_Leros_HI:
      FixupKind = Leros::fixup_Leros_hi20;
      break;
    case LerosMCExpr::VK_Leros_PCREL_LO:
      if (MIFrm == LerosII::InstFormatI)
        FixupKind = Leros::fixup_Leros_pcrel_lo12_i;
      else if (MIFrm == LerosII::InstFormatS)
        FixupKind = Leros::fixup_Leros_pcrel_lo12_s;
      else
        llvm_unreachable(
            "VK_Leros_PCREL_LO used with unexpected instruction format");
      break;
    case LerosMCExpr::VK_Leros_PCREL_HI:
      FixupKind = Leros::fixup_Leros_pcrel_hi20;
      break;
    case LerosMCExpr::VK_Leros_CALL:
      FixupKind = Leros::fixup_Leros_call;
      break;
    }
  } else if (Kind == MCExpr::SymbolRef &&
             cast<MCSymbolRefExpr>(Expr)->getKind() ==
                 MCSymbolRefExpr::VK_None) {
    if (Desc.getOpcode() == Leros::JAL) {
      FixupKind = Leros::fixup_Leros_jal;
    } else if (MIFrm == LerosII::InstFormatB) {
      FixupKind = Leros::fixup_Leros_branch;
    } else if (MIFrm == LerosII::InstFormatCJ) {
      FixupKind = Leros::fixup_Leros_rvc_jump;
    } else if (MIFrm == LerosII::InstFormatCB) {
      FixupKind = Leros::fixup_Leros_rvc_branch;
    }
  }

  assert(FixupKind != Leros::fixup_Leros_invalid && "Unhandled expression!");

  Fixups.push_back(
      MCFixup::create(0, Expr, MCFixupKind(FixupKind), MI.getLoc()));
  ++MCNumFixups;

  if (EnableRelax) {
    if (FixupKind == Leros::fixup_Leros_call) {
      Fixups.push_back(MCFixup::create(
          0, Expr, MCFixupKind(Leros::fixup_Leros_relax), MI.getLoc()));
      ++MCNumFixups;
    }
  }

  return 0;
}

#include "LerosGenMCCodeEmitter.inc"
