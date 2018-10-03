//===-- LerosMCInstLower.cpp - Convert Leros MachineInstr to an MCInst ------=//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains code to lower Leros MachineInstrs to their corresponding
// MCInst records.
//
//===----------------------------------------------------------------------===//

#include "Leros.h"
#include "MCTargetDesc/LerosMCExpr.h"
#include "llvm/CodeGen/AsmPrinter.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"

namespace llvm {

static MCOperand lowerSymbolOperand(const MachineOperand &MO, MCSymbol *Sym,
                                    const AsmPrinter &AP) {
  MCContext &Ctx = AP.OutContext;
  LerosMCExpr::VariantKind Kind;

  auto tf = MO.getTargetFlags();

  switch (tf) {
  default:
    llvm_unreachable("Unknown target flag on GV operand");
  case LEROSTF::MO_None:
    Kind = LerosMCExpr::VK_Leros_None;
  case LEROSTF::MO_B0:
    Kind = LerosMCExpr::VK_Leros_B0;
    break;
  case LEROSTF::MO_B1:
    Kind = LerosMCExpr::VK_Leros_B1;
    break;
  case LEROSTF::MO_B2:
    Kind = LerosMCExpr::VK_Leros_B2;
    break;
  case LEROSTF::MO_B3:
    Kind = LerosMCExpr::VK_Leros_B3;
    break;
  }

  const MCExpr *ME =
      MCSymbolRefExpr::create(Sym, MCSymbolRefExpr::VK_None, Ctx);

  if (!MO.isJTI() && !MO.isMBB() && MO.getOffset())
    ME = MCBinaryExpr::createAdd(
        ME, MCConstantExpr::create(MO.getOffset(), Ctx), Ctx);
  return MCOperand::createExpr(ME);
}

static MCOperand createLerosImm(int64_t Val) {
  // Create Leros immediates are always to be signed
  return MCOperand::createImm(SignExtend64(Val, 8));
}

bool LowerLerosMachineOperandToMCOperand(const MachineOperand &MO,
                                         MCOperand &MCOp,
                                         const AsmPrinter &AP) {
  MachineOperand::MachineOperandType MOTy = MO.getType();

  switch (MOTy) {
  default:
    llvm_unreachable("unknown operand type");
  case MachineOperand::MO_Register:
    // Ignore all implicit register operands.
    if (MO.isImplicit()) {
      break;
    }
    MCOp = MCOperand::createReg(MO.getReg());
    break;
  case MachineOperand::MO_RegisterMask:
    // Regmasks are like implicit defs.
    return false;
  case MachineOperand::MO_Immediate:
    MCOp = createLerosImm(MO.getImm());
    break;
  case MachineOperand::MO_MachineBasicBlock:
    MCOp = lowerSymbolOperand(MO, MO.getMBB()->getSymbol(), AP);
    break;
  case MachineOperand::MO_GlobalAddress:
    MCOp = lowerSymbolOperand(MO, AP.getSymbol(MO.getGlobal()), AP);
    break;
  case MachineOperand::MO_BlockAddress:
    MCOp = lowerSymbolOperand(
        MO, AP.GetBlockAddressSymbol(MO.getBlockAddress()), AP);
    break;
  case MachineOperand::MO_ExternalSymbol:
    MCOp = lowerSymbolOperand(
        MO, AP.GetExternalSymbolSymbol(MO.getSymbolName()), AP);
    break;
  case MachineOperand::MO_ConstantPoolIndex:
    MCOp = lowerSymbolOperand(MO, AP.GetCPISymbol(MO.getIndex()), AP);
    break;
  }

  return true;
}

void LowerLerosMachineInstrToMCInst(const MachineInstr *MI, MCInst &OutMI,
                                    const AsmPrinter &AP) {
  OutMI.setOpcode(MI->getOpcode());

  volatile auto a = MI->getOpcode();

  for (const MachineOperand &MO : MI->operands()) {
    MCOperand MCOp;
    if (LowerLerosMachineOperandToMCOperand(MO, MCOp, AP))
      OutMI.addOperand(MCOp);
  }
}
} // namespace llvm
