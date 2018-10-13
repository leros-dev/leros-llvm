//===-- LerosMCExpr.cpp - Leros specific MC expression classes ------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the implementation of the assembly expression modifiers
// accepted by the Leros architecture (e.g. ":lo12:", ":gottprel_g1:", ...).
//
//===----------------------------------------------------------------------===//

#include "LerosMCExpr.h"
#include "Leros.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSymbolELF.h"
#include "llvm/MC/MCValue.h"
#include "llvm/Object/ELF.h"
#include "llvm/Support/ErrorHandling.h"

#define DEBUG_TYPE "Lerosmcexpr"

namespace llvm {

const LerosMCExpr *LerosMCExpr::create(const MCExpr *Expr, VariantKind Kind,
                                       MCContext &Ctx) {
  return new (Ctx) LerosMCExpr(Expr, Kind);
}

void LerosMCExpr::printImpl(raw_ostream &OS, const MCAsmInfo *MAI) const {
  Expr->print(OS, MAI);
}

bool LerosMCExpr::evaluateAsRelocatableImpl(MCValue &Res,
                                            const MCAsmLayout *Layout,
                                            const MCFixup *Fixup) const {
  return false;
}

void LerosMCExpr::visitUsedExpr(MCStreamer &Streamer) const {
  Streamer.visitUsedExpr(*getSubExpr());
}

LerosMCExpr::VariantKind LerosMCExpr::getVariantKindForName(StringRef name) {
  return StringSwitch<LerosMCExpr::VariantKind>(name)
      .Case("b0", VK_Leros_B0)
      .Case("b1", VK_Leros_B1)
      .Case("b2", VK_Leros_B2)
      .Case("b3", VK_Leros_B3)
      .Default(VK_Leros_Invalid);
}

StringRef LerosMCExpr::getVariantKindName(VariantKind Kind) {
  switch (Kind) {
  default:
    llvm_unreachable("Invalid ELF symbol kind");
  case VK_Leros_B0:
    return "b0";
  case VK_Leros_B1:
    return "b1";
  case VK_Leros_B2:
    return "b2";
  case VK_Leros_B3:
    return "b3";
  }
}

bool LerosMCExpr::evaluateAsConstant(int64_t &Res) const {
  MCValue Value;

  if (!getSubExpr()->evaluateAsRelocatable(Value, nullptr, nullptr))
    return false;

  if (!Value.isAbsolute())
    return false;

  Res = evaluateAsInt64(Value.getConstant());
  return true;
}

int64_t LerosMCExpr::evaluateAsInt64(int64_t Value) const {
  switch (Kind) {
  default:
    llvm_unreachable("Invalid kind");
  case VK_Leros_B0:
    return SignExtend64<8>(Value);
  case VK_Leros_B1:
    return SignExtend64<8>(Value >> 8);
  case VK_Leros_B2:
    return SignExtend64<8>(Value >> 16);
  case VK_Leros_B3:
    return SignExtend64<8>(Value >> 24);
  }
}
}
