//===-- LerosELFObjectWriter.cpp - Leros ELF Writer -----------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/LerosFixupKinds.h"
#include "MCTargetDesc/LerosMCTargetDesc.h"
#include "memory"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCFixup.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;

namespace {
class LerosELFObjectWriter : public MCELFObjectTargetWriter {
public:
  LerosELFObjectWriter(uint8_t OSABI, bool Is64Bit);

  ~LerosELFObjectWriter() override;

  // Return true if the given relocation must be with a symbol rather than
  // section plus offset.
  bool needsRelocateWithSymbol(const MCSymbol &Sym,
                               unsigned Type) const override {

    return true;
  }

  unsigned getRelocType(MCContext &Ctx, const MCValue &Target,
                        const MCFixup &Fixup, bool IsPCRel) const override;
};
} // namespace

LerosELFObjectWriter::LerosELFObjectWriter(uint8_t OSABI, bool Is64Bit)
    : MCELFObjectTargetWriter(Is64Bit, OSABI, ELF::EM_LEROS,
                              /*HasRelocationAddend*/ true) {}

LerosELFObjectWriter::~LerosELFObjectWriter() {}

unsigned LerosELFObjectWriter::getRelocType(MCContext &Ctx,
                                            const MCValue &Target,
                                            const MCFixup &Fixup,
                                            bool IsPCRel) const {
  // Determine the type of the relocation
  switch ((unsigned)Fixup.getKind()) {
  default:
    llvm_unreachable("invalid fixup kind!");
  case FK_Data_4:
    return ELF::R_LEROS_32;
  case FK_Data_8:
    return ELF::R_LEROS_64;
  case Leros::fixup_leros_branch:
    return ELF::R_LEROS_BRANCH;
  case Leros::fixup_leros_b0:
    return ELF::R_LEROS_BYTE0;
  case Leros::fixup_leros_b1:
    return ELF::R_LEROS_BYTE1;
  case Leros::fixup_leros_b2:
    return ELF::R_LEROS_BYTE2;
  case Leros::fixup_leros_b3:
    return ELF::R_LEROS_BYTE3;
  }
}

std::unique_ptr<MCObjectTargetWriter>
llvm::createLerosELFObjectWriter(uint8_t OSABI, bool Is64Bit) {
  return llvm::make_unique<LerosELFObjectWriter>(OSABI, Is64Bit);
}
