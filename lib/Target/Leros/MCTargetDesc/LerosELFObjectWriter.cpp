//===-- LerosELFObjectWriter.cpp - Leros ELF Writer -----------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

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
    // TODO: this is very conservative, update once RISC-V psABI requirements
    //       are clarified.
    return true;
  }

  unsigned getRelocType(MCContext &Ctx, const MCValue &Target,
                        const MCFixup &Fixup, bool IsPCRel) const override {
    return false;
  }
};
}

LerosELFObjectWriter::LerosELFObjectWriter(uint8_t OSABI, bool Is64Bit)
    : MCELFObjectTargetWriter(Is64Bit, OSABI, ELF::EM_NONE,
                              /*HasRelocationAddend*/ true) {}

LerosELFObjectWriter::~LerosELFObjectWriter() {}

std::unique_ptr<MCObjectTargetWriter>
llvm::createLerosELFObjectWriter(uint8_t OSABI, bool Is64Bit) {
  return llvm::make_unique<LerosELFObjectWriter>(OSABI, Is64Bit);
}
