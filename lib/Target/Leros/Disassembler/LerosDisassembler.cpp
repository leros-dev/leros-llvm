//===-- LerosDisassembler.cpp - Disassembler for Leros --------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the LerosDisassembler class.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/LerosMCTargetDesc.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCDisassembler/MCDisassembler.h"
#include "llvm/MC/MCFixedLenDisassembler.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/Endian.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

#define DEBUG_TYPE "Leros-disassembler"

typedef MCDisassembler::DecodeStatus DecodeStatus;

namespace {
class LerosDisassembler : public MCDisassembler {

public:
  LerosDisassembler(const MCSubtargetInfo &STI, MCContext &Ctx)
      : MCDisassembler(STI, Ctx) {}

  DecodeStatus getInstruction(MCInst &Instr, uint64_t &Size,
                              ArrayRef<uint8_t> Bytes, uint64_t Address,
                              raw_ostream &VStream,
                              raw_ostream &CStream) const override;
};
} // end anonymous namespace

static MCDisassembler *createLerosDisassembler(const Target &T,
                                               const MCSubtargetInfo &STI,
                                               MCContext &Ctx) {
  return new LerosDisassembler(STI, Ctx);
}

extern "C" void LLVMInitializeLerosDisassembler() {
  // Register the disassembler for each target.
  TargetRegistry::RegisterMCDisassembler(getTheLeros32Target(),
                                         createLerosDisassembler);
  TargetRegistry::RegisterMCDisassembler(getTheLeros64Target(),
                                         createLerosDisassembler);
}

static const unsigned GPRDecoderTable[] = {
    Leros::R0,   Leros::R1,   Leros::R2,   Leros::R3,   Leros::R4,
    Leros::R5,   Leros::R6,   Leros::R7,   Leros::R8,   Leros::R9,
    Leros::R10,  Leros::R11,  Leros::R12,  Leros::R13,  Leros::R14,
    Leros::R15,  Leros::R16,  Leros::R17,  Leros::R18,  Leros::R19,
    Leros::R20,  Leros::R21,  Leros::R22,  Leros::R23,  Leros::R24,
    Leros::R25,  Leros::R26,  Leros::R27,  Leros::R28,  Leros::R29,
    Leros::R30,  Leros::R31,  Leros::R32,  Leros::R33,  Leros::R34,
    Leros::R35,  Leros::R36,  Leros::R37,  Leros::R38,  Leros::R39,
    Leros::R40,  Leros::R41,  Leros::R42,  Leros::R43,  Leros::R44,
    Leros::R45,  Leros::R46,  Leros::R47,  Leros::R48,  Leros::R49,
    Leros::R50,  Leros::R51,  Leros::R52,  Leros::R53,  Leros::R54,
    Leros::R55,  Leros::R56,  Leros::R57,  Leros::R58,  Leros::R59,
    Leros::R60,  Leros::R61,  Leros::R62,  Leros::R63,  Leros::R64,
    Leros::R65,  Leros::R66,  Leros::R67,  Leros::R68,  Leros::R69,
    Leros::R70,  Leros::R71,  Leros::R72,  Leros::R73,  Leros::R74,
    Leros::R75,  Leros::R76,  Leros::R77,  Leros::R78,  Leros::R79,
    Leros::R80,  Leros::R81,  Leros::R82,  Leros::R83,  Leros::R84,
    Leros::R85,  Leros::R86,  Leros::R87,  Leros::R88,  Leros::R89,
    Leros::R90,  Leros::R91,  Leros::R92,  Leros::R93,  Leros::R94,
    Leros::R95,  Leros::R96,  Leros::R97,  Leros::R98,  Leros::R99,
    Leros::R100, Leros::R101, Leros::R102, Leros::R103, Leros::R104,
    Leros::R105, Leros::R106, Leros::R107, Leros::R108, Leros::R109,
    Leros::R110, Leros::R111, Leros::R112, Leros::R113, Leros::R114,
    Leros::R115, Leros::R116, Leros::R117, Leros::R118, Leros::R119,
    Leros::R120, Leros::R121, Leros::R122, Leros::R123, Leros::R124,
    Leros::R125, Leros::R126, Leros::R127, Leros::R128, Leros::R129,
    Leros::R130, Leros::R131, Leros::R132, Leros::R133, Leros::R134,
    Leros::R135, Leros::R136, Leros::R137, Leros::R138, Leros::R139,
    Leros::R140, Leros::R141, Leros::R142, Leros::R143, Leros::R144,
    Leros::R145, Leros::R146, Leros::R147, Leros::R148, Leros::R149,
    Leros::R150, Leros::R151, Leros::R152, Leros::R153, Leros::R154,
    Leros::R155, Leros::R156, Leros::R157, Leros::R158, Leros::R159,
    Leros::R160, Leros::R161, Leros::R162, Leros::R163, Leros::R164,
    Leros::R165, Leros::R166, Leros::R167, Leros::R168, Leros::R169,
    Leros::R170, Leros::R171, Leros::R172, Leros::R173, Leros::R174,
    Leros::R175, Leros::R176, Leros::R177, Leros::R178, Leros::R179,
    Leros::R180, Leros::R181, Leros::R182, Leros::R183, Leros::R184,
    Leros::R185, Leros::R186, Leros::R187, Leros::R188, Leros::R189,
    Leros::R190, Leros::R191, Leros::R192, Leros::R193, Leros::R194,
    Leros::R195, Leros::R196, Leros::R197, Leros::R198, Leros::R199,
    Leros::R200, Leros::R201, Leros::R202, Leros::R203, Leros::R204,
    Leros::R205, Leros::R206, Leros::R207, Leros::R208, Leros::R209,
    Leros::R210, Leros::R211, Leros::R212, Leros::R213, Leros::R214,
    Leros::R215, Leros::R216, Leros::R217, Leros::R218, Leros::R219,
    Leros::R220, Leros::R221, Leros::R222, Leros::R223, Leros::R224,
    Leros::R225, Leros::R226, Leros::R227, Leros::R228, Leros::R229,
    Leros::R230, Leros::R231, Leros::R232, Leros::R233, Leros::R234,
    Leros::R235, Leros::R236, Leros::R237, Leros::R238, Leros::R239,
    Leros::R240, Leros::R241, Leros::R242, Leros::R243, Leros::R244,
    Leros::R245, Leros::R246, Leros::R247, Leros::R248, Leros::R249,
    Leros::R250, Leros::R251, Leros::R252, Leros::R253, Leros::R254,
    Leros::R255};

static DecodeStatus DecodeGPRRegisterClass(MCInst &Inst, uint64_t RegNo,
                                           uint64_t Address,
                                           const void *Decoder) {
  if (RegNo > sizeof(GPRDecoderTable))
    return MCDisassembler::Fail;

  // We must define our own mapping from RegNo to register identifier.
  // Accessing index RegNo in the register class will work in the case that
  // registers were added in ascending order, but not in general.
  unsigned Reg = GPRDecoderTable[RegNo];
  Inst.addOperand(MCOperand::createReg(Reg));
  return MCDisassembler::Success;
}

static DecodeStatus DecodeGPRCRegisterClass(MCInst &Inst, uint64_t RegNo,
                                            uint64_t Address,
                                            const void *Decoder) {
  if (RegNo > 8)
    return MCDisassembler::Fail;

  unsigned Reg = GPRDecoderTable[RegNo + 8];
  Inst.addOperand(MCOperand::createReg(Reg));
  return MCDisassembler::Success;
}

template <unsigned N>
static DecodeStatus decodeUImmOperand(MCInst &Inst, uint64_t Imm,
                                      int64_t Address, const void *Decoder) {
  assert(isUInt<N>(Imm) && "Invalid immediate");
  Inst.addOperand(MCOperand::createImm(Imm));
  return MCDisassembler::Success;
}

template <unsigned N>
static DecodeStatus decodeSImmOperand(MCInst &Inst, uint64_t Imm,
                                      int64_t Address, const void *Decoder) {
  assert(isUInt<N>(Imm) && "Invalid immediate");
  // Sign-extend the number in the bottom N bits of Imm
  Inst.addOperand(MCOperand::createImm(SignExtend64<N>(Imm)));
  return MCDisassembler::Success;
}

template <unsigned N>
static DecodeStatus decodeSImmOperandAndLsl1(MCInst &Inst, uint64_t Imm,
                                             int64_t Address,
                                             const void *Decoder) {
  assert(isUInt<N>(Imm) && "Invalid immediate");
  // Sign-extend the number in the bottom N bits of Imm after accounting for
  // the fact that the N bit immediate is stored in N-1 bits (the LSB is
  // always zero)
  Inst.addOperand(MCOperand::createImm(SignExtend64<N>(Imm << 1)));
  return MCDisassembler::Success;
}

#include "LerosGenDisassemblerTables.inc"

DecodeStatus LerosDisassembler::getInstruction(MCInst &MI, uint64_t &Size,
                                               ArrayRef<uint8_t> Bytes,
                                               uint64_t Address,
                                               raw_ostream &OS,
                                               raw_ostream &CS) const {
  // TODO: This will need modification when supporting instruction set
  // extensions with instructions > 32-bits (up to 176 bits wide).
  uint32_t Insn;
  DecodeStatus Result;

  Insn = support::endian::read16be(Bytes.data());

  LLVM_DEBUG(dbgs() << "Trying Leros_C table (16-bit Instruction):\n");
  // Calling the auto-generated decoder function.
  Result = decodeInstruction(DecoderTable16, MI, Insn, Address, this, STI);
  Size = 2;

  return Result;
}
