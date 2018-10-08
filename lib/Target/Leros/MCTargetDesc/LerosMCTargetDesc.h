//===- LerosMCTargetDesc.h - Leros Target Descriptions --------------*- C++
//-*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file provides Leros specific target descriptions.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_Leros_MCTARGETDESC_LerosMCTARGETDESC_H
#define LLVM_LIB_TARGET_Leros_MCTARGETDESC_LerosMCTARGETDESC_H

#include "memory"
#include "llvm/MC/MCTargetOptions.h"
#include "llvm/Support/DataTypes.h"

namespace llvm {

class MCAsmBackend;
class MCCodeEmitter;
class MCContext;
class MCInstrInfo;
class MCObjectTargetWriter;
class MCRegisterInfo;
class MCSubtargetInfo;
class StringRef;
class Target;
class Triple;
class raw_ostream;
class raw_pwrite_stream;

Target &getTheLeros16Target();
Target &getTheLeros32Target();
Target &getTheLeros64Target();

MCCodeEmitter *createLerosMCCodeEmitter(const MCInstrInfo &MCII,
                                        const MCRegisterInfo &MRI,
                                        MCContext &Ctx);

MCAsmBackend *createLerosAsmBackend(const Target &T, const MCSubtargetInfo &STI,
                                    const MCRegisterInfo &MRI,
                                    const MCTargetOptions &Options);

std::unique_ptr<MCObjectTargetWriter> createLerosELFObjectWriter(uint8_t OSABI,
                                                                 bool Is64Bit);

} // end namespace llvm

// Defines symbolic names for Leros registers.  This defines a mapping from
// register name to register number.
#define GET_REGINFO_ENUM
#include "LerosGenRegisterInfo.inc"

// Defines symbolic names for the Leros instructions.
#define GET_INSTRINFO_ENUM
#include "LerosGenInstrInfo.inc"

#define GET_SUBTARGETINFO_ENUM
#include "LerosGenSubtargetInfo.inc"

#endif // LLVM_LIB_TARGET_Leros_MCTARGETDESC_LerosMCTARGETDESC_H
