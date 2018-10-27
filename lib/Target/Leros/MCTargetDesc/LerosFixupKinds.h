//===-- LerosFixupKinds.h - Leros Specific Fixup Entries --------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_Leros_MCTARGETDESC_LerosFIXUPKINDS_H
#define LLVM_LIB_TARGET_Leros_MCTARGETDESC_LerosFIXUPKINDS_H

#include "llvm/MC/MCFixup.h"

#undef Leros

namespace llvm {
namespace Leros {
enum Fixups {
  // fixup_leros_branch - 11-bit fixup value for a pc-relative branch offset
  fixup_leros_branch = FirstTargetFixupKind,
  // fixup_leros_b0 - 8-bit fixup, for byte 0 of a symbol
  fixup_leros_b0,
  // fixup_leros_b1 - 8-bit fixup, for byte 1 of a symbol
  fixup_leros_b1,
  // fixup_leros_b2 - 8-bit fixup, for byte 2 of a symbol
  fixup_leros_b2,
  // fixup_leros_b3 - 8-bit fixup, for byte 3 of a symbol
  fixup_leros_b3,

  // fixup_Leros_invalid - used as a sentinel and a marker, must be last fixup
  fixup_leros_invalid,
  NumTargetFixupKinds = fixup_leros_invalid - FirstTargetFixupKind
};
} // end namespace Leros
} // end namespace llvm

#endif
