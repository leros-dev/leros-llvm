//===- LerosMCAsmInfo.h - Leros asm properties ----------------------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the declaration of the LerosMCAsmInfo class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_Leros_MCTARGETDESC_LerosMCASMINFO_H
#define LLVM_LIB_TARGET_Leros_MCTARGETDESC_LerosMCASMINFO_H

#include "llvm/MC/MCAsmInfoELF.h"

namespace llvm {

class Triple;

class LerosMCAsmInfo : public MCAsmInfoELF {
  void anchor() override;

public:
  explicit LerosMCAsmInfo(const Triple &TT);
};

} // end namespace llvm

#endif // LLVM_LIB_TARGET_Leros_MCTARGETDESC_LerosMCASMINFO_H
