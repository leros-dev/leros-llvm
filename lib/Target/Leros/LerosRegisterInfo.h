//===-- LerosRegisterInfo.h - Leros Register Information Impl ---*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the Leros implementation of the TargetRegisterInfo class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_Leros_LerosREGISTERINFO_H
#define LLVM_LIB_TARGET_Leros_LerosREGISTERINFO_H

#include "llvm/CodeGen/TargetRegisterInfo.h"

#define GET_REGINFO_HEADER
#include "LerosGenRegisterInfo.inc"

namespace llvm {

struct LerosRegisterInfo : public LerosGenRegisterInfo {

  LerosRegisterInfo(unsigned HwMode);
};
}

#endif
