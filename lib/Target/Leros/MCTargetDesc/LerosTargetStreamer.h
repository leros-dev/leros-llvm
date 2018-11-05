//===-- LerosTargetStreamer.h - Leros Target Streamer ----------*- C++ -*--===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_Leros_LerosTARGETSTREAMER_H
#define LLVM_LIB_TARGET_Leros_LerosTARGETSTREAMER_H

#include "llvm/MC/MCStreamer.h"

namespace llvm {

class LerosTargetStreamer : public MCTargetStreamer {
public:
  LerosTargetStreamer(MCStreamer &S);

  virtual void emitDirectiveOptionRVC() = 0;
  virtual void emitDirectiveOptionNoRVC() = 0;
};

// This part is for ascii assembly output
class LerosTargetAsmStreamer : public LerosTargetStreamer {
  formatted_raw_ostream &OS;

public:
  LerosTargetAsmStreamer(MCStreamer &S, formatted_raw_ostream &OS);

  void emitDirectiveOptionRVC() override;
  void emitDirectiveOptionNoRVC() override;
};

} // namespace llvm
#endif
