//===-- LerosTargetStreamer.cpp - Leros Target Streamer Methods -----------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file provides Leros specific target streamer methods.
//
//===----------------------------------------------------------------------===//

#include "LerosTargetStreamer.h"
#include "llvm/Support/FormattedStream.h"

using namespace llvm;

LerosTargetStreamer::LerosTargetStreamer(MCStreamer &S) : MCTargetStreamer(S) {}

// This part is for ascii assembly output
LerosTargetAsmStreamer::LerosTargetAsmStreamer(MCStreamer &S,
                                               formatted_raw_ostream &OS)
    : LerosTargetStreamer(S), OS(OS) {}

void LerosTargetAsmStreamer::emitDirectiveOptionRVC() {
  OS << "\t.option\trvc\n";
}

void LerosTargetAsmStreamer::emitDirectiveOptionNoRVC() {
  OS << "\t.option\tnorvc\n";
}
