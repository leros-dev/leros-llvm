//===- Leros.td - Describe the Leros Target Machine --------*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

include "llvm/Target/Target.td"

include "LerosRegisterInfo.td"
include "LerosInstrInfo.td"
include "LerosCallingConv.td"

def LerosInstrInfo : InstrInfo;

class Proc<string Name, list<SubtargetFeature> Features>
 : Processor<Name, NoItineraries, Features>;

def : Proc<"generic", []>;

def Leros : Target {
  let InstructionSet = LerosInstrInfo;
}
