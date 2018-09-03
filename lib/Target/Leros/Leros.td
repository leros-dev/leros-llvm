//===- Leros.td - Describe the Leros Target Machine --------*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

include "llvm/Target/Target.td"

//===----------------------------------------------------------------------===//
// Leros subtarget features
//===----------------------------------------------------------------------===//

def Leros16           : HwMode<"+16bit">;
def Leros32           : HwMode<"+32bit">;
def Leros64           : HwMode<"+64bit">;

/* @todo: These are not subtarget specific features, but rather our processor model specifications
def Fe
def IsLeros16 : Predicate<"Subtarget->is16Bit()">, 
  AssemblerPredicate<"Leros16Bit">;

def IsLeros32 : Predicate<"Subtarget->is32Bit()">, 
  AssemblerPredicate<"Leros32Bit">;

def IsLeros64 : Predicate<"Subtarget->is64Bit()">, 
  AssemblerPredicate<"Leros64Bit">;


def
*/

//===----------------------------------------------------------------------===//
// Registers, calling conventions, instruction descriptions.
//===----------------------------------------------------------------------===//

include "LerosRegisterInfo.td"
include "LerosInstrInfo.td"
include "LerosCallingConv.td"

//===----------------------------------------------------------------------===//
// Leros processors supported.
//===----------------------------------------------------------------------===//

def : ProcessorModel<"generic-leros16", NoSchedModel, []>;

def : ProcessorModel<"generic-leros32", NoSchedModel, []>;

def : ProcessorModel<"generic-leros64", NoSchedModel, []>;

//===----------------------------------------------------------------------===//
// Define the Leros target.
//===----------------------------------------------------------------------===//

def LerosInstrInfo : InstrInfo;

class Proc<string Name, list<SubtargetFeature> Features>
 : Processor<Name, NoItineraries, Features>;

def : Proc<"generic", []>;

def Leros : Target {
  let InstructionSet = LerosInstrInfo;
}
