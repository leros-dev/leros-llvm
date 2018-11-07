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

def Feature64Bit
    : SubtargetFeature<"64bit", "HasLeros64", "true", "Implements Leros64">;
def IsRV64 : Predicate<"Subtarget->is64Bit()">,
                       AssemblerPredicate<"Feature64Bit">;
def IsRV32 : Predicate<"!Subtarget->is64Bit()">,
                       AssemblerPredicate<"!Feature64Bit">;

def Leros32           : HwMode<"-64bit">;
def Leros64           : HwMode<"+64bit">;

/* @todo: These are not subtarget specific features, but rather our processor model specifications
def Fe
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
def : ProcessorModel<"generic-leros32", NoSchedModel, []>;

def : ProcessorModel<"generic-leros64", NoSchedModel, [Feature64Bit]>;

//===----------------------------------------------------------------------===//
// Define the Leros target.
//===----------------------------------------------------------------------===//

def LerosInstrInfo : InstrInfo;

def LerosAsmParser : AsmParser {
  let ShouldEmitMatchRegisterAltName = 1;
  let AllowDuplicateRegisterNames = 1;
}


class Proc<string Name, list<SubtargetFeature> Features>
 : Processor<Name, NoItineraries, Features>;

def : Proc<"generic", []>;

def Leros : Target {
  let InstructionSet = LerosInstrInfo;
  let AssemblyParsers = [LerosAsmParser];
}
