//===--- LerosInstrFormats.td ----------------------------*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

// Opcodes for instructions operating on the Accumulator or a Register
class LerosImmRegOpcode<bits<7> val> {
    bits<7>Value = val;
}

// Opcode class for branch instructions
class LerosBOpcode<bits<8> val> {
    bits<8>Value = val;
}

// Opcode class for indirect load/store and jal operations
class LerosLSOpcode<bits<5> val> {
    bits<5>Value = val;
}

class LerosJalOpcode<bits<5> val>{
    bits<5>Value = val;
}

// Accumulator and Register-using opcodes

def OPC_NOP     : LerosImmRegOpcode<0b0000000>;
def OPC_ADD     : LerosImmRegOpcode<0b0000100>;
def OPC_SUB     : LerosImmRegOpcode<0b0000110>;
def OPC_SHR     : LerosImmRegOpcode<0b0010000>;
def OPC_UNUSED  : LerosImmRegOpcode<0b0001100>;
def OPC_LOAD    : LerosImmRegOpcode<0b0010000>;
def OPC_AND     : LerosImmRegOpcode<0b0010001>;
def OPC_OR      : LerosImmRegOpcode<0b0010010>;
def OPC_XOR     : LerosImmRegOpcode<0b0010011>;
def OPC_LOADH   : LerosImmRegOpcode<0b0010100>;
def OPC_STORE   : LerosImmRegOpcode<0b0011000>;
def OPC_OUT     : LerosImmRegOpcode<0b0011100>;
def OPC_IN      : LerosImmRegOpcode<0b0000010>;


// Branch opcodes
def OPC_BR      : LerosBOpcode<0b01001000>;
def OPC_BRZ     : LerosBOpcode<0b01001001>;
def OPC_BRNZ    : LerosBOpcode<0b01001010>;
def OPC_BRP     : LerosBOpcode<0b01001011>;
def OPC_BRN     : LerosBOpcode<0b01001100>;

// Load/Store opcodes
def OPC_LDADDR  : LerosLSOpcode<0b01010>;
def OPC_LDIND   : LerosLSOpcode<0b01100>;
def OPC_STIND   : LerosLSOpcode<0b01110>;

// JAL
def OPC_JAL     : LerosJalOpcode<0b01000>;


//===----------------------------------------------------------------------===//
// Instruction format superclass
//===----------------------------------------------------------------------===//

class LerosInst<dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern> : Instruction {
  field bits<16> Inst;
  bits<8> RegOrImm;

  let Inst{7-0} = RegOrImm;

  let Namespace = "Leros";

  dag OutOperandList = outs;
  dag InOperandList = ins;
  let AsmString = opcodestr # "\t" # argstr;
  let Pattern = pattern;
}

class LerosImmRegInst<LerosImmRegOpcode opcode, dag outs, dag ins, string opcodestr, string argstr>
    : LerosInst<outs, ins, opcodestr, argstr, []>{
    let Inst{15-9} = opcode.Value;
}

class LerosRegInst<LerosImmRegOpcode opcode, dag outs, dag ins,  string opcodestr, string argstr>
    : LerosImmRegInst<opcode, outs, ins, opcodestr, argstr>{
    let Inst{8} = 0;
}

class LerosImmInst<LerosImmRegOpcode opcode, dag outs, dag ins, string opcodestr, string argstr>
    : LerosImmRegInst<opcode, outs, ins, opcodestr, argstr>{
    let Inst{8} = 1;
}

class LerosBranchInst<LerosBOpcode opcode, dag outs, dag ins, string opcodestr, string argstr>
    : LerosInst<outs, ins, opcodestr, argstr, []>{
    let Inst{15-8} = opcode.Value;
} 