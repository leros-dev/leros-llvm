//===--- LerosInstrFormats.td ----------------------------*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

// Opcodes for instructions operating on the Accumulator or a Register
class LerosAROpcode<bits<7> val {
    bits<7>Value = val;
}

// Opcode class for branch instructions
class LerosBOpcode<bits<8> val {
    bits<8>Value = val;
}

// Opcode class for load/store operations
class LerosLSOpcode<bits<5> val {
    bits<4>Value = val;
}

// Accumulator and Register-using opcodes

def OPC_NOP     : LerosAROpcode<0b0000000>;
def OPC_ADD     : LerosAROpcode<0b0000100>;
def OPC_SUB     : LerosAROpcode<0b0000110>;
def OPC_SHR     : LerosAROpcode<0b0010000>;
def OPC_UNUSED  : LerosAROpcode<0b0001100>;
def OPC_LOAD    : LerosAROpcode<0b0010000>;
def OPC_AND     : LerosAROpcode<0b0010001>;
def OPC_OR      : LerosAROpcode<0b0010010>;
def OPC_XOR     : LerosAROpcode<0b0010011>;
def OPC_LOADH   : LerosAROpcode<0b0010100>;
def OPC_STORE   : LerosAROpcode<0b0011000>;
def OPC_OUT     : LerosAROpcode<0b0011100>;
def OPC_IN      : LerosAROpcode<0b0000010>;

// Branch opcodes
def OPC_BR      : LerosBOpcode<0b01001000>;
def OPC_BRZ     : LerosBOpcode<0b01001001>;
def OPC_BRNZ    : LerosBOpcode<0b01001010>;
def OPC_BRP     : LerosBOpcode<0b01001011>;
def OPC_BRN     : LerosBOpcode<0b01001100>;

// Load/Store opcodes
def OPC_LDADDR      : LerosLSOpcode<0b01010>;
def OPC_LOADIND     : LerosLSOpcode<0b01100>;
def OPC_STOREIND    : LerosLSOpcode<0b01110>;


//===----------------------------------------------------------------------===//
// Instruction format superclass
//===----------------------------------------------------------------------===//

class LerosInst<dag outs, dag ins, string asmstr, list<dag> pattern> : Instruction {
  field bits<32> Inst;

  let Namespace = "Leros";

  dag OutOperandList = outs;
  dag InOperandList = ins;
  let AsmString = asmstr;
  let Pattern = pattern;
}

class LerosARInst<bits<1>AR, LerosAROpcode opcode, dag outs, dag ins, string asmstr>
    : LerosInst<outs, ins, asmstr, []>{
    
    let Inst{31-}

}