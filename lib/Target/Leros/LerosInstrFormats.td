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
def OPC_LOADH2 : LerosImmRegOpcode<0b0010101>;
def OPC_LOADH3 : LerosImmRegOpcode<0b0010110>;
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

// Pseudoinstruction formats
class PseudoFormat<bits<4> val>{
    bits<4>Value = val;
}

// Format specifies the encoding used by the pseudoinstruction. This is used in
// expandPostRAPseudo to determine which form of accumulator-using expansion to use.
// These definitions must be kept in-sync with LerosISelLowering.h.
def F_NA : PseudoFormat<0>;     // No special formatting
def F_RRR : PseudoFormat<1>;    // Register -> Register-Register formatting
def F_RRI : PseudoFormat<2>;    // Register -> Register, Immediate formatting
def F_BRCC : PseudoFormat<3>;   // Conditional branch formatting
def F_RI : PseudoFormat<4>;     // Register -> Immediate
def F_BR : PseudoFormat<5>;     // Unconditional branch formatting
def F_LS : PseudoFormat<6>;     // Memory load/store formatting
def F_S8L : PseudoFormat<7>;    // sign-extended 8-bit memory load
def F_S16L: PseudoFormat<8>;    // Sign-extended 16-bit memory load
def F_U8L : PseudoFormat<9>;    // zero-extended 8-bit memory load
def F_U16L : PseudoFormat<10>;  // zero-extended 16-bit memory load




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

class LerosPseudo<dag outs, dag ins, PseudoFormat format, list<dag> pattern>
    : LerosInst<outs, ins, "", "", pattern> {
  let isPseudo = 1;
  let isCodeGenOnly = 1;

  // The pseudo instruction format is used in expandPostRAPseudo for determining
  // the sequence of accumulator loads and stores
  let TSFlags{3-0} = format.Value;
}

class LerosImmRegInst<LerosImmRegOpcode opcode, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : LerosInst<outs, ins, opcodestr, argstr, pattern>{
    let Inst{15-9} = opcode.Value;
}

class LerosRegInst<LerosImmRegOpcode opcode, dag outs, dag ins,  string opcodestr, string argstr,
    list<dag> pattern>
    : LerosImmRegInst<opcode, outs, ins, opcodestr, argstr, pattern>{
    let Inst{8} = 0;
}

class LerosImmInst<LerosImmRegOpcode opcode, dag outs, dag ins, string opcodestr, string argstr,
    list<dag> pattern>
    : LerosImmRegInst<opcode, outs, ins, opcodestr, argstr, pattern>{
    let Inst{8} = 1;
}

class LerosBranchInst<LerosBOpcode opcode, dag outs, dag ins, string opcodestr, string argstr,
    list<dag> pattern>
    : LerosInst<outs, ins, opcodestr, argstr, pattern>{
    let Inst{15-8} = opcode.Value;
    let isBranch = 1;
} 
