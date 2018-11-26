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

class LerosLSOpcode<bits<8> val> {
    bits<8>Value = val;
}

// Opcode class for branch instructions
class LerosBOpcode<bits<4> val> {
    bits<4>Value = val;
}

class LerosJalOpcode<bits<5> val>{
    bits<5>Value = val;
}

// Accumulator and Register-using opcodes

def OPC_NOP     : LerosImmRegOpcode<0b0000000>;
def OPC_ADD     : LerosImmRegOpcode<0b0000100>;
def OPC_SUB     : LerosImmRegOpcode<0b0000110>;
def OPC_SHR     : LerosImmRegOpcode<0b0001000>;
def OPC_UNUSED  : LerosImmRegOpcode<0b0001100>;
def OPC_LOAD    : LerosImmRegOpcode<0b0010000>;
def OPC_AND     : LerosImmRegOpcode<0b0010001>;
def OPC_OR      : LerosImmRegOpcode<0b0010010>;
def OPC_XOR     : LerosImmRegOpcode<0b0010011>;

def OPC_STORE   : LerosImmRegOpcode<0b0011000>;
def OPC_OUT     : LerosImmRegOpcode<0b0011100>;
def OPC_IN      : LerosImmRegOpcode<0b0000010>;


// Branch opcodes
def OPC_BR      : LerosBOpcode<0b1000>;
def OPC_BRZ     : LerosBOpcode<0b1001>;
def OPC_BRNZ    : LerosBOpcode<0b1010>;
def OPC_BRP     : LerosBOpcode<0b1011>;
def OPC_BRN     : LerosBOpcode<0b1100>;

// Load/Store opcodes
def OPC_LDADDR  : LerosImmRegOpcode<0b0101000>;
def OPC_LDIND   : LerosLSOpcode<0b01100000>;
def OPC_LDINDBU : LerosLSOpcode<0b01100001>;
def OPC_STIND   : LerosLSOpcode<0b01110000>;
def OPC_STINDB  : LerosLSOpcode<0b01110001>;
def OPC_LOADH   : LerosLSOpcode<0b00101001>;
def OPC_LOADH2  : LerosLSOpcode<0b00101010>;
def OPC_LOADH3  : LerosLSOpcode<0b00101011>;

// JAL
def OPC_JAL     : LerosJalOpcode<0b01000>;

// Pseudoinstruction formats
class PseudoFormat<bits<4> val>{
    bits<4>Value = val;
}

// Format specifies the encoding used by the pseudoinstruction. This is used in
// expandPostRAPseudo to determine which form of accumulator-using expansion to use.
// These definitions must be kept in-sync with LerosBaseInfo.h.
def F_NA : PseudoFormat<0>;     // No special formatting
def F_RRR : PseudoFormat<1>;    // Register -> Register-Register formatting
def F_RRI : PseudoFormat<2>;    // Register -> Register, Immediate formatting
def F_BRCmp : PseudoFormat<3>;  // Conditional branch with 2 registers (comparisons)
def F_RI : PseudoFormat<4>;     // Register -> Immediate
def F_BRIND : PseudoFormat<5>;     // Indirect branch formatting
def F_LS : PseudoFormat<6>;     // Memory load/store formatting
def F_S8L : PseudoFormat<7>;    // sign-extended 8-bit memory load
def F_S16L: PseudoFormat<8>;    // Sign-extended 16-bit memory load
def F_U8L : PseudoFormat<9>;    // zero-extended 8-bit memory load
def F_U16L : PseudoFormat<10>;  // zero-extended 16-bit memory load
def F_8S    : PseudoFormat<11>; // 8-bit (byte) memory store
def F_BRRs : PseudoFormat<12>;  // Conditional branch with 1 register



//===----------------------------------------------------------------------===//
// Instruction format superclass
//===----------------------------------------------------------------------===//

class LerosInst<dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern> : Instruction {
  // SoftFail is a field the disassembler can use to provide a way for
  // instructions to not match without killing the whole decode process. It is
  // mainly used for ARM, but Tablegen expects this field to exist or it fails
  // to build the decode table.
  field bits<32> SoftFail = 0;

  field bits<16> Inst;

  let Namespace = "Leros";

  let Size = 2; // All leros instructions are 2 bytes

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

class LerosImmRegInst<dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : LerosInst<outs, ins, opcodestr, argstr, pattern>{
    bits<8> RegOrImm;
    let Inst{7-0} = RegOrImm;
}

class LerosRegInst<LerosImmRegOpcode opcode, dag outs, dag ins,  string opcodestr, string argstr,
    list<dag> pattern>
    : LerosImmRegInst<outs, ins, opcodestr, argstr, pattern>{
    bits<8> r;
    let RegOrImm = r;
    let Inst{8} = 0;
    let Inst{15-9} = opcode.Value;
}

// Instruction with no operands, that work exclusively on the accumulator, such as shr
class LerosAccInst<LerosImmRegOpcode opcode, string opcodestr, list<dag> pattern>
    : LerosInst<(outs), (ins), opcodestr, "", pattern>{
    let Inst{8-0} = 0;
    let Inst{15-9} = opcode.Value;
}


class LerosImmInst<LerosImmRegOpcode opcode, dag outs, dag ins, string opcodestr, string argstr,
    list<dag> pattern>
    : LerosImmRegInst<outs, ins, opcodestr, argstr, pattern>{
    bits<8> imm8;
    let RegOrImm = imm8;
    let Inst{8} = 1;
    let Inst{15-9} = opcode.Value;
}

class LerosImmInst_LS<LerosLSOpcode opcode, dag outs, dag ins, string opcodestr, string argstr,
    list<dag> pattern>
    : LerosImmRegInst<outs, ins, opcodestr, argstr, pattern>{
    bits<8> imm8;
    let RegOrImm = imm8;
    let Inst{15-8} = opcode.Value;
}


class LerosBranchInst<LerosBOpcode opcode, dag outs, dag ins, string opcodestr, string argstr,
    list<dag> pattern>
    : LerosInst<outs, ins, opcodestr, argstr, pattern>{
    bits<12> imm12;
    let Inst{11-0} = imm12;
    let Inst{15-12} = opcode.Value;
    let isBranch = 1;
} 
