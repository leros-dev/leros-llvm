//===--- LerosInstrFormats.td ----------------------------*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

class LerosOpcode<bits<8> val> {
    bits<8>Value = val;
}
class LerosBOpcode<bits<4> val> {
    bits<4>Value = val;
}

// Accumulator and Register-using opcodes

def OPC_NOP     : LerosOpcode<0b00000000>;
def OPC_ADDI    : LerosOpcode<0b00001001>;
def OPC_ADDR    : LerosOpcode<0b00001000>;
def OPC_SUBI    : LerosOpcode<0b00001101>;
def OPC_SUBR    : LerosOpcode<0b00001100>;
def OPC_SRA     : LerosOpcode<0b00010000>;
def OPC_LOADR   : LerosOpcode<0b00100000>;
def OPC_LOADI   : LerosOpcode<0b00100001>;
def OPC_ANDI    : LerosOpcode<0b00100011>;
def OPC_ANDR    : LerosOpcode<0b00100010>;
def OPC_ORI     : LerosOpcode<0b00100101>;
def OPC_ORR     : LerosOpcode<0b00100100>;
def OPC_XORI    : LerosOpcode<0b00100111>;
def OPC_XORR    : LerosOpcode<0b00100110>;

def OPC_STORE   : LerosOpcode<0b00110000>;
def OPC_OUT     : LerosOpcode<0b00111000>;
def OPC_IN      : LerosOpcode<0b00000100>;


// Branch opcodes
def OPC_BR      : LerosBOpcode<0b1000>;
def OPC_BRZ     : LerosBOpcode<0b1001>;
def OPC_BRNZ    : LerosBOpcode<0b1010>;
def OPC_BRP     : LerosBOpcode<0b1011>;
def OPC_BRN     : LerosBOpcode<0b1100>;

// Load/Store opcodes
def OPC_LDADDR  : LerosOpcode<0b01010000>;
def OPC_LDIND   : LerosOpcode<0b01100000>;
def OPC_LDINDBU : LerosOpcode<0b01100001>;
def OPC_STIND   : LerosOpcode<0b01110000>;
def OPC_STINDB  : LerosOpcode<0b01110001>;
def OPC_LOADH   : LerosOpcode<0b00101001>;
def OPC_LOADH2  : LerosOpcode<0b00101010>;
def OPC_LOADH3  : LerosOpcode<0b00101011>;

// JAL
def OPC_JAL     : LerosOpcode<0b01000000>;

// Pseudoinstruction formats
class PseudoFormat<bits<4> val>{
    bits<4>Value = val;
}

// Format specifies the encoding used by the pseudoinstruction. This is used in
// expandPostRAPseudo to determine which form of expansion to use.
// These definitions must be kept in-sync with LerosBaseInfo.h.
def F_NA : PseudoFormat<0>;     // No special formatting
def F_RRR : PseudoFormat<1>;    // Register -> Register-Register formatting
def F_RRI : PseudoFormat<2>;    // Register -> Register, Immediate formatting
def F_BRCmp : PseudoFormat<3>;  // Conditional branch with 2 registers (comparisons)
def F_RI : PseudoFormat<4>;     // Register -> Immediate
def F_LS : PseudoFormat<5>;     // Memory load/store formatting
def F_BRRs : PseudoFormat<6>;  // Conditional branch with 1 register



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

class LerosImmRegInst<LerosOpcode opcode, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : LerosInst<outs, ins, opcodestr, argstr, pattern>{
    bits<8> RegOrImm;
    let Inst{7-0} = RegOrImm;
    let Inst{15-8} = opcode.Value;
}

// Instruction with no operands, that work exclusively on the accumulator, such as shr
class LerosAccInst<LerosOpcode opcode, string opcodestr>
    : LerosInst<(outs), (ins), opcodestr, "", []>{
    let Inst{8-0} = 0;
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
