//===-- LerosInstrInfo.td - Target Description for RISCV ---*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file describes the Leros instructions in TableGen format.
//
//===----------------------------------------------------------------------===//

include "LerosInstrFormats.td"

//===----------------------------------------------------------------------===//
// Operands
//===----------------------------------------------------------------------===//

class ImmAsmOperand<string prefix, int width, string suffix> : AsmOperandClass {
  let Name = prefix # "Imm" # width # suffix;
  let RenderMethod = "addImmOperands";
  let DiagnosticType = !strconcat("Invalid", Name);
}

class SImmAsmOperand<int width, string suffix = "">
    : ImmAsmOperand<"S", width, suffix> {
}

class UImmAsmOperand<int width, string suffix = "">
    : ImmAsmOperand<"U", width, suffix> {
}

// Leros only has two forms of immediate operands - a signed and unsigned 
// byte-sized immediate.
// Both operands are mapped to XLenVT for calls
def uimm8 : Operand<XLenVT>, ImmLeaf<XLenVT, [{return isUInt<8>(Imm);}]> {
  let ParserMatchClass = UImmAsmOperand<5>;
  let DecoderMethod = "decodeUImmOperand<8>";
}

def simm8 : Operand<XLenVT>, ImmLeaf<XLenVT, [{return isInt<8>(Imm);}]> {
  let ParserMatchClass = SImmAsmOperand<5>;
  let DecoderMethod = "decodeSImmOperand<8>";
}


//===----------------------------------------------------------------------===//
// Instructions
//===----------------------------------------------------------------------===//


// Instruction class definitions

// Instructions with dependency: Acc -> (Acc, Imm)
let Constraints = "$acci = $acco" in {
    class INSTR_AccAccImm<LerosImmRegOpcode opcode, string opcodestr>
        : LerosImmInst<opcode, (outs ACC:$acco), (ins ACC:$acci, uimm8:$imm8), 
            opcodestr, "$imm8">;

    // Instructions with dependency: Acc -> (Acc, Reg)
    class INSTR_AccAccReg<LerosImmRegOpcode opcode, string opcodestr>
        : LerosImmInst<opcode, (outs ACC:$acco), (ins ACC:$acci, GPR:$r),
            opcodestr, "$r">;
}

    // Instructions with dependency: Reg -> (Acc)
    class INSTR_RegAcc<LerosImmRegOpcode opcode, string opcodestr>
        : LerosImmInst<opcode, (outs GPR:$r), (ins ACC:$acc),
            opcodestr, "$r">;

multiclass INSTR_ImmReg<LerosImmRegOpcode opcode, string TGName, string opcodestr>{
    def #TGName#_I  : INSTR_AccAccImm<opcode, opcodestr>;
    def #TGName#_R  : INSTR_AccAccReg<opcode, opcodestr>; 
}


class INSTR_Branch<LerosBOpcode opcode, string opcodestr>
    : LerosBranchInst<opcode, (outs), (ins uimm8:$imm8), opcodestr, "$imm8">{
}


// Instruction declarations
defm : INSTR_ImmReg<OPC_NOP     , "INSTR_NOP",    "nop">;
defm : INSTR_ImmReg<OPC_ADD     , "INSTR_ADD",    "add">;
defm : INSTR_ImmReg<OPC_SUB     , "INSTR_SUB",    "sub">;
defm : INSTR_ImmReg<OPC_SHR     , "INSTR_SHR",    "shr">;
defm : INSTR_ImmReg<OPC_UNUSED  , "INSTR_UNUSED", "">;
defm : INSTR_ImmReg<OPC_LOAD    , "INSTR_LOAD",   "load">;
defm : INSTR_ImmReg<OPC_AND     , "INSTR_AND",    "and">;
defm : INSTR_ImmReg<OPC_OR      , "INSTR_OR",     "or">;
defm : INSTR_ImmReg<OPC_XOR     , "INSTR_XOR",    "xor">;
defm : INSTR_ImmReg<OPC_LOADH   , "INSTR_LOADH",  "loadh">;
defm : INSTR_ImmReg<OPC_OUT     , "INSTR_OUT",    "out">;
defm : INSTR_ImmReg<OPC_IN      , "INSTR_IN",     "in">;

def STORE   : INSTR_RegAcc<OPC_STORE, "store">;

def BR      : INSTR_Branch<OPC_BR, "br">;
def BRZ     : INSTR_Branch<OPC_BRZ, "brz">;
def BRNZ    : INSTR_Branch<OPC_BRNZ, "brnz">;
def BRP     : INSTR_Branch<OPC_BRP, "brp">;
def BRN     : INSTR_Branch<OPC_BRN, "brn">;

// Instructions where format is specific to the instruction

def LDADDR  : LerosInst<(outs GPR:$r), (ins ACC:$acc), "ldaddr", "$r", []>{
    let Inst{15-11} = OPC_LDADDR.Value;
}

def LDIND : LerosInst<(outs ACC:$acc), (ins GPR:$r), "ldind", "$r", []>{
    let Inst{15-11} = OPC_LDIND.Value;
}

def STIND : LerosInst<(outs GPR:$r), (ins ACC:$acc), "stind", "$r", []>{
    let Inst{15-11} = OPC_STIND.Value;
}

def JAL     : LerosInst<(outs GPR:$r), (ins ACC:$acc), "jal", "$r", []>{
    let Inst{15-11} = OPC_JAL.Value;
}

//===----------------------------------------------------------------------===//
// Immediate patterns
//===----------------------------------------------------------------------===//


//===----------------------------------------------------------------------===//
// Codegen patterns
//===----------------------------------------------------------------------===//

/*
class PatGprGpr<SDPatternOperator OpNode, LerosARInst Inst>
    : Pat<(OpNode GPR:$rs1, GPR:$rs2), (Inst GPR:$rs1, GPR:$rs2)>;

class PatGprImm<SDPatternOperator OpNode, LerosARInst Inst>
    : Pat<(OpNode GPR:$rs1, GPR:$rs2), (Inst GPR:$rs1, GPR:$rs2)>;

// Branches
class BPat<PatFrag CondOp, RVInstB Inst>
    : Pat<(brcond (i32 (CondOp GPR:$rs1, GPR:$rs2)), bb:$imm12),
          (Inst GPR:$rs1, GPR:$rs2, simm13_lsb0:$imm12)>;

def : BPat<seteq, BR>;
def : BPat<setne, BRZ>;
def : BPat<setlt, BRNZ>;
def : BPat<setge, BRP>;
def : BPat<setge, BRN>;
*/

// 'icmp' instruction
