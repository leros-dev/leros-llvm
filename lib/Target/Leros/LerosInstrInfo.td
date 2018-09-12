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
// Leros specific DAG profiles & Nodes.
//===----------------------------------------------------------------------===//
def JAL_RET : SDNode<"LEROSISD::JAL_RET", SDTNone,
                          [SDNPHasChain, SDNPOptInGlue, SDNPVariadic]>;

// For Loadh ops, both an imm8 as well as the accumulator are inputs
def SDT_LOADH   :SDTypeProfile<1, 2, [SDTCisVT<0, untyped>, SDTCisInt<1>, 
                                      SDTCisSameAs<1, 2>]>;

def LOADH : SDNode<"LEROSISD::LOADH", SDT_LOADH>;
def LOADH2 : SDNode<"LEROSISD::LOADH2", SDT_LOADH>;
def LOADH3 : SDNode<"LEROSISD::LOADH3", SDT_LOADH>;

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
let Constraints = "$acci = $acco" in {
    // Instructions with dependency: Acc -> (Acc, Imm)
    class INSTR_AccAccImm<LerosImmRegOpcode opcode, string opcodestr, SDNode opnode>
        : LerosImmInst<opcode, (outs ACC:$acco), (ins ACC:$acci, uimm8:$imm8), 
            opcodestr, "$imm8", [(set i32:$acco, (opnode i32:$acci, i32:$imm8))]>;

    // Instructions with dependency: Acc -> (Acc, Reg)
    class INSTR_AccAccReg<LerosImmRegOpcode opcode, string opcodestr, SDNode opnode>
        : LerosImmInst<opcode, (outs ACC:$acco), (ins ACC:$acci, GPR:$r),
            opcodestr, "$r", [(set i32:$acco, (opnode i32:$acci, i32:$r))]>;
}

    // Instructions with dependency: Reg -> (Acc)
    class INSTR_RegAcc<LerosImmRegOpcode opcode, string opcodestr, SDNode opnode>
        : LerosImmInst<opcode, (outs GPR:$r), (ins ACC:$acc),
            opcodestr, "$r", [(set i32:$r, (opnode i32:$acc))]>;
    
    // Instructions with dependency: Acc -> (reg)
    class INSTR_AccReg<LerosImmRegOpcode opcode, string opcodestr, SDNode opnode>
        : LerosImmInst<opcode, (outs ACC:$acc), (ins GPR:$r),
            opcodestr, "$r", [(set i32:$acc, (opnode i32:$r))]>;
    
    // Instructions with dependency: Acc -> (imm)
    class INSTR_AccImm<LerosImmRegOpcode opcode, string opcodestr, SDNode opnode>
        : LerosImmInst<opcode, (outs ACC:$acc), (ins uimm8:$imm8),
            opcodestr, "$imm8", [(set i32:$acc, (opnode i32:$imm8))]>;

multiclass INSTR_ImmReg<LerosImmRegOpcode opcode, string TGName, string opcodestr, SDNode opnode>{
    def #TGName#_I  : INSTR_AccAccImm<opcode, opcodestr, opnode>;
    def #TGName#_R  : INSTR_AccAccReg<opcode, opcodestr, opnode>; 
}


class INSTR_BranchCond<LerosBOpcode opcode, string opcodestr, PatFrag CondOp>
    : LerosBranchInst<opcode, (outs), (ins ACC:$acc, uimm8:$imm8), opcodestr, "$imm8",
    [(brcond (i32 (CondOp i32:$acc, uimm8:$imm8)))]>{
}


// Instruction declarations
// defm : INSTR_ImmReg<OPC_NOP     , "INSTR_NOP",    "nop",    nop>;
defm : INSTR_ImmReg<OPC_ADD     , "INSTR_ADD",    "add",    add>;
defm : INSTR_ImmReg<OPC_SUB     , "INSTR_SUB",    "sub",    sub>;
defm : INSTR_ImmReg<OPC_SHR     , "INSTR_SHR",    "shr",    srl>;
// defm : INSTR_ImmReg<OPC_UNUSED  , "INSTR_UNUSED", "">;
defm : INSTR_ImmReg<OPC_AND     , "INSTR_AND",    "and",    and>;
defm : INSTR_ImmReg<OPC_OR      , "INSTR_OR",     "or",     or>;
defm : INSTR_ImmReg<OPC_XOR     , "INSTR_XOR",    "xor",    xor>;

// Loads
def INSTR_LOAD_R : INSTR_AccReg<OPC_LOAD, "load", ld>;
def INSTR_LOAD_I : INSTR_AccImm<OPC_LOAD, "load", ld>;


//defm : INSTR_ImmReg<OPC_LOADH   , "INSTR_LOADH",  "loadh",  LOADH>;
//defm : INSTR_ImmReg<OPC_LOADH_2   , "INSTR_LOADH2",  "loadh2",  LOADH2>;
//defm : INSTR_ImmReg<OPC_LOADH_3   , "INSTR_LOADH3",  "loadh3",  LOADH3>;
// defm : INSTR_ImmReg<OPC_OUT     , "INSTR_OUT",    "out",    out>;
// defm : INSTR_ImmReg<OPC_IN      , "INSTR_IN",     "in",     in>;

// def STORE   : INSTR_RegAcc<OPC_STORE, "store", store>;

// def BR      : INSTR_BranchCond<OPC_BR,      "br",       BEQZ>; // unconditional branch
//def BRZ     : INSTR_BranchCond<OPC_BRZ,     "brz",      seteq>;
//def BRNZ    : INSTR_BranchCond<OPC_BRNZ,    "brnz",     brnz>;
//def BRP     : INSTR_BranchCond<OPC_BRP,     "brp",      brp>;
//  def BRN     : INSTR_BranchCond<OPC_BRN,     "brn",      brn>;

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


// Function return using jal
def PseudoRET : Pseudo<(outs), (ins), [(JAL_RET)]>,
                PseudoInstExpansion<(JAL R0)>;

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
