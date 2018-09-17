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

// type is: Acc -> (Acc, imm)
def SDT_LOADH   :   SDTypeProfile<1, 2, [SDTCisVT<0, i32>, SDTCisSameAs<0, 1>,
                                      SDTCisInt<2>]>;

def LerosRet :      SDNode<"LEROSISD::Ret", SDTNone,
                          [SDNPHasChain, SDNPOptInGlue, SDNPVariadic]>;
def LerosMov :      SDNode<"LEROSISD::Mov", SDTIntUnaryOp>;

def LOADH   : SDNode<"LEROSISD::LOADH", SDT_LOADH>;
def LOADH2  : SDNode<"LEROSISD::LOADH2", SDT_LOADH>;
def LOADH3  : SDNode<"LEROSISD::LOADH3", SDT_LOADH>;

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
  let EncoderMethod = "getImmOpValue";
  let ParserMatchClass = UImmAsmOperand<5>;
  let DecoderMethod = "decodeUImmOperand<8>";
}

def simm8 : Operand<XLenVT>, ImmLeaf<XLenVT, [{return isInt<8>(Imm);}]> {
  let ParserMatchClass = SImmAsmOperand<5>;
  let EncoderMethod = "getImmOpValue";
  let DecoderMethod = "decodeSImmOperand<8>";
}

def bb_simm8 : Operand<OtherVT> {
  let ParserMatchClass = SImmAsmOperand<5>;
  let EncoderMethod = "getImmOpValue";
  let DecoderMethod = "decodeSImmOperand<8>";
}

// Standalone (codegen-only) immleaf patterns - used for loading constants greater than
// 8 bit into the accumulator. Order is important - we want to reduce the number of loads
// to what actually represents the immediate
def simm16 : ImmLeaf<XLenVT, [{return isInt<16>(Imm);}]>;
def simm24 : ImmLeaf<XLenVT, [{return isInt<24>(Imm);}]>;
def simm32 : ImmLeaf<XLenVT, [{return isInt<32>(Imm);}]>;

// Byte extraction node transformation, used for sequentialising immediate
// generation of larger than 8-bit immediates using loadh instructions
def BYTE0 : SDNodeXForm<imm, [{
  return CurDAG->getTargetConstant((N->getZExtValue() >> (0 * 8)) & 0xff,
                                   SDLoc(N), N->getValueType(0));}]>;
def BYTE1 : SDNodeXForm<imm, [{
  return CurDAG->getTargetConstant((N->getZExtValue() >> (1 * 8)) & 0xff,
                                   SDLoc(N), N->getValueType(0));}]>;
def BYTE2 : SDNodeXForm<imm, [{
  return CurDAG->getTargetConstant((N->getZExtValue() >> (2 * 8)) & 0xff,
                                   SDLoc(N), N->getValueType(0));}]>;
def BYTE3 : SDNodeXForm<imm, [{
  return CurDAG->getTargetConstant((N->getZExtValue() >> (3 * 8)) & 0xff,
                                   SDLoc(N), N->getValueType(0));}]>;

//===----------------------------------------------------------------------===//
// Instructions
//===----------------------------------------------------------------------===//

let Uses=[ACC] in {
    // Instructions with dependency: Acc -> (Acc, Reg)
    class INSTR_AccAccReg<LerosImmRegOpcode opcode, string opcodestr, SDNode opnode>
        : LerosImmInst<opcode, (outs), (ins GPR:$r),
            opcodestr, "$r", []>;

    // Instructions with dependency: Acc -> (Acc, Imm)
    class INSTR_AccAccImm<LerosImmRegOpcode opcode, string opcodestr, SDNode opnode>
        : LerosImmInst<opcode, (outs), (ins simm8:$imm8), 
            opcodestr, "$imm8", []>;

    // Instructions with dependency: Reg -> (Acc)
    class INSTR_RegAcc<LerosImmRegOpcode opcode, string opcodestr>
        : LerosImmInst<opcode, (outs GPR:$r), (ins),
            opcodestr, "$r", []>;
    
    // Instructions with dependency: Acc -> (reg)
    class INSTR_AccReg<LerosImmRegOpcode opcode, string opcodestr, SDNode opnode>
        : LerosImmInst<opcode, (outs), (ins GPR:$r),
            opcodestr, "$r", []>;
    
    // Instructions with dependency: Acc -> (imm)
    class INSTR_AccImm<LerosImmRegOpcode opcode, string opcodestr, SDNode opnode>
        : LerosImmInst<opcode, (outs), (ins simm8:$imm8),
            opcodestr, "$imm8", []>;

    // Pseudo instruction wrapper for an Acc -> (Acc, Reg) instruction
    class PSEUDO_AccAccReg<SDNode opnode> : LerosPseudo<(outs GPR:$rd), (ins GPR:$r1,   GPR:$r2), F_RRR, [(set i32:$rd, (opnode i32:$r1, i32:$r2))]>;

    // Pseudo instruction wrapper for an Reg -> (imm) instruction
    class PSEUDO_RegImm : LerosPseudo<(outs GPR:$rd), (ins simm8:$imm), F_RI, []>;

    // Pseudo instruction wrapper for an Acc -> (Acc, Reg) instruction
    class PSEUDO_AccAccImm<SDNode opnode> : LerosPseudo<(outs GPR:$rd), (ins GPR:$r1,   uimm8:$uimm8), F_RRI, [(set i32:$rd, (opnode i32:$r1, uimm8:$uimm8))]>;

    class INSTR_BranchCond<LerosBOpcode opcode, string opcodestr>
        : LerosBranchInst<opcode, (outs), (ins bb_simm8:$simm8), opcodestr, "$simm8", []>;

    // Pseudo instruction wrapper for instruction format: brcond rs1 rs2 imm
    class Pseudo_Branch : LerosPseudo<(outs), (ins GPR:$r1, GPR:$r2, bb_simm8:$simm8), F_BR, []> {
        let isBranch = 1;
        let isTerminator = 1;
    }
}

multiclass INSTR_ImmReg<LerosImmRegOpcode opcode, string opcodestr, SDNode opnode>{
    // Wrapper pseudo instruction definitions
    def _RR_PSEUDO  : PSEUDO_AccAccReg<opnode>; // Register-register instruction
    def _RI_PSEUDO  : PSEUDO_AccAccImm<opnode>; // Register-immediate instruction

    // Instruction definitions
    def _AI  : INSTR_AccAccImm<opcode, opcodestr, opnode>;
    def _AR  : INSTR_AccAccReg<opcode, opcodestr, opnode>; 
}

multiclass INSTR_BrImpl<LerosBOpcode opcode, string opcodestr>{
    def _PSEUDO : Pseudo_Branch;
    def  _IMPL     : INSTR_BranchCond<opcode, opcodestr>;
}

// Instruction declarations
// defm : INSTR_ImmReg<OPC_NOP     , "INSTR_NOP",    "nop",    nop>;
defm ADD  : INSTR_ImmReg<OPC_ADD     ,    "add",    add>;
defm SUB  : INSTR_ImmReg<OPC_SUB     ,    "sub",    sub>;
defm SHR  : INSTR_ImmReg<OPC_SHR     ,    "shr",    srl>;
defm AND  : INSTR_ImmReg<OPC_AND     ,    "and",    and>;
defm OR   : INSTR_ImmReg<OPC_OR      ,    "or",     or>;
defm XOR  : INSTR_ImmReg<OPC_XOR     ,    "xor",    xor>;

// Loads
def LOAD_R : INSTR_AccReg<OPC_LOAD, "load", ld>;
def LOAD_I : INSTR_AccImm<OPC_LOAD, "load", ld>;
def LOAD_RI_PSEUDO : PSEUDO_RegImm;

defm LOADH    : INSTR_ImmReg<OPC_LOADH    ,  "loadh",   LOADH>;
defm LOADH2   : INSTR_ImmReg<OPC_LOADH2   ,  "loadh2",  LOADH2>;
defm LOADH3   : INSTR_ImmReg<OPC_LOADH3   ,  "loadh3",  LOADH3>;

// defm : INSTR_ImmReg<OPC_OUT     , "INSTR_OUT",    "out",    out>;
// defm : INSTR_ImmReg<OPC_IN      , "INSTR_IN",     "in",     in>;

def STORE   : INSTR_RegAcc<OPC_STORE, "store">;

defm BR      : INSTR_BrImpl<OPC_BR,      "br"       >; // unconditional branch
defm BRZ     : INSTR_BrImpl<OPC_BRZ,     "brz"      >;
defm BRNZ    : INSTR_BrImpl<OPC_BRNZ,    "brnz"     >;
defm BRP     : INSTR_BrImpl<OPC_BRP,     "brp"      >;
defm BRN     : INSTR_BrImpl<OPC_BRN,     "brn"     >;

// Unconditional branch is defined as a separate instruction - needed for
// small jumps
let isBarrier = 1, isBranch = 1, isTerminator = 1 in
def PseudoBR : LerosPseudo<(outs), (ins bb_simm8:$simm8), F_NA, [(br bb:$simm8)]>,
               PseudoInstExpansion<(BR_IMPL bb_simm8:$simm8)>;

// Branch instruction where branch condition ($rc) has been precalculated
class Pseudo_BranchCC : LerosPseudo<(outs), (ins GPR:$rc, bb_simm8:$simm8), F_BR, []> {
    let isBranch = 1;
    let isTerminator = 1;
    let Uses = [ACC];
}
def PseudoBRC : Pseudo_BranchCC;
            

// Instructions where format is specific to the instruction
let Uses=[ACC] in {
    def LDADDR  : LerosInst<(outs GPR:$r), (ins), "ldaddr", "$r", []>{
        let Inst{15-11} = OPC_LDADDR.Value;
    }

    def LDIND : LerosInst<(outs), (ins GPR:$r), "ldind", "$r", []>{
        let Inst{15-11} = OPC_LDIND.Value;
    }

    def STIND : LerosInst<(outs GPR:$r), (ins), "stind", "$r", []>{
        let Inst{15-11} = OPC_STIND.Value;
    }

    def JAL     : LerosInst<(outs GPR:$r), (ins), "jal", "$r", []>{
        let Inst{15-11} = OPC_JAL.Value;
    }
}

//===----------------------------------------------------------------------===//
// Pseudo Instructions
//===----------------------------------------------------------------------===//

// Function return using jal
let isReturn=1, isTerminator=1, isBarrier=1, Uses=[R0, ACC] in {
  def RET : LerosPseudo<(outs), (ins), F_NA, [(LerosRet)]>;
}

// Register copy instruction
let Uses=[ACC] in
    def MOV : LerosInst<(outs GPR:$rd), (ins GPR:$rs), "mov","$rd $rs",
        [(set GPR:$rd, (LerosMov GPR:$rs))]>; 

//===----------------------------------------------------------------------===//
// Immediate patterns
//===----------------------------------------------------------------------===//

def : Pat<(simm8:$imm), 
        (LOAD_RI_PSEUDO imm:$imm)>;

def : Pat<(simm16:$imm), 
    (LOADH_RI_PSEUDO 
        (LOAD_RI_PSEUDO (BYTE0 imm:$imm)), 
    (BYTE1 imm:$imm))>;

def : Pat<(simm24:$imm), 
    (LOADH2_RI_PSEUDO
        (LOADH_RI_PSEUDO 
            (LOAD_RI_PSEUDO (BYTE0 imm:$imm)), 
        (BYTE1 imm:$imm)), 
    (BYTE2 imm:$imm))>;

def : Pat<(simm32:$imm),
    (LOADH3_RI_PSEUDO
        (LOADH2_RI_PSEUDO
            (LOADH_RI_PSEUDO 
                (LOAD_RI_PSEUDO (BYTE0 imm:$imm)), 
            (BYTE1 imm:$imm)), 
        (BYTE2 imm:$imm)),
    (BYTE3 imm:$imm))>;

//===----------------------------------------------------------------------===//
// Codegen patterns
//===----------------------------------------------------------------------===//
def : Pat<(seteq GPR:$rs1, GPR:$rs2), (SUB_RR_PSEUDO GPR:$rs1, GPR:$rs2)>;
def : Pat<(setne GPR:$rs1, GPR:$rs2), (SUB_RR_PSEUDO GPR:$rs1, GPR:$rs2)>;
def : Pat<(setgt GPR:$rs1, GPR:$rs2), (SUB_RR_PSEUDO GPR:$rs1, GPR:$rs2)>;
def : Pat<(setlt GPR:$rs1, GPR:$rs2), (SUB_RR_PSEUDO GPR:$rs1, GPR:$rs2)>;


class BccPat<PatFrag CondOp, Pseudo_Branch Inst>
    : Pat<(brcond (i32 (CondOp GPR:$rs1, GPR:$rs2)), bb:$imm8),
          (Inst GPR:$rs1, GPR:$rs2, bb_simm8:$imm8)>;

def : BccPat<seteq, BRZ_PSEUDO>;
def : BccPat<setne, BRNZ_PSEUDO>;
def : BccPat<setgt, BRP_PSEUDO>;
def : BccPat<setlt, BRN_PSEUDO>;

// An extra pattern is needed for a brcond without a setcc (i.e. where the
// condition was calculated elsewhere).
def : Pat<(brcond GPR:$cond, bb:$imm8), (PseudoBRC GPR:$cond, bb:$imm8)>;


// 'icmp' instruction