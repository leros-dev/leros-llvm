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
  let EncoderMethod = "getImmOpValue";
  let ParserMatchClass = SImmAsmOperand<5>;
  let DecoderMethod = "decodeSImmOperand<8>";
}

// Byte extraction node transformation, used for sequentialising immediate
// generation of larger than 8-bit immediates using loadh instructions
class ImmByteExtracter<int byteN> : SDNodeXForm<imm, [{
  return CurDAG->getTargetConstant((N->getZExtValue() >> (byteN * 0xff)) & 0xff,
                                   SDLoc(N), N->getValueType(0));
}]>;

def BYTE0 : ImmByteExtracter<0>;
def BYTE1 : ImmByteExtracter<1>;
def BYTE2 : ImmByteExtracter<2>;
def BYTE3 : ImmByteExtracter<3>;

def simm32     : ImmLeaf<XLenVT, [{return isInt<32>(Imm);}]>;


//===----------------------------------------------------------------------===//
// Instructions
//===----------------------------------------------------------------------===//

// Test instructcion without accumulator
class ADD_INST : Instruction {
  field bits<16> Inst;
  bits<8> RegOrImm;

  let Namespace = "Leros";

  dag OutOperandList = (outs GPR:$dst);
  dag InOperandList = (ins GPR:$src1, GPR:$src2);
  let AsmString = "add $src1 $src2";
  let Pattern = [(set i32:$dst, (add i32:$src1, i32:$src2))];
}

def ADD_INST_IMPL : ADD_INST;


// Instruction class definitions
let Constraints = "$acci = $acco", Uses=[ACC] in {
    // Instructions with dependency: Acc -> (Acc, Imm)
    class INSTR_AccAccImm<LerosImmRegOpcode opcode, string opcodestr, SDNode opnode>
        : LerosImmInst<opcode, (outs ACC_OP:$acco), (ins ACC_OP:$acci, simm8:$imm8), 
            opcodestr, "$imm8", [(set ACC_OP:$acco, (opnode ACC_OP:$acci, simm8:$imm8))]>;

    // Instructions with dependency: Acc -> (Acc, Reg)
    class INSTR_AccAccReg<LerosImmRegOpcode opcode, string opcodestr, SDNode opnode>
        : LerosImmInst<opcode, (outs ACC_OP:$acco), (ins ACC_OP:$acci, GPR:$r),
            opcodestr, "$r", [(set ACC_OP:$acco, (opnode ACC_OP:$acci, i32:$r))]>;
}

    // Instructions with dependency: Reg -> (Acc)
    class INSTR_RegAcc<LerosImmRegOpcode opcode, string opcodestr, SDNode opnode>
        : LerosImmInst<opcode, (outs GPR:$r), (ins ACC_OP:$acc),
            opcodestr, "$r", [(set i32:$r, (opnode i32:$acc))]>;
    
    // Instructions with dependency: Acc -> (reg)
    class INSTR_AccReg<LerosImmRegOpcode opcode, string opcodestr, SDNode opnode>
        : LerosImmInst<opcode, (outs ACC_OP:$acc), (ins GPR:$r),
            opcodestr, "$r", [(set i32:$acc, (opnode i32:$r))]>;
    
    // Instructions with dependency: Acc -> (imm)
    class INSTR_AccImm<LerosImmRegOpcode opcode, string opcodestr, SDNode opnode>
        : LerosImmInst<opcode, (outs ACC_OP:$acc), (ins simm8:$imm8),
            opcodestr, "$imm8", [(set i32:$acc, (opnode i32:$imm8))]>;

    multiclass INSTR_ImmReg<LerosImmRegOpcode opcode, string opcodestr, SDNode opnode>{
        def _I  : INSTR_AccAccImm<opcode, opcodestr, opnode>;
        def _R  : INSTR_AccAccReg<opcode, opcodestr, opnode>; 
}


class INSTR_BranchCond<LerosBOpcode opcode, string opcodestr, PatFrag CondOp>
    : LerosBranchInst<opcode, (outs), (ins ACC_OP:$acc, uimm8:$imm8), opcodestr, "$imm8",
    [(brcond (i32 (CondOp i32:$acc, uimm8:$imm8)))]>{
}

// Instruction declarations
// defm : INSTR_ImmReg<OPC_NOP     , "INSTR_NOP",    "nop",    nop>;
defm INSTR_ADD  : INSTR_ImmReg<OPC_ADD     ,    "add",    add>;
defm INSTR_SUB  : INSTR_ImmReg<OPC_SUB     ,    "sub",    sub>;
defm INSTR_SHR  : INSTR_ImmReg<OPC_SHR     ,    "shr",    srl>;
defm INSTR_AND  : INSTR_ImmReg<OPC_AND     ,    "and",    and>;
defm INSTR_OR   : INSTR_ImmReg<OPC_OR      ,    "or",     or>;
defm INSTR_XOR  : INSTR_ImmReg<OPC_XOR     ,    "xor",    xor>;


// Loads
def INSTR_LOAD_R : INSTR_AccReg<OPC_LOAD, "load", ld>;
def INSTR_LOAD_I : INSTR_AccImm<OPC_LOAD, "load", ld>;

defm INSTR_LOADH    : INSTR_ImmReg<OPC_LOADH    ,  "loadh",   LOADH>;
defm INSTR_LOADH2   : INSTR_ImmReg<OPC_LOADH2   ,  "loadh2",  LOADH2>;
defm  INSTR_LOADH3  : INSTR_ImmReg<OPC_LOADH3   ,  "loadh3",  LOADH3>;

// defm : INSTR_ImmReg<OPC_OUT     , "INSTR_OUT",    "out",    out>;
// defm : INSTR_ImmReg<OPC_IN      , "INSTR_IN",     "in",     in>;

// def STORE   : INSTR_RegAcc<OPC_STORE, "store", store>;

// def BR      : INSTR_BranchCond<OPC_BR,      "br",       BEQZ>; // unconditional branch
//def BRZ     : INSTR_BranchCond<OPC_BRZ,     "brz",      seteq>;
//def BRNZ    : INSTR_BranchCond<OPC_BRNZ,    "brnz",     brnz>;
//def BRP     : INSTR_BranchCond<OPC_BRP,     "brp",      brp>;
//  def BRN     : INSTR_BranchCond<OPC_BRN,     "brn",      brn>;

// Instructions where format is specific to the instruction

def INSTR_LDADDR  : LerosInst<(outs GPR:$r), (ins ACC_OP:$acc), "ldaddr", "$r", []>{
    let Inst{15-11} = OPC_LDADDR.Value;
}

def INSTR_LDIND : LerosInst<(outs ACC_OP:$acc), (ins GPR:$r), "ldind", "$r", []>{
    let Inst{15-11} = OPC_LDIND.Value;
}

def INSTR_STIND : LerosInst<(outs GPR:$r), (ins ACC_OP:$acc), "stind", "$r", []>{
    let Inst{15-11} = OPC_STIND.Value;
}

def INSTR_JAL     : LerosInst<(outs GPR:$r), (ins ACC_OP:$acc), "jal", "$r", []>{
    let Inst{15-11} = OPC_JAL.Value;
}

//===----------------------------------------------------------------------===//
// Pseudo Instructions
//===----------------------------------------------------------------------===//

// Function return using jal
let isReturn=1, isTerminator=1, isBarrier=1, Uses=[R0] in {
  def INSTR_RET : LerosPseudo<(outs), (ins), "jal", " r0", [(LerosRet)]>;
}

// Register copy instruction
let Uses=[ACC] in
    def INSTR_MOV : LerosInst<(outs GPR:$rd), (ins GPR:$rs), "mov","$rd $rs",
        [(set GPR:$rd, (LerosMov GPR:$rs))]>; 

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