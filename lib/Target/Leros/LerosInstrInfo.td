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
// Codegen patterns
//===----------------------------------------------------------------------===//

class PatReg<SDPatternOperator OpNode, LerosARInst Inst>
    : Pat<(OpNode GPR:$rs1, GPR:$rs2), (Inst GPR:$rs1, GPR:$rs2)>;

class PatAcc<SDPatternOperator OpNode, LerosARInst Inst>
    : Pat<(OpNode GPR:$rs1, GPR:$rs2), (Inst GPR:$rs1, GPR:$rs2)>;

//===----------------------------------------------------------------------===//
// Instructions
//===----------------------------------------------------------------------===//


// Instruction class definitions

// Instructions with dependency: Acc -> (Acc, Imm)
class INSTR_AccAccImm<LerosImmRegOpcode opcode, string opcodestr>
    : LerosImmInst<opcode, (outs ACC), (ins ACC, TODO: some immediate...), 
        opcodestr, "$imm8">;

// Instructions with dependency: Acc -> (Acc, Reg)
class INSTR_AccAccReg<LerosImmRegOpcode opcode, string opcodestr>
    : LerosImmInst<opcode, (outs ACC), (ins ACC, GPR:$r),
        opcodestr, "$r">;

// Instructions with dependency: Reg -> (Acc)
class INSTR_RegAcc<LerosImmRegOpcode opcode, string opcodestr>
    : LerosImmInst<opcode, (outs GPR:$r), (ins ACC),
        opcodestr, "$r">;


multiclass INSTR_ImmReg<LerosImmRegOpcode opcode, string TGName string opcodestr>{
    def TGName#_I  : INSTR_AccAccImm<opcode, opcodestr>;
    def TGName#_R  : INSTR_AccAccReg<opcode, opcodestr>; 
}

class INSTR_Branch<LerosBOpcode opcode, string opcodestr>{
    : LerosBranchInst<opcode, [], (ins TODO: some immediate...), opcodestr, "$imm8">;
}

class INSTR_IndLS<LerosBOpcode opcode, string opcodestr>{
    : LerosBranchInst<opcode, [], (outs ACC), (ins GPR:$r), opcodestr, "$r">;
}

class INSTR_Jal<LerosJalOpcode opcode, string opcodestr>{
    : LerosBranchInst<opcode, [], (outs GRP:$r), (ins GPR:$r), opcodestr, "$r">;
}


// Instruction declarations
defm INSTR_ImmReg<OPC_NOP     , "nop">;
defm INSTR_ImmReg<OPC_ADD     , "add">;
defm INSTR_ImmReg<OPC_SUB     , "sub">;
defm INSTR_ImmReg<OPC_SHR     , "shr">;
defm INSTR_ImmReg<OPC_UNUSED  , "">;
defm INSTR_ImmReg<OPC_LOAD    , "load">;
defm INSTR_ImmReg<OPC_AND     , "and">;
defm INSTR_ImmReg<OPC_OR      , "or">;
defm INSTR_ImmReg<OPC_XOR     , "xor">;
defm INSTR_ImmReg<OPC_LOADH   , "loadh">;
defm INSTR_ImmReg<OPC_OUT     , "out">;
defm INSTR_ImmReg<OPC_IN      , "in">;

def STORE   : INSTR_RegAcc<OPC_STORE, "store">;

def BR      : INSTR_Branch<OPC_BR, "br">;
def BRZ     : INSTR_Branch<OPC_BRZ, "brz">;
def BRNZ    : INSTR_Branch<OPC_BRNZ, "brnz">;
def BRP     : INSTR_Branch<OPC_BRP, "brp">;
def BRN     : INSTR_Branch<OPC_BRN, "brn">;

def LDADDR  : INSTR_IndLS<OPC_LDADDR,   "ldaddr">;
def LDIND   : INSTR_IndLS<OPC_LOADIND,  "ldind">;
def STIND   : INSTR_IndLS<OPC_STOREIND, "stind">;

// JAL
def OPC_JAL     : LerosJalOpcode<0b01000>;