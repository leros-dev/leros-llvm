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

def SDT_lerosCall         : SDTypeProfile<0, -1, [SDTCisVT<0, XLenVT>]>;
def SDT_LerosCallSeqStart : SDCallSeqStart<[SDTCisVT<0, i32>, SDTCisVT<1, i32>]>;
def SDT_LerosCallSeqEnd   : SDCallSeqEnd<[SDTCisVT<0, i32>, SDTCisVT<1, i32>]>;

def SDT_LerosSelectCC     : SDTypeProfile<1, 3,  [SDTCisSameAs<0, 1>,
                                                 SDTCisSameAs<1, 2>]>;

def Callseq_start : SDNode<"ISD::CALLSEQ_START", SDT_LerosCallSeqStart,
                           [SDNPHasChain, SDNPSideEffect, SDNPOutGlue]>;
def Callseq_end   : SDNode<"ISD::CALLSEQ_END", SDT_LerosCallSeqEnd,
                           [SDNPHasChain, SDNPSideEffect,
                            SDNPOptInGlue, SDNPOutGlue]>;
def Call         : SDNode<"LEROSISD::Call", SDT_lerosCall,
                          [SDNPHasChain, SDNPOptInGlue, SDNPOutGlue,
                           SDNPVariadic]>;


// type is: Acc -> (Acc, imm)
def SDT_LOADH   :   SDTypeProfile<1, 2, [SDTCisVT<0, i32>, SDTCisSameAs<0, 1>,
                                      SDTCisInt<2>]>;

def LerosRet :      SDNode<"LEROSISD::Ret", SDTNone,
                          [SDNPHasChain, SDNPOptInGlue, SDNPVariadic]>;
def LerosMov :      SDNode<"LEROSISD::Mov", SDTIntUnaryOp>;

def LOADH   : SDNode<"LEROSISD::LOADH", SDT_LOADH>;
def LOADH2  : SDNode<"LEROSISD::LOADH2", SDT_LOADH>;
def LOADH3  : SDNode<"LEROSISD::LOADH3", SDT_LOADH>;

def SelectCC     : SDNode<"LEROSISD::SELECT_CC", SDT_LerosSelectCC,
                          [SDNPInGlue]>;

//===----------------------------------------------------------------------===//
// Operands
//===----------------------------------------------------------------------===//

class ImmXLenAsmOperand<string prefix, string suffix = ""> : AsmOperandClass {
  let Name = prefix # "ImmXLen" # suffix;
  let RenderMethod = "addImmOperands";
  let DiagnosticType = !strconcat("Invalid", Name);
}

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
  let ParserMatchClass = UImmAsmOperand<8>;
  let DecoderMethod = "decodeUImmOperand<8>";
}

def simm8 : Operand<XLenVT>, ImmLeaf<XLenVT, [{return isInt<8>(Imm);}]> {
  let ParserMatchClass = SImmAsmOperand<8>;
  let EncoderMethod = "getImmOpValue";
  let DecoderMethod = "decodeSImmOperand<8>";
  let MCOperandPredicate = [{
    int64_t Imm;
    if (MCOp.evaluateAsConstantImm(Imm))
      return isInt<8>(Imm);
    return MCOp.isBareSymbolRef();
  }];
}


// Boolean operations are only able to safely parse immediates that can be expressed
// as 7-bit immediates. If they immediate is in the 8th bit, we risk sign extension and
// thus expressing the wrong operation. ie. 'and -1' for bitmasking the lower byte is incorrect,
// because -1 will sign extend. 
def uimm7 : Operand<XLenVT>, ImmLeaf<XLenVT, [{return isUInt<7>(Imm);}]> {
  let ParserMatchClass = UImmAsmOperand<7>;
  let EncoderMethod = "getImmOpValue";
  let DecoderMethod = "decodeUImmOperand<7>";
}

def bb_simm12 : Operand<OtherVT> {
  let ParserMatchClass = SImmAsmOperand<12>;
  let EncoderMethod = "getImmOpValueAsr1";
  let DecoderMethod = "decodeSImmOperandAndLsl1<12>";
  let MCOperandPredicate = [{
    int64_t Imm;
    if (MCOp.evaluateAsConstantImm(Imm))
      return isShiftedInt<11, 1>(Imm);
    return MCOp.isBareSymbolRef();
  }];
}

// Standalone (codegen-only) immleaf patterns - used for loading constants greater than
// 8 bit into the accumulator. Order is important - we want to reduce the number of loads
// to what actually represents the immediate
def simm16 : ImmLeaf<XLenVT, [{return isInt<16>(Imm);}]>;
def simm24 : ImmLeaf<XLenVT, [{return isInt<24>(Imm);}]>;
def simm32 : ImmLeaf<XLenVT, [{return isInt<32>(Imm);}]>;

// Symbol operands for calls
def BareSymbol : AsmOperandClass {
  let Name = "BareSymbol";
  let RenderMethod = "addImmOperands";
  let DiagnosticType = "InvalidBareSymbol";
}

// A bare symbol.
def bare_symbol : Operand<XLenVT> {
  let ParserMatchClass = BareSymbol;
  let MCOperandPredicate = [{
     return MCOp.isBareSymbolRef();
  }];
}

// A parameterized register class alternative to i32imm/i64imm from Target.td.
def ixlenimm : Operand<XLenVT> {
  let ParserMatchClass = ImmXLenAsmOperand<"">;
}

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
    // Instructions with dependency: Acc -> (Acc, Reg)
    class INSTR_AccAccReg<LerosImmRegOpcode opcode, string opcodestr>
        : LerosRegInst<opcode, (outs), (ins GPR:$r),
            opcodestr, "$r", []>{
                let Inst{7-0} = r;
            }

    // Instructions with dependency: Acc -> (Acc, Imm)
    class INSTR_AccAccImm<LerosImmRegOpcode opcode, string opcodestr>
        : LerosImmInst<opcode, (outs), (ins simm8:$imm8), 
            opcodestr, "$imm8", []>{
                let Inst{7-0} = imm8;
            }

    // Instructions with dependency: Reg -> (Acc)
    class INSTR_RegAcc<LerosImmRegOpcode opcode, string opcodestr>
        : LerosRegInst<opcode, (outs GPR:$r), (ins),
            opcodestr, "$r", []>{
                let Inst{7-0} = r;
            }
    
    // Instructions with dependency: Acc -> (reg)
    class INSTR_AccReg<LerosImmRegOpcode opcode, string opcodestr>
        : LerosRegInst<opcode, (outs), (ins GPR:$r),
            opcodestr, "$r", []>{
                let RegOrImm{7-0} = r;
            }
    
    // Instructions with dependency: Acc -> (imm)
    class INSTR_AccImm<LerosImmRegOpcode opcode, string opcodestr>
        : LerosImmInst<opcode, (outs), (ins simm8:$imm8),
            opcodestr, "$imm8", []>{
                let Inst{7-0} = imm8;
            }

    class INSTR_BranchCond<LerosBOpcode opcode, string opcodestr>
        : LerosBranchInst<opcode, (outs), (ins bb_simm12:$imm11), opcodestr, "$imm11", []>{
                let Inst{10-0} = imm11;
            }

// ------------- Pseudo ISA wrapper classes -------------
// Pseudo instruction wrapper for instruction format: brcond rs1 rs2 imm
class Pseudo_Branch : LerosPseudo<(outs), (ins GPR:$r1, GPR:$r2, bb_simm12:$imm11), F_BRCC, []> {
    let isBranch = 1;
    let isTerminator = 1;
}

// Pseudo instruction wrapper for an Acc -> (Acc, Reg) instruction
class PSEUDO_AccAccImm<SDNode opnode, Operand ImmClass> : LerosPseudo<(outs GPR:$rd), (ins GPR:$r1,
   ImmClass:$simm), F_RRI, [(set i32:$rd, (opnode i32:$r1, ImmClass:$simm))]>;

    // Pseudo instruction wrapper for memory access functions
let hasSideEffects = 0, mayLoad = 0, mayStore = 1 in
class PSEUDO_Store_M : LerosPseudo<(outs), (ins GPR:$rs2, GPR:$rs1, simm8:$imm8), F_LS, []>;
let hasSideEffects = 0, mayLoad = 1, mayStore = 0 in
class PSEUDO_Load_M<PseudoFormat format>  : LerosPseudo<(outs GPR:$rs2), (ins GPR:$rs1, simm8:$imm8), format, []>;

// Pseudo instruction wrapper for an Acc -> (Acc, Reg) i    nstruction
class PSEUDO_AccAccReg<SDNode opnode> : LerosPseudo<(outs GPR:$rd), (ins GPR:$r1,
   GPR:$r2), F_RRR, [(set i32:$rd, (opnode i32:$r1, i32:$r2))]>;

// Pseudo instruction wrapper for an Reg -> (imm) instruction
class PSEUDO_RegImm : LerosPseudo<(outs GPR:$rd), (ins simm8:$imm), F_RI, []>;


multiclass INSTR_ImmReg<LerosImmRegOpcode opcode, string opcodestr, SDNode opnode, Operand immClass>{
    // Wrapper pseudo instruction definitions
    def _RI_PSEUDO  : PSEUDO_AccAccImm<opnode, immClass>; // Register-immediate instruction
    def _RR_PSEUDO  : PSEUDO_AccAccReg<opnode>; // Register-register instruction

    // Instruction definitions
    def _AI  : INSTR_AccAccImm<opcode, opcodestr>;
    def _AR  : INSTR_AccAccReg<opcode, opcodestr>; 
}

multiclass INSTR_Pseudo<LerosImmRegOpcode opcode, SDNode opnode, Operand immClass>{
    let usesCustomInserter = 1 in {
        def _RI_PSEUDO  : PSEUDO_AccAccImm<opnode, immClass>; // Register-immediate instruction
        def _RR_PSEUDO  : PSEUDO_AccAccReg<opnode>; // Register-register instruction
    }
}

multiclass INSTR_BrImpl<LerosBOpcode opcode, string opcodestr>{
    let isTerminator=1 in {
    def _PSEUDO : Pseudo_Branch;
    def  _IMPL     : INSTR_BranchCond<opcode, opcodestr>;
    }
}

// Instruction declarations
let isReMaterializable = 1 in
defm ADD  : INSTR_ImmReg<OPC_ADD     ,    "add",    add,    simm8>;

defm SUB  : INSTR_ImmReg<OPC_SUB     ,    "sub",    sub,    simm8>;

defm AND  : INSTR_ImmReg<OPC_AND     ,    "and",    and,    uimm7>;
defm OR   : INSTR_ImmReg<OPC_OR      ,    "or",     or,     uimm7>;
defm XOR  : INSTR_ImmReg<OPC_XOR     ,    "xor",    xor,    uimm7>;

def LDADDR : INSTR_AccAccReg<OPC_LDADDR, "ldaddr">;

let hasSideEffects = 0, mayLoad = 1, mayStore = 0 in
def LDIND : INSTR_AccAccImm<OPC_LDIND, "ldind">;

let hasSideEffects = 0, mayLoad = 0, mayStore = 1 in
def STIND : INSTR_AccAccImm<OPC_STIND, "stind">;

// Shifts are all pseudo instructions since they require custom emmission. 
// shr has an actual leros instruction, whereas shl and sra are emissions of small algorithms
def SHRByOne_Pseudo: LerosPseudo<(outs GPR:$rd), (ins GPR:$rs), F_NA, []>;
def SHR_IMPL  : LerosImmInst<OPC_SHR, (outs), (ins simm8:$imm8), 
            "shr", "", []>;

defm SRL  : INSTR_Pseudo<OPC_SHR     , srl,  simm8>; // This instruction will get expanded by EmitSRL
defm SHL  : INSTR_Pseudo<OPC_NOP     , shl,  simm8>; // This instruction will get expanded by EmitSHL
defm SRA  : INSTR_Pseudo<OPC_NOP     , sra,  simm8>; // This instruction will get expanded by EmitSRA

// Set pseudo-instructions will be emitted as custom branch sequences with phi nodes
let usesCustomInserter = 1 in {
    def SETEQ_PSEUDO    : LerosPseudo<(outs GPR:$rd), (ins GPR:$rs1, GPR:$rs2), F_NA, []>;
    def SETGE_PSEUDO   : LerosPseudo<(outs GPR:$rd), (ins GPR:$rs1, GPR:$rs2), F_NA, []>;
    def SETLT_PSEUDO    : LerosPseudo<(outs GPR:$rd), (ins GPR:$rs1, GPR:$rs2), F_NA, []>;
}

// Register->accumulator Loads
def LOAD_R : INSTR_AccReg<OPC_LOAD, "load">;
def LOAD_I : INSTR_AccImm<OPC_LOAD, "load">;
def LOAD_RI_PSEUDO : PSEUDO_RegImm;

defm LOADH    : INSTR_ImmReg<OPC_LOADH    ,  "loadh",   LOADH,  simm8>;
defm LOADH2   : INSTR_ImmReg<OPC_LOADH2   ,  "loadh2",  LOADH2,  simm8>;
defm LOADH3   : INSTR_ImmReg<OPC_LOADH3   ,  "loadh3",  LOADH3,  simm8>;

// Accumulator->register store
def STORE_R   : INSTR_RegAcc<OPC_STORE, "store">;

let isBarrier = 1, isBranch = 1, isTerminator = 1 in
def  BR_IMPL : INSTR_BranchCond<OPC_BR,  "br"       >; // unconditional branch, pseudo version is handled by PseudoBR
defm BRZ     : INSTR_BrImpl<OPC_BRZ,     "brz"      >;
defm BRNZ    : INSTR_BrImpl<OPC_BRNZ,    "brnz"     >;
defm BRP     : INSTR_BrImpl<OPC_BRP,     "brp"      >;
defm BRN     : INSTR_BrImpl<OPC_BRN,     "brn"     >;

// Memory access pseudoinstruction definitions
def STORE_M_PSEUDO      : PSEUDO_Store_M;
def LOAD_M_PSEUDO       : PSEUDO_Load_M<F_LS>;
// Sign-extending loads use custom inserter for the sign extension mechanism, see emitInstrUsingCustomInserter
let usesCustomInserter=1 in {
    def LOAD_S8_M_PSEUDO    : PSEUDO_Load_M<F_S8L>;
    def LOAD_S16_M_PSEUDO   : PSEUDO_Load_M<F_S16L>;
}
// Unsigned 8- and 16 bit loads just sets topmost bits to 0 (LerosInstrInfo::expandLS)
def LOAD_U8_M_PSEUDO    : PSEUDO_Load_M<F_U8L>;
def LOAD_U16_M_PSEUDO   : PSEUDO_Load_M<F_U8L>;

// NOP instruction. Pseudo NOP can be used without any arguments, later expanded to NOP_IMPL.
// NOP_IMPL contains a register argument because llvm complains about instructions with no in/out nodes
def  NOP   : LerosPseudo<(outs ), (ins), F_NA, []>;
def  NOP_IMPL   : LerosInst<(outs), (ins simm8:$imm), "nop", "", []>{
        let Inst{15-9} = OPC_NOP.Value;
        let Inst{8} = 0;
        let Inst{7-0} = 0;
    }


// Unconditional branch is defined as a separate instruction - needed for
// small jumps
def : Pat<(br bb:$imm11), (BR_IMPL bb_simm12:$imm11)>;
let isBarrier = 1, isBranch = 1, isTerminator = 1 in
def PseudoBR : LerosPseudo<(outs), (ins bb_simm12:$imm11), F_BR, [(br bb:$imm11)]>,
               PseudoInstExpansion<(BR_IMPL bb_simm12:$imm11)>;

// Branch instruction where branch condition ($rc) has been precalculated
class Pseudo_BranchCC : LerosPseudo<(outs), (ins GPR:$rc, bb_simm12:$imm11), F_NA, []> {
    let isBranch = 1;
    let isTerminator = 1;
}
def PseudoBRNZ : Pseudo_BranchCC;
def PseudoBRZ : Pseudo_BranchCC;
def PseudoBRP : Pseudo_BranchCC;
def PseudoBRN : Pseudo_BranchCC;

def PseudoBRIND : LerosPseudo<(outs), (ins GPR:$rd), F_BR, [(brind GPR:$rd)]>;


// Instructions where format is specific to the instruction 
    let isCall = 1, Defs=[RA] in
    def JAL_call     : LerosInst<(outs GPR:$r), (ins), "jal", "$r", []>{
        bits<8> r;
        let Inst{7-0} = r;
        let Inst{15-11} = OPC_JAL.Value;
        let Inst{10-8} = 0;
        
    }

    let isReturn=1, isTerminator=1, isBarrier=1, Uses=[RA], isCodeGenOnly=1 in
    def JAL_ret     : LerosInst<(outs GPR:$r), (ins), "jal", "$r", []>{
        bits<8> r;
        let Inst{7-0} = r;
        let Inst{15-11} = OPC_JAL.Value;
        let Inst{10-8} = 0;
    }

//===----------------------------------------------------------------------===//
// Pseudo Instructions
//===----------------------------------------------------------------------===//

// Function return using jal
let isReturn=1, isTerminator=1, isBarrier=1, Uses=[RA] in
def RET : LerosPseudo<(outs), (ins), F_NA, [(LerosRet)]>;

// Will later be expanded to a sequence of immediate loads and a jal instruction
let isCall = 1, Defs = [RA], isCodeGenOnly = 0 in
def PseudoCALL : LerosPseudo<(outs), (ins bare_symbol:$func), F_NA,
                        [(Call tglobaladdr:$func)]> {
    let AsmString = "call\t$func";
}

def : Pat<(Call texternalsym:$func), (PseudoCALL texternalsym:$func)>;

// Register copy instruction
def MOV : LerosPseudo<(outs GPR:$rd), (ins GPR:$rs), F_NA,
    [(set GPR:$rd, (LerosMov GPR:$rs))]>; 

/// Predicates

def IsOrAdd: PatFrag<(ops node:$A, node:$B), (or node:$A, node:$B), [{
  return isOrEquivalentToAdd(N);
}]>;

//===----------------------------------------------------------------------===//
// Immediate patterns
//===----------------------------------------------------------------------===//

// We want to be able to capture immediate loading patterns through the target
// description files. The following sequence minimizes the required loads and stores
// which are selected for an immediate
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

// Todo: Extend for 64-bit

//===----------------------------------------------------------------------===//
// Codegen patterns
//===----------------------------------------------------------------------===//   
 
//Todo: Implement these using a future set instruction, or expand into a phi node
// using branch logic for the set results


def : Pat<(seteq GPR:$rs1, GPR:$rs2), (SETEQ_PSEUDO GPR:$rs1, GPR:$rs2)>;
def : Pat<(setne GPR:$rs1, GPR:$rs2), (XOR_RI_PSEUDO (SETEQ_PSEUDO GPR:$rs1, GPR:$rs2), 1)>;
def : Pat<(setgt GPR:$rs1, GPR:$rs2), (SETLT_PSEUDO GPR:$rs2, GPR:$rs1)>;
def : Pat<(setge GPR:$rs1, GPR:$rs2), (SETGE_PSEUDO GPR:$rs1, GPR:$rs2)>;
def : Pat<(setlt GPR:$rs1, GPR:$rs2), (SETLT_PSEUDO GPR:$rs1, GPR:$rs2)>;
def : Pat<(setle GPR:$rs1, GPR:$rs2), (SETGE_PSEUDO GPR:$rs2, GPR:$rs1)>;
def : Pat<(setugt GPR:$rs1, GPR:$rs2), (SETLT_PSEUDO GPR:$rs2, GPR:$rs1)>;
def : Pat<(setuge GPR:$rs1, GPR:$rs2), (SETGE_PSEUDO GPR:$rs1, GPR:$rs2)>;
def : Pat<(setult GPR:$rs1, GPR:$rs2), (SETLT_PSEUDO GPR:$rs1, GPR:$rs2)>;
def : Pat<(setule GPR:$rs1, GPR:$rs2), (SETGE_PSEUDO GPR:$rs2, GPR:$rs1)>;


class BccPat<PatFrag CondOp, Pseudo_Branch Inst>
    : Pat<(brcond (i32 (CondOp GPR:$rs1, GPR:$rs2)), bb:$imm11),
          (Inst GPR:$rs1, GPR:$rs2, bb_simm12:$imm11)>;

def : BccPat<seteq, BRZ_PSEUDO>;
def : BccPat<setne, BRNZ_PSEUDO>;
def : BccPat<setge, BRP_PSEUDO>;
def : BccPat<setlt, BRN_PSEUDO>;
def : BccPat<setuge, BRP_PSEUDO>;
def : BccPat<setult, BRN_PSEUDO>;

class BccSwapPat<PatFrag CondOp, Pseudo_Branch Inst>
    : Pat<(brcond (i32 (CondOp GPR:$rs1, GPR:$rs2)), bb:$imm8),
          (Inst GPR:$rs2, GPR:$rs1, bb:$imm8)>;

// Condition codes that don't have matching RISC-V branch instructions, but
// are trivially supported by swapping the two input operands
def : BccSwapPat<setgt, BRN_PSEUDO>;
def : BccSwapPat<setle, BRP_PSEUDO>;
def : BccSwapPat<setugt, BRN_PSEUDO>;
def : BccSwapPat<setule, BRP_PSEUDO>;

// An extra pattern is needed for a brcond without a setcc (i.e. where the
// condition was calculated elsewhere).
def : Pat<(brcond GPR:$cond, bb:$imm8), (PseudoBRNZ GPR:$cond, bb:$imm8)>;

// Addressing modes.
// Necessary because a frameindex can't be matched directly in a pattern.
def AddrFI : ComplexPattern<iPTR, 1, "SelectAddrFI", [frameindex], []>;

/// FrameIndex calculations
def : Pat<(add (i32 AddrFI:$Rs), simm8:$imm8),
          (ADD_RI_PSEUDO (i32 AddrFI:$Rs), simm8:$imm8)>;
def : Pat<(IsOrAdd (i32 AddrFI:$Rs), simm8:$imm8),
          (ADD_RI_PSEUDO (i32 AddrFI:$Rs), simm8:$imm8)>;

//  ----------- SELECT pseudo instruction  -----------
// will be expanded in EmitInstrWithCustomInserter
let usesCustomInserter = 1 in
class SelectCC_rrr<RegisterClass valty, RegisterClass cmpty>
    : LerosPseudo<(outs valty:$dst),
             (ins cmpty:$cc, valty:$truev, valty:$falsev), F_NA,
             [(set valty:$dst, (SelectCC cmpty:$cc, valty:$truev, valty:$falsev))]>;

def Select_GPR_Using_CC_GPR : SelectCC_rrr<GPR, GPR>;

//  ----------- Memory load/store patterns -----------

// Loads
multiclass LdPat<PatFrag LoadOp, PSEUDO_Load_M Inst> {
  def : Pat<(LoadOp (add GPR:$rs1, simm8:$imm8)),
            (Inst GPR:$rs1, simm8:$imm8)>;
  def : Pat<(LoadOp (add AddrFI:$rs1, simm8:$imm8)),
            (Inst AddrFI:$rs1, simm8:$imm8)>;
  def : Pat<(LoadOp GPR:$rs1), 
            (Inst GPR:$rs1, 0)>;
  def : Pat<(LoadOp AddrFI:$rs1), 
            (Inst AddrFI:$rs1, 0)>;
  def : Pat<(LoadOp (IsOrAdd AddrFI:$rs1, simm8:$imm8)),
            (Inst AddrFI:$rs1, simm8:$imm8)>;
}

defm : LdPat<load, LOAD_M_PSEUDO>;
defm : LdPat<load, LOAD_M_PSEUDO>;
defm : LdPat<sextloadi8, LOAD_S8_M_PSEUDO>;
defm : LdPat<extloadi8, LOAD_S8_M_PSEUDO>;
defm : LdPat<sextloadi16, LOAD_S16_M_PSEUDO>;
defm : LdPat<extloadi16, LOAD_S16_M_PSEUDO>;
defm : LdPat<zextloadi8, LOAD_U8_M_PSEUDO>;
defm : LdPat<zextloadi16, LOAD_U8_M_PSEUDO>;

// Stores
multiclass StPat<PatFrag StoreOp> {
  def : Pat<(StoreOp GPR:$rs2, GPR:$rs1), 
            (STORE_M_PSEUDO GPR:$rs2, GPR:$rs1, 0)>;
  def : Pat<(StoreOp GPR:$rs2, AddrFI:$rs1), 
            (STORE_M_PSEUDO GPR:$rs2, AddrFI:$rs1, 0)>;
  def : Pat<(StoreOp GPR:$rs2, (add GPR:$rs1, simm8:$imm8)),
            (STORE_M_PSEUDO GPR:$rs2, GPR:$rs1, simm8:$imm8)>;
  def : Pat<(StoreOp GPR:$rs2, (add AddrFI:$rs1, simm8:$imm8)),
            (STORE_M_PSEUDO GPR:$rs2, AddrFI:$rs1, simm8:$imm8)>;
  def : Pat<(StoreOp GPR:$rs2, (IsOrAdd AddrFI:$rs1, simm8:$imm8)),
            (STORE_M_PSEUDO GPR:$rs2, AddrFI:$rs1, simm8:$imm8)>;
}


defm : StPat<store>;
defm : StPat<truncstorei8>;
defm : StPat<truncstorei16>;


// Stack pointer adjustments
let Defs = [SP], Uses = [SP] in {
def ADJCALLSTACKDOWN : LerosPseudo<(outs), (ins i32imm:$amt1, i32imm:$amt2), F_NA,
                                  [(Callseq_start timm:$amt1, timm:$amt2)]>;
def ADJCALLSTACKUP   : LerosPseudo<(outs), (ins i32imm:$amt1, i32imm:$amt2), F_NA,
                                  [(Callseq_end timm:$amt1, timm:$amt2)]>;
}