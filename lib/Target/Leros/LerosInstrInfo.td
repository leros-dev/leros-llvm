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
                           [SDNPHasChain, SDNPOutGlue]>;
def Callseq_end   : SDNode<"ISD::CALLSEQ_END", SDT_LerosCallSeqEnd,
                           [SDNPHasChain,
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

// Special case signed 8-bit immediate that can match immediates through LLVM IR -> Leros
// or symbol references through Leros assembly. Used for Load operations that
// reference symbols. We create a separate operand type as to not allow all other
// immediate instructions to refer to symbols.
def simm8_symbolref : Operand<XLenVT>, ImmLeaf<XLenVT, [{return isInt<8>(Imm);}]> {
  let ParserMatchClass = SImmAsmOperand<8, "SymbolRef">;
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

// A 9 bit signed immediate where the lsb is zero
def simm9_lsb0 : Operand<XLenVT>, ImmLeaf<XLenVT, [{return isShiftedInt<8,1>(Imm);}]> {
  let ParserMatchClass = SImmAsmOperand<9, "Lsb0">;
  let EncoderMethod = "getImmOpValueAsr<1>";
  let DecoderMethod = "decodeSImmOperandAndLsl<9,1>";
  let MCOperandPredicate = [{
    int64_t Imm;
    if (MCOp.evaluateAsConstantImm(Imm))
      return isShiftedInt<8, 1>(Imm);
    return MCOp.isBareSymbolRef();
  }];
}

// A 10 bit signed immediate where the b0/b1 is always zero (width = 8).
// Used for ldind/stind operands, which are 32-bit aligned
def simm10_lsb00 : Operand<XLenVT>, ImmLeaf<XLenVT, [{return isShiftedInt<8, 2>(Imm);}]> {
  let ParserMatchClass = SImmAsmOperand<10, "Lsb00">;
  let EncoderMethod = "getImmOpValueAsr<2>";
  let DecoderMethod = "decodeSImmOperandAndLsl<10,2>";
  let MCOperandPredicate = [{
    int64_t Imm;
    if (MCOp.evaluateAsConstantImm(Imm))
      return isShiftedInt<8, 2>(Imm);
    return MCOp.isBareSymbolRef();
  }];
}

// A 13 bit signed immediate where the lsb is zero
def simm13_lsb0 : Operand<OtherVT> {
  let ParserMatchClass = SImmAsmOperand<13, "Lsb0">;
  let EncoderMethod = "getImmOpValueAsr<1>";
  let DecoderMethod = "decodeSImmOperandAndLsl<13,1>";
  let MCOperandPredicate = [{
    int64_t Imm;
    if (MCOp.evaluateAsConstantImm(Imm))
      return isShiftedInt<12, 1>(Imm);
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
  let ParserMethod = "parseBareSymbol";
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
    // Instructions with dependency: Reg -> (Acc)
    class INSTR_RRRAcc<LerosOpcode opcode, string opcodestr>
        : LerosImmRegInst<opcode, (outs GPR:$RegOrImm), (ins),
            opcodestr, "$RegOrImm", []>;
    
    // Instructions with dependency: Acc -> (reg)
    class INSTR_AccReg<LerosOpcode opcode, string opcodestr>
        : LerosImmRegInst<opcode, (outs), (ins GPR:$RegOrImm),
            opcodestr, "$RegOrImm", []>;
    
    // Instructions with dependency: Acc -> (imm)
    class INSTR_AccImm<LerosOpcode opcode, string opcodestr, Operand immClass>
        : LerosImmRegInst<opcode, (outs), (ins immClass:$RegOrImm),
            opcodestr, "$RegOrImm", []>;

    class INSTR_BranchCond<LerosBOpcode opcode, string opcodestr>
        : LerosBranchInst<opcode, (outs), (ins simm13_lsb0:$imm12), opcodestr, "$imm12", []>{
                let Inst{11-0} = imm12;
                let isBranch = 1;
                let isTerminator = 1;
            }

// ------------- Pseudo ISA wrapper classes -------------
// Pseudo instruction wrapper for instruction format: brcond rs1 rs2 imm
// Ie. breq rs1 rs2 bb => leros sub & leros brz
class Pseudo_BrCmpRs1Rs2 : LerosPseudo<(outs), (ins GPR:$r1, GPR:$r2, simm13_lsb0:$imm12), F_BRCmp, []> {
    let isBranch = 1;
    let isTerminator = 1;
}

// Ie. brgtz rs1 bb => leros brp
class Pseudo_BrRs : LerosPseudo<(outs), (ins GPR:$r1, simm13_lsb0:$imm12), F_BRRs, []> {
    let isBranch = 1;
    let isTerminator = 1;
}

// Multiclass for definition of single register branch pseudoinstructions and their
// Leros instruction counterpart
multiclass Implement_BrRs<LerosBOpcode opcode, string opcodestr>{
    def _PSEUDO :   Pseudo_BrRs;
    def _MI   :   INSTR_BranchCond<opcode, opcodestr>;
}

// Pseudo instruction wrapper for a Reg -> Reg, Imm instruction
class PSEUDO_RRI<SDNode opnode, Operand ImmClass> : LerosPseudo<(outs GPR:$rd), (ins GPR:$r1,
   ImmClass:$simm), F_RRI, [(set i32:$rd, (opnode i32:$r1, ImmClass:$simm))]>;
class PSEUDO_RRI_NoPat<Operand ImmClass, PseudoFormat format> : LerosPseudo<(outs GPR:$rd), (ins GPR:$r1,
  ImmClass:$simm), format, []>;


// Pseudo instruction wrapper for a Reg -> Reg, Reg instruction
class PSEUDO_RRR<SDNode opnode> : LerosPseudo<(outs GPR:$rd), (ins GPR:$r1,
   GPR:$r2), F_RRR, [(set i32:$rd, (opnode i32:$r1, i32:$r2))]>;

// Pseudo instruction wrapper for a Reg -> Imm instruction
class PSEUDO_RI : LerosPseudo<(outs GPR:$rd), (ins simm8:$imm), F_RI, []>;

// Pseudo instruction classes for memory access functions
let hasSideEffects = 0, mayLoad = 0, mayStore = 1 in
class PSEUDO_Store_M<Operand immClass> : LerosPseudo<(outs), (ins GPR:$rs2, GPR:$rs1, immClass:$imm8), F_LS, []>;

let hasSideEffects = 0, mayLoad = 1, mayStore = 0 in
class PSEUDO_Load_M<Operand immClass>  : LerosPseudo<(outs GPR:$rs2), (ins GPR:$rs1, immClass:$imm8), F_LS, []>;

multiclass INSTR_RRI<LerosOpcode opcode, string opcodestr, SDNode opnode, Operand immClass>{
    // Pseudo instruction definition
    def _PSEUDO  : PSEUDO_RRI<opnode, immClass>; // Register-immediate instruction

    // Machine instruction definitions
    def _MI  : INSTR_AccImm<opcode, !strconcat(opcodestr, "i"), immClass>;
}

multiclass INSTR_Store<LerosOpcode opcode, string opcodestr, Operand immClass>{
    def _PSEUDO  : PSEUDO_Store_M<immClass>;
    def _MI  : INSTR_AccImm<opcode, opcodestr, immClass>;
}

multiclass INSTR_Load<LerosOpcode opcode, string opcodestr, Operand immClass>{
    def _PSEUDO  : PSEUDO_Load_M<immClass>;
    def _MI  : INSTR_AccImm<opcode, opcodestr, immClass>;
}

multiclass INSTR_RRR<LerosOpcode opcode, string opcodestr, SDNode opnode>{
    // Pseudo instruction definition
    def _PSEUDO  : PSEUDO_RRR<opnode>; // Register-register instruction

    // Machine instruction definitions
    def _MI  : INSTR_AccReg<opcode, opcodestr>;
}

multiclass INSTR_Pseudo<SDNode opnode, Operand immClass>{
    let usesCustomInserter = 1 in {
        def I_PSEUDO  : PSEUDO_RRI<opnode, immClass>; // Register-immediate instruction
        def R_PSEUDO  : PSEUDO_RRR<opnode>; // Register-register instruction
    }
}

// Instruction declarations
let isReMaterializable = 1 in
defm ADD    : INSTR_RRR<OPC_ADDR    ,    "add",    add>;
defm ADDI   : INSTR_RRI<OPC_ADDI    ,    "add",    add,   simm8>;

defm SUB    : INSTR_RRR<OPC_SUBR     ,    "sub",    sub>;
defm SUBI   : INSTR_RRI<OPC_SUBI     ,    "sub",    sub,    simm8>;

defm AND    : INSTR_RRR<OPC_ANDR     ,    "and",    and>;
defm ANDI   : INSTR_RRI<OPC_ANDI     ,    "and",    and,    uimm7>;

defm OR     : INSTR_RRR<OPC_ORR      ,    "or",     or>;
defm ORI    : INSTR_RRI<OPC_ORI      ,    "or",     or,     uimm7>;

defm XOR    : INSTR_RRR<OPC_XORR     ,    "xor",    xor>;
defm XORI   : INSTR_RRI<OPC_XORI     ,    "xor",    xor,    uimm7>;

def LDADDR : INSTR_AccReg<OPC_LDADDR, "ldaddr">;

let hasSideEffects = 0, mayLoad = 1, mayStore = 0 in{
    defm LDINDB  : INSTR_Load<OPC_LDINDB,  "ldindb",    simm8>;
    defm LDINDH  : INSTR_Load<OPC_LDINDH,  "ldindh",    simm9_lsb0>;
    defm LDIND   : INSTR_Load<OPC_LDIND,   "ldind",     simm10_lsb00>;

    // Unsigned load pseudo-instructions, does not have a corresponding MI
    def LDINDBU_PSEUDO    : PSEUDO_Load_M<simm8>;
    def LDINDHU_PSEUDO    : PSEUDO_Load_M<simm9_lsb0>;
}

let hasSideEffects = 0, mayLoad = 0, mayStore = 1 in {
    defm STIND   : INSTR_Store<OPC_STIND,     "stind",      simm10_lsb00>;
    defm STINDH  : INSTR_Store<OPC_STINDH,    "stindh",     simm9_lsb0>;
    defm STINDB  : INSTR_Store<OPC_STINDB,    "stindb",     simm8>;
}


// Shifts are all pseudo instructions since they require custom emmission. 
// sra has an actual leros instruction, whereas shl and sra are emissions of small algorithms
def SRA1_PSEUDO: LerosPseudo<(outs GPR:$rd), (ins GPR:$rs), F_NA, []>;
def SRA1_MI  : LerosAccInst<OPC_SRA, "sra">;

// Set pseudo-instructions will be emitted as custom branch sequences with phi nodes
let usesCustomInserter = 1 in {
    def SETEQ_PSEUDO    : LerosPseudo<(outs GPR:$rd), (ins GPR:$rs1, GPR:$rs2), F_NA, []>;
    def SETGE_PSEUDO    : LerosPseudo<(outs GPR:$rd), (ins GPR:$rs1, GPR:$rs2), F_NA, []>;
    def SETLT_PSEUDO    : LerosPseudo<(outs GPR:$rd), (ins GPR:$rs1, GPR:$rs2), F_NA, []>;


    def SETUGE_PSEUDO   : LerosPseudo<(outs GPR:$rd), (ins GPR:$rs1, GPR:$rs2), F_NA, []>;
    def SETULT_PSEUDO   : LerosPseudo<(outs GPR:$rd), (ins GPR:$rs1, GPR:$rs2), F_NA, []>;
}

// Machine load instructions
def LOAD_MI     : INSTR_AccReg<OPC_LOADR, "load">;
def LOADI_MI    : INSTR_AccImm<OPC_LOADI, "loadi", simm8_symbolref>;

// Register-immediate load. Only class which needs PSEUDO_RI, given that no registers
// are input nodes to the pseudo instruction. We need this pseudo operation since it may be
// emitted during instruction selection to select immediate operands
def LOADI_PSEUDO : PSEUDO_RI;

defm LOADHI    : INSTR_RRI<OPC_LOADH    ,  "loadh",   LOADH,   simm8_symbolref>;
defm LOADH2I   : INSTR_RRI<OPC_LOADH2   ,  "loadh2",  LOADH2,  simm8_symbolref>;
defm LOADH3I   : INSTR_RRI<OPC_LOADH3   ,  "loadh3",  LOADH3,  simm8_symbolref>;

// Accumulator->register store
def STORE_MI   : INSTR_RRRAcc<OPC_STORE, "store">;

// NOP instruction. Pseudo NOP can be used without any arguments, later expanded to NOP_IMPL.
// NOP_IMPL contains a register argument because llvm complains about instructions with no in/out nodes
def  NOP_PSEUDO   : LerosPseudo<(outs ), (ins), F_NA, []>;
def  NOP_MI   : LerosAccInst<OPC_NOP, "nop">;

// Simulator-only scall (simulator call) instruction
def SCALL : LerosInst<(outs), (ins uimm8:$imm), "scall", "$imm", []>{
    bits<8> imm;
    let Inst{7-0} = imm;
    let Inst{15-8} = 0b11111111;
}

// =================== BRANCHES ===================
// Unconditional branch
let isBarrier = 1, isBranch = 1, isTerminator = 1 in
def BR_MI: LerosBranchInst<OPC_BR, (outs), (ins simm13_lsb0:$imm12), "br", "$imm12", [(br bb:$imm12)]>{
                let Inst{11-0} = imm12;
}

// Leros branch instructions - Defined as Implement_BrRs for:
//  - 2 operand pseudo-instruction (Register and basic block)
//  - 1 operand machine instructcion (basic block)
defm BRNZ            : Implement_BrRs<OPC_BRNZ,    "brnz"   >;
defm BRZ             : Implement_BrRs<OPC_BRZ,     "brz"    >;
defm BRP             : Implement_BrRs<OPC_BRP,     "brp"    >;
defm BRN             : Implement_BrRs<OPC_BRN,     "brn"    >;

// Dual register (comparison) branch Pseudo instructions
def BRGTE_PSEUDO       : Pseudo_BrCmpRs1Rs2;
def BRLT_PSEUDO        : Pseudo_BrCmpRs1Rs2;
def BREQ_PSEUDO        : Pseudo_BrCmpRs1Rs2;
def BRNEQ_PSEUDO       : Pseudo_BrCmpRs1Rs2;

// Indirect branch (expands to jal in LerosInstrInfo::expandBRIND)
let isBranch = 1, isTerminator = 1, isIndirectBranch = 1, isBarrier=1 in
def BRIND_PSEUDO : LerosPseudo<(outs), (ins GPR:$rd), F_NA, [(brind GPR:$rd)]>;

// ================================================


// Instructions where format is specific to the instruction 
let isCall = 1, Uses=[R0] in
def JAL_call     : INSTR_AccReg<OPC_JAL, "jal">;

let isReturn=1, isTerminator=1, isBarrier=1, Uses=[R0], isCodeGenOnly=1 in
def JAL_ret     : INSTR_RRRAcc<OPC_JAL, "jal">;

// Function return using jal
let isReturn=1, isTerminator=1, isBarrier=1, Uses=[R0] in
def RET : LerosPseudo<(outs), (ins), F_NA, [(LerosRet)]>;

// Will later be expanded to a sequence of immediate loads and a jal instruction
let isCall = 1, Defs = [R0], isCodeGenOnly = 0 in
def PseudoCALL : LerosPseudo<(outs), (ins bare_symbol:$func), F_NA,
                        [(Call tglobaladdr:$func)]> {
    let AsmString = "call\t$func";
}

def : Pat<(Call texternalsym:$func), (PseudoCALL texternalsym:$func)>;

// Will expand to the same instruction sequence as branch indirect, but marked as a separate
// pseudoinstructio n sice wee need to add the decorators of isCall and Defs
let isCall = 1, Defs = [R0] in
def PseudoCALLIndirect : LerosPseudo<(outs), (ins GPR:$rs1), F_NA, [(Call GPR:$rs1)]>;

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
        (LOADI_PSEUDO imm:$imm)>;

def : Pat<(simm16:$imm), 
    (LOADHI_PSEUDO
        (LOADI_PSEUDO (BYTE0 imm:$imm)), 
    (BYTE1 imm:$imm))>;

def : Pat<(simm24:$imm), 
    (LOADH2I_PSEUDO
        (LOADHI_PSEUDO
            (LOADI_PSEUDO (BYTE0 imm:$imm)), 
        (BYTE1 imm:$imm)), 
    (BYTE2 imm:$imm))>;

def : Pat<(simm32:$imm),
    (LOADH3I_PSEUDO
        (LOADH2I_PSEUDO
            (LOADHI_PSEUDO
                (LOADI_PSEUDO (BYTE0 imm:$imm)), 
            (BYTE1 imm:$imm)), 
        (BYTE2 imm:$imm)),
    (BYTE3 imm:$imm))>;

// Todo: Extend for 64-bit

//===----------------------------------------------------------------------===//
// Codegen patterns
//===----------------------------------------------------------------------===//   

// Branch register-0 comparison patterns
class BrCmp0Pat<PatFrag CondOp, Pseudo_BrRs Inst>
    : Pat<(brcond (i32 (CondOp GPR:$rs1, 0)), bb:$imm12),
          (Inst GPR:$rs1, simm13_lsb0:$imm12)>;

def : BrCmp0Pat<seteq, BRZ_PSEUDO>;
def : BrCmp0Pat<setne, BRNZ_PSEUDO>;
def : BrCmp0Pat<setge, BRP_PSEUDO>;
def : BrCmp0Pat<setlt, BRN_PSEUDO>;


// Branch register-register comparison patterns
class BrCmpPat<PatFrag CondOp, Pseudo_BrCmpRs1Rs2 Inst>
    : Pat<(brcond (i32 (CondOp GPR:$rs1, GPR:$rs2)), bb:$imm12),
          (Inst GPR:$rs1, GPR:$rs2, simm13_lsb0:$imm12)>;

def : BrCmpPat<seteq, BREQ_PSEUDO>;
def : BrCmpPat<setne, BRNEQ_PSEUDO>;
def : BrCmpPat<setge, BRGTE_PSEUDO>;
def : BrCmpPat<setlt, BRLT_PSEUDO>;

class BccSwapPat<PatFrag CondOp, Pseudo_BrCmpRs1Rs2 Inst>
    : Pat<(brcond (i32 (CondOp GPR:$rs1, GPR:$rs2)), bb:$imm8),
          (Inst GPR:$rs2, GPR:$rs1, bb:$imm8)>;

// Condition codes that don't have matching Leros branch instructions, but
// are trivially supported by swapping the two input operands
def : BccSwapPat<setgt, BRLT_PSEUDO>;
def : BccSwapPat<setle, BRGTE_PSEUDO>;

// An extra pattern is needed for a brcond without a setcc (i.e. where the
// condition was calculated elsewhere).
def : Pat<(brcond GPR:$cond, bb:$imm8), (BRNZ_PSEUDO GPR:$cond, bb:$imm8)>;

// Set patterns for CC calculation
def : Pat<(seteq GPR:$rs1, GPR:$rs2), (SETEQ_PSEUDO GPR:$rs1, GPR:$rs2)>;
def : Pat<(setueq GPR:$rs1, GPR:$rs2), (SETEQ_PSEUDO GPR:$rs1, GPR:$rs2)>;
def : Pat<(setne GPR:$rs1, GPR:$rs2), (XORI_PSEUDO (SETEQ_PSEUDO GPR:$rs1, GPR:$rs2), 1)>;
def : Pat<(setgt GPR:$rs1, GPR:$rs2), (SETLT_PSEUDO GPR:$rs2, GPR:$rs1)>;
def : Pat<(setge GPR:$rs1, GPR:$rs2), (SETGE_PSEUDO GPR:$rs1, GPR:$rs2)>;
def : Pat<(setlt GPR:$rs1, GPR:$rs2), (SETLT_PSEUDO GPR:$rs1, GPR:$rs2)>;
def : Pat<(setle GPR:$rs1, GPR:$rs2), (SETGE_PSEUDO GPR:$rs2, GPR:$rs1)>;

// We do not have patterns where we catch the CC for unsigned comparison for branches
//(as with BccPat), but we can do unsigned compare and set
def : Pat<(setugt GPR:$rs1, GPR:$rs2), (SETULT_PSEUDO GPR:$rs2, GPR:$rs1)>;
def : Pat<(setuge GPR:$rs1, GPR:$rs2), (SETUGE_PSEUDO GPR:$rs1, GPR:$rs2)>;
def : Pat<(setult GPR:$rs1, GPR:$rs2), (SETULT_PSEUDO GPR:$rs1, GPR:$rs2)>;
def : Pat<(setule GPR:$rs1, GPR:$rs2), (SETUGE_PSEUDO GPR:$rs2, GPR:$rs1)>;

// Addressing modes.
// Necessary because a frameindex can't be matched directly in a pattern.
def AddrFI : ComplexPattern<iPTR, 1, "SelectAddrFI", [frameindex], []>;

/// FrameIndex calculations
def : Pat<(add (i32 AddrFI:$Rs), simm8:$imm8),
          (ADDI_PSEUDO (i32 AddrFI:$Rs), simm8:$imm8)>;
def : Pat<(IsOrAdd (i32 AddrFI:$Rs), simm8:$imm8),
          (ADDI_PSEUDO (i32 AddrFI:$Rs), simm8:$imm8)>;

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
multiclass LdPat<PatFrag LoadOp, PSEUDO_Load_M Inst, Operand immClass> {
   def : Pat<(LoadOp (add GPR:$rs1, immClass:$imm)),
            (Inst GPR:$rs1, immClass:$imm)>;
   def : Pat<(LoadOp (add AddrFI:$rs1, immClass:$imm)),
             (Inst AddrFI:$rs1, immClass:$imm)>;
   def : Pat<(LoadOp GPR:$rs1),
             (Inst GPR:$rs1, 0)>;
   def : Pat<(LoadOp AddrFI:$rs1),
             (Inst AddrFI:$rs1, 0)>;
   def : Pat<(LoadOp (IsOrAdd AddrFI:$rs1, immClass:$imm)),
             (Inst AddrFI:$rs1, immClass:$imm)>;
}

defm : LdPat<load,          LDIND_PSEUDO,       simm10_lsb00>;
defm : LdPat<sextloadi8,    LDINDB_PSEUDO,      simm8>;
defm : LdPat<extloadi8,     LDINDBU_PSEUDO,     simm8>;
defm : LdPat<zextloadi8,    LDINDBU_PSEUDO,     simm8>;
defm : LdPat<sextloadi16,   LDINDH_PSEUDO,      simm9_lsb0>;
defm : LdPat<extloadi16,    LDINDHU_PSEUDO,     simm9_lsb0>;
defm : LdPat<zextloadi16,   LDINDHU_PSEUDO,     simm9_lsb0>;

// Stores
multiclass StPat<PatFrag StoreOp, LerosPseudo Inst, Operand immClass> {
  def : Pat<(StoreOp GPR:$rs2, GPR:$rs1), 
            (Inst GPR:$rs2, GPR:$rs1, 0)>;
  def : Pat<(StoreOp GPR:$rs2, AddrFI:$rs1), 
            (Inst GPR:$rs2, AddrFI:$rs1, 0)>;
  def : Pat<(StoreOp GPR:$rs2, (add GPR:$rs1, immClass:$imm)),
            (Inst GPR:$rs2, GPR:$rs1, immClass:$imm)>;
  def : Pat<(StoreOp GPR:$rs2, (add AddrFI:$rs1, immClass:$imm)),
            (Inst GPR:$rs2, AddrFI:$rs1, immClass:$imm)>;
  def : Pat<(StoreOp GPR:$rs2, (IsOrAdd AddrFI:$rs1, immClass:$imm)),
            (Inst GPR:$rs2, AddrFI:$rs1, immClass:$imm)>;
}


defm : StPat<store,         STIND_PSEUDO,   simm10_lsb00>;
defm : StPat<truncstorei8,  STINDB_PSEUDO,  simm8>;
defm : StPat<truncstorei16, STINDH_PSEUDO,  simm9_lsb0>;


// Stack pointer adjustments
let Defs = [R1], Uses = [R1] in {
def ADJCALLSTACKDOWN : LerosPseudo<(outs), (ins i32imm:$amt1, i32imm:$amt2), F_NA,
                                  [(Callseq_start timm:$amt1, timm:$amt2)]>;
def ADJCALLSTACKUP   : LerosPseudo<(outs), (ins i32imm:$amt1, i32imm:$amt2), F_NA,
                                  [(Callseq_end timm:$amt1, timm:$amt2)]>;
}
