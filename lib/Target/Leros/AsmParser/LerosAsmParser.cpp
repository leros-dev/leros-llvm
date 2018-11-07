//===-- LerosAsmParser.cpp - Parse Leros assembly to MCInst instructions --===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "Leros.h"
#include "MCTargetDesc/LerosMCExpr.h"
#include "MCTargetDesc/LerosMCTargetDesc.h"
#include "MCTargetDesc/LerosTargetStreamer.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/StringSwitch.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstBuilder.h"
#include "llvm/MC/MCParser/MCAsmLexer.h"
#include "llvm/MC/MCParser/MCParsedAsmOperand.h"
#include "llvm/MC/MCParser/MCTargetAsmParser.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/SMLoc.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

namespace {
class LerosAsmParser : public MCTargetAsmParser {
  SMLoc getLoc() const { return getParser().getTok().getLoc(); }

  LerosTargetStreamer &getTargetStreamer() {
    MCTargetStreamer &TS = *getParser().getStreamer().getTargetStreamer();
    return static_cast<LerosTargetStreamer &>(TS);
  }

  unsigned validateTargetOperandClass(MCParsedAsmOperand &Op,
                                      unsigned Kind) override;

  bool MatchAndEmitInstruction(SMLoc IDLoc, unsigned &Opcode,
                               OperandVector &Operands, MCStreamer &Out,
                               uint64_t &ErrorInfo,
                               bool MatchingInlineAsm) override;

  bool ParseRegister(unsigned &RegNo, SMLoc &StartLoc, SMLoc &EndLoc) override;

  bool ParseInstruction(ParseInstructionInfo &Info, StringRef Name,
                        SMLoc NameLoc, OperandVector &Operands) override;

  bool ParseDirective(AsmToken DirectiveID) override { return true; }

// Auto-generated instruction matching functions
#define GET_ASSEMBLER_HEADER
#include "LerosGenAsmMatcher.inc"

  bool parseOperand(OperandVector &Operands, StringRef Mnemonic);
  OperandMatchResultTy parseImmediate(OperandVector &Operands);
  OperandMatchResultTy parseRegister(OperandVector &Operands);
  OperandMatchResultTy parseBareSymbol(OperandVector &Operands);

public:
  enum LerosMatchResultTy {
    Match_Dummy = FIRST_TARGET_MATCH_RESULT_TY,
#define GET_OPERAND_DIAGNOSTIC_TYPES
#include "LerosGenAsmMatcher.inc"
#undef GET_OPERAND_DIAGNOSTIC_TYPES
  };

  static bool classifySymbolRef(const MCExpr *Expr,
                                LerosMCExpr::VariantKind &Kind,
                                int64_t &Addend);

  LerosAsmParser(const MCSubtargetInfo &STI, MCAsmParser &Parser,
                 const MCInstrInfo &MII, const MCTargetOptions &Options)
      : MCTargetAsmParser(Options, STI, MII) {}
};

/// LerosOperand - Instances of this class represent a parsed machine
/// instruction
struct LerosOperand : public MCParsedAsmOperand {

  enum KindTy { Token, Register, Immediate } Kind;

  struct RegOp {
    unsigned RegNum;
  };

  struct ImmOp {
    const MCExpr *Val;
  };

  struct SysRegOp {
    const char *Data;
    unsigned Length;
    unsigned Encoding;
    // FIXME: Add the Encoding parsed fields as needed for checks,
    // e.g.: read/write or user/supervisor/machine privileges.
  };

  SMLoc StartLoc, EndLoc;
  union {
    StringRef Tok;
    RegOp Reg;
    ImmOp Imm;
    struct SysRegOp SysReg;
  };

  LerosOperand(KindTy K) : MCParsedAsmOperand(), Kind(K) {}

public:
  LerosOperand(const LerosOperand &o) : MCParsedAsmOperand() {
    Kind = o.Kind;
    StartLoc = o.StartLoc;
    EndLoc = o.EndLoc;
    switch (Kind) {
    case Register:
      Reg = o.Reg;
      break;
    case Immediate:
      Imm = o.Imm;
      break;
    case Token:
      Tok = o.Tok;
      break;
    }
  }

  bool isToken() const override { return Kind == Token; }
  bool isReg() const override { return Kind == Register; }
  bool isImm() const override { return Kind == Immediate; }

  /// getStartLoc - Gets location of the first token of this operand
  SMLoc getStartLoc() const override { return StartLoc; }
  /// getEndLoc - Gets location of the last token of this operand
  SMLoc getEndLoc() const override { return EndLoc; }

  unsigned getReg() const override {
    assert(Kind == Register && "Invalid type access!");
    return Reg.RegNum;
  }

  void print(raw_ostream &OS) const override {
    switch (Kind) {
    case Immediate:
      OS << *getImm();
      break;
    case Register:
      OS << "<register r";
      OS << getReg() << ">";
      break;
    case Token:
      OS << "'" << getToken() << "'";
      break;
    }
  }

  static bool evaluateConstantImm(const MCExpr *Expr, int64_t &Imm,
                                  LerosMCExpr::VariantKind &VK) {
    if (auto *RE = dyn_cast<LerosMCExpr>(Expr)) {
      VK = RE->getKind();
      return RE->evaluateAsConstant(Imm);
    }

    if (auto CE = dyn_cast<MCConstantExpr>(Expr)) {
      VK = LerosMCExpr::VK_Leros_None;
      Imm = CE->getValue();
      return true;
    }

    return false;
  }

  bool isBareSymbol() const {
    int64_t Imm;
    LerosMCExpr::VariantKind VK;
    // Must be of 'immediate' type but not a constant.
    if (!isImm() || evaluateConstantImm(getImm(), Imm, VK))
      return false;
    return LerosAsmParser::classifySymbolRef(getImm(), VK, Imm) &&
           VK == LerosMCExpr::VK_Leros_None;
  }

  void addExpr(MCInst &Inst, const MCExpr *Expr) const {
    assert(Expr && "Expr shouldn't be null!");
    int64_t Imm = 0;
    LerosMCExpr::VariantKind VK;
    bool IsConstant = evaluateConstantImm(Expr, Imm, VK);

    if (IsConstant)
      Inst.addOperand(MCOperand::createImm(Imm));
    else
      Inst.addOperand(MCOperand::createExpr(Expr));
  }

  bool isUImm8() const {
    int64_t Imm;
    LerosMCExpr::VariantKind VK;
    if (!isImm())
      return false;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    return IsConstantImm && isUInt<8>(Imm) && VK == LerosMCExpr::VK_Leros_None;
  }

  bool isUImm7() const {
    int64_t Imm;
    LerosMCExpr::VariantKind VK;
    if (!isImm())
      return false;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    return IsConstantImm && isUInt<7>(Imm) && VK == LerosMCExpr::VK_Leros_None;
  }

  bool isSImm8() const {
    int64_t Imm;
    LerosMCExpr::VariantKind VK;
    if (!isImm())
      return false;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    return IsConstantImm && isInt<8>(Imm) && VK == LerosMCExpr::VK_Leros_None;
  }

  // True if operand is a symbol with no modifiers, or a constant with no
  // modifiers and isShiftedInt<N-1, 1>(Op).
  template <int N> bool isBareSimmNLsb0() const {
    int64_t Imm;
    LerosMCExpr::VariantKind VK;
    if (!isImm())
      return false;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    bool IsValid;
    if (!IsConstantImm)
      IsValid = LerosAsmParser::classifySymbolRef(getImm(), VK, Imm);
    else
      IsValid = isShiftedInt<N - 1, 1>(Imm);
    return IsValid && VK == LerosMCExpr::VK_Leros_None;
  }

  // True if operand is a symbol with no modifiers, or a constant with no
  // modifiers
  template <int N> bool isBareSimm() const {
    int64_t Imm;
    LerosMCExpr::VariantKind VK;
    if (!isImm())
      return false;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    bool IsValid;
    if (!IsConstantImm)
      IsValid = LerosAsmParser::classifySymbolRef(getImm(), VK, Imm);
    return IsValid && VK == LerosMCExpr::VK_Leros_None;
  }

  bool isSImm12Lsb0() const { return isBareSimmNLsb0<12>(); }

  // Operand parser for signed 8-bit immediates which are either constants
  // or symbol references (used for load operations)
  bool isSImm8SymbolRef() const { return isBareSimm<8>(); }

  bool isImmXLen() const {
    int64_t Imm;
    LerosMCExpr::VariantKind VK;
    if (!isImm())
      return false;
    bool IsConstantImm = evaluateConstantImm(getImm(), Imm, VK);
    // Given only Imm, ensuring that the actually specified constant is either
    // a signed or unsigned 64-bit number is unfortunately impossible.
    bool IsInRange = isInt<32>(Imm);
    return IsConstantImm && IsInRange && VK == LerosMCExpr::VK_Leros_None;
  }

  const MCExpr *getImm() const {
    assert(Kind == Immediate && "Invalid type access!");
    return Imm.Val;
  }

  StringRef getToken() const {
    assert(Kind == Token && "Invalid type access!");
    return Tok;
  }
  bool isMem() const override { return false; }

  void addRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getReg()));
  }

  void addImmOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    addExpr(Inst, getImm());
  }

  static std::unique_ptr<LerosOperand> createToken(StringRef Str, SMLoc S) {
    auto Op = make_unique<LerosOperand>(Token);
    Op->Tok = Str;
    Op->StartLoc = S;
    Op->EndLoc = S;
    return Op;
  }

  static std::unique_ptr<LerosOperand> createReg(unsigned RegNo, SMLoc S,
                                                 SMLoc E) {
    auto Op = make_unique<LerosOperand>(Register);
    Op->Reg.RegNum = RegNo;
    Op->StartLoc = S;
    Op->EndLoc = E;
    return Op;
  }

  static std::unique_ptr<LerosOperand> createImm(const MCExpr *Val, SMLoc S,
                                                 SMLoc E) {
    auto Op = make_unique<LerosOperand>(Immediate);
    Op->Imm.Val = Val;
    Op->StartLoc = S;
    Op->EndLoc = E;
    return Op;
  }
};

#define GET_REGISTER_MATCHER
#define GET_MATCHER_IMPLEMENTATION
#include "LerosGenAsmMatcher.inc"

bool LerosAsmParser::ParseRegister(unsigned &RegNo, SMLoc &StartLoc,
                                   SMLoc &EndLoc) {
  const AsmToken &Tok = getParser().getTok();
  StartLoc = Tok.getLoc();
  EndLoc = Tok.getEndLoc();
  RegNo = 0;
  StringRef Name = getLexer().getTok().getIdentifier();

  if (!MatchRegisterName(Name)) {
    getParser().Lex(); // Eat identifier token.
    return false;
  }

  return Error(StartLoc, "invalid register name");
}

OperandMatchResultTy LerosAsmParser::parseRegister(OperandVector &Operands) {
  AsmToken Buf[2];

  switch (getLexer().getKind()) {
  default:
    return MatchOperand_NoMatch;
  case AsmToken::Identifier:
    StringRef Name = getLexer().getTok().getIdentifier();
    unsigned RegNo = MatchRegisterName(Name);
    if (RegNo == 0) {
      RegNo = MatchRegisterAltName(Name);
      if (RegNo == 0) {
        return MatchOperand_NoMatch;
      }
    }
    SMLoc S = getLoc();
    SMLoc E = SMLoc::getFromPointer(S.getPointer() - 1);
    Operands.push_back(LerosOperand::createReg(RegNo, S, E));
    getLexer().Lex();
  }
  return MatchOperand_Success;
}

bool LerosAsmParser::parseOperand(OperandVector &Operands, StringRef Mnemonic) {
  // Check if the current operand has a custom associated parser, if so, try to
  // custom parse the operand, or fallback to the general approach.
  OperandMatchResultTy Result =
      MatchOperandParserImpl(Operands, Mnemonic, /*ParseForAllFeatures=*/true);
  if (Result == MatchOperand_Success)
    return false;
  if (Result == MatchOperand_ParseFail)
    return true;

  // Attempt to parse token as a register.
  if (parseRegister(Operands) == MatchOperand_Success)
    return false;

  // Attempt to parse token as an immediate
  if (parseImmediate(Operands) == MatchOperand_Success)
    return false;

  // Finally we have exhausted all options and must declare defeat.
  Error(getLoc(), "unknown operand");
  return true;
}

OperandMatchResultTy LerosAsmParser::parseBareSymbol(OperandVector &Operands) {
  SMLoc S = getLoc();
  SMLoc E = SMLoc::getFromPointer(S.getPointer() - 1);
  const MCExpr *Res;

  if (getLexer().getKind() != AsmToken::Identifier)
    return MatchOperand_NoMatch;

  StringRef Identifier;
  if (getParser().parseIdentifier(Identifier))
    return MatchOperand_ParseFail;

  MCSymbol *Sym = getContext().getOrCreateSymbol(Identifier);
  Res = MCSymbolRefExpr::create(Sym, MCSymbolRefExpr::VK_None, getContext());
  Operands.push_back(LerosOperand::createImm(Res, S, E));
  return MatchOperand_Success;
}

OperandMatchResultTy LerosAsmParser::parseImmediate(OperandVector &Operands) {
  SMLoc S = getLoc();
  SMLoc E = SMLoc::getFromPointer(S.getPointer() - 1);
  const MCExpr *Res;

  switch (getLexer().getKind()) {
  default:
    return MatchOperand_NoMatch;
  case AsmToken::LParen:
  case AsmToken::Minus:
  case AsmToken::Plus:
  case AsmToken::Integer:
  case AsmToken::String:
    if (getParser().parseExpression(Res))
      return MatchOperand_ParseFail;
    break;
  case AsmToken::Identifier: {
    StringRef Identifier;
    if (getParser().parseIdentifier(Identifier))
      return MatchOperand_ParseFail;
    MCSymbol *Sym = getContext().getOrCreateSymbol(Identifier);
    Res = MCSymbolRefExpr::create(Sym, MCSymbolRefExpr::VK_None, getContext());
    break;
  }
  }

  Operands.push_back(LerosOperand::createImm(Res, S, E));
  return MatchOperand_Success;
}

bool LerosAsmParser::ParseInstruction(ParseInstructionInfo &Info,
                                      StringRef Name, SMLoc NameLoc,
                                      OperandVector &Operands) {
  // First operand is token for instruction
  Operands.push_back(LerosOperand::createToken(Name, NameLoc));

  // If there are no more operands, then finish
  if (getLexer().is(AsmToken::EndOfStatement))
    return false;

  // Parse the potential first instruction operand
  if (getLexer().isNot(AsmToken::EndOfStatement)) {

    if (parseOperand(Operands, Name))
      return true;

    // At this point, next token must always be end of statement
    if (getLexer().isNot(AsmToken::EndOfStatement)) {
      SMLoc Loc = getLexer().getLoc();
      getParser().eatToEndOfStatement();
      return Error(Loc, "unexpected token");
    }
  }

  getParser().Lex(); // Consume the EndOfStatement.
  return false;
}

unsigned LerosAsmParser::validateTargetOperandClass(MCParsedAsmOperand &AsmOp,
                                                    unsigned) {
  LerosOperand &Op = static_cast<LerosOperand &>(AsmOp);
  if (!Op.isReg())
    return Match_InvalidOperand;
  return Match_Success;
}

bool LerosAsmParser::MatchAndEmitInstruction(SMLoc IDLoc, unsigned &Opcode,
                                             OperandVector &Operands,
                                             MCStreamer &Out,
                                             uint64_t &ErrorInfo,
                                             bool MatchingInlineAsm) {
  MCInst Inst;

  auto Result =
      MatchInstructionImpl(Operands, Inst, ErrorInfo, MatchingInlineAsm);
  switch (Result) {
  default:
    break;
  case Match_Success: {
    Out.EmitInstruction(Inst, getSTI());
    return false;
  }
  case Match_MissingFeature:
    return Error(IDLoc, "instruction use requires an option to be enabled");
  case Match_MnemonicFail:
    return Error(IDLoc, "unrecognized instruction mnemonic");
  case Match_InvalidOperand: {
    SMLoc ErrorLoc = IDLoc;
    if (ErrorInfo != ~0U) {
      if (ErrorInfo >= Operands.size())
        return Error(ErrorLoc, "too few operands for instruction");

      ErrorLoc = ((LerosOperand &)*Operands[ErrorInfo]).getStartLoc();
      if (ErrorLoc == SMLoc())
        ErrorLoc = IDLoc;
    }
    return Error(ErrorLoc, "invalid operand for instruction");
  }
  }

  // Handle the case when the error message is of specific type
  // other than the generic Match_InvalidOperand, and the
  // corresponding operand is missing.
  if (Result > FIRST_TARGET_MATCH_RESULT_TY) {
    SMLoc ErrorLoc = IDLoc;
    if (ErrorInfo != ~0U && ErrorInfo >= Operands.size())
      return Error(ErrorLoc, "too few operands for instruction");
  }

  llvm_unreachable("Unknown match type detected!");
}

bool LerosAsmParser::classifySymbolRef(const MCExpr *Expr,
                                       LerosMCExpr::VariantKind &Kind,
                                       int64_t &Addend) {
  Kind = LerosMCExpr::VK_Leros_None;
  Addend = 0;

  if (const LerosMCExpr *RE = dyn_cast<LerosMCExpr>(Expr)) {
    Kind = RE->getKind();
    Expr = RE->getSubExpr();
  }

  // It's a simple symbol reference or constant with no addend.
  if (isa<MCConstantExpr>(Expr) || isa<MCSymbolRefExpr>(Expr))
    return true;

  const MCBinaryExpr *BE = dyn_cast<MCBinaryExpr>(Expr);
  if (!BE)
    return false;

  if (!isa<MCSymbolRefExpr>(BE->getLHS()))
    return false;

  if (BE->getOpcode() != MCBinaryExpr::Add &&
      BE->getOpcode() != MCBinaryExpr::Sub)
    return false;

  // We are able to support the subtraction of two symbol references
  if (BE->getOpcode() == MCBinaryExpr::Sub &&
      isa<MCSymbolRefExpr>(BE->getRHS()))
    return true;

  // See if the addend is a constant, otherwise there's more going
  // on here than we can deal with.
  auto AddendExpr = dyn_cast<MCConstantExpr>(BE->getRHS());
  if (!AddendExpr)
    return false;

  Addend = AddendExpr->getValue();
  if (BE->getOpcode() == MCBinaryExpr::Sub)
    Addend = -Addend;

  // It's some symbol reference + a constant addend
  return Kind != LerosMCExpr::VK_Leros_Invalid;
}

} // namespace

extern "C" void LLVMInitializeLerosAsmParser() {
  RegisterMCAsmParser<LerosAsmParser> X(getTheLeros32Target());
  RegisterMCAsmParser<LerosAsmParser> Y(getTheLeros64Target());
}
