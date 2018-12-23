//===-- LerosTargetMachine.cpp - Define TargetMachine for Leros -----------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// Implements the info about Leros target spec.
//
//===----------------------------------------------------------------------===//

#include "LerosTargetMachine.h"
#include "Leros.h"
#include "LerosTargetObjectFile.h"
#include "MCTargetDesc/LerosMCTargetDesc.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/TargetLoweringObjectFileImpl.h"
#include "llvm/CodeGen/TargetPassConfig.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Target/TargetOptions.h"

namespace llvm {
extern "C" void LLVMInitializeLerosTarget() {
  RegisterTargetMachine<LerosTargetMachine> X(getTheLeros32Target());
  RegisterTargetMachine<LerosTargetMachine> Y(getTheLeros64Target());
}

static std::string computeDataLayout(const Triple &TT) {
  if (TT.isArch64Bit()) {
    llvm_unreachable("64 bit Leros not yet supported");
  } else {
    return "E-m:e-p:32:32-i8:8:8-i16:32:32-i32:32:32-i64:64-n32-S128";
  }
}

static Reloc::Model getEffectiveRelocModel(const Triple &TT,
                                           Optional<Reloc::Model> RM) {
  if (!RM.hasValue())
    return Reloc::Static;
  return *RM;
}

static CodeModel::Model getEffectiveCodeModel(Optional<CodeModel::Model> CM) {
  if (CM)
    return *CM;
  return CodeModel::Small;
}

LerosTargetMachine::LerosTargetMachine(const Target &T, const Triple &TT,
                                       StringRef CPU, StringRef FS,
                                       const TargetOptions &Options,
                                       Optional<Reloc::Model> RM,
                                       Optional<CodeModel::Model> CM,
                                       CodeGenOpt::Level OL, bool JIT)
    : LLVMTargetMachine(T, computeDataLayout(TT), TT, CPU, FS, Options,
                        getEffectiveRelocModel(TT, RM),
                        getEffectiveCodeModel(CM), OL),
      Subtarget(TT, CPU, FS, *this),
      TLOF(make_unique<LerosELFTargetObjectFile>()) {
  initAsmInfo();
}

namespace {
class LerosPassConfig : public TargetPassConfig {
public:
  LerosPassConfig(LerosTargetMachine &TM, PassManagerBase &PM)
      : TargetPassConfig(TM, PM) {}

  LerosTargetMachine &getLerosTargetMachine() const {
    return getTM<LerosTargetMachine>();
  }

  bool addInstSelector() override;
  void addPreEmitPass() override;
};
} // namespace

TargetPassConfig *LerosTargetMachine::createPassConfig(PassManagerBase &PM) {
  return new LerosPassConfig(*this, PM);
}

void LerosPassConfig::addPreEmitPass() {
  addPass(createLerosUseAccumulatorPass());
}

bool LerosPassConfig::addInstSelector() {
  addPass(createLerosISelDag(getLerosTargetMachine()));

  return false;
}
} // namespace llvm
