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
#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/TargetLoweringObjectFileImpl.h"
#include "llvm/CodeGen/TargetPassConfig.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Target/TargetOptions.h"

using namespace llvm;

extern "C" void LLVMInitializeLerosTarget() {
  RegisterTargetMachine<LerosTargetMachine> X(getTheLeros32Target());
  RegisterTargetMachine<LerosTargetMachine> Y(getTheLeros64Target());
}

static std::string computeDataLayout(const Triple &TT) {
  if (TT.isArch16Bit()) {
    return "e-p16:16-i1:8:16-i8:8:16-i16:16-i32:16:32-i64:16:64";
  } else if (TT.isArch32Bit()) {
    return "e-p32:32-i1:8:32-i8:8:32-i16:16:32-i32:32:32-i64:32:64";
  } else {
    return "e-p64:64-i1:8:64-i8:8:64-i16:16:64-i32:32:64-i64:64";
  }
}

LerosTargetMachine::LerosTargetMachine(const Target &T, const Triple &TT,
                                       StringRef CPU, StringRef FS,
                                       const TargetOptions &Options,
                                       Optional<Reloc::Model> RM,
                                       Optional<CodeModel::Model> CM,
                                       CodeGenOpt::Level OL, bool JIT)
    : LLVMTargetMachine(T, computeDataLayout(TT), TT, CPU, FS, Options, RM, CM,
                        OL),
      Subtarget(TT, CPU, FS, *this) {
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
};
}

TargetPassConfig *LerosTargetMachine::createPassConfig(PassManagerBase &PM) {
  return new LerosPassConfig(*this, PM);
}

bool LerosPassConfig::addInstSelector() {
  addPass(createLerosISelDag(getLerosTargetMachine()));

  return false;
}
