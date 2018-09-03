set(LLVM_TARGET_DEFINITIONS Leros.td)

#tablegen(LLVM LerosGenAsmWriter.inc -gen-asm-writer)
#tablegen(LLVM LerosGenCallingConv.inc -gen-callingconv)
#tablegen(LLVM LerosGenDAGISel.inc -gen-dag-isel)
#tablegen(LLVM LerosGenInstrInfo.inc -gen-instr-info)
tablegen(LLVM LerosGenRegisterInfo.inc -gen-register-info)
#tablegen(LLVM LerosGenSubtargetInfo.inc -gen-subtarget)
add_public_tablegen_target(LerosCommonTableGen)

add_llvm_target(LerosCodeGen
 LerosInstrInfo.cpp
 LerosTargetMachine.cpp
 LerosSubtarget.cpp
 LerosISelDAGToDAG.cpp
 LerosRegisterInfo.cpp
)

add_subdirectory(MCTargetDesc)
add_subdirectory(TargetInfo)
