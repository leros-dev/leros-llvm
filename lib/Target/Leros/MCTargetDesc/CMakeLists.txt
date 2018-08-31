add_llvm_library(LLVMLerosDesc
    LerosMCTargetDesc.cpp
    LerosMCAsmInfo.cpp
)

add_dependencies(LLVMLerosDesc LerosCommonTableGen)