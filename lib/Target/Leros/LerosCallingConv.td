//===-- LerosCallingConv.td - Calling Conventions Leros ----*- tabLerosen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This describes the calling conventions for the Leros architecture.
//
//===----------------------------------------------------------------------===//

def CSR : CalleeSavedRegs<(add R0, R1, R2, R3, (sequence "R%u", 61, 117))>;

// Needed for implementation of LerosRegisterInfo::getNoPreservedMask()
def CSR_NoRegs : CalleeSavedRegs<(add)>;

//===----------------------------------------------------------------------===//
// Leros Return Value Calling Convention
//===----------------------------------------------------------------------===//

def RetCC_Leros : CallingConv
<[
  // i32 are returned in registers R4-R7
  CCIfType<[i32], CCAssignToReg<[R4, R5, R6, R7]>>,

  // Integer values get stored in stack slots that are 4 bytes in
  // size and 4-byte aligned.
  CCIfType<[i32], CCAssignToStack<4, 4>>
]>;

//===----------------------------------------------------------------------===//
// Leros Argument Calling Conventions
//===----------------------------------------------------------------------===//
def CC_Leros : CallingConv<[
  // Promote i8/i16 arguments to i32.
  CCIfType<[i8, i16], CCPromoteToType<i32>>,

  // The first 4 integer arguments are passed in arg/return registers.
  CCIfType<[i32], CCAssignToReg<[R4, R5, R6, R7]>>,

  // Integer values get stored in stack slots that are 4 bytes in
  // size and 4-byte aligned.
  CCIfType<[i32], CCAssignToStack<4, 4>>
]>;

