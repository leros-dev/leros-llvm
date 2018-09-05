//===-- LerosRegisterInfo.td - RISC-V Register defs --------*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
//  Declarations that describe the Leros register files
//===----------------------------------------------------------------------===//

let Namespace = "Leros" in {
  
// Declare the accumulator
def ACC : Register<"acc">, DwarfRegNum<[-1]>;


class LerosReg<bits<8> Enc, string n> : Register<n> {
    let HWEncoding{7-0} = Enc;
}

// Declare the 256 GPR registers
foreach Index = 0-255 in {
    def R#Index# : LerosReg<Index, "r"#Index>, DwarfRegNum<[Index]>;
  }
}

def XLenVT : ValueTypeByHwMode<[Leros16, Leros32, Leros64, DefaultMode],
                               [i16,     i32,     i64,     i32]>;

// General purpose registers
def GPR : RegisterClass<"Leros", [XLenVT], 32, (add
    (sequence "R%u", 0, 255)
  )> {
    let RegInfos = RegInfoByHwMode<[Leros16,            Leros32,           Leros64,            DefaultMode],
                                   [RegInfo<16,16,16>,  RegInfo<32,32,32>, RegInfo<64,64,64>,  RegInfo<32,32,32>]>;
}

// Stack pointer
def SP : RegisterClass<"leros", [XLenVT], 32, (add R1)> {
    let RegInfos = RegInfoByHwMode<[Leros16,            Leros32,           Leros64,            DefaultMode],
                                   [RegInfo<16,16,16>,  RegInfo<32,32,32>, RegInfo<64,64,64>,  RegInfo<32,32,32>]>;
}

// Grouping with stack pointer excluded
def GPRNoSP : RegisterClass<"Leros", [XLenVT], 32, (add
    R0,
    (sequence "R%u", 2, 255)
  )> {
    let RegInfos = RegInfoByHwMode<[Leros16,            Leros32,           Leros64,            DefaultMode],
                                   [RegInfo<16,16,16>,  RegInfo<32,32,32>, RegInfo<64,64,64>,  RegInfo<32,32,32>]>;
}