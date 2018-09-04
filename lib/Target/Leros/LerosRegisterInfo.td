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


class LerosReg<bits<5> Enc, string n> : Register<n> {
    let HWEncoding{4-0} = Enc;
}

// Declare the 32 GPR registers
foreach Index = 0-31 in {
    def R#Index# : LerosReg<Index, "r"#Index>, DwarfRegNum<[Index]>;
  }
}

def XLenVT : ValueTypeByHwMode<[Leros16, Leros32, Leros64, DefaultMode],
                               [i16,     i32,     i64,     i32]>;

// General purpose registers
// The order of registers represents the preferred allocation sequence
// Registers are listed in the order caller-save, callee-save, specials.
def GPR : RegisterClass<"Leros", [XLenVT], 32, (add
    (sequence "R%u", 2, 9),
    (sequence "R%u", 23, 31),
    (sequence "R%u", 10, 22),
    (sequence "R%u", 0, 1)
  )> {
    let RegInfos = RegInfoByHwMode<[Leros16,            Leros32,           Leros64,            DefaultMode],
                                   [RegInfo<16,16,16>,  RegInfo<32,32,32>, RegInfo<64,64,64>,  RegInfo<32,32,32>]>;
}

// Grouping with stack pointer excluded
def GPRNoR0 : RegisterClass<"Leros", [XLenVT], 32, (add
    (sequence "R%u", 2, 9),
    (sequence "R%u", 23, 31),
    (sequence "R%u", 10, 22),
    R1
  )> {
    let RegInfos = RegInfoByHwMode<[Leros16,            Leros32,           Leros64,            DefaultMode],
                                   [RegInfo<16,16,16>,  RegInfo<32,32,32>, RegInfo<64,64,64>,  RegInfo<32,32,32>]>;
}

// Stack pointer
def SP : RegisterClass<"leros", [XLenVT], 32, (add R0)> {
    let RegInfos = RegInfoByHwMode<[Leros16,            Leros32,           Leros64,            DefaultMode],
                                   [RegInfo<16,16,16>,  RegInfo<32,32,32>, RegInfo<64,64,64>,  RegInfo<32,32,32>]>;
}