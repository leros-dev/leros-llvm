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
    
class LerosReg<bits<8> Enc, string n, list<string> alt = []> : Register<n> {
    let HWEncoding{7-0} = Enc;
    let AltNames = alt;
}

def ABIRegAltName : RegAltNameIndex;

// Declare the 256 GPR registers
let RegAltNameIndices = [ABIRegAltName] in {
    def RA : LerosReg<0, "r0", ["ra"]>, DwarfRegNum<[0]>;
    def SP : LerosReg<1, "r1", ["sp"]>, DwarfRegNum<[1]>;
    def GP : LerosReg<2, "r2", ["gp"]>, DwarfRegNum<[2]>;
    def FP : LerosReg<3, "r3", ["fp"]>, DwarfRegNum<[3]>;
}
let RegAltNameIndices = [NoRegAltName] in {
    foreach Index = 4-255 in {
        def R#Index# : LerosReg<Index, "r"#Index>, DwarfRegNum<[Index]>;
    }
}


def XLenVT : ValueTypeByHwMode<[Leros32, Leros64, DefaultMode],
                               [i32,     i64,     i32]>;

// General purpose registers
def GPR : RegisterClass<"Leros", [XLenVT], 32, (add
    RA, SP, GP, FP,
    (sequence "R%u", 4, 250)
  )> {
    let RegInfos = RegInfoByHwMode<[Leros32,           Leros64,            DefaultMode],
                                   [RegInfo<32,32,32>, RegInfo<64,64,64>,  RegInfo<32,32,32>]>;
}

// Non-reserved registers
def GPRNoReserve : RegisterClass<"Leros", [XLenVT], 32, (add
    (sequence "R%u", 4, 250)
  )> {
    let RegInfos = RegInfoByHwMode<[Leros32,           Leros64,            DefaultMode],
                                   [RegInfo<32,32,32>, RegInfo<64,64,64>,  RegInfo<32,32,32>]>;
}

// Registers that can be used post-register allocation by pseudo instr. expansion
def GPRPseudoExpand : RegisterClass<"Leros", [XLenVT], 32, (add
    (sequence "R%u", 251, 255)
  )> {
    let RegInfos = RegInfoByHwMode<[Leros32,           Leros64,            DefaultMode],
                                   [RegInfo<32,32,32>, RegInfo<64,64,64>,  RegInfo<32,32,32>]>;
}

// Define the accumulator as a register - is this the best way to define an
// implicit operand that still should be considered in the DAGs?
def ACC : Register<"acc">, DwarfRegNum<[-1]>;

// Accumulator - we treat this as a separate register outside of the GPR definitions
def ACC_OP : RegisterClass<"Leros", [XLenVT], 32, (add
    ACC
)> {
    let RegInfos = RegInfoByHwMode<[Leros32,           Leros64,            DefaultMode],
                                   [RegInfo<32,32,32>, RegInfo<64,64,64>,  RegInfo<32,32,32>]>;
}

}