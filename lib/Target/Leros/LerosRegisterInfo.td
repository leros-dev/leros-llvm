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
let RegAltNameIndices = [NoRegAltName] in {
    foreach Index = 0-255 in {
        def R#Index# : LerosReg<Index, "r"#Index>, DwarfRegNum<[Index]>;
    }
}
} // namespace Leros


def XLenVT : ValueTypeByHwMode<[Leros32, Leros64, DefaultMode],
                               [i32,     i64,     i32]>;

// General purpose registers
def GPR : RegisterClass<"Leros", [XLenVT], 32, (add
    (sequence "R%u", 0, 250)
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
