//===- LerosISelDAGToDAG.cpp - A dag to dag inst selector for Leros ---===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines an instruction selector for the Leros target.
//
//===----------------------------------------------------------------------===//

#include "Leros.h"
#include "LerosTargetMachine.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/SelectionDAGISel.h"
#include "llvm/Support/Compiler.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"

namespace llvm {

#define DEBUG_TYPE "leros-isel"

// Leros  specific code to select Leros machine
// instructions for SelectionDAG operations.
namespace {
class LerosDAGToDAGISel : public SelectionDAGISel {
  const LerosSubtarget *Subtarget;

public:
  LerosDAGToDAGISel(LerosTargetMachine &TM) : SelectionDAGISel(TM) {}

  StringRef getPassName() const override {
    return "Leros DAG->DAG Pattern Instruction Selection";
  }

  void PostprocessISelDAG() override;

  bool runOnMachineFunction(MachineFunction &MF) override {
    Subtarget = &MF.getSubtarget<LerosSubtarget>();
    return SelectionDAGISel::runOnMachineFunction(MF);
  }

  void Select(SDNode *N) override;

  bool SelectAddrFI(SDValue Addr, SDValue &Base);

public:
// Include the pieces autogenerated from the target description.
#include "LerosGenDAGISel.inc"

private:
  void doPeepholeLoadStoreADDI();
};
} // end anonymous namespace

void LerosDAGToDAGISel::PostprocessISelDAG() { doPeepholeLoadStoreADDI(); }

void LerosDAGToDAGISel::Select(SDNode *Node) {
  // If we have a custom node, we have already selected.
  if (Node->isMachineOpcode()) {
    LLVM_DEBUG(dbgs() << "== "; Node->dump(CurDAG); dbgs() << "\n");
    Node->setNodeId(-1);
    return;
  }

  // Instruction Selection not handled by the auto-generated tablegen selection
  // should be handled here.
  unsigned Opcode = Node->getOpcode();
  MVT XLenVT = Subtarget->getXLenVT();
  SDLoc DL(Node);
  EVT VT = Node->getValueType(0);

  if (Opcode == ISD::FrameIndex) {
    SDValue Imm = CurDAG->getTargetConstant(0, DL, XLenVT);
    int FI = cast<FrameIndexSDNode>(Node)->getIndex();
    SDValue TFI = CurDAG->getTargetFrameIndex(FI, VT);
    ReplaceNode(Node,
                CurDAG->getMachineNode(Leros::ADD_RI_PSEUDO, DL, VT, TFI, Imm));
    return;
  }

  // Select the default instruction.
  SelectCode(Node);
}

bool LerosDAGToDAGISel::SelectAddrFI(SDValue Addr, SDValue &Base) {
  if (auto FIN = dyn_cast<FrameIndexSDNode>(Addr)) {
    Base = CurDAG->getTargetFrameIndex(FIN->getIndex(), Subtarget->getXLenVT());
    return true;
  }
  return false;
}

// Merge an ADD_RI_PSEUDO into the offset of a load/store instruction where
// possible. This is only possible for offsets in a signed 8 bit range, and as
// such we only do it if the source operand for a load/store instruction stems
// from a ADD_RI_PSEUDO and not a loadh sequence
void LerosDAGToDAGISel::doPeepholeLoadStoreADDI() {
  SelectionDAG::allnodes_iterator Position(CurDAG->getRoot().getNode());
  ++Position;

  while (Position != CurDAG->allnodes_begin()) {
    SDNode *N = &*--Position;
    // Skip dead nodes and any non-machine opcodes.
    if (N->use_empty() || !N->isMachineOpcode())
      continue;

    int OffsetOpIdx;
    int BaseOpIdx;

    // Attempt optimization for loads and stores
    switch (N->getMachineOpcode()) {
    default:
      continue;
    case Leros::LOAD_M_PSEUDO:
      BaseOpIdx = 0;
      OffsetOpIdx = 1;
      break;
    case Leros::STORE_M_PSEUDO:
      BaseOpIdx = 1;
      OffsetOpIdx = 2;
      break;
    }

    // Currently, the load/store offset must be 0 to be considered for this
    // peephole optimisation.
    if (!isa<ConstantSDNode>(N->getOperand(OffsetOpIdx)) ||
        N->getConstantOperandVal(OffsetOpIdx) != 0)
      continue;

    SDValue Base = N->getOperand(BaseOpIdx);

    // If the base is an LOAD_RI_PSEUDO, we can merge it in to the load/store.
    if (!Base.isMachineOpcode() ||
        Base.getMachineOpcode() != Leros::ADD_RI_PSEUDO)
      continue;

    SDValue ImmOperand = Base.getOperand(1);

    if (auto Const = dyn_cast<ConstantSDNode>(ImmOperand)) {
      ImmOperand = CurDAG->getTargetConstant(
          Const->getSExtValue(), SDLoc(ImmOperand), ImmOperand.getValueType());
    } else if (auto GA = dyn_cast<GlobalAddressSDNode>(ImmOperand)) {
      ImmOperand = CurDAG->getTargetGlobalAddress(
          GA->getGlobal(), SDLoc(ImmOperand), ImmOperand.getValueType(),
          GA->getOffset(), GA->getTargetFlags());
    } else {
      continue;
    }

    LLVM_DEBUG(dbgs() << "Folding add-immediate into mem-op:\nBase:    ");
    LLVM_DEBUG(Base->dump(CurDAG));
    LLVM_DEBUG(dbgs() << "\nN: ");
    LLVM_DEBUG(N->dump(CurDAG));
    LLVM_DEBUG(dbgs() << "\n");

    // Modify the offset operand of the load/store.
    if (BaseOpIdx == 0) // Load
      CurDAG->UpdateNodeOperands(N, Base.getOperand(0), ImmOperand,
                                 N->getOperand(2));
    else // Store
      CurDAG->UpdateNodeOperands(N, N->getOperand(0), Base.getOperand(0),
                                 ImmOperand, N->getOperand(3));

    // The add-immediate may now be dead, in which case remove it.
    if (Base.getNode()->use_empty())
      CurDAG->RemoveDeadNode(Base.getNode());
  }
}
} // namespace llvm

/// createLerosISelDag - This pass converts a Lerosalized DAG into a
/// Leros-specific DAG, ready for instruction scheduling.
///
llvm::FunctionPass *llvm::createLerosISelDag(LerosTargetMachine &TM) {
  return new LerosDAGToDAGISel(TM);
}
