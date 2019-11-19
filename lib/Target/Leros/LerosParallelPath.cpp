#include "Leros.h"
#include "LerosInstrInfo.h"
#include "LerosTargetMachine.h"

#include "llvm/Support/Debug.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/ADT/Statistic.h"

#include "llvm/CodeGen/LivePhysRegs.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineDominators.h"
#include "llvm/CodeGen/MachinePostDominators.h"
#include "llvm/CodeGen/MachineLoopInfo.h"

using namespace llvm;

#define LEROS_PARALLEL_PATH_NAME "Leros Parallel Path pass"

#define DEBUG_TYPE "ppath"

STATISTIC(NumTrianglePPaths, "Number of parallel paths inserted (triangle shape)");
STATISTIC(NumDiamondPPaths, "Number of parallel paths inserted (diamond shape)");
STATISTIC(NumBranchPPaths, "Number of branch parallel path instructions");
STATISTIC(MaxPPaths, "Max parallel path parallelism");
STATISTIC(NumEndPPaths, "Number of end parallel path instructions");

static cl::opt<bool>
    ParallelPathOpts("ppath-opt", cl::Hidden,
		     cl::desc("Activate the parallel path transformation"),
		     cl::init(false));

namespace {

class LerosParallelPath : public MachineFunctionPass {
public:
  const LerosInstrInfo *TII;
  static char ID;

  MachineDominatorTree *MDT;
  MachinePostDominatorTree *MPDT;
  MachineLoopInfo *MLI;

  LerosParallelPath() : MachineFunctionPass(ID) {
    initializeLerosParallelPathPass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &MF) override;

  void getAnalysisUsage(AnalysisUsage &AU) const override {
    AU.addRequired<MachineDominatorTree>();
    AU.addRequired<MachinePostDominatorTree>();
    AU.addRequiredTransitive<MachineLoopInfo>();    
    MachineFunctionPass::getAnalysisUsage(AU);
  }
  
  StringRef getPassName() const override { return LEROS_PARALLEL_PATH_NAME; }
};

#define OPCASEPP(instr)                                                      \
  case instr##_PSEUDO:                                                       \
    opcode = instr##P_PSEUDO;                                                \
    break;

bool LerosParallelPath::runOnMachineFunction(MachineFunction &MF) {
  LLVM_DEBUG(dbgs() << "Function: " << MF.getName() << "\n");
  // Did we modified a BB?
  bool Modified = false;
  
  if (ParallelPathOpts) {
    TII = static_cast<const LerosInstrInfo *>(MF.getSubtarget().getInstrInfo());    
    // Are we in a parallel path
    bool parallelPathEna = false;

    // If we want to work before Instruction Selection
    //    const Function &Fn = MF.getFunction();
    MLI = &getAnalysis<MachineLoopInfo>();
    MDT = &getAnalysis<MachineDominatorTree>();
    MPDT = &getAnalysis<MachinePostDominatorTree>();

    // To store the current parallel path parallelism
    unsigned int currentNbPPaths = 1;    

    // If we want to work before Instruction Selection    
    for (MachineBasicBlock &block : MF) {
      MachineLoop *loopInfo; 
      LLVM_DEBUG(dbgs() << "bb." << block.getNumber() << ": "
		 << block.size() << " instructions \n");
      LLVM_DEBUG(dbgs()  << "Before PPath:\n" << block);

      // TODO: get the terminator and check if it is a end path instruction
      // If so, decrement currentNbPPaths
      
      // No branch statements for loops can use the parallel path feature
      // Conditional statements inner loops can however ...
      loopInfo = MLI->getLoopFor(&block);
      if (loopInfo != NULL && loopInfo->isLoopExiting(&block)) {
	LLVM_DEBUG(dbgs() << "We can exit loop from here " << *loopInfo << ", eliminating ...\n");
	continue;
      }
      
      for (MachineBasicBlock::reverse_iterator MBBI = block.rbegin();
	   MBBI != block.rend(); ++MBBI) {
	MachineInstr *instr = &*MBBI;

	// NOTE: start path instruction should be marked as conditional branches!
	// so there is a need to distinguish !!!
	if (instr->isConditionalBranch()) {
	  LLVM_DEBUG(dbgs() << "Instruction:" << *instr << "\n");

	  // Retrieve the immediate post dominator, i.e. where the paths join/merge
	  assert(MPDT->getNode(&block) &&
		 MPDT->getNode(&block)->getIDom());// &&
		 //MPDT->getNode(&block)->getIDom()->getBlock());
	  // TODO: store a list of join blocks. If the new join block
	  // is found in this list, this means that several end path
	  // instructions will follow in predecessors of this join
	  // block
	  MachineBasicBlock* joinBlock = MPDT->getNode(&block)->getIDom()->getBlock();

	  // Retrieve the first basic blocks of both branch and fall-through paths	 
	  MachineBasicBlock* branchBlock = NULL;
	  MachineBasicBlock* fallThroughBlock = NULL;	  
	  for (MachineOperand &MO : instr->operands()) {
	    if (MO.isMBB()) {
	      LLVM_DEBUG(dbgs() << "Found branch target:" << MO << "\n");
	      branchBlock = MO.getMBB();
	    }
	  }
	  unsigned nbSuc = 0;
	  for (MachineBasicBlock::succ_iterator SBB = block.succ_begin(),
		 SBBE = block.succ_end(); SBB != SBBE; ++SBB) {
	    MachineBasicBlock* sucBlock = *SBB;
	    if (++nbSuc > 2) {
	      LLVM_DEBUG(dbgs() << "More than 2 successors, eliminating\n");
	      break;
	    }
	    if (sucBlock != branchBlock) fallThroughBlock = sucBlock;
	  }
	  if (nbSuc > 2) break;
	  
	  // Let's identify the last blocks of both branch and fall-through paths 
	  unsigned nbPred = 0;
	  MachineInstr* branchPathEndInst = NULL;
	  MachineInstr* fallThroughPathEndInst = NULL;
	  MachineBasicBlock* branchPathEndBlock = NULL;
	  MachineBasicBlock* fallThroughPathEndBlock = NULL;	  	  
	  for (MachineBasicBlock::pred_iterator PBB = joinBlock->pred_begin(),
		 PBBE = joinBlock->pred_end(); PBB != PBBE; ++PBB) {
	    MachineBasicBlock* predBlock = *PBB;
	    
	    // Triangle and diamonds shapes only
	    if (++nbPred > 2) break;
	    
	    assert (!(MDT->dominates(branchBlock, predBlock) &&
		      MDT->dominates(fallThroughBlock, predBlock)) &&
		    "Pred block can not be dominated by both branch and fall through blocks");
	    
	    // Checking the type of the last instruction
	    // TODO: this is incorrect as it could already have a end path instruction
	    // We need to add another end path instruction if we found one.
	    MachineInstr* predLastInst = NULL;
	    for (MachineBasicBlock::iterator TBBI = predBlock->getFirstTerminator(),
		   TBBE = predBlock->end(); TBBI != TBBE; ++TBBI) {
	      predLastInst = &*TBBI;
	      if (predLastInst == NULL || predLastInst->isConditionalBranch()) {
		LLVM_DEBUG(dbgs() << "Last instruction must be an unconditional branch\n");
		// TODO: if it is a conditional branch, then it is the fall trough, no?
		// TODO: check this ...
		// We have to continue in case we first see the fall-through path and not
		// the branch path
		continue;
	      }	    
	      
	      // Checking to which paths the instruction belongs to
	      if (MDT->dominates(branchBlock, predBlock)) {
		branchPathEndInst = predLastInst;
		branchPathEndBlock = predBlock;
	      }
	      if (MDT->dominates(fallThroughBlock, predBlock)) {
		fallThroughPathEndInst = predLastInst;
		fallThroughPathEndBlock = predBlock;
	      }
	    }
	  }
	  
	  // Triangle and diamond shapes only
	  if (nbPred > 2) {
	    LLVM_DEBUG(dbgs() << "Merge block has more than 2 predecessors, eliminating ...\n");
	    continue;
	  }
	  if (branchPathEndInst == NULL) {
	    LLVM_DEBUG(dbgs() << "Conditional branch is at least a triangle shape\n");
	    continue;
	  }
	  
	  LLVM_DEBUG(dbgs() << "TODO: change LLVM IR for parallel path instruction\n");
	  // Now let's modify the LLVM IR for parallel path instructions
	  // TODO: we must add information from the initial instructions
	  // auto *newInst = new branchSPPathInst(Type:: , 0, "brspp");
	  // ReplaceInstWithInst(brFrom, newInst);
	  ++NumBranchPPaths;
	  if (++currentNbPPaths > MaxPPaths) MaxPPaths = currentNbPPaths;
	  // newInst = new branchEPPathInst(Type:: , 0, "brepp");	  
	  // ReplaceInstWithInst(branchPathEndInst, newInst);
	  ++NumEndPPaths;
	  if (fallThroughPathEndInst != NULL) {
	    LLVM_DEBUG(dbgs() << "Diamond conditional branch shape\n");
	    //   newInst = new branchEPPathInst(Type:: , 0, "brepp");
	    //   ReplaceInstWithInst(fallThroughPathEndInst, newInst);
	    ++NumDiamondPPaths;
	    ++NumEndPPaths;
	  } else {
	    LLVM_DEBUG(dbgs() << "Triangle conditional branch shape\n");
	    ++NumTrianglePPaths;	  
	  }
	
	  // For display purpose (BB can have no names ...)
	  std::string Str;
	  raw_string_ostream OS(Str);
	  block.printAsOperand(OS,false);
	  OS << " fw (";
	  branchBlock->printAsOperand(OS,false);	  	  
	  OS << ",";
	  fallThroughBlock->printAsOperand(OS,false);	  	  	  
	  OS << ") merging ";
	  joinBlock->printAsOperand(OS,false);
	  OS << ", last (";
	  branchPathEndBlock->printAsOperand(OS,false);
	  OS << ",";
	  fallThroughPathEndBlock->printAsOperand(OS,false);
	  LLVM_DEBUG(dbgs() << "Branch from: " << OS.str() << ")\n");
	  
	} else if (instr->isUnconditionalBranch()) {
	  LLVM_DEBUG(dbgs() << "Unconditional branch instruction, skipping\n");
	} else {
	  LLVM_DEBUG(dbgs() << "No more branch instructions, skipping\n");
	  break;
	}
      }
      
      //LLVM_DEBUG(dbgs()  << "After PPath:\n" << &block);
    }
    
	//   if (!parallelPathEna) {
	//     MachineBasicBlock::iterator NMBBI = std::next(MBBI);	    	    
	//     // We switch to the path instruction
	//     unsigned opcode = MBBI->getOpcode();
	//     switch (opcode) {
	//       OPCASEPP(Leros::BRLT)
	//       OPCASEPP(Leros::BRGTE)
	//       OPCASEPP(Leros::BREQ)
	//       OPCASEPP(Leros::BRNEQ)
	//       OPCASEPP(Leros::BRNZ)
	//       OPCASEPP(Leros::BRZ)
	//       OPCASEPP(Leros::BRP)
	//       OPCASEPP(Leros::BRN)		
	//     }
	//     // TODO: we should be able to know which registers are alive and use that
	//     // to set a register to tell the parallel path HW extension which registers
	//     // should be sent to the other core
	//     MBBI->setDesc(TII->get(opcode));	    
	//     MBBI = NMBBI;
	//     NMBBI = std::next(MBBI);	    	    	    
	//     // We erase the next instruction which is BR_MI (we should check it)
	//     if (MBBI->getOpcode() == Leros::BR_MI) {
	//       MBBI->eraseFromParent();
	//     } else {
	//       LLVM_DEBUG(dbgs() << "Strange pattern, next instruction: " << *MBBI);
	//     }
	//     MBBI = NMBBI;	    
	//     Modified = true;
	//     parallelPathEna = true;
	//     ++NumPPaths;
	//   } else {
	//     LLVM_DEBUG(dbgs() << "We are already in a parallel path and a depth higher than 1 is not supported currently!\n");
	//     // TODO: allow parallel paths of depth higher than 1 (as in switch) 
	//   }
	//   break;
	// default:
	//   break;
	// }
	// MBBI++;
  }
  return Modified;
}

} // end of anonymous namespace

char LerosParallelPath::ID = 0;

INITIALIZE_PASS_BEGIN(LerosParallelPath, "Leros-parallel-path",
		      LEROS_PARALLEL_PATH_NAME, false, false)
INITIALIZE_PASS_DEPENDENCY(MachineDominatorTree)
INITIALIZE_PASS_DEPENDENCY(MachinePostDominatorTree)
INITIALIZE_PASS_DEPENDENCY(MachineLoopInfo)
INITIALIZE_PASS_END(LerosParallelPath, "Leros-parallel-path",
		    LEROS_PARALLEL_PATH_NAME, false, false)

namespace llvm {

FunctionPass *createLerosParallelPathPass() {
  return new LerosParallelPath();
}

} // end of namespace llvm
