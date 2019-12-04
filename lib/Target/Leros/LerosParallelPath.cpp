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

  std::set<int> modifiedBranchBlock;
  
  if (ParallelPathOpts) {
    TII = static_cast<const LerosInstrInfo *>(MF.getSubtarget().getInstrInfo());    

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
      Modified = false;
      
      LLVM_DEBUG(dbgs() << "bb." << block.getNumber() << "." << block.getName());
      LLVM_DEBUG(dbgs() << ": " << block.size() << " instructions \n");
      LLVM_DEBUG(dbgs() << "Before PPath:\n" << block);

      // TODO: get the terminator and check if it is a end path instruction
      // If so, decrement currentNbPPaths
      
      // No branch statements for loops can use the parallel path feature
      // Conditional statements inner loops can however ...
      loopInfo = MLI->getLoopFor(&block);
      if (loopInfo != NULL && loopInfo->isLoopExiting(&block)) {
	LLVM_DEBUG(dbgs() << "We can exit loop from this block, eliminating ...\n");
	continue;
      }
      
      for (MachineBasicBlock::reverse_iterator MBBI = block.rbegin();
	   MBBI != block.rend(); ++MBBI) {
	MachineInstr *instr = &*MBBI;

	// NOTE: start path instruction are marked as conditional branches,
	// so there is a need to distinguish from classical one?
	if (instr->isConditionalBranch()) {
	  LLVM_DEBUG(dbgs() << "Instruction:" << *instr);

	  // Retrieve the immediate post dominator, i.e. where the paths join/merge
	  assert(MPDT->getNode(&block) &&
		 MPDT->getNode(&block)->getIDom());// &&
		 //MPDT->getNode(&block)->getIDom()->getBlock());
	  // TODO: store a list of join blocks. If the new join block
	  // is found in this list, this means that several end path
	  // instructions will follow in predecessors of this join
	  // block
	  MachineBasicBlock* joinBlock = MPDT->getNode(&block)->getIDom()->getBlock();
	  LLVM_DEBUG(dbgs() << "Join block: bb." << joinBlock->getNumber()
		     << "." << joinBlock->getName() << "\n");

	  // Retrieve the first basic blocks of both branch and fall-through paths	 
	  MachineBasicBlock* branchBlock = NULL;
	  MachineBasicBlock* fallThroughBlock = NULL;	  
	  for (MachineOperand &MO : instr->operands()) {
	    if (MO.isMBB()) {
	      branchBlock = MO.getMBB();
	      LLVM_DEBUG(dbgs() << "Found branch target: bb." << branchBlock->getNumber()
			 << "." << branchBlock->getName() << "\n");	      
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
	    if (sucBlock != branchBlock) {
	      fallThroughBlock = sucBlock;
	      LLVM_DEBUG(dbgs() << "Fall Through target: bb."
			 << fallThroughBlock->getNumber()
			 << "." << fallThroughBlock->getName() << "\n");
	    }
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
	    LLVM_DEBUG(dbgs() << "Pred to join: bb." << predBlock->getNumber()
		       << "." << predBlock->getName() << "\n");
	    
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
	      if (predBlock == &block) {
		LLVM_DEBUG(dbgs() << "Predblock is current block, going for next predblock\n");
		break;
	      }
	      if (predLastInst == NULL || predLastInst->isConditionalBranch()) {
		LLVM_DEBUG(dbgs() << "Last instruction should not be a conditional branch\n");
		continue;
	      }

	      // Checking to which paths the instruction belongs to
	      if (MDT->dominates(branchBlock, predBlock)) {
		LLVM_DEBUG(dbgs() << "branch block dominates predBlock: bb." <<
			   predBlock->getNumber() << "." << predBlock->getName() << "\n");
		branchPathEndInst = predLastInst;
		branchPathEndBlock = predBlock;
	      }
	      if (MDT->dominates(fallThroughBlock, predBlock)) {
		LLVM_DEBUG(dbgs() << "fallTrough block dominates predBlock: bb." <<
			   predBlock->getNumber() << "." << predBlock->getName() << "\n");
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
	  if (branchPathEndInst == NULL && fallThroughPathEndInst == NULL) {
	    LLVM_DEBUG(dbgs() << "Something went wrong when analyzing this conditional branch, skipping ...\n");
	    continue;
	  }
	  
	  LLVM_DEBUG(dbgs() << "Going to insert parallel path instructions ...\n");
	  unsigned opcode = instr->getOpcode();
	  switch (opcode) {
	    OPCASEPP(Leros::BRLT)
	    OPCASEPP(Leros::BRGTE)
	    OPCASEPP(Leros::BREQ)
	    OPCASEPP(Leros::BRNEQ)
	    OPCASEPP(Leros::BRNZ)
	    OPCASEPP(Leros::BRZ)
	    OPCASEPP(Leros::BRP)
	    OPCASEPP(Leros::BRN)		
	  }
	  // TODO: we should be able to know which registers are alive and use that
	  // to set a register to tell the parallel path HW extension which registers
	  // should be sent to the other core
	  instr->setDesc(TII->get(opcode));	    
	  instr = &*(--MBBI);
	  // We erase the next instruction which is BR_MI
	  if (MBBI->getOpcode() == Leros::BR_MI) {
	    LLVM_DEBUG(dbgs() << "Removing previous instruction: " << *MBBI);	    
	    MBBI->eraseFromParent();
	  } else {
	    LLVM_DEBUG(dbgs() << "Strange previous instruction: " << *MBBI);
	  }
	  Modified = true;

	  DebugLoc DL;

	  // TODO: incorrect as we could reach a block which has already been modified.
	  ++NumBranchPPaths;
	  if (++currentNbPPaths > MaxPPaths) MaxPPaths = currentNbPPaths;
	  ++NumEndPPaths;
	  
	  if (branchPathEndInst == NULL && fallThroughPathEndInst != NULL) {
	    LLVM_DEBUG(dbgs() << "Triangle conditional branch shape\n");
	    
	    if (fallThroughPathEndInst->getOpcode() == Leros::BREPP_MI) {
	      LLVM_DEBUG(dbgs() << "In fall through, already an end path instruction\n");
	      BuildMI(fallThroughPathEndBlock, DL, TII->get(Leros::BREPP_MI)).addMBB(joinBlock);
	    } else fallThroughPathEndInst->setDesc(TII->get(Leros::BREPP_MI));
	    
	    ++NumTrianglePPaths;
	    
	  } else if (branchPathEndInst != NULL && fallThroughPathEndInst != NULL) {
	    LLVM_DEBUG(dbgs() << "Diamond conditional branch shape\n");
	    
	    if (fallThroughPathEndInst->getOpcode() == Leros::BREPP_MI) {
	      LLVM_DEBUG(dbgs() << "In fall through, already an end path instruction\n");
	      BuildMI(fallThroughPathEndBlock, DL, TII->get(Leros::BREPP_MI)).addMBB(joinBlock);  
	    } else fallThroughPathEndInst->setDesc(TII->get(Leros::BREPP_MI));
	    
	    // If the branch block targets an branch block already analyze then
	    // we should simply do nothing (a single end of path instruction is correct)	  
	    if (modifiedBranchBlock.find(branchBlock->getNumber()) == modifiedBranchBlock.end()) {	    	  
	      if (branchPathEndInst->getOpcode() == Leros::BREPP_MI) {
		LLVM_DEBUG(dbgs() << "In branch, already an end path instruction\n");
		BuildMI(branchPathEndBlock, DL, TII->get(Leros::BREPP_MI)).addMBB(joinBlock);
	      } else branchPathEndInst->setDesc(TII->get(Leros::BREPP_MI));

	      modifiedBranchBlock.insert(branchBlock->getNumber());
	      LLVM_DEBUG(dbgs() << "Inserting bb number: " << branchBlock->getNumber() << "\n");

	      ++NumEndPPaths;	      
	      
	    } else {
	      LLVM_DEBUG(dbgs() << "Branch block already modified, no need for an additional end path instruction\n");

	      
	    
	    ++NumDiamondPPaths;
	  } else {
	    llvm_unreachable("Invalid branch should have been detected before ...\n");
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
	  if (branchPathEndBlock != NULL) {
	    branchPathEndBlock->printAsOperand(OS,false);
	  } else {
	    OS << "TR-shape";
	  }
	  OS << ",";
	  fallThroughPathEndBlock->printAsOperand(OS,false);
	  LLVM_DEBUG(dbgs() << "Branch from: " << OS.str() << ")\n");

	  // We break as we have mess up the iterator ...
	  break;
	  
	} else if (instr->isUnconditionalBranch()) {
	  LLVM_DEBUG(dbgs() << "Unconditional branch instruction, skipping\n");
	} else {
	  LLVM_DEBUG(dbgs() << "No more branch instructions, skipping\n");
	  break;
	}	
      }
      if (Modified) LLVM_DEBUG(dbgs()  << "After PPath:\n" << block);
    }
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
