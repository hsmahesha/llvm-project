//===-- X86CFCSS.cpp - CFCSS Implementation -------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "X86.h"
#include "X86InstrBuilder.h"
#include "X86InstrInfo.h"
#include "X86Subtarget.h"
#include  "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/IR/DebugLoc.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/InitializePasses.h"
#include "llvm/Support/Debug.h"
#include "llvm/Transforms/Utils/BasicBlockUtils.h"

using namespace llvm;

#define DEBUG_TYPE "x86-cfcss"

namespace {

class X86CFCSS : public ModulePass {
public:
  static char ID;

  X86CFCSS() : ModulePass(ID) {
    initializeX86CFCSSPass(*PassRegistry::getPassRegistry());
  }

  void getAnalysisUsage(AnalysisUsage &AU) const override {
    AU.addRequired<MachineModuleInfoWrapperPass>();
    AU.addPreserved<MachineModuleInfoWrapperPass>();
    AU.setPreservesAll();
    ModulePass::getAnalysisUsage(AU);
  }

  bool runOnModule(Module &M) override;
  bool run(MachineFunction &MF, MachineFunction &ErrMF);

  StringRef getPassName() const override { return "X86 CFCSS"; }
};

} // namespace

char X86CFCSS::ID = 0;

INITIALIZE_PASS_BEGIN(X86CFCSS, DEBUG_TYPE, "X86 CFCSS Implementation",
                      false, false)
INITIALIZE_PASS_END(X86CFCSS, DEBUG_TYPE, "X86 CFCSS Implementation",
                    false, false)

ModulePass *llvm::createX86CFCSSPass() {
  return new X86CFCSS();
}

static MachineInstr *InsertMovRIInst(MachineBasicBlock &MBB,
     MachineBasicBlock::iterator I, const DebugLoc &DL, const X86InstrInfo *TII,
     unsigned Imm, bool Is64Bit, bool IsG) {
  if (Is64Bit)
    return BuildMI(MBB, I, DL, TII->get(X86::MOV64ri),
                   IsG ? X86::RCX : X86::RDX).addImm(Imm);
  else
    return BuildMI(MBB, I, DL, TII->get(X86::MOV32ri),
            IsG ? X86::ECX : X86::EDX).addImm(Imm);
}

static MachineInstr *InsertXORRIInst(MachineBasicBlock &MBB,
     MachineBasicBlock::iterator I, const DebugLoc &DL, const X86InstrInfo *TII,
     unsigned Imm, bool Is64Bit, bool IsG) {
  if (Is64Bit) {
    Register Reg = IsG ? X86::RCX : X86::RDX;
    return BuildMI(MBB, I, DL, TII->get(X86::XOR64ri32), Reg)
        .addReg(Reg)
        .addImm(Imm);
  } else {
    Register Reg = IsG ? X86::ECX : X86::EDX;
    return BuildMI(MBB, I, DL, TII->get(X86::XOR32ri), Reg)
        .addReg(Reg)
        .addImm(Imm);
  }
}

static MachineInstr *InsertXORRRInst(MachineBasicBlock &MBB,
     MachineBasicBlock::iterator I, const DebugLoc &DL, const X86InstrInfo *TII,
     bool Is64Bit) {
  if (Is64Bit) {
    Register Reg1 = X86::RCX;
    Register Reg2 = X86::RDX;
    return BuildMI(MBB, I, DL, TII->get(X86::XOR64rr), Reg1)
        .addReg(Reg1)
        .addReg(Reg2);
  } else {
    Register Reg1 = X86::ECX;
    Register Reg2 = X86::EDX;
    return BuildMI(MBB, I, DL, TII->get(X86::XOR32rr), Reg1)
        .addReg(Reg1)
        .addReg(Reg2);
  }
}

static MachineInstr *InsertCallInst(MachineBasicBlock &MBB, MachineInstr &MI,
     const DebugLoc &DL, const X86InstrInfo *TII, bool Is64Bit,
     MachineFunction &ErrF) {
}

static MachineFunction *GetMachineFunction(Function *F,
    MachineModuleInfo *MMI) {
  MachineFunction &MF = MMI->getOrCreateMachineFunction(*F);
  MF.getProperties().reset(MachineFunctionProperties::Property::TracksLiveness);
  MF.getProperties().set(MachineFunctionProperties::Property::NoVRegs);
  MF.getRegInfo().freezeReservedRegs(MF);

  // FIXME: Not working
  //MF.CreateMachineBasicBlock(EntryB);
  //MF.CreateMachineBasicBlock(ExitB);
  //MF.CreateMachineBasicBlock(RetB);

  return &MF;
}

static Function *CreateErrorFunction(Module *M, StringRef Name) {
  // Create error function.
  LLVMContext &C = M->getContext();
  SmallVector<Type *, 2> I32Tys(2, Type::getInt32Ty(C));
  FunctionType *ErrFnTy = FunctionType::get(Type::getVoidTy(C), I32Tys, false);
  Function *ErrF =
      Function::Create(ErrFnTy, GlobalValue::InternalLinkage, Name, M);

  // Create exit(0) function if required.
  Function *ExitF = nullptr;
  if (!(ExitF = M->getFunction("exit"))) {
    //SmallVector<Type *, 1> I32Tys(1, Type::getInt32Ty(C));
    //FunctionType *ExitFnTy =
    //    FunctionType::get(Type::getVoidTy(C), I32Tys, false);
    FunctionType *ExitFnTy = FunctionType::get(Type::getVoidTy(C), false);
    ExitF = Function::Create(ExitFnTy, GlobalValue::ExternalLinkage, "exit", M);
  }

  // Entry block
  auto *EntryB = BasicBlock::Create(C, "entry", ErrF);
  IRBuilder<> builder(EntryB);
  Instruction *CmpI =
      cast<Instruction>(builder.CreateICmpNE(ErrF->getArg(0), ErrF->getArg(1)));
  Instruction *RetI = cast<Instruction>(builder.CreateRetVoid());

  // Exit (on error) block
  auto *ExitB= BasicBlock::Create(C, "exit", ErrF);
  builder.SetInsertPoint(ExitB);
  builder.CreateCall(ExitF);

  // Return Block
  SplitBlockAndInsertIfThen(CmpI, RetI, false, nullptr,
                            static_cast<DomTreeUpdater *>(nullptr), nullptr,
                            ExitB);
  return ErrF;
}


// Apply CFCSS algorithm on machine function MF.
bool X86CFCSS::run(MachineFunction &MF, MachineFunction &ErrMF) {
  bool Changed = false;
  const X86Subtarget &STI = MF.getSubtarget<X86Subtarget>();
  const X86InstrInfo *TII = STI.getInstrInfo();
  const bool Is64Bit = STI.is64Bit();

  // Assign unique, compile time reference signature to each machine basic
  // block.
  unsigned MBBNo = 0;
  DenseMap<MachineBasicBlock *, unsigned> SMap;
  for (MachineBasicBlock &MBB : MF)
      SMap.insert(std::make_pair(&MBB, ++MBBNo));

  // At the beginning of each machine basic block MBB, insert a set of
  // instructions to compute the run time signature of MBB, and run time
  // adjusting signature wherever required.
  for (MachineBasicBlock &MBB : MF) {
    MachineBasicBlock::iterator I = MBB.getFirstNonDebugInstr();
    MachineInstr &MI = *I;
    unsigned S = SMap[&MBB];

    if (MBB.pred_empty()) {
      // MBB is entry block, run time siganture of MBB is nothing but the
      // compile time reference signature of MBB.
      InsertMovRIInst(MBB, I, MI.getDebugLoc(), TII, S, Is64Bit, /*IsG=*/true);
    } else {
      // MBB has one or more predecessors. Get the reference signature of first
      // predecessor, and use it for computing the run time signature of MBB.
      unsigned FPS = SMap[*MBB.pred_begin()];
      unsigned d = FPS ^ S;

      InsertXORRIInst(MBB, I, MI.getDebugLoc(), TII, d, Is64Bit, /*IsG=*/true);

      // If MBB has more than one predecessors, then, run time signature need to
      // be adjusted with run time adjusting signature.
      if (MBB.pred_size()  > 1) {
        // Make sure that run time adjusting signature is properly computed
        // within all predecessors block.
        for (MachineBasicBlock *Pred : MBB.predecessors()) {
          MachineBasicBlock::iterator PI = Pred->getFirstTerminator();
          MachineInstr *PMI = nullptr;
          if (PI == Pred->end())
            PMI = &*Pred->getLastNonDebugInstr();
          else
            PMI = &*PI;
          unsigned D = 0;
          if (Pred != *MBB.pred_begin()) {
            unsigned CPS = SMap[Pred];
            D = FPS ^ CPS;
          }
          InsertMovRIInst(*Pred, PI, PMI->getDebugLoc(), TII, D, Is64Bit,
                          /*IsG=*/false);
        }

        // Adjust run time signature of MBB using run time adjusting signature.
        InsertXORRRInst(MBB, I, MI.getDebugLoc(), TII, Is64Bit);
      }
    }

    Changed = true;
  }

  return Changed;
}

bool X86CFCSS::runOnModule(Module &M) {
  MachineModuleInfo *MMI =
       &getAnalysis<MachineModuleInfoWrapperPass>().getMMI();

  // Insert definition for "exit on control flow error" function.
  Function *ErrF = CreateErrorFunction(&M, "__cfcss_error");
  MachineFunction *ErrMF = GetMachineFunction(ErrF, MMI);

  // Apply CFCSS algorithm to each machine function within the module M.
  bool Changed = false;
  for (Function &F : M) {
    if (F.isDeclaration())
       continue;

    if (&F == ErrF)
       continue;

     MachineFunction *MF = MMI->getMachineFunction(F);
     if (!MF)
       continue;

     Changed |= run(*MF, *ErrMF);
  }

  return Changed;

  return true;
}
