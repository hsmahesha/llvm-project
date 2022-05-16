//===-- CFCSS.cpp -  CFCSS Implementation ---------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "llvm/Transforms/Instrumentation/CFCSS.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/Transforms/Utils/BasicBlockUtils.h"

using namespace llvm;

// Insert "exit on control error" function within the module M. The error
// function looks like below.
//
// define void @cttErr() {
//   entry:
//     call void @exit()
//     ret void
// }
static Function *insertControlFlowErrorHandlingFunction(Module &M) {
  // Create "exit on control error" function.
  auto *FnTy = FunctionType::get(Type::getVoidTy(M.getContext()), false);
  auto *ErrFunc =
      Function::Create(FnTy, GlobalValue::ExternalLinkage, "cttErr", M);

  // Insert a basic block within above created error function which has a call
  // to "exit()".
  auto *BB = BasicBlock::Create(M.getContext(), "entry", ErrFunc);
  auto *exitFunc =
      Function::Create(FnTy, GlobalValue::ExternalLinkage, "exit", M);
  IRBuilder<> builder(BB);
  builder.CreateCall(exitFunc);
  builder.CreateRetVoid();

  return ErrFunc;
}

// Apply CFCSS algorithm to function F as described in the paper:
//   http://crc.stanford.edu/crc_papers/CRC-TR-00-4.pdf
void CFCSSPass::applyCFCSS(Function &F, ModuleAnalysisManager &AM) {
  Module *M = F.getParent();
  LLVMContext &Ctx = M->getContext();
  Type *I64Ty = Type::getInt64Ty(Ctx);

  // Insert an "exit on error" basic block ErrBB within function F so that the
  // control gets transfered to ErrBB upon control flow error. Within ErrBB, a
  // call is inserted to "exit on control flow error" function "cttErr()".
  BasicBlock *ErrBB = BasicBlock::Create(Ctx, "err.exit", &F);
  IRBuilder<> builder(ErrBB);
  builder.CreateCall(CttErrFunc);
  if (F.getReturnType() == Type::getVoidTy(Ctx))
    builder.CreateRetVoid();
  else
    builder.CreateRet(UndefValue::get(F.getReturnType()));

  // Assign unique, compile time reference signature to each basic block.
  unsigned bbNo = 0;
  DenseMap<BasicBlock *, unsigned> SMap;
  for (BasicBlock &BB : F) {
    if (&BB != ErrBB)
      SMap.insert(std::make_pair(&BB, ++bbNo));
  }

  // At the beginning of each basic block BB, insert a set of instructions to
  // compute the run time signature of BB, and run time adjusting signature
  // wherever required.
  DenseMap<BasicBlock *, Instruction *> StoreIMap;
  DenseMap<BasicBlock *, Instruction *> XorIMap;
  for (BasicBlock &BB : F) {
    if (&BB == ErrBB)
      continue;

    unsigned BBS = SMap[&BB];
    IRBuilder<> builder(&(*(BB.getFirstInsertionPt())));

    if (BB.hasNPredecessorsOrMore(1)) {
      unsigned FPS = SMap[*pred_begin(&BB)];

      // Run time adjusting signature logic is required when BB has 2 or more
      // predecessors.
      //
      // FIXME: According to me, this logic is broken for the cases, where a
      // predecessor P has two (or more succssors), say S1 and S2, and both S1
      // and S2 are branch-fan-in nodes, meaning both S1 and S2 have two (or
      // more) predecessors, with P as common predecessor of both S1 and S2.
      //
      if (BB.hasNPredecessorsOrMore(2)) {
        for (auto it = pred_begin(&BB), end = pred_end(&BB); it != end; ++it) {
          BasicBlock *PBB = *it;
          unsigned CPS = SMap[PBB];
          unsigned AS = (it == pred_begin(&BB)) ? 0 : FPS ^ CPS;
          IRBuilder<> builder(PBB->getTerminator());
          builder.CreateStore(Constant::getIntegerValue(I64Ty, APInt(64, AS)),
                              D);
        }
      }

      unsigned SD = FPS ^ BBS;
      Instruction *XorI = cast<Instruction>(
          builder.CreateXor(builder.CreateLoad(I64Ty, G),
                            Constant::getIntegerValue(I64Ty, APInt(64, SD))));
      if (BB.hasNPredecessorsOrMore(2)) {
        XorI = cast<Instruction>(
            builder.CreateXor(XorI, builder.CreateLoad(I64Ty, D)));
      }
      Instruction *StoreI = cast<Instruction>(builder.CreateStore(XorI, G));
      XorIMap.insert(std::make_pair(&BB, XorI));
      StoreIMap.insert(std::make_pair(&BB, StoreI));
    } else {
      builder.CreateStore(Constant::getIntegerValue(I64Ty, APInt(64, BBS)), G);
    }
  }

  // Within each basic block BB, insert "exit on control flow error" branch and
  // accordingly split BB.
  for (auto &Entry : XorIMap) {
    BasicBlock *BB = Entry.first;
    Instruction *XorI = Entry.second;
    Instruction *StoreI = StoreIMap[BB];

    IRBuilder<> builder(StoreI);
    Value *Cond = builder.CreateICmpNE(
        XorI, Constant::getIntegerValue(I64Ty, APInt(64, SMap[BB])));
    SplitBlockAndInsertIfThen(Cond, StoreI, false, nullptr,
                              static_cast<DomTreeUpdater *>(nullptr), nullptr,
                              ErrBB);
  }

  // Invalidate all analysis results for function F
  auto &FAM = AM.getResult<FunctionAnalysisManagerModuleProxy>(*M).getManager();
  FAM.invalidate(F, PreservedAnalyses::none());
}

PreservedAnalyses CFCSSPass::run(Module &M, ModuleAnalysisManager &AM) {
  // Within very first visited module, insert the definition of "exit on control
  // flow error" function.
  if (!CttErrFunc) {
    CttErrFunc = insertControlFlowErrorHandlingFunction(M);
  }

  // Within very first visited module, insert a global variable which holds the
  // run time computed signature of each basic block BB upon the entry to BB.
  if (!G) {
    auto *I64Ty = Type::getInt64Ty(M.getContext());
    G = new GlobalVariable(M, I64Ty, false, GlobalValue::ExternalLinkage,
                           UndefValue::get(I64Ty), "cfcss.signature");
  }

  // Within very first visited module, insert a global variable which holds the
  // run time adjusting signature wherever required.
  if (!D) {
    auto *I64Ty = Type::getInt64Ty(M.getContext());
    D = new GlobalVariable(M, I64Ty, false, GlobalValue::ExternalLinkage,
                           UndefValue::get(I64Ty), "cfcss.adjusting.signature");
  }

  // Apply CFCSS algorithm to each function within the module M.
  for (Function &F : M) {
    if (F.isDeclaration())
      continue;

    if (&F == CttErrFunc)
      continue;

    applyCFCSS(F, AM);
  }

  // Invalidate all analysis results.
  AM.invalidate(M, PreservedAnalyses::none());

  return PreservedAnalyses::none();
}
