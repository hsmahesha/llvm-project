//===-- CFCSS.h - CFCSS Implementation --------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_TRANSFORMS_CFCSS_CFCSS_H
#define LLVM_TRANSFORMS_CFCSS_CFCSS_H

#include "llvm/IR/PassManager.h"

namespace llvm {

class CFCSSPass : public PassInfoMixin<CFCSSPass> {
public:
  CFCSSPass() : CttErrFunc(nullptr), G(nullptr), D(nullptr) {}
  PreservedAnalyses run(Module &M, ModuleAnalysisManager &AM);

private:
  void applyCFCSS(Function &F, ModuleAnalysisManager &AM);

private:
  Function *CttErrFunc;
  GlobalVariable *G;
  GlobalVariable *D;
};

} // namespace llvm

#endif // LLVM_TRANSFORMS_CFCSS_CFCSS_H
